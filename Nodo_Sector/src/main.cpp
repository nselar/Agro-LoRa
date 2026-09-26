#include <Arduino.h>
#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <mbedtls/md.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <Preferences.h>

// ==========================================
// 1. PINES (Heltec WiFi LoRa 32 V3)
// ==========================================
#define VEXT          36   // Control VEXT: LOW=ON, HIGH=OFF

// SX1262
#define LORA_NSS      8
#define LORA_DIO1     14
#define LORA_NRST     12
#define LORA_BUSY     13
#define LORA_SCK      9
#define LORA_MISO     11
#define LORA_MOSI     10

// OLED SSD1306
#define OLED_SDA      17
#define OLED_SCL      18
#define OLED_RST      21
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

// DRV8833 — pines GPIO 2,4,5,6,7 (cluster compacto, fácil de cablear)
// Canal A → Válvula 1 (solenoide latch Baccara 9V)
#define PIN_AIN1       4   // AIN1: pulso ABRIR  V1
#define PIN_AIN2       2   // AIN2: pulso CERRAR V1 (GPIO3 era strap → GPIO2)
// Canal B → Válvula 2
#define PIN_BIN1       6   // BIN1: pulso ABRIR  V2
#define PIN_BIN2       5   // BIN2: pulso CERRAR V2
// Enable
#define PIN_STBY      7   // STBY/nSLEEP: LOW=standby, HIGH=activo
#define PIN_FAULT     3   // nFAULT DRV8833 (open-drain, pull-up R10 en placa): LOW=fallo
// Boost MT3608 (10.2V) — GPIO47 (GPIO15/16 NO existen en headers Heltec V3)
#define PIN_BOOST_EN  47  // HIGH=ON, 10k pulldown en placa (OFF al boot)

// Diagnóstico nFAULT: 1 = imprime [DIAG] con nFAULT en cada etapa de accionarValvula()
#define FAULT_DIAG 1

// Batería (Heltec V3): divisor 390k/100k en GPIO1, habilitado con ADC_CTRL en LOW
#define VBAT_ADC_PIN      1
#define VBAT_ADC_CTRL     37
#define VBAT_DIV_FACTOR   4.9f  // (390k + 100k) / 100k

// ==========================================
// 2. CONFIGURACIÓN DEL NODO
// ==========================================
#define MAX_NODES        8
#define SLEEP_DAY_S      10    // Ciclo día: 8h-19h (latencia aprox. 0-10s)
#define SLEEP_NIGHT_S   300    // Ciclo noche: 19h-8h (5 min)
#define LISTEN_DAY_MS   2000   // Ventana escucha LoRa — día
#define LISTEN_NIGHT_MS  500   // Ventana mínima — noche (irrigación inactiva)
#define HB_EVERY_DAY     10   // Heartbeat cada 10 ciclos = ~2 min (día)
#define HB_EVERY_NIGHT    2   // Heartbeat cada 2 ciclos = 10 min (noche, < 12 min timeout GW)
#define SECTOR_DISABLE_DEEP_SLEEP 0  // TEST: 1=escucha continua, 0=producción batería
#define LISTEN_AWAKE_MS  1000        // Ventana por ciclo sin deep sleep (latencia ~0-1s)
#define HB_EVERY_AWAKE     60        // Heartbeat cada ~60s en modo escucha continua
#define WDT_TIMEOUT_S    10
#define JOIN_TIMEOUT_MS  8000
// Compatibilidad con código que usa SLEEP_INTERVAL_S y LISTEN_WINDOW_MS
#define SLEEP_INTERVAL_S SLEEP_DAY_S
#define LISTEN_WINDOW_MS LISTEN_DAY_MS

// Display on-demand (batería)
#define BUTTON_PIN       0
#define DISPLAY_ON_MS    20000  // 20s visible tras PRG (más tiempo para menú manual)
#define BTN_LONG_MS      800

// Ciclos de wakeup sin ACK del gateway antes de considerarlo caído.
// Día:   60 ciclos × ~12s = ~12 min (= HB_TIMEOUT gateway)
// Noche:  3 ciclos × 300s = 15 min (> HB_TIMEOUT de 12 min)
#define GW_CONN_MAX_WAKES_DAY    60
#define GW_CONN_MAX_WAKES_NIGHT   3

// ==========================================
// 3. SEGURIDAD – PROTOCOLO Y CLAVES
// ==========================================
#define PKT_COMMAND   0xA1  // Gateway → Nodo
#define PKT_STATUS    0xB2  // Nodo → Gateway

// Subtipos PKT_STATUS
#define STATUS_ACK        0x00  // ACK de comando
#define STATUS_HEARTBEAT  0x01  // Pulso de vida
#define STATUS_BOOT       0x02  // Reinicio anormal
#define STATUS_MANUAL     0x03  // Acción manual desde nodo → informar al gateway

#define PKT_JOIN     0xC3
#define PKT_REGISTER 0xD4
#define PKT_SYNC     0xE5  // Gateway → nodos: hora actual para modo día/noche

#define HMAC_KEY_LEN 16
#define MAC_LEN      4
#define MAX_PKT_LEN  12

#include "secrets.h"

// ==========================================
// 4. ESTRUCTURAS DE PAQUETES
// ==========================================
struct __attribute__((packed)) LoRaPacket {
  uint8_t  pktType;
  uint8_t  targetNode;
  uint8_t  valve;
  uint8_t  command;      // 1=ABRIR 2=CERRAR 3=PING
  uint32_t messageId;
  uint8_t  mac[MAC_LEN];
};

struct __attribute__((packed)) LoRaStatus {
  uint8_t  pktType;
  uint8_t  fromNode;
  uint8_t  type;
  uint8_t  detail;
  uint32_t messageId;
};

struct __attribute__((packed)) LoRaJoin {
  uint8_t  pktType;
  uint32_t chipId;
  uint8_t  mac[MAC_LEN];
};

struct __attribute__((packed)) LoRaRegister {
  uint8_t  pktType;
  uint32_t chipId;
  uint8_t  assignedId;
  uint8_t  mac[MAC_LEN];
};

struct __attribute__((packed)) LoRaSync {
  uint8_t pktType;  // PKT_SYNC 0xE5
  uint8_t hour;     // 0-23
  uint8_t mac[4];   // HMAC-SHA256 truncado sobre {pktType, hour}
};

// ==========================================
// 5. VARIABLES RTC (sobreviven al deep sleep)
// ==========================================
RTC_DATA_ATTR uint32_t lastMessageId  = 0;
RTC_DATA_ATTR uint32_t wakeCount      = 0;
RTC_DATA_ATTR bool     valve1Open     = false;
RTC_DATA_ATTR bool     valve2Open     = false;
// UINT32_MAX = nunca recibido ACK; 0 = ACK recibido en ciclo actual; incrementa cada ciclo.
// Seguro en deep sleep: millis() se resetea pero wakesSinceAck no depende del tiempo.
RTC_DATA_ATTR uint32_t wakesSinceAck = UINT32_MAX;
RTC_DATA_ATTR bool     nightMode      = false;  // Actualizado por PKT_SYNC del gateway
RTC_DATA_ATTR bool     driverFault    = false;  // nFAULT del DRV8833 visto en el último pulso

// ==========================================
// 6. OBJETOS GLOBALES
// ==========================================
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
Preferences prefs;

uint8_t  nodeId = 0;
volatile bool receivedFlag = false;

// Display on-demand
bool          displayOn      = false;
unsigned long displayOnStart = 0;

// Botón PRG
bool          btnPressed     = false;
bool          btnLongFired   = false;
unsigned long btnPressStart  = 0;

// Pantallas del nodo campo
// PAGE_INFO: estado general (pantalla 1)
// PAGE_MANUAL: control manual de válvulas (pantalla 2)
enum NodePage { PAGE_INFO = 0, PAGE_MANUAL = 1 };
NodePage currentPage  = PAGE_INFO;
uint8_t  manualCursor = 0;   // 0=V1, 1=V2

// ==========================================
// 7. ISR
// ==========================================
#if defined(ESP32)
  IRAM_ATTR
#endif
void setFlag() { receivedFlag = true; }

// ==========================================
// 8. HMAC
// ==========================================
void computeHMAC(const LoRaPacket* p, uint8_t out[MAC_LEN]) {
  uint8_t full[32];
  const uint8_t payload[8] = {
    p->pktType, p->targetNode, p->valve, p->command,
    (uint8_t)p->messageId, (uint8_t)(p->messageId>>8),
    (uint8_t)(p->messageId>>16), (uint8_t)(p->messageId>>24)
  };
  mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                  HMAC_KEY, HMAC_KEY_LEN, payload, 8, full);
  memcpy(out, full, MAC_LEN);
}

bool verifyHMAC(const LoRaPacket* p) {
  uint8_t exp[MAC_LEN]; computeHMAC(p, exp);
  return memcmp(exp, p->mac, MAC_LEN) == 0;
}

void computeHMACJoin(const LoRaJoin* j, uint8_t out[MAC_LEN]) {
  uint8_t full[32];
  const uint8_t payload[5] = {
    j->pktType,
    (uint8_t)j->chipId, (uint8_t)(j->chipId>>8),
    (uint8_t)(j->chipId>>16), (uint8_t)(j->chipId>>24)
  };
  mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                  HMAC_KEY, HMAC_KEY_LEN, payload, 5, full);
  memcpy(out, full, MAC_LEN);
}

void computeHMACRegister(const LoRaRegister* r, uint8_t out[MAC_LEN]) {
  uint8_t full[32];
  const uint8_t payload[6] = {
    r->pktType,
    (uint8_t)r->chipId, (uint8_t)(r->chipId>>8),
    (uint8_t)(r->chipId>>16), (uint8_t)(r->chipId>>24),
    r->assignedId
  };
  mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                  HMAC_KEY, HMAC_KEY_LEN, payload, 6, full);
  memcpy(out, full, MAC_LEN);
}

bool verifyHMACRegister(const LoRaRegister* r) {
  uint8_t exp[MAC_LEN]; computeHMACRegister(r, exp);
  return memcmp(exp, r->mac, MAC_LEN) == 0;
}

bool verifyHMACSync(const LoRaSync* s) {
  uint8_t full[32];
  uint8_t d[2] = {s->pktType, s->hour};
  mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                  HMAC_KEY, HMAC_KEY_LEN, d, 2, full);
  return memcmp(full, s->mac, 4) == 0;
}

// ==========================================
// 9. CONEXIÓN CON GATEWAY
// ==========================================
bool gatewayConnected() {
  if (wakesSinceAck == UINT32_MAX) return false;
  uint32_t maxW = nightMode ? GW_CONN_MAX_WAKES_NIGHT : GW_CONN_MAX_WAKES_DAY;
  return wakesSinceAck < maxW;
}

// ==========================================
// 10. DISPLAY — HELPERS (portrait 64×128)
// ==========================================
// dispLine: size 1, row×8 px, máx 10 chars.
static void dispLine(uint8_t row, const char* fmt, ...) {
  char buf[12];
  va_list args; va_start(args, fmt); vsnprintf(buf, sizeof(buf), fmt, args); va_end(args);
  display.setTextSize(1);
  display.setCursor(0, row * 8);
  display.print(buf);
}
// dispBig: size 2 (12×16 px/char), y en píxeles, máx 5 chars.
static void dispBig(uint8_t y_px, const char* fmt, ...) {
  char buf[7];
  va_list args; va_start(args, fmt); vsnprintf(buf, sizeof(buf), fmt, args); va_end(args);
  display.setTextSize(2);
  display.setCursor(0, y_px);
  display.print(buf);
  display.setTextSize(1);
}

// Lectura de batería: el divisor del Heltec V3 solo conduce con ADC_CTRL en LOW.
// Se devuelve a INPUT tras leer para no drenar la batería por el divisor.
static float readBatteryV() {
  pinMode(VBAT_ADC_CTRL, OUTPUT);
  digitalWrite(VBAT_ADC_CTRL, LOW);
  delay(5);  // asentar divisor
  uint32_t mv = 0;
  for (uint8_t i = 0; i < 8; i++) mv += analogReadMilliVolts(VBAT_ADC_PIN);
  pinMode(VBAT_ADC_CTRL, INPUT);
  return (mv / 8) * VBAT_DIV_FACTOR / 1000.0f;
}

// ==========================================
// 11. DISPLAY — PANTALLA 1: INFO (portrait)
// ==========================================
// Layout 64×128 px:
// y=0  s1: ID + modo día/noche
// y=9  s2: GW:OK / GW:X  (big — crítico)
// y=26 s2: V1:ON / V1:--
// y=43 s2: V2:ON / V2:--
// y=61 s1: batería
// y=70 s1: temperatura
// y=79 s1: ciclos
// y=88 s1: último ACK
// y=112 s1: hints navegación
void renderPageInfo() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  bool conn = gatewayConnected();

  dispLine(0, "NODO#%d %s", nodeId, nightMode ? "NOC" : "DIA");
  dispBig(9,  "GW:%s", conn ? "OK" : "X ");
  dispBig(26, "V1:%s", valve1Open ? "ON" : "--");
  dispBig(43, "V2:%s", valve2Open ? "ON" : "--");

  dispLine(7,  "Bat:%.2fV", readBatteryV());
  dispLine(8,  "T:%.0fC", temperatureRead());
  dispLine(9,  "C:%d", wakeCount);
  if (wakesSinceAck == UINT32_MAX) {
    dispLine(10, "ACK:nunca");
  } else {
    dispLine(10, "ACK:%dW", wakesSinceAck);  // ciclos desde último ACK
  }
  dispLine(11, "DRV:%s", driverFault ? "FALLO" : "ok");
  dispLine(14, "LNG=manual");
  dispLine(15, "PRG=pant");

  display.display();
}

// ==========================================
// 12. DISPLAY — PANTALLA 2: CONTROL MANUAL
// ==========================================
// Menú de 3 opciones: V1, V2, SALIR
// Short press: mueve cursor · Long press: ejecuta acción
void renderPageManual() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  dispLine(0, "CTRL MAN");

  // V1
  dispLine(2, "%cV1:%s", manualCursor == 0 ? '>' : ' ',
    valve1Open ? "ABIER" : "cerrd");
  dispLine(3, " [%s]", valve1Open ? "CERRAR" : "ABRIR ");

  // V2
  dispLine(5, "%cV2:%s", manualCursor == 1 ? '>' : ' ',
    valve2Open ? "ABIER" : "cerrd");
  dispLine(6, " [%s]", valve2Open ? "CERRAR" : "ABRIR ");

  // Volver
  dispLine(8, "%c[VOLVER]", manualCursor == 2 ? '>' : ' ');

  dispLine(11, "PRG=mover");
  dispLine(12, "LNG=OK");

  display.display();
}


// ==========================================
// 12. DISPLAY — CONTROL GENERAL
// ==========================================
void wakeDisplay() {
  display.ssd1306_command(SSD1306_DISPLAYON);
  displayOn      = true;
  displayOnStart = millis();
}

void sleepDisplay() {
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  displayOn = false;
}

void renderDisplay() {
  if (!displayOn) return;
  switch (currentPage) {
    case PAGE_INFO:   renderPageInfo();   break;
    case PAGE_MANUAL: renderPageManual(); break;
  }
}

// ==========================================
// 13. ACCIONAR VÁLVULA + NOTIFICAR GATEWAY
// ==========================================
// forward-declaration
void sendStatus(uint8_t type, uint8_t detail, uint32_t msgId);

void accionarValvula(uint8_t valve, bool abrir) {
  uint8_t pinA = (valve == 1) ? PIN_AIN1 : PIN_BIN1;
  uint8_t pinB = (valve == 1) ? PIN_AIN2 : PIN_BIN2;

  Serial.printf(">>> V%d %s <<<\n", valve, abrir ? "APERTURA" : "CIERRE");

#if FAULT_DIAG
  // Muestras de nFAULT (1=alto=OK, 0=bajo=fallo). Se imprimen al final para no alargar el pulso.
  uint8_t fIdle, fBoost, fEnabled, fP1, fP10, fP30, fRel, fStby;
  fIdle = digitalRead(PIN_FAULT);   // Driver en standby, boost OFF: debe ser 1 (pull-up R10)
#endif

  // Pre-carga del reservorio (40mF) antes del pulso: MT3608 limita a ~0.5A
  digitalWrite(PIN_BOOST_EN, HIGH);  // Boost ON → 10.2V
  delay(1500);                       // ~1s carga + margen de regulación
#if FAULT_DIAG
  fBoost = digitalRead(PIN_FAULT);  // Boost ON, driver aún en standby
#endif

  digitalWrite(PIN_STBY, HIGH);  // Activar driver DRV8833
  delay(1);
#if FAULT_DIAG
  fEnabled = digitalRead(PIN_FAULT);  // Driver activo, salidas en LOW (sin corriente)
#endif

  if (abrir) { digitalWrite(pinA, HIGH); digitalWrite(pinB, LOW); }
  else       { digitalWrite(pinA, LOW);  digitalWrite(pinB, HIGH); }

#if FAULT_DIAG
  delay(1);  fP1  = digitalRead(PIN_FAULT);
  delay(9);  fP10 = digitalRead(PIN_FAULT);
  delay(19); fP30 = digitalRead(PIN_FAULT);
  delay(5);  // Pulso latch total 35ms para Baccara 9V
  driverFault = (fP30 == LOW);
#else
  delay(30);
  // nFAULT LOW = sobrecorriente / térmico / UVLO del DRV8833. Muestrear con el pulso activo.
  driverFault = (digitalRead(PIN_FAULT) == LOW);
  delay(5);   // Pulso latch total 35ms para Baccara 9V
#endif

  digitalWrite(pinA, LOW);
  digitalWrite(pinB, LOW);
  delay(1);
#if FAULT_DIAG
  fRel = digitalRead(PIN_FAULT);  // Salidas ya en LOW, driver aún activo
#endif
  digitalWrite(PIN_STBY, LOW);  // Standby → ahorro energía
#if FAULT_DIAG
  delay(1);
  fStby = digitalRead(PIN_FAULT);  // De vuelta en standby
#endif

  digitalWrite(PIN_BOOST_EN, LOW);  // Boost OFF → Iq < 1µA (drenaje 0 batería)

#if FAULT_DIAG
  Serial.printf("[DIAG] nFAULT V%d idle:%d boost:%d en:%d p1ms:%d p10ms:%d p30ms:%d rel:%d stby:%d\n",
    valve, fIdle, fBoost, fEnabled, fP1, fP10, fP30, fRel, fStby);
#endif
  if (driverFault) {
    Serial.printf("[WARN] DRV8833 nFAULT activo durante pulso V%d\n", valve);
  }

  // Actualizar estado en RTC (persiste en deep sleep)
  if (valve == 1) valve1Open = abrir;
  else            valve2Open = abrir;

  Serial.println("--- Válvula en reposo (alta impedancia) ---");
}

// Abrir/cerrar válvula iniciado MANUALMENTE desde el nodo
// y notificar al gateway con STATUS_MANUAL
void accionarValvulaManual(uint8_t valve, bool abrir) {
  accionarValvula(valve, abrir);

  // Notificar al gateway: type=STATUS_MANUAL, detail=valve, messageId=apertura/cierre
  // detail: bit0=valve, bit1=0(cerrar)/1(abrir) → encode en un byte
  uint8_t detail = (valve & 0x0F) | (abrir ? 0x10 : 0x00);
  sendStatus(STATUS_MANUAL, detail, ++lastMessageId);

  Serial.printf("[MANUAL] V%d %s → notificado al gateway\n",
    valve, abrir ? "ABIERTA" : "CERRADA");
}

// ==========================================
// 14. ENVIAR STATUS AL GATEWAY
// ==========================================
void sendStatus(uint8_t type, uint8_t detail, uint32_t msgId) {
  LoRaStatus s;
  s.pktType   = PKT_STATUS;
  s.fromNode  = nodeId;
  s.type      = type;
  s.detail    = detail;
  s.messageId = msgId;
  radio.transmit((uint8_t*)&s, sizeof(LoRaStatus));
}

// ==========================================
// 15. BOTÓN PRG — LÓGICA COMPLETA
// ==========================================
void processButton() {
  bool raw = (digitalRead(BUTTON_PIN) == LOW);

  if (raw && !btnPressed) {
    btnPressed    = true;
    btnLongFired  = false;
    btnPressStart = millis();
  }

  if (btnPressed && !btnLongFired &&
      (millis() - btnPressStart) > BTN_LONG_MS) {
    btnLongFired = true;

    // --- Long press ---
    if (!displayOn) {
      // Pantalla apagada: encender y mostrar info
      wakeDisplay();
      currentPage = PAGE_INFO;
      renderDisplay();
      return;
    }

    if (currentPage == PAGE_INFO) {
      // Long press en info → ir a menú manual
      currentPage  = PAGE_MANUAL;
      manualCursor = 0;
      displayOnStart = millis();  // Resetear timeout
      renderDisplay();
    }
    else if (currentPage == PAGE_MANUAL) {
      // Long press en menú → ejecutar acción del cursor
      if (manualCursor == 0) {
        // Toggle V1
        accionarValvulaManual(1, !valve1Open);
        renderDisplay();
      }
      else if (manualCursor == 1) {
        // Toggle V2
        accionarValvulaManual(2, !valve2Open);
        renderDisplay();
      }
      else {
        // VOLVER: salir al menú de info
        currentPage = PAGE_INFO;
        renderDisplay();
      }
    }
  }

  if (!raw && btnPressed) {
    if (!btnLongFired) {
      // --- Short press ---
      if (!displayOn) {
        // Apagada: encender y mostrar info
        wakeDisplay();
        currentPage = PAGE_INFO;
        renderDisplay();
      }
      else if (currentPage == PAGE_INFO) {
        // Info → Manual (ciclo)
        currentPage  = PAGE_MANUAL;
        manualCursor = 0;
        displayOnStart = millis();
        renderDisplay();
      }
      else if (currentPage == PAGE_MANUAL) {
        // Mover cursor: 0→1→2→0
        manualCursor = (manualCursor + 1) % 3;
        displayOnStart = millis();  // Resetear timeout al interactuar
        renderDisplay();
      }
    }
    btnPressed = false;
  }

  // Auto-apagado
  if (displayOn && (millis() - displayOnStart) > DISPLAY_ON_MS) {
    sleepDisplay();
    currentPage = PAGE_INFO;  // Volver a info al apagarse
  }
}

// ==========================================
// 16. DEEP SLEEP
// ==========================================
void goToSleep() {
  uint32_t sleepS = nightMode ? SLEEP_NIGHT_S : SLEEP_DAY_S;
  Serial.printf("[SLEEP] → %ds (%s) V1:%s V2:%s\n",
    sleepS, nightMode ? "NOCHE" : "DIA",
    valve1Open ? "ABT" : "CER",
    valve2Open ? "ABT" : "CER");

  sleepDisplay();
  digitalWrite(VEXT, HIGH);  // Apagar OLED y TCXO

  // Incrementar contador de wakes sin ACK antes de dormir (se resetea a 0 al recibir ACK)
  if (wakesSinceAck != UINT32_MAX) wakesSinceAck++;

  esp_sleep_enable_timer_wakeup((uint64_t)sleepS * 1000000ULL);
  esp_deep_sleep_start();
}

// ==========================================
// 17. JOIN PROTOCOL
// ==========================================
bool performJoin() {
  uint32_t myChipId = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFF);
  Serial.printf("[JOIN] ChipId: 0x%08X\n", myChipId);
  wakeDisplay();
  renderPageInfo();

  LoRaJoin j;
  j.pktType = PKT_JOIN;
  j.chipId  = myChipId;
  computeHMACJoin(&j, j.mac);

  radio.setDio1Action(setFlag);

  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    Serial.printf("[JOIN] Intento %d/3\n", attempt + 1);
    radio.transmit((uint8_t*)&j, sizeof(j));
    radio.startReceive();

    unsigned long t = millis();
    while (millis() - t < JOIN_TIMEOUT_MS) {
      esp_task_wdt_reset();
      if (receivedFlag) {
        receivedFlag = false;
        uint8_t buf[MAX_PKT_LEN] = {0};
        if (radio.readData(buf, sizeof(buf)) == RADIOLIB_ERR_NONE &&
            buf[0] == PKT_REGISTER) {
          const LoRaRegister* reg = (const LoRaRegister*)buf;
          if (reg->chipId == myChipId && verifyHMACRegister(reg)) {
            nodeId = reg->assignedId;
            prefs.begin("node", false);
            prefs.putUChar("id", nodeId);
            prefs.end();
            wakesSinceAck = 0;  // Primer contacto exitoso
            Serial.printf("[JOIN] ID asignado: %d\n", nodeId);
            return true;
          }
        }
        radio.startReceive();
      }
      delay(10);
    }
  }

  Serial.println("[JOIN] Timeout.");
  return false;
}

// ==========================================
// 18. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
  bool isTimerWakeup = (wakeup_cause == ESP_SLEEP_WAKEUP_TIMER);

  if (!isTimerWakeup) {
    delay(2000);
    Serial.println("\n--- NODO CAMPO v2.0 ---");
  }

  // VEXT ON → alimenta OLED y TCXO del SX1262
  pinMode(VEXT, OUTPUT);
  digitalWrite(VEXT, LOW);
  delay(50);

  // Botón PRG
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // DRV8833
  pinMode(PIN_AIN1,  OUTPUT); digitalWrite(PIN_AIN1,  LOW);
  pinMode(PIN_AIN2,  OUTPUT); digitalWrite(PIN_AIN2,  LOW);
  pinMode(PIN_BIN1,  OUTPUT); digitalWrite(PIN_BIN1,  LOW);
  pinMode(PIN_BIN2,  OUTPUT); digitalWrite(PIN_BIN2,  LOW);
  pinMode(PIN_STBY, OUTPUT); digitalWrite(PIN_STBY, LOW);  // Standby → ahorro (se sube en accionarValvula)
  pinMode(PIN_FAULT, INPUT);  // pull-up externo R10; GPIO3 strap solo relevante con eFuse JTAG_SEL
  pinMode(PIN_BOOST_EN, OUTPUT); digitalWrite(PIN_BOOST_EN, LOW);  // Boost OFF (10k pulldown en placa)

  // OLED: inicializar APAGADO (ahorrar batería)
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[WARN] Fallo OLED");
  }
  display.clearDisplay(); display.display();
  display.setRotation(1);  // 90° CW — nodo montado vertical → canvas 64×128 px
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  displayOn = false;

  // En primer arranque: mostrar pantalla brevemente
  if (!isTimerWakeup) {
    wakeDisplay();
    currentPage = PAGE_INFO;
    renderDisplay();
  }

  // SX1262 LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  // Sin antena: ≤5 dBm. Con antena FRP 3-5 dBi: 10 dBm (≤25 mW ERP, legal EU 868 MHz).
  int state = radio.begin(868.0, 125.0, 9, 7, 18, 10, 8, 1.8, false);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ERROR] LoRa: %d\n", state);
    goToSleep();
  }
  radio.setDio2AsRfSwitch(true);

  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  // Recuperar nodeId desde NVS
  prefs.begin("node", true);
  nodeId = prefs.getUChar("id", 0);
  prefs.end();

  if (nodeId == 0) {
    if (!performJoin()) goToSleep();
  }

  // Reportar reset anormal
  esp_reset_reason_t rc = esp_reset_reason();
  if (rc == ESP_RST_PANIC || rc == ESP_RST_INT_WDT ||
      rc == ESP_RST_TASK_WDT || rc == ESP_RST_BROWNOUT) {
    Serial.printf("[ALERTA] Reset anormal: %d\n", (int)rc);
    sendStatus(STATUS_BOOT, (uint8_t)rc, 0);
    delay(300);
  }

  radio.setDio1Action(setFlag);
  Serial.printf("[OK] Nodo #%d  V1:%s V2:%s  Ciclo:%d\n",
    nodeId,
    valve1Open ? "ABT" : "CER",
    valve2Open ? "ABT" : "CER",
    wakeCount);
#if SECTOR_DISABLE_DEEP_SLEEP
  Serial.println("[TEST] Deep sleep desactivado: escucha continua");
#endif
}

// ==========================================
// 19. LOOP — CICLO PRODUCCIÓN (wake → escucha → sleep)
// ==========================================
void loop() {
  wakeCount++;
  esp_task_wdt_reset();

  uint32_t listenMs = nightMode ? LISTEN_NIGHT_MS : LISTEN_DAY_MS;
  uint8_t  hbEvery  = nightMode ? HB_EVERY_NIGHT   : HB_EVERY_DAY;
#if SECTOR_DISABLE_DEEP_SLEEP
  listenMs = LISTEN_AWAKE_MS;
  hbEvery  = HB_EVERY_AWAKE;
#endif

  // Heartbeat periódico
  if (wakeCount % hbEvery == 0) {
    Serial.printf("[HB] Ciclo %d (%s)\n", wakeCount, nightMode ? "NOCHE" : "DIA");
    sendStatus(STATUS_HEARTBEAT, 0, wakeCount);
  }

  // Ventana de escucha LoRa
  receivedFlag = false;
  radio.startReceive();
  unsigned long listenStart = millis();

  while (millis() - listenStart < listenMs) {
    esp_task_wdt_reset();
    processButton();   // Botón PRG activo durante la ventana de escucha

    if (receivedFlag) {
      receivedFlag = false;

      uint8_t buf[MAX_PKT_LEN] = {0};
      int rxState = radio.readData(buf, sizeof(buf));

      if (rxState == RADIOLIB_ERR_NONE) {
        if (buf[0] == PKT_COMMAND) {
          const LoRaPacket* pkt = (const LoRaPacket*)buf;

          Serial.printf("[RX] N:%d V:%d Cmd:%d ID:%d\n",
            pkt->targetNode, pkt->valve, pkt->command, pkt->messageId);

          if (pkt->targetNode != nodeId) {
            Serial.println("[RX] No es para este nodo.");
          }
          else if (!verifyHMAC(pkt)) {
            Serial.println("[SEG] HMAC invalido, descartado.");
          }
          else if (pkt->messageId <= lastMessageId) {
            Serial.printf("[SEG] ID %d repetido, descartado.\n", pkt->messageId);
          }
          else {
            lastMessageId = pkt->messageId;
            wakesSinceAck = 0;

            if (pkt->command == 1 || pkt->command == 2) {
              bool abrir = (pkt->command == 1);
              if (displayOn) renderDisplay();
              accionarValvula(pkt->valve, abrir);
              sendStatus(STATUS_ACK, pkt->valve, pkt->messageId);
              if (displayOn) renderDisplay();
            }
            else if (pkt->command == 3) {
              sendStatus(STATUS_ACK, 0, pkt->messageId);
            }
          }
        }
        else if (buf[0] == PKT_SYNC) {
          const LoRaSync* sync = (const LoRaSync*)buf;
          if (verifyHMACSync(sync)) {
            nightMode = (sync->hour < 8 || sync->hour >= 19);
            Serial.printf("[SYNC] Hora %02dh → %s\n",
              sync->hour, nightMode ? "NOCHE" : "DIA");
          } else {
            Serial.println("[SYNC] HMAC invalido, descartado.");
          }
        }
      }

      radio.startReceive();
    }

    delay(10);
  }

  // Auto-apagado display (transcurrido DISPLAY_ON_MS tras última interacción)
  if (displayOn && (millis() - displayOnStart) > DISPLAY_ON_MS) {
    sleepDisplay();
    currentPage = PAGE_INFO;
  }

#if SECTOR_DISABLE_DEEP_SLEEP
  delay(10);
#else
  goToSleep();
#endif
}
