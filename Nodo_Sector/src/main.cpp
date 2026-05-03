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

// DRV8833 – Canal A (Válvula 1)
#define PIN_IN1       4
#define PIN_IN2       5
// DRV8833 – Canal B (Válvula 2)
#define PIN_IN3       38
#define PIN_IN4       39
// DRV8833 – STBY
#define PIN_STBY      7

// ==========================================
// 2. CONFIGURACIÓN DEL NODO
// ==========================================
#define MAX_NODES        8
#define SLEEP_INTERVAL_S 30
#define LISTEN_WINDOW_MS 3000
#define HEARTBEAT_EVERY  10
#define WDT_TIMEOUT_S    10
#define JOIN_TIMEOUT_MS  8000

// Display on-demand (batería)
#define BUTTON_PIN       0
#define DISPLAY_ON_MS    20000  // 20s visible tras PRG (más tiempo para menú manual)
#define BTN_LONG_MS      800

// Timeout de conexión con gateway (2 ciclos sin ACK = sin conexión)
#define GW_CONN_TIMEOUT_MS  (2UL * SLEEP_INTERVAL_S * 1000 + 10000)

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

// ==========================================
// 5. VARIABLES RTC (sobreviven al deep sleep)
// ==========================================
RTC_DATA_ATTR uint32_t lastMessageId  = 0;
RTC_DATA_ATTR uint32_t wakeCount      = 0;
RTC_DATA_ATTR bool     valve1Open     = false;
RTC_DATA_ATTR bool     valve2Open     = false;
RTC_DATA_ATTR unsigned long lastGwAck = 0;  // Timestamp del último ACK exitoso del gateway

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

// ==========================================
// 9. CONEXIÓN CON GATEWAY
// ==========================================
// Devuelve true si hemos recibido un ACK del gateway en los últimos GW_CONN_TIMEOUT_MS
// lastGwAck sobrevive al deep sleep en RTC_DATA_ATTR
bool gatewayConnected() {
  if (lastGwAck == 0) return false;  // Nunca recibido ACK
  return (millis() - lastGwAck) < GW_CONN_TIMEOUT_MS;
}

// ==========================================
// 10. DISPLAY — PANTALLA 1: INFO
// ==========================================
void renderPageInfo() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Línea 0: ID + modo
  display.setCursor(0, 0);
  display.printf("=NODO #%d  868MHz=", nodeId > 0 ? nodeId : 0);

  // Línea 1: conexión con gateway
  bool conn = gatewayConnected();
  display.setCursor(0, 8);
  display.printf("GW: %s", conn ? "CONECTADO   " : "SIN CONEXION");

  // Línea 2: estado válvulas
  display.setCursor(0, 16);
  display.printf("V1: %-8s  V2: %-8s",
    valve1Open ? "ABIERTA" : "cerrada",
    valve2Open ? "ABIERTA" : "cerrada");

  // Línea 3: ciclos y último mensaje
  display.setCursor(0, 24);
  display.printf("Ciclos:%-5d MsgID:%-4d", wakeCount, lastMessageId);

  // Línea 4: batería (ADC GPIO1 con divisor ×2 en Heltec V3)
  int raw  = analogRead(1);
  float vBat = raw * (3.3f / 4095.0f) * 2.0f;
  display.setCursor(0, 32);
  display.printf("Bat: %.2fV  Temp:%.0fC", vBat, temperatureRead());

  // Línea 5: tiempo desde último ACK del gateway
  display.setCursor(0, 40);
  if (lastGwAck == 0) {
    display.print("GW ult ACK: nunca");
  } else {
    unsigned long ago = (millis() - lastGwAck) / 1000;
    display.printf("GW ult ACK: %lus ago", ago);
  }

  // Línea 6: vacía / separador
  display.setCursor(0, 48);
  display.print("--------------------");

  // Línea 7: instrucción navegación
  display.setCursor(0, 56);
  display.print("PRG:pant  LNG:manual");

  display.display();
}

// ==========================================
// 11. DISPLAY — PANTALLA 2: CONTROL MANUAL
// ==========================================
// Menú de 3 opciones: V1, V2, SALIR
// Short press: mueve cursor · Long press: ejecuta acción
void renderPageManual() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("=CONTROL MANUAL=");

  // Opción 0: Válvula 1
  display.setCursor(0, 12);
  display.printf("%cV1: %-8s [%s]",
    manualCursor == 0 ? '>' : ' ',
    valve1Open ? "ABIERTA" : "cerrada",
    valve1Open ? "CERRAR" : "ABRIR ");

  // Opción 1: Válvula 2
  display.setCursor(0, 24);
  display.printf("%cV2: %-8s [%s]",
    manualCursor == 1 ? '>' : ' ',
    valve2Open ? "ABIERTA" : "cerrada",
    valve2Open ? "CERRAR" : "ABRIR ");

  // Opción 2: Salir al menú de info
  display.setCursor(0, 36);
  display.printf("%c[VOLVER A INFO]",
    manualCursor == 2 ? '>' : ' ');

  display.setCursor(0, 48);
  display.print("--------------------");
  display.setCursor(0, 56);
  display.print("PRG=mover  LNG=OK");

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
  uint8_t pinA = (valve == 1) ? PIN_IN1 : PIN_IN3;
  uint8_t pinB = (valve == 1) ? PIN_IN2 : PIN_IN4;

  Serial.printf(">>> V%d %s <<<\n", valve, abrir ? "APERTURA" : "CIERRE");

  digitalWrite(PIN_STBY, HIGH);
  delay(1);

  if (abrir) { digitalWrite(pinA, HIGH); digitalWrite(pinB, LOW); }
  else       { digitalWrite(pinA, LOW);  digitalWrite(pinB, HIGH); }

  delay(100);  // Pulso latch 100ms para Baccara 9V

  digitalWrite(pinA, LOW);
  digitalWrite(pinB, LOW);
  delay(1);
  digitalWrite(PIN_STBY, LOW);

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
  Serial.printf("[SLEEP] → %ds  V1:%s V2:%s\n",
    SLEEP_INTERVAL_S,
    valve1Open ? "ABT" : "CER",
    valve2Open ? "ABT" : "CER");

  sleepDisplay();
  digitalWrite(VEXT, HIGH);  // Apagar OLED y TCXO

  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_INTERVAL_S * 1000000ULL);
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
            lastGwAck = millis();  // Primer contacto exitoso
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
  pinMode(PIN_IN1,  OUTPUT); digitalWrite(PIN_IN1,  LOW);
  pinMode(PIN_IN2,  OUTPUT); digitalWrite(PIN_IN2,  LOW);
  pinMode(PIN_IN3,  OUTPUT); digitalWrite(PIN_IN3,  LOW);
  pinMode(PIN_IN4,  OUTPUT); digitalWrite(PIN_IN4,  LOW);
  pinMode(PIN_STBY, OUTPUT); digitalWrite(PIN_STBY, LOW);

  // OLED: inicializar APAGADO (ahorrar batería)
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[WARN] Fallo OLED");
  }
  display.clearDisplay(); display.display();
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
  int state = radio.begin(868.0, 125.0, 9, 7, 18, 22, 8, 1.8, false);
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
}

// ==========================================
// 19. LOOP (un ciclo → deep sleep al final)
// ==========================================
void loop() {
  wakeCount++;
  esp_task_wdt_reset();

  // Heartbeat periódico
  if (wakeCount % HEARTBEAT_EVERY == 0) {
    Serial.printf("[HB] Ciclo %d\n", wakeCount);
    sendStatus(STATUS_HEARTBEAT, 0, wakeCount);
  }

  // Ventana de escucha LoRa
  receivedFlag = false;
  radio.startReceive();
  unsigned long listenStart = millis();

  while (millis() - listenStart < LISTEN_WINDOW_MS) {
    esp_task_wdt_reset();
    processButton();   // Botón PRG activo durante la ventana de escucha

    if (receivedFlag) {
      receivedFlag = false;

      uint8_t buf[MAX_PKT_LEN] = {0};
      int rxState = radio.readData(buf, sizeof(buf));

      if (rxState == RADIOLIB_ERR_NONE && buf[0] == PKT_COMMAND) {
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
          lastGwAck     = millis();  // Gateway respondió → conexión activa

          if (pkt->command == 1 || pkt->command == 2) {
            bool abrir = (pkt->command == 1);
            if (displayOn) renderDisplay();  // Actualizar si visible
            accionarValvula(pkt->valve, abrir);
            sendStatus(STATUS_ACK, pkt->valve, pkt->messageId);
            if (displayOn) renderDisplay();  // Refrescar estado válvulas
          }
          else if (pkt->command == 3) {
            // PING del gateway → ACK + actualizar timestamp conexión
            sendStatus(STATUS_ACK, 0, pkt->messageId);
          }
        }
      }

      radio.startReceive();
    }

    delay(10);
  }

  // Apagar pantalla si llevan más de DISPLAY_ON_MS
  if (displayOn && (millis() - displayOnStart) > DISPLAY_ON_MS) {
    sleepDisplay();
    currentPage = PAGE_INFO;
  }

  goToSleep();
}
