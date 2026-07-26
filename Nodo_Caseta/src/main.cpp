// =============================================================================
// GATEWAY CASETA — Agrónic IoT  v2.0
// Hardware: Heltec WiFi LoRa 32 V3 (ESP32-S3 dual-core)
//
// ARQUITECTURA FREERTOS:
//   Core 1 (alta prioridad) — taskRealTime:
//     · Lectura optoacopladores DST-1R8P (sectores Agrónic)
//     · TX/RX LoRa Star P2P (comandos, ACK, JOIN, heartbeat)
//     · Gestión botón PRG (corto = ciclar pantalla, largo = menú manual)
//     · Cola de comandos FIFO
//   Core 0 (baja prioridad) — taskConnectivity:
//     · WiFi + Blynk
//     · Modbus RTU master VFD ABB ACQ80
//     · Push telemetría periódico
//   Core 0 (baja prioridad) — taskDisplay:
//     · OLED: 4 pantallas + menú control manual
//     · Auto-encendido cuando hay riego activo
//     · Carousel para contenido largo
//
// OPTOACOPLADOR DST-1R8P (NPN): LOW = sector activo
// VCC-OUT del DST-1R8P → pin 3V3 del ESP32
// RS485 VFD: GPIO6=RX (←MAX3485 TX), GPIO7=TX (→MAX3485 RX), GPIO45=DE
// =============================================================================

#include "secrets.h"  // must precede BlynkSimpleEsp32.h (defines BLYNK_TEMPLATE_ID etc.)
#include <Arduino.h>
#include <RadioLib.h>
#include <ModbusRTU.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <mbedtls/md.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ============================================================
// 1. PINES
// ============================================================
#define LORA_NSS    8
#define LORA_DIO1   14
#define LORA_NRST   12
#define LORA_BUSY   13
#define LORA_SCK    9
#define LORA_MISO   11
#define LORA_MOSI   10

#define OLED_SDA    17
#define OLED_SCL    18
#define OLED_RST    21
#define VEXT_PIN    36  // Heltec V3: LOW = OLED power ON
#define SCREEN_W    128
#define SCREEN_H    64

// Optoacoplador DST-1R8P — sectores 4-11 del Agrónic (8 pines cableados, 5 activos)
// NPN: LOW = sector activo. Todos con INPUT_PULLUP.
#define OPTO_S4   5
#define OPTO_S5   4
#define OPTO_S6   3
#define OPTO_S7   2
#define OPTO_S8   1
#define OPTO_S9  38   // futuro
#define OPTO_S10 39   // futuro
#define OPTO_S11 40   // futuro

#define NUM_SECTORS  5
#define DEBOUNCE_MS  100

// RS485 MAX3485 → VFD ABB ACQ80
// GPIO 6 = ESP32 RX ← MAX3485 TX (lee respuestas del VFD)
// GPIO 7 = ESP32 TX → MAX3485 RX (envía comandos Modbus al VFD)
#define VFD_RX    6
#define VFD_TX    7
#define VFD_DE   45

#define BUTTON_PIN  0

// ============================================================
// 2. MAPEO SECTOR → NODO LORA
// ============================================================
struct SectorMap { uint8_t gpio; uint8_t loraNode; uint8_t loraValve; };

static const SectorMap SECTOR_MAP[NUM_SECTORS] = {
  { OPTO_S4, 1, 1 },   // Sector 4 → Nodo 1 V-A
  { OPTO_S5, 1, 2 },   // Sector 5 → Nodo 1 V-B
  { OPTO_S6, 2, 1 },   // Sector 6 → Nodo 2 V-A
  { OPTO_S7, 2, 2 },   // Sector 7 → Nodo 2 V-B
  { OPTO_S8, 3, 1 },   // Sector 8 → Nodo 3 V-A
};

// ============================================================
// 3. PROTOCOLO LORA
// ============================================================
#define PKT_COMMAND  0xA1
#define PKT_STATUS   0xB2
#define PKT_JOIN     0xC3
#define PKT_REGISTER 0xD4
#define PKT_SYNC     0xE5  // Gateway → todos los nodos: hora actual (modo día/noche)

#define HMAC_KEY_LEN   16
#define MAC_LEN         4
#define MAX_PKT_LEN    12
#define MAX_RETRIES     3
#define RETRY_MS        1000
#define ACK_TIMEOUT_MS  4000
// B2: cola persistente para cmds sector/menu. Cubre ciclo sleep del sector:
//   día 30s  → deadline 65s reintenta 2 ventanas escucha
//   noche 300s → no cubre; cmd se descarta pero próximo sector下次/reintentar manualmente
#define CMD_RETRY_MS        2000   // separación entre reintentos
#define CMD_RETRY_DEADLINE  65000   // ms total antes de declarar sin ACK

struct __attribute__((packed)) LoRaPacket  { uint8_t t; uint8_t node; uint8_t valve; uint8_t cmd; uint32_t id; uint8_t mac[4]; };
struct __attribute__((packed)) LoRaStatus  { uint8_t t; uint8_t from; uint8_t type; uint8_t detail; uint32_t id; };
struct __attribute__((packed)) LoRaJoin    { uint8_t t; uint32_t chipId; uint8_t mac[4]; };
struct __attribute__((packed)) LoRaRegister{ uint8_t t; uint32_t chipId; uint8_t assignedId; uint8_t mac[4]; };
struct __attribute__((packed)) LoRaSync    { uint8_t t; uint8_t hour; uint8_t mac[4]; };  // 6 bytes

// ============================================================
// 4. NODOS
// ============================================================
#define MAX_NODES  8
#define HB_TIMEOUT_MS  (12UL * 60 * 1000)

struct NodeHealth { unsigned long lastSeen; bool alive; bool fault; uint8_t resetCause; };
NodeHealth nodes[MAX_NODES + 1];
uint32_t   chipIdMap[MAX_NODES + 1];
volatile uint8_t registeredCount = 0;

// ============================================================
// 5. VFD — MODBUS
// ============================================================
#define VFD_ID    1
#define VFD_BAUD  9600
#define REG_SW    2
#define REG_FREQ  3
#define REG_CURR  4

#define VP_FREQ        V0
#define VP_CURR        V1
#define VP_STATUS      V2
#define VP_TEMP        V3
// Nodos 1-8: V4-V11. Widget LED. 255=ON (con color), 0=OFF.
// Color verde #23C48E = OK, rojo #D3435C = caído/fallo.
#define VP_VALVE_TIMER V12  // Slider: minutos apertura manual (1-120)
// Válvulas: V13=N1V1, V14=N1V2, V15=N2V1, ..., V27=N8V1, V28=N8V2
// Widget Switch (no Button): envía 1=ABRIR, 0=CERRAR.
static inline int valveVpin(uint8_t n, uint8_t v) { return 12 + (n-1)*2 + (v-1); }

// ============================================================
// 6. DISPLAY — MÁQUINA DE ESTADOS
// ============================================================
enum DisplayPage {
  PAGE_NODES = 0,   // Pantalla 1: nodos + health
  PAGE_ALERTS,      // Pantalla 2: alertas recientes
  PAGE_SECTORS,     // Pantalla 3: sectores en riego
  PAGE_VFD,         // Pantalla 4: info VFD
  PAGE_MANUAL,      // Menú control manual (desde pantalla 3 long-press)
  PAGE_COUNT = 4    // Solo 4 páginas normales (MANUAL es overlay)
};

#define DISPLAY_TIMEOUT_MS  30000   // Apagar pantalla tras 30s sin actividad
#define CAROUSEL_MS          2500   // Intervalo scroll de carrusel
#define BTN_LONG_MS           800   // Tiempo para long-press

// Log de alertas (circular, 5 entradas)
#define ALERT_LOG_SIZE  5
struct AlertEntry { char msg[32]; unsigned long ts; };
AlertEntry alertLog[ALERT_LOG_SIZE];
uint8_t    alertLogHead = 0;
uint8_t    alertLogCount = 0;

// ============================================================
// 7. ESTADO COMPARTIDO (protegido con SemaphoreHandle_t)
// ============================================================
volatile bool     sectorState[NUM_SECTORS]     = {false};
volatile bool     prevSectorState[NUM_SECTORS] = {false};
volatile unsigned long lastChange[NUM_SECTORS] = {0};

uint16_t vfdRaw[3]     = {0};  // [SW, FREQ, CURR]
bool     vfdRunning    = false;
bool     vfdFault      = false;
volatile bool vfdPollPending = false;
unsigned long lastVfdPoll    = 0;
unsigned long lastBlynkPush  = 0;
unsigned long lastHealthPush = 0;
unsigned long lastTimeout    = 0;

// Control manual válvulas desde Blynk
bool          valveBlynkOpen[MAX_NODES + 1][3]  = {};   // [node][valve 1-2] estado confirmado (ACK)
unsigned long manualCloseAtMs[MAX_NODES + 1][3] = {};   // 0=sin timer activo
volatile uint8_t manualTimerMin = 30;                   // minutos, desde VP_VALVE_TIMER
volatile bool valveBlynkDirty   = false;                // señal taskRealTime → taskConnectivity

struct ManualCmd {
  bool     active;
  uint8_t  node, valve, cmd;   // cmd: 1=ABRIR 2=CERRAR
  uint32_t deadline;    // millis() límite reintentos (~65s)
  uint32_t lastTryMs;   // millis() último intento
  uint32_t durationMs;  // sólo para ABRIR: ms que queda abierta
};
ManualCmd pendingManual = {};

// B2: cola persistente para cmds sector Agrónic + menú PRG.
// msgId se fija en el encolar → sector dedupe por lastMessageId (idempotente).
// Sólo un cmd activo; nuevo cmd sobreescribe (aceptado: opto debounce + manual serialized).
struct CmdRetry {
  bool     active;
  uint8_t  node, valve, cmd;
  uint32_t msgId;
  uint32_t deadline;
  uint32_t lastTryMs;
};
CmdRetry pendingCmd = {};

uint32_t          msgCounter = 0;
volatile bool     gwRxFlag   = false;

// ============================================================
// 8. FREERTOS — HANDLES
// ============================================================
QueueHandle_t     cmdQueue;
SemaphoreHandle_t displayMutex;
SemaphoreHandle_t radioMutex;
SemaphoreHandle_t nodesMutex;
SemaphoreHandle_t manualMutex;

// ============================================================
// 9. OBJETOS HARDWARE
// ============================================================
SX1262           radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RST);
ModbusRTU        vfd;
Preferences      prefs;

// ============================================================
// 10. ISR LORA
// ============================================================
IRAM_ATTR void gwSetFlag() { gwRxFlag = true; }

// ============================================================
// 11. HMAC
// ============================================================
static void _hmac(const uint8_t* data, size_t len, uint8_t out[MAC_LEN]) {
  uint8_t full[32];
  mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                  HMAC_KEY, HMAC_KEY_LEN, data, len, full);
  memcpy(out, full, MAC_LEN);
}
void hmacPacket(const LoRaPacket* p, uint8_t out[4]) {
  uint8_t d[8] = { p->t,p->node,p->valve,p->cmd,
    (uint8_t)p->id,(uint8_t)(p->id>>8),(uint8_t)(p->id>>16),(uint8_t)(p->id>>24) };
  _hmac(d, 8, out);
}
void hmacJoin(const LoRaJoin* j, uint8_t out[4]) {
  uint8_t d[5] = { j->t,(uint8_t)j->chipId,(uint8_t)(j->chipId>>8),
    (uint8_t)(j->chipId>>16),(uint8_t)(j->chipId>>24) };
  _hmac(d, 5, out);
}
void hmacReg(const LoRaRegister* r, uint8_t out[4]) {
  uint8_t d[6] = { r->t,(uint8_t)r->chipId,(uint8_t)(r->chipId>>8),
    (uint8_t)(r->chipId>>16),(uint8_t)(r->chipId>>24),r->assignedId };
  _hmac(d, 6, out);
}
bool verifyJoin(const LoRaJoin* j) {
  uint8_t exp[4]; hmacJoin(j, exp);
  return memcmp(exp, j->mac, 4) == 0;
}

// ============================================================
// 11b. SINCRONIZACIÓN HORARIA — PKT_SYNC
// ============================================================
static bool isNightHour(int h) { return h < 8 || h >= 19; }

// Envía PKT_SYNC a todos los nodos de campo (broadcast sin dirección).
// Llamar desde taskConnectivity con radioMutex disponible.
void sendSyncToAll(uint8_t hour) {
  LoRaSync s;
  s.t    = PKT_SYNC;
  s.hour = hour;
  uint8_t d[2] = {s.t, s.hour};
  _hmac(d, 2, s.mac);

  if (xSemaphoreTake(radioMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    radio.transmit((uint8_t*)&s, sizeof(s));
    radio.startReceive();
    xSemaphoreGive(radioMutex);
  }
  Serial.printf("[SYNC] Hora %02dh → nodos (%s)\n",
    hour, isNightHour(hour) ? "NOCHE" : "DIA");
}

// ============================================================
// 12. ALERTAS — LOG CIRCULAR
// ============================================================
void pushAlert(const char* msg) {
  Serial.printf("[ALERTA] %s\n", msg);
  AlertEntry& e = alertLog[alertLogHead];
  strncpy(e.msg, msg, 31); e.msg[31] = '\0';
  e.ts = millis();
  alertLogHead = (alertLogHead + 1) % ALERT_LOG_SIZE;
  if (alertLogCount < ALERT_LOG_SIZE) alertLogCount++;

  if (Blynk.connected()) Blynk.logEvent("alerta", msg);
}

// ============================================================
// 12b. CONTROL MANUAL VÁLVULAS — BLYNK
// ============================================================
void pushValveStates() {
  if (!Blynk.connected()) return;
  for (uint8_t n = 1; n <= MAX_NODES; n++) {
    for (uint8_t v = 1; v <= 2; v++) {
      int vpin = valveVpin(n, v);
      if (n > registeredCount) {
        Blynk.virtualWrite(vpin, 0);
        continue;
      }
      bool open = valveBlynkOpen[n][v];
      char lbl[20];
      Blynk.virtualWrite(vpin, open ? 1 : 0);
      if (open && manualCloseAtMs[n][v] > millis()) {
        uint32_t minLeft = (manualCloseAtMs[n][v] - millis()) / 60000UL + 1;
        snprintf(lbl, sizeof(lbl), "N%d-V%d %dmin", n, v, (int)minLeft);
        Blynk.setProperty(vpin, "color", "#23C48E");
      } else if (open) {
        snprintf(lbl, sizeof(lbl), "N%d-V%d ABIERTO", n, v);
        Blynk.setProperty(vpin, "color", "#23C48E");
      } else {
        snprintf(lbl, sizeof(lbl), "N%d-V%d cerrado", n, v);
        Blynk.setProperty(vpin, "color", "#808080");
      }
      Blynk.setProperty(vpin, "label", lbl);
    }
  }
}

void handleValveBlynk(uint8_t n, uint8_t v, int val) {
  if (n < 1 || n > MAX_NODES || v < 1 || v > 2) return;
  bool open  = (val == 1);
  uint8_t cmd = open ? 1 : 2;
  if (!open) manualCloseAtMs[n][v] = 0;
  if (xSemaphoreTake(manualMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    pendingManual = {true, n, v, cmd, millis() + 65000UL, 0,
                     open ? (uint32_t)manualTimerMin * 60000UL : 0};
    xSemaphoreGive(manualMutex);
  }
  Serial.printf("[BLYNK] N%d-V%d %s timer=%dmin\n", n, v,
    open ? "ABRIR" : "CERRAR", (int)manualTimerMin);
}

BLYNK_WRITE(V12)  { manualTimerMin = (uint8_t)constrain(param.asInt(), 1, 120); }
BLYNK_WRITE(V13)  { handleValveBlynk(1,1,param.asInt()); }
BLYNK_WRITE(V14)  { handleValveBlynk(1,2,param.asInt()); }
BLYNK_WRITE(V15)  { handleValveBlynk(2,1,param.asInt()); }
BLYNK_WRITE(V16)  { handleValveBlynk(2,2,param.asInt()); }
BLYNK_WRITE(V17)  { handleValveBlynk(3,1,param.asInt()); }
BLYNK_WRITE(V18)  { handleValveBlynk(3,2,param.asInt()); }
BLYNK_WRITE(V19)  { handleValveBlynk(4,1,param.asInt()); }
BLYNK_WRITE(V20)  { handleValveBlynk(4,2,param.asInt()); }
BLYNK_WRITE(V21)  { handleValveBlynk(5,1,param.asInt()); }
BLYNK_WRITE(V22)  { handleValveBlynk(5,2,param.asInt()); }
BLYNK_WRITE(V23)  { handleValveBlynk(6,1,param.asInt()); }
BLYNK_WRITE(V24)  { handleValveBlynk(6,2,param.asInt()); }
BLYNK_WRITE(V25)  { handleValveBlynk(7,1,param.asInt()); }
BLYNK_WRITE(V26)  { handleValveBlynk(7,2,param.asInt()); }
BLYNK_WRITE(V27)  { handleValveBlynk(8,1,param.asInt()); }
BLYNK_WRITE(V28)  { handleValveBlynk(8,2,param.asInt()); }

// ============================================================
// 13. DISPLAY — SISTEMA COMPLETO
// ============================================================
static DisplayPage  currentPage      = PAGE_NODES;
static bool         displayOn        = false;
static unsigned long displayLastActivity = 0;
static uint8_t      carouselOffset   = 0;
static unsigned long carouselLastTick = 0;
static bool         inManualMenu     = false;
static uint8_t      manualCursor     = 0;   // sector seleccionado (0..NUM_SECTORS-1)

// Portrait 64×128: ~10 chars/línea a size 1, ~5 chars a size 2.
// dispLine: size 1, row × 8 px.
void dispLine(uint8_t row, const char* fmt, ...) {
  char buf[12];
  va_list args; va_start(args, fmt); vsnprintf(buf, sizeof(buf), fmt, args); va_end(args);
  display.setTextSize(1);
  display.setCursor(0, row * 8);
  display.print(buf);
}
// dispBig: size 2 (12×16 px/char), y en píxeles absolutos.
void dispBig(uint8_t y_px, const char* fmt, ...) {
  char buf[7];
  va_list args; va_start(args, fmt); vsnprintf(buf, sizeof(buf), fmt, args); va_end(args);
  display.setTextSize(2);
  display.setCursor(0, y_px);
  display.print(buf);
  display.setTextSize(1);
}

// Devuelve true si algún sector está activo
bool anySectorActive() {
  for (uint8_t i = 0; i < NUM_SECTORS; i++) if (sectorState[i]) return true;
  return false;
}

void displayWake() {
  displayOn = true;
  displayLastActivity = millis();
  display.ssd1306_command(SSD1306_DISPLAYON);
}

void displaySleep() {
  displayOn = false;
  display.ssd1306_command(SSD1306_DISPLAYOFF);
}

// Pantalla 1 — Nodos y health (portrait 64×128)
// Header s1 (3 líneas) + nodos s2 (16px cada uno, espaciado 20px)
// Estado s2: "N1:OK" "N3:FL" "N8:X " — máx 5 chars × 12px = 60px ✓
// Hasta 5 nodos visibles; carrusel si hay más.
void renderPageNodes() {
  uint8_t count;
  if (xSemaphoreTake(nodesMutex, 5) == pdTRUE) {
    count = registeredCount;
    xSemaphoreGive(nodesMutex);
  } else { count = registeredCount; }

  dispLine(0, "NODOS%d/%d", count, MAX_NODES);
  dispLine(1, "W:%s B:%s", WiFi.isConnected() ? "Y" : "N", Blynk.connected() ? "Y" : "N");
  dispLine(2, "T:%.0fC", temperatureRead());

  uint8_t visible = min((uint8_t)5, count);
  uint8_t start   = (count > 5) ? (carouselOffset % (count - 4)) : 0;
  for (uint8_t i = 0; i < visible; i++) {
    uint8_t n = start + i + 1;
    if (n > count) break;
    // "OK"=activo OK, "FL"=fallo, "X "=caído
    const char* st = !nodes[n].alive ? "X " : nodes[n].fault ? "FL" : "OK";
    dispBig(27 + i * 20, "N%d:%s", n, st);
  }
}

// Pantalla 2 — Alertas recientes (portrait 64×128)
// Contador s2 (grande) + 3 alertas más recientes en s1 (10 chars/línea), 2 líneas cada una.
// Mensajes largos se parten en línea 1 (chars 0-9) y línea 2 (chars 10-19).
void renderPageAlerts() {
  dispLine(0, "ALERTAS:");
  if (alertLogCount == 0) {
    dispBig(9, "0");
    dispLine(5, "(ninguna)");
    return;
  }
  dispBig(9, "%d", alertLogCount);  // Número grande — visible de lejos

  uint8_t show = min((uint8_t)3, alertLogCount);
  for (uint8_t i = 0; i < show; i++) {
    // Índice: de más reciente (i=0) a más antiguo (i=show-1)
    uint8_t idx = (alertLogHead - 1 - i + ALERT_LOG_SIZE) % ALERT_LOG_SIZE;
    const char* msg = alertLog[idx].msg;
    uint8_t baseRow = 4 + i * 4;   // Filas 4,5 | 8,9 | 12,13
    dispLine(baseRow,     "%.10s", msg);
    if (strlen(msg) > 10) dispLine(baseRow + 1, "%.10s", msg + 10);
  }
}

// Pantalla 3 — Sectores en riego (portrait, size 2 por sector — lectura desde lejos)
// Layout: 128px tall. Header s1 (8px) + 5×sector s2 (16px, espaciado 23px) + hint s1.
void renderPageSectors() {
  dispLine(0, "SECTORES");
  for (uint8_t i = 0; i < NUM_SECTORS; i++) {
    // "S4:ON" o "S4:--" — 5 chars × 12px = 60px (cabe en 64px)
    dispBig(9 + i * 23, "S%d:%s", i + 4, sectorState[i] ? "ON" : "--");
  }
  dispLine(14, "LNG=MENU");
}

// Pantalla 4 — VFD info (portrait, mix size 1/2)
// y=0  s1: título         (8px)
// y=9  s2: RUN/STOP       (16px) → hasta y=25
// y=26 s1: fallo          (8px)
// y=35 s1: label freq     (8px)
// y=44 s2: valor freq     (16px) → hasta y=60
// y=61 s1: label corr     (8px)
// y=70 s2: valor corr     (16px) → hasta y=86
// y=96 s1: ID y poll      (8px)
void renderPageVfd() {
  dispLine(0, "=VFD ABB=");
  dispBig(9, vfdRunning ? "RUN" : "STOP");
  dispLine(3, vfdFault ? "!!FALLO" : "");
  dispLine(4, "Freq(Hz):");
  dispBig(44, "%.1f", vfdRaw[1] / 100.0f);
  dispLine(7, "Corr(A):");
  dispBig(70, "%.1f", vfdRaw[2] / 10.0f);
  dispLine(12, "ID:%d 5s", VFD_ID);
}

// Menú control manual — overlay (portrait, size 1)
void renderMenuManual() {
  dispLine(0, "CTRL MAN");
  for (uint8_t i = 0; i < NUM_SECTORS; i++) {
    char arrow = (i == manualCursor) ? '>' : ' ';
    dispLine(i + 2, "%cS%d:%s", arrow, i + 4, sectorState[i] ? "ON" : "--");
  }
  dispLine(9,  "PRG=mover");
  dispLine(10, "LNG=activ");
}

// Función principal de render — llamada desde taskDisplay
void renderDisplay() {
  if (!displayOn) return;

  if (xSemaphoreTake(displayMutex, portMAX_DELAY) != pdTRUE) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (inManualMenu) {
    renderMenuManual();
  } else {
    switch (currentPage) {
      case PAGE_NODES:   renderPageNodes();   break;
      case PAGE_ALERTS:  renderPageAlerts();  break;
      case PAGE_SECTORS: renderPageSectors(); break;
      case PAGE_VFD:     renderPageVfd();     break;
      default: break;
    }
  }

  display.display();
  xSemaphoreGive(displayMutex);
}

// ============================================================
// 14. LÓGICA BOTÓN PRG
// ============================================================
// Llamada desde taskRealTime — no bloqueante
struct BtnState {
  bool     pressed;
  bool     longFired;
  unsigned long pressStart;
};
BtnState btn = {false, false, 0};

// Enqueue externo (definido adelante)
struct CmdItem { uint8_t node; uint8_t valve; uint8_t cmd; };
extern QueueHandle_t cmdQueue;

void btnShortPress() {
  displayWake();
  displayLastActivity = millis();

  if (inManualMenu) {
    // Avanzar cursor en el menú
    manualCursor = (manualCursor + 1) % NUM_SECTORS;
    return;
  }
  // Ciclar entre pantallas normales
  currentPage = (DisplayPage)((currentPage + 1) % PAGE_COUNT);
}

void btnLongPress() {
  displayWake();
  displayLastActivity = millis();

  if (inManualMenu) {
    // Ejecutar apertura/cierre del sector seleccionado
    uint8_t node  = SECTOR_MAP[manualCursor].loraNode;
    uint8_t valve = SECTOR_MAP[manualCursor].loraValve;
    uint8_t cmd   = sectorState[manualCursor] ? 2 : 1;  // toggle
    CmdItem item  = {node, valve, cmd};
    xQueueSend(cmdQueue, &item, 0);
    Serial.printf("[MANUAL] S%d → N%dV%d %s\n",
      manualCursor+4, node, valve, cmd==1 ? "ABRIR" : "CERRAR");
    return;
  }

  // Long press en pantalla 3 → entrar menú manual
  if (currentPage == PAGE_SECTORS) {
    inManualMenu  = true;
    manualCursor  = 0;
  }
  // Long press en menú pero no en sector → salir
  // (ya cubierto arriba: si inManualMenu → ejecuta, no sale)
  // Para salir del menú: ciclar con short press hasta PAGE_SECTORS y volver a entrar
  // O añadir double-press en el futuro
}

void processButton() {
  bool raw = (digitalRead(BUTTON_PIN) == LOW);

  if (raw && !btn.pressed) {
    btn.pressed   = true;
    btn.longFired = false;
    btn.pressStart = millis();
  }

  if (btn.pressed && !btn.longFired &&
      (millis() - btn.pressStart) > BTN_LONG_MS) {
    btn.longFired = true;
    btnLongPress();
  }

  if (!raw && btn.pressed) {
    if (!btn.longFired) btnShortPress();
    btn.pressed = false;
  }
}

// ============================================================
// 15. PROCESADO DE PAQUETES LORA
// ============================================================
void processStatus(const LoRaStatus* s) {
  uint8_t n = s->from;
  if (n < 1 || n > registeredCount) return;
  bool wasAlive = nodes[n].alive;
  nodes[n].lastSeen = millis();
  nodes[n].alive    = true;
  switch (s->type) {
    case 0x00:  // ACK de comando
      Serial.printf("[ACK] N%d V%d ID=%d\n", n, s->detail, s->id);
      break;
    case 0x01:  // Heartbeat
      Serial.printf("[HB] N%d\n", n);
      if (!wasAlive) { nodes[n].fault = false; pushAlert(("N" + String(n) + " recuperado").c_str()); }
      break;
    case 0x02:  // Reset anormal
      nodes[n].resetCause = s->detail;
      pushAlert(("N" + String(n) + " reset=" + String(s->detail)).c_str());
      break;
    case 0x03: {  // STATUS_MANUAL: el nodo accionó una válvula localmente
      // detail: bits 0-3 = nº válvula, bit 4 = 1(abrir)/0(cerrar)
      uint8_t valve  = s->detail & 0x0F;
      bool    abrir  = (s->detail >> 4) & 0x01;
      Serial.printf("[MANUAL NODO] N%d V%d %s (iniciado en campo)\n",
        n, valve, abrir ? "ABIERTA" : "CERRADA");
      // Actualizar estado del sector en pantalla del gateway
      // El sector correspondiente se deduce del nodo y válvula
      for (uint8_t i = 0; i < NUM_SECTORS; i++) {
        if (SECTOR_MAP[i].loraNode == n && SECTOR_MAP[i].loraValve == valve) {
          sectorState[i] = abrir;
          prevSectorState[i] = abrir;  // Evitar re-envío del gateway
          Serial.printf("[MANUAL NODO] → sector %d actualizado en gateway\n", i+4);
          break;
        }
      }
      char alertMsg[40];
      snprintf(alertMsg, sizeof(alertMsg), "N%d V%d %s (manual campo)",
               n, valve, abrir ? "ABRIO" : "CERRO");
      pushAlert(alertMsg);
      break;
    }
  }
}


void processJoin(const LoRaJoin* j) {
  if (!verifyJoin(j)) { Serial.println("[JOIN] HMAC inv"); return; }
  for (uint8_t n = 1; n <= registeredCount; n++) {
    if (chipIdMap[n] == j->chipId) {
      nodes[n].lastSeen = millis();  // nodo se reincorporó (reset o re-JOIN)
      nodes[n].alive    = true;
      nodes[n].fault    = false;
      LoRaRegister r = {PKT_REGISTER, j->chipId, n};
      hmacReg(&r, r.mac);
      if (xSemaphoreTake(radioMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        radio.transmit((uint8_t*)&r, sizeof(r));
        radio.startReceive();
        xSemaphoreGive(radioMutex);
      }
      Serial.printf("[JOIN] N%d re-JOIN (0x%08X)\n", n, j->chipId);
      return;
    }
  }
  if (registeredCount >= MAX_NODES) return;
  registeredCount++;
  uint8_t nid = registeredCount;
  chipIdMap[nid] = j->chipId;
  nodes[nid] = {millis(), true, false, 0};
  prefs.putUChar("rc", registeredCount);
  char key[8]; snprintf(key, 8, "c%d", nid); prefs.putUInt(key, j->chipId);

  LoRaRegister r = {PKT_REGISTER, j->chipId, nid};
  hmacReg(&r, r.mac);
  if (xSemaphoreTake(radioMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    radio.transmit((uint8_t*)&r, sizeof(r));
    radio.startReceive();
    xSemaphoreGive(radioMutex);
  }
  Serial.printf("[JOIN] N%d registrado (0x%08X)\n", nid, j->chipId);
  pushAlert(("Nodo " + String(nid) + " unido").c_str());
}

// ============================================================
// 16. ENVÍO LORA — TX ÚNICA + ESPERA ACK
// ============================================================
// Una TX + ventana ACK_TIMEOUT_MS. NO incrementa msgCounter (msgId fijado por caller
// → idempotente entre reintentos). Procesa STATUS colaterales vía processStatus.
bool txAndWaitAck(uint8_t node, uint8_t valve, uint8_t cmd, uint32_t msgId) {
  LoRaPacket pkt = {PKT_COMMAND, node, valve, cmd, msgId};
  hmacPacket(&pkt, pkt.mac);

  if (xSemaphoreTake(radioMutex, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
  radio.transmit((uint8_t*)&pkt, sizeof(pkt));
  radio.startReceive();
  xSemaphoreGive(radioMutex);

  Serial.printf("[TX] N%d V%d C%d ID%d\n", node, valve, cmd, msgId);

  unsigned long t = millis();
  while (millis() - t < ACK_TIMEOUT_MS) {
    if (gwRxFlag) {
      gwRxFlag = false;
      uint8_t buf[MAX_PKT_LEN] = {0};
      if (xSemaphoreTake(radioMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        int st = radio.readData(buf, sizeof(buf));
        radio.startReceive();
        xSemaphoreGive(radioMutex);
        if (st == RADIOLIB_ERR_NONE && buf[0] == PKT_STATUS) {
          const LoRaStatus* s = (const LoRaStatus*)buf;
          processStatus(s);
          if (s->from == node && s->type == 0x00 && s->id == msgId) {
            if (node <= registeredCount) nodes[node].fault = false;
            return true;
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return false;
}

// Encolar cmd sector/menu → reintentado por taskRealTime hasta ACK o CMD_RETRY_DEADLINE.
// msgId fijo entre reintentos → sector dedupe por lastMessageId (nodo descarta repite sin re-fire).
bool sendCmd(uint8_t node, uint8_t valve, uint8_t cmd) {
  msgCounter++;
  prefs.putUInt("msgId", msgCounter);
  pendingCmd = {true, node, valve, cmd, msgCounter, millis() + CMD_RETRY_DEADLINE, 0};
  Serial.printf("[ENQ] N%d V%d C%d ID%d +%lus\n",
    node, valve, cmd, msgCounter, CMD_RETRY_DEADLINE / 1000);
  return true;
}

// ============================================================
// 17. LEER OPTOACOPLADORES
// ============================================================
void readOptocouplers() {
  for (uint8_t i = 0; i < NUM_SECTORS; i++) {
    bool raw = (digitalRead(SECTOR_MAP[i].gpio) == LOW);
    if (raw != sectorState[i]) {
      if (millis() - lastChange[i] > DEBOUNCE_MS) {
        sectorState[i] = raw;
        Serial.printf("[OPTO] S%d %s\n", i+4, raw ? "ACTIVO" : "libre");
      }
    } else {
      lastChange[i] = millis();
    }
  }
}

// ============================================================
// 18. TASK — TIEMPO REAL (Core 1)
// ============================================================
void taskRealTime(void* pv) {
  // Configurar LoRa en este core
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  // Sin antena: ≤5 dBm. Con antena FRP 3-5 dBi: 10 dBm (≤25 mW ERP, legal EU 868 MHz).
  int st = radio.begin(868.0, 125.0, 9, 7, 18, 10, 8, 1.8, false);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("[ERROR] LoRa: %d\n", st);
    while (true) vTaskDelay(portMAX_DELAY);
  }
  radio.setDio2AsRfSwitch(true);
  radio.setDio1Action(gwSetFlag);
  radio.startReceive();
  Serial.println("✓ LoRa SX1262 OK (Core 1)");

  CmdItem item;
  for (;;) {
    // 1. Botón PRG
    processButton();

    // 2. Optoacopladores
    readOptocouplers();

    // 3. Detectar flancos → encolar
    for (uint8_t i = 0; i < NUM_SECTORS; i++) {
      if (sectorState[i] != prevSectorState[i]) {
        prevSectorState[i] = sectorState[i];
        // Despertar pantalla si hay cambio de sector
        if (sectorState[i]) displayWake();
        CmdItem ci = {
          SECTOR_MAP[i].loraNode,
          SECTOR_MAP[i].loraValve,
          (uint8_t)(sectorState[i] ? 1 : 2)
        };
        if (xQueueSend(cmdQueue, &ci, 0) != pdTRUE) {
          pushAlert("Cola CMDs llena!");
        }
        Serial.printf("[FLANCO] S%d → N%dV%d %s (encolado)\n",
          i+4, ci.node, ci.valve, ci.cmd==1 ? "ABRIR" : "CERRAR");
      }
    }

    // 4. Procesar UN comando de la cola (sector Agrónic)
    if (xQueueReceive(cmdQueue, &item, 0) == pdTRUE) {
      sendCmd(item.node, item.valve, item.cmd);
      if (xSemaphoreTake(radioMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        radio.startReceive();
        xSemaphoreGive(radioMutex);
      }
    }

    // 4b. Comando manual Blynk — reintenta hasta 65s para pillar ventana sleep 30s
    {
      bool     mpActive = false;
      uint8_t  mpNode = 0, mpValve = 0, mpCmd = 0;
      uint32_t mpDeadline = 0, mpDuration = 0;

      if (xSemaphoreTake(manualMutex, 0) == pdTRUE) {
        if (pendingManual.active && millis() - pendingManual.lastTryMs > 2000) {
          mpActive   = true;
          mpNode     = pendingManual.node;
          mpValve    = pendingManual.valve;
          mpCmd      = pendingManual.cmd;
          mpDeadline = pendingManual.deadline;
          mpDuration = pendingManual.durationMs;
          pendingManual.lastTryMs = millis();
        }
        xSemaphoreGive(manualMutex);
      }

      if (mpActive) {
        // Nuevo msgId por intento — sector dedupe por lastMessageId (IDs ascenden).
        msgCounter++;
        prefs.putUInt("msgId", msgCounter);
        bool ack = txAndWaitAck(mpNode, mpValve, mpCmd, msgCounter);
        if (ack) {
          valveBlynkOpen[mpNode][mpValve] = (mpCmd == 1);
          if (mpCmd == 1 && mpDuration > 0) {
            manualCloseAtMs[mpNode][mpValve] = millis() + mpDuration;
          } else if (mpCmd == 2) {
            manualCloseAtMs[mpNode][mpValve] = 0;
          }
          if (xSemaphoreTake(manualMutex, portMAX_DELAY) == pdTRUE) {
            pendingManual.active = false;
            xSemaphoreGive(manualMutex);
          }
          valveBlynkDirty = true;
          Serial.printf("[BLYNK] N%d-V%d ACK OK\n", mpNode, mpValve);
        } else if (millis() >= mpDeadline) {
          if (xSemaphoreTake(manualMutex, portMAX_DELAY) == pdTRUE) {
            pendingManual.active = false;
            xSemaphoreGive(manualMutex);
          }
          pushAlert(("BLY N" + String(mpNode) + "V" + String(mpValve) + " sinACK").c_str());
          valveBlynkDirty = true;  // pushValveStates revierte botón al estado real
        }
        // Si aún dentro de deadline y sin ACK → lastTryMs ya actualizado, reintentará
      }
    }

    // 4c. Comando sector/menu — reintento persistente (msgId fijo, idempotente)
    if (pendingCmd.active && millis() - pendingCmd.lastTryMs > CMD_RETRY_MS) {
      pendingCmd.lastTryMs = millis();
      bool ack = txAndWaitAck(pendingCmd.node, pendingCmd.valve, pendingCmd.cmd, pendingCmd.msgId);
      if (ack) {
        Serial.printf("[CMD] N%dV%d ACK OK\n", pendingCmd.node, pendingCmd.valve);
        pendingCmd.active = false;
      } else if (millis() >= pendingCmd.deadline) {
        if (pendingCmd.node <= registeredCount) nodes[pendingCmd.node].fault = true;
        pushAlert(("Sin ACK N" + String(pendingCmd.node) + "V" + String(pendingCmd.valve)).c_str());
        Serial.printf("[CMD] N%dV%d sinACK deadline\n", pendingCmd.node, pendingCmd.valve);
        pendingCmd.active = false;
      }
    }

    // 5. Recepción LoRa (heartbeats, JOINs)
    if (gwRxFlag) {
      gwRxFlag = false;
      uint8_t buf[MAX_PKT_LEN] = {0};
      if (xSemaphoreTake(radioMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        int ret = radio.readData(buf, sizeof(buf));
        radio.startReceive();
        xSemaphoreGive(radioMutex);
        if (ret == RADIOLIB_ERR_NONE) {
          if (buf[0] == PKT_STATUS) processStatus((const LoRaStatus*)buf);
          else if (buf[0] == PKT_JOIN) processJoin((const LoRaJoin*)buf);
        }
      }
    }

    // 6. Timeout nodos (cada 30s)
    if (millis() - lastTimeout > 30000) {
      lastTimeout = millis();
      for (uint8_t n = 1; n <= registeredCount; n++) {
        if (nodes[n].alive && (millis() - nodes[n].lastSeen) > HB_TIMEOUT_MS) {
          nodes[n].alive = false;
          nodes[n].fault = true;
          pushAlert(("N" + String(n) + " timeout").c_str());
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ============================================================
// 19. TASK — CONECTIVIDAD (Core 0)
// ============================================================
bool vfdReadCb(Modbus::ResultCode ev, uint16_t, void*) {
  vfdPollPending = false;
  if (ev != Modbus::EX_SUCCESS) { Serial.printf("[VFD] Err %d\n", ev); return true; }
  bool prevFault = vfdFault;
  vfdRunning = (vfdRaw[0] >> 2) & 1;
  vfdFault   = (vfdRaw[0] >> 3) & 1;
  if (vfdFault && !prevFault) pushAlert("FALLO VFD: trip ABB");
  Serial.printf("[VFD] %s %.2fHz %.1fA\n",
    vfdRunning ? "RUN" : "STOP", vfdRaw[1]/100.0f, vfdRaw[2]/10.0f);
  return true;
}

void taskConnectivity(void* pv) {
  // Modbus
  Serial1.begin(VFD_BAUD, SERIAL_8N1, VFD_RX, VFD_TX);
  vfd.begin(&Serial1, VFD_DE);
  vfd.master();
  Serial.println("✓ Modbus RTU master VFD (Core 0)");

  // WiFi + Blynk + NTP
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long wt = millis();
  while (!WiFi.isConnected() && millis() - wt < 15000) {
    vTaskDelay(pdMS_TO_TICKS(500)); Serial.print(".");
  }
  if (WiFi.isConnected()) {
    Serial.printf("\n✓ WiFi (%s)\n", WiFi.localIP().toString().c_str());
    Blynk.config(BLYNK_TOKEN);
    Blynk.connect(5000);
    // España: UTC+1 invierno (CET), UTC+2 verano (CEST)
    configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");
    Serial.println("✓ NTP configurado");
  } else {
    Serial.println("\n[WARN] WiFi timeout — offline");
  }

  for (;;) {
    if (WiFi.isConnected()) Blynk.run();
    vfd.task();

    // Poll VFD cada 5s
    if (!vfdPollPending && millis() - lastVfdPoll > 5000) {
      lastVfdPoll = millis();
      vfdPollPending = true;
      vfd.readHreg(VFD_ID, REG_SW, vfdRaw, 3, vfdReadCb);
    }

    // Push Blynk VFD cada 30s
    if (millis() - lastBlynkPush > 30000) {
      lastBlynkPush = millis();
      if (Blynk.connected()) {
        Blynk.virtualWrite(VP_FREQ,   vfdRaw[1] / 100.0f);
        Blynk.virtualWrite(VP_CURR,   vfdRaw[2] / 10.0f);
        Blynk.virtualWrite(VP_STATUS, vfdRunning ? "RUN" : "STOP");
      }
    }

    // Push salud + LEDs de nodos cada 60s
    if (millis() - lastHealthPush > 60000) {
      lastHealthPush = millis();
      if (Blynk.connected()) {
        Blynk.virtualWrite(VP_TEMP, temperatureRead());
        // LEDs de nodos: V4-V11 (uno por nodo, máx 8)
        for (uint8_t n = 1; n <= MAX_NODES; n++) {
          int vpin = 3 + n;  // V4..V11
          if (n > registeredCount) {
            Blynk.virtualWrite(vpin, 0);           // Apagado: no registrado
          } else if (!nodes[n].alive || nodes[n].fault) {
            Blynk.setProperty(vpin, "color", "#D3435C");  // Rojo: caído/fallo
            Blynk.virtualWrite(vpin, 255);
          } else {
            Blynk.setProperty(vpin, "color", "#23C48E");  // Verde: OK
            Blynk.virtualWrite(vpin, 255);
          }
        }
      }
    }

    // Auto-cierre válvulas manuales por timer expirado
    for (uint8_t n = 1; n <= registeredCount; n++) {
      for (uint8_t v = 1; v <= 2; v++) {
        if (manualCloseAtMs[n][v] > 0 && millis() >= manualCloseAtMs[n][v]) {
          manualCloseAtMs[n][v] = 0;
          if (xSemaphoreTake(manualMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (!pendingManual.active) {
              pendingManual = {true, n, v, 2, millis() + 65000UL, 0, 0};
            }
            xSemaphoreGive(manualMutex);
          }
          pushAlert(("Timer N" + String(n) + "V" + String(v) + " auto-cierre").c_str());
        }
      }
    }

    // Push estados válvulas Blynk (dirty flag o cada 30s)
    {
      static unsigned long lastValvePush = 0;
      if (valveBlynkDirty || millis() - lastValvePush > 30000) {
        lastValvePush   = millis();
        valveBlynkDirty = false;
        pushValveStates();
      }
    }

    // Sincronización horaria con nodos de campo (modo día/noche)
    {
      static bool lastNightMode   = false;
      static bool initialSyncDone = false;
      struct tm t;
      if (getLocalTime(&t, 0)) {
        bool nightNow = isNightHour(t.tm_hour);
        if (!initialSyncDone || nightNow != lastNightMode) {
          lastNightMode   = nightNow;
          initialSyncDone = true;
          sendSyncToAll((uint8_t)t.tm_hour);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ============================================================
// 20. TASK — DISPLAY (Core 0)
// ============================================================
void taskDisplay(void* pv) {
  // Inicializar OLED
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);  // enable OLED power rail
  delay(10);
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[WARN] Fallo OLED");
  }
  display.clearDisplay(); display.display();
  display.setRotation(1);  // 90° CW — placa montada vertical → canvas 64×128 px
  Serial.println("✓ OLED OK (Core 0 taskDisplay)");
  displayWake();  // show boot screen; auto-sleeps after DISPLAY_TIMEOUT_MS

  for (;;) {
    // Auto-apagado por inactividad (solo si no hay riego activo)
    bool active = anySectorActive();
    if (active && !displayOn) displayWake();
    if (!active && displayOn &&
        (millis() - displayLastActivity) > DISPLAY_TIMEOUT_MS) {
      displaySleep();
    }

    // Carousel tick
    if (millis() - carouselLastTick > CAROUSEL_MS) {
      carouselLastTick = millis();
      carouselOffset++;
    }

    // Render
    renderDisplay();

    vTaskDelay(pdMS_TO_TICKS(200));  // 5 fps — suficiente para OLED
  }
}

// ============================================================
// 21. SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== GATEWAY CASETA — AGRONIC IOT v2.0 (FreeRTOS) ===");

  // GPIOs optoacoplador
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  for (uint8_t i = 0; i < NUM_SECTORS; i++) {
    pinMode(SECTOR_MAP[i].gpio, INPUT_PULLUP);
  }
  Serial.println("✓ GPIOs optoacoplador configurados");

  // FreeRTOS primitivas
  cmdQueue     = xQueueCreate(16, sizeof(CmdItem));
  displayMutex = xSemaphoreCreateMutex();
  radioMutex   = xSemaphoreCreateMutex();
  nodesMutex   = xSemaphoreCreateMutex();
  manualMutex  = xSemaphoreCreateMutex();
  memset(&pendingManual,    0, sizeof(pendingManual));
  memset(&pendingCmd,        0, sizeof(pendingCmd));
  memset(valveBlynkOpen,    0, sizeof(valveBlynkOpen));
  memset(manualCloseAtMs,   0, sizeof(manualCloseAtMs));

  // NVS
  prefs.begin("agro", false);
  msgCounter      = prefs.getUInt("msgId", 0);
  registeredCount = prefs.getUChar("rc", 0);
  for (uint8_t n = 1; n <= registeredCount; n++) {
    char key[8]; snprintf(key, 8, "c%d", n);
    chipIdMap[n] = prefs.getUInt(key, 0);
    nodes[n]     = {millis(), true, false, 0};
    Serial.printf("  Nodo %d: 0x%08X\n", n, chipIdMap[n]);
  }
  Serial.printf("✓ NVS: msgId=%d, nodos=%d\n", msgCounter, registeredCount);

  // Lanzar tareas FreeRTOS
  // taskRealTime → Core 1, prioridad 3 (alta)
  xTaskCreatePinnedToCore(taskRealTime,    "RealTime", 8192, NULL, 3, NULL, 1);
  // taskConnectivity → Core 0, prioridad 1
  xTaskCreatePinnedToCore(taskConnectivity,"Conn",     6144, NULL, 1, NULL, 0);
  // taskDisplay → Core 0, prioridad 1
  xTaskCreatePinnedToCore(taskDisplay,     "Display",  4096, NULL, 1, NULL, 0);

  Serial.println("=== TAREAS FREERTOS LANZADAS ===");
  Serial.println("Core 1: RealTime (LoRa + optoacoplador + botón)");
  Serial.println("Core 0: Connectivity (WiFi + Blynk + VFD)");
  Serial.println("Core 0: Display (OLED 4 pantallas + menú manual)");
}

// loop() vacío — todo va en tareas FreeRTOS
void loop() {
  vTaskDelay(portMAX_DELAY);
}
