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
// =============================================================================

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
#include "secrets.h"

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
#define SCREEN_W    128
#define SCREEN_H    64

// Optoacoplador DST-1R8P — sectores 4-8 del Agrónic
#define OPTO_S4  2
#define OPTO_S5  3
#define OPTO_S6  4
#define OPTO_S7  5
#define OPTO_S8  6

#define NUM_SECTORS  5
#define DEBOUNCE_MS  100

// RS485 MAX3485 → VFD ABB ACQ80
#define VFD_RX   47
#define VFD_TX   48
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

#define HMAC_KEY_LEN   16
#define MAC_LEN         4
#define MAX_PKT_LEN    12
#define MAX_RETRIES     3
#define RETRY_MS        1000
#define ACK_TIMEOUT_MS  4000

struct __attribute__((packed)) LoRaPacket  { uint8_t t; uint8_t node; uint8_t valve; uint8_t cmd; uint32_t id; uint8_t mac[4]; };
struct __attribute__((packed)) LoRaStatus  { uint8_t t; uint8_t from; uint8_t type; uint8_t detail; uint32_t id; };
struct __attribute__((packed)) LoRaJoin    { uint8_t t; uint32_t chipId; uint8_t mac[4]; };
struct __attribute__((packed)) LoRaRegister{ uint8_t t; uint32_t chipId; uint8_t assignedId; uint8_t mac[4]; };

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

#define VP_FREQ   V0
#define VP_CURR   V1
#define VP_STATUS V2
#define VP_TEMP   V3
#define VP_N1     V4
#define VP_N2     V5
#define VP_N3     V6

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

uint32_t          msgCounter = 0;
volatile bool     gwRxFlag   = false;

// ============================================================
// 8. FREERTOS — HANDLES
// ============================================================
QueueHandle_t     cmdQueue;
SemaphoreHandle_t displayMutex;
SemaphoreHandle_t radioMutex;
SemaphoreHandle_t nodesMutex;

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
// 13. DISPLAY — SISTEMA COMPLETO
// ============================================================
static DisplayPage  currentPage      = PAGE_NODES;
static bool         displayOn        = false;
static unsigned long displayLastActivity = 0;
static uint8_t      carouselOffset   = 0;
static unsigned long carouselLastTick = 0;
static bool         inManualMenu     = false;
static uint8_t      manualCursor     = 0;   // sector seleccionado (0..NUM_SECTORS-1)

// Helper: imprime línea con truncado a 21 chars (ancho pantalla a size 1)
void dispLine(uint8_t row, const char* fmt, ...) {
  char buf[24];
  va_list args; va_start(args, fmt); vsnprintf(buf, sizeof(buf), fmt, args); va_end(args);
  display.setCursor(0, row * 8);
  display.print(buf);
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

// Pantalla 1 — Nodos y health
void renderPageNodes() {
  uint8_t count;
  if (xSemaphoreTake(nodesMutex, 5) == pdTRUE) {
    count = registeredCount;
    xSemaphoreGive(nodesMutex);
  } else { count = registeredCount; }

  dispLine(0, "=NODOS  %d/%d=", count, MAX_NODES);
  float espTemp = temperatureRead();
  dispLine(1, "GW %.0fC W:%s B:%s", espTemp,
    WiFi.isConnected() ? "Y" : "N", Blynk.connected() ? "Y" : "N");

  // Hasta 6 nodos visibles; si hay más, el carrusel desplaza
  uint8_t visible = min((uint8_t)6, count);
  uint8_t start   = (count > 6) ? (carouselOffset % (count - 5)) : 0;
  for (uint8_t i = 0; i < visible; i++) {
    uint8_t n = start + i + 1;
    if (n > count) break;
    const char* st = !nodes[n].alive ? "CAIDO" : nodes[n].fault ? "FALLO" : "OK   ";
    unsigned long ago = (millis() - nodes[n].lastSeen) / 1000;
    dispLine(i + 2, "N%d %s %lus", n, st, ago);
  }
}

// Pantalla 2 — Alertas recientes
void renderPageAlerts() {
  dispLine(0, "=ALERTAS  %d=", alertLogCount);
  if (alertLogCount == 0) { dispLine(1, "(sin alertas)"); return; }
  uint8_t visible = min((uint8_t)7, alertLogCount);
  uint8_t start   = (alertLogCount > 7) ? (carouselOffset % (alertLogCount - 6)) : 0;
  for (uint8_t i = 0; i < visible; i++) {
    uint8_t idx = (alertLogHead - alertLogCount + start + i + ALERT_LOG_SIZE) % ALERT_LOG_SIZE;
    display.setCursor(0, (i + 1) * 8);
    char buf[22]; strncpy(buf, alertLog[idx].msg, 21); buf[21] = '\0';
    display.print(buf);
  }
}

// Pantalla 3 — Sectores en riego
void renderPageSectors() {
  dispLine(0, "=SECTORES=");
  for (uint8_t i = 0; i < NUM_SECTORS; i++) {
    dispLine(i + 1, "S%d->N%dV%d %s", i + 4,
      SECTOR_MAP[i].loraNode, SECTOR_MAP[i].loraValve,
      sectorState[i] ? "RIEGO " : "parado");
  }
  dispLine(6, "");
  dispLine(7, "[MANT]=prg largo");
}

// Pantalla 4 — VFD info
void renderPageVfd() {
  dispLine(0, "=VFD ABB ACQ80=");
  dispLine(1, "Estado: %s%s", vfdRunning ? "RUN" : "STOP", vfdFault ? " FALLO" : "");
  dispLine(2, "Freq:  %.2f Hz", vfdRaw[1] / 100.0f);
  dispLine(3, "Corr:  %.1f A",  vfdRaw[2] / 10.0f);
  dispLine(4, "VFD ID:%d", VFD_ID);
  dispLine(5, "Poll: 5s");
}

// Menú control manual — overlay
void renderMenuManual() {
  dispLine(0, "=CONTROL MANUAL=");
  for (uint8_t i = 0; i < NUM_SECTORS; i++) {
    char arrow = (i == manualCursor) ? '>' : ' ';
    dispLine(i + 1, "%cS%d %s", arrow, i + 4, sectorState[i] ? "ABIERTO" : "cerrado");
  }
  dispLine(6, "");
  dispLine(7, "PRG=mover LNG=act");
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
      LoRaRegister r = {PKT_REGISTER, j->chipId, n};
      hmacReg(&r, r.mac);
      if (xSemaphoreTake(radioMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        radio.transmit((uint8_t*)&r, sizeof(r));
        radio.startReceive();
        xSemaphoreGive(radioMutex);
      }
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
// 16. ENVÍO LORA CON REINTENTOS
// ============================================================
bool sendCmd(uint8_t node, uint8_t valve, uint8_t cmd) {
  msgCounter++;
  prefs.putUInt("msgId", msgCounter);
  LoRaPacket pkt = {PKT_COMMAND, node, valve, cmd, msgCounter};
  hmacPacket(&pkt, pkt.mac);

  for (uint8_t attempt = 0; attempt < MAX_RETRIES; attempt++) {
    if (xSemaphoreTake(radioMutex, pdMS_TO_TICKS(1000)) != pdTRUE) continue;
    radio.transmit((uint8_t*)&pkt, sizeof(pkt));
    radio.startReceive();
    xSemaphoreGive(radioMutex);

    Serial.printf("[TX] %d/%d N%d V%d C%d ID%d\n",
      attempt+1, MAX_RETRIES, node, valve, cmd, msgCounter);

    // Espera ACK
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
            if (s->from == node && s->type == 0x00 && s->id == msgCounter) {
              if (node <= registeredCount) nodes[node].fault = false;
              return true;
            }
          }
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
  }

  if (node <= registeredCount) nodes[node].fault = true;
  pushAlert(("Sin ACK N" + String(node) + " V" + String(valve)).c_str());
  return false;
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
  int st = radio.begin(868.0, 125.0, 9, 7, 18, 22, 8, 1.8, false);
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

    // 4. Procesar UN comando de la cola
    if (xQueueReceive(cmdQueue, &item, 0) == pdTRUE) {
      sendCmd(item.node, item.valve, item.cmd);
      if (xSemaphoreTake(radioMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        radio.startReceive();
        xSemaphoreGive(radioMutex);
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

  // WiFi + Blynk
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long wt = millis();
  while (!WiFi.isConnected() && millis() - wt < 15000) {
    vTaskDelay(pdMS_TO_TICKS(500)); Serial.print(".");
  }
  if (WiFi.isConnected()) {
    Serial.printf("\n✓ WiFi (%s)\n", WiFi.localIP().toString().c_str());
    Blynk.config(BLYNK_TOKEN);
    Blynk.connect(5000);
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

    // Push salud cada 60s
    if (millis() - lastHealthPush > 60000) {
      lastHealthPush = millis();
      if (Blynk.connected()) {
        Blynk.virtualWrite(VP_TEMP, temperatureRead());
        for (uint8_t n = 1; n <= 3; n++) {
          uint8_t st = (n > registeredCount) ? 0 : (nodes[n].fault ? 2 : (nodes[n].alive ? 1 : 2));
          if (n==1) Blynk.virtualWrite(VP_N1, st);
          else if (n==2) Blynk.virtualWrite(VP_N2, st);
          else Blynk.virtualWrite(VP_N3, st);
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
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[WARN] Fallo OLED");
  }
  display.clearDisplay(); display.display();
  Serial.println("✓ OLED OK (Core 0 taskDisplay)");

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
  cmdQueue    = xQueueCreate(16, sizeof(CmdItem));
  displayMutex = xSemaphoreCreateMutex();
  radioMutex   = xSemaphoreCreateMutex();
  nodesMutex   = xSemaphoreCreateMutex();

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
