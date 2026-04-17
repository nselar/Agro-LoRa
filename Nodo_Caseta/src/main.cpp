#include <Arduino.h>
#include <RadioLib.h>
#include <ModbusRTU.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <mbedtls/md.h>

// ==========================================
// 1. PINES (Heltec WiFi LoRa 32 V3)
// ==========================================
// SX1262
#define LORA_NSS    8
#define LORA_DIO1   14
#define LORA_NRST   12
#define LORA_BUSY   13
#define LORA_SCK    9
#define LORA_MISO   11
#define LORA_MOSI   10

// OLED SSD1306
#define OLED_SDA    17
#define OLED_SCL    18
#define OLED_RST    21
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

// RS485 (MAX3485) – UART1
#define RX_PIN      47
#define TX_PIN      48
#define DE_RE_PIN   45   // HIGH=TX, LOW=RX

// Botón de usuario (PRG)
#define BUTTON_PIN  0

// ==========================================
// 2. SEGURIDAD – PROTOCOLO Y CLAVES
// ==========================================
#define PKT_COMMAND  0xA1   // Gateway → Nodo: comando de válvula
#define PKT_STATUS   0xB2   // Nodo → Gateway: ACK / heartbeat / reset
#define PKT_JOIN     0xC3   // Nodo → Gateway: solicitud de registro
#define PKT_REGISTER 0xD4   // Gateway → Nodo: ID asignado

#define HMAC_KEY_LEN 16
#define MAC_LEN      4      // HMAC-SHA256 truncado a 4 bytes
#define MAX_PKT_LEN  12     // Tamaño del paquete más grande (LoRaPacket)

// Clave compartida – DEBE ser idéntica en gateway y nodo
// IMPORTANTE: cambiar antes del despliegue en campo
static const uint8_t HMAC_KEY[HMAC_KEY_LEN] = {
  /* HMAC key removed — see secrets.h.example */
  /* HMAC key removed */
};

// ==========================================
// 3. ESTRUCTURAS DE PAQUETES
// ==========================================

// Gateway → Nodo (12 bytes)
struct __attribute__((packed)) LoRaPacket {
  uint8_t  pktType;       // PKT_COMMAND (0xA1)
  uint8_t  targetNode;    // ID del nodo destino
  uint8_t  valve;         // 1 o 2 (canal del DRV8833)
  uint8_t  command;       // 1=ABRIR, 2=CERRAR, 3=PING
  uint32_t messageId;     // Contador anti-replay (persistente en NVS)
  uint8_t  mac[MAC_LEN];  // HMAC-SHA256 truncado
};

// Nodo → Gateway (8 bytes)
struct __attribute__((packed)) LoRaStatus {
  uint8_t  pktType;       // PKT_STATUS (0xB2)
  uint8_t  fromNode;      // ID del nodo emisor
  uint8_t  type;          // 0x00=ACK, 0x01=HEARTBEAT, 0x02=BOOT_REASON
  uint8_t  detail;        // ACK: válvula; HB: 0; BOOT: reset cause
  uint32_t messageId;     // ACK: echo cmd ID; HB: wakeCount del nodo
};

// Nodo → Gateway: solicitud de registro (9 bytes)
struct __attribute__((packed)) LoRaJoin {
  uint8_t  pktType;       // PKT_JOIN (0xC3)
  uint32_t chipId;        // ID único del chip (ESP.getEfuseMac() & 0xFFFFFFFF)
  uint8_t  mac[MAC_LEN];  // HMAC(pktType + chipId)
};

// Gateway → Nodo: ID asignado (10 bytes)
struct __attribute__((packed)) LoRaRegister {
  uint8_t  pktType;       // PKT_REGISTER (0xD4)
  uint32_t chipId;        // Echo del chipId solicitante (para que el nodo lo verifique)
  uint8_t  assignedId;    // ID asignado: 1..MAX_NODES
  uint8_t  mac[MAC_LEN];  // HMAC(pktType + chipId + assignedId)
};

// ==========================================
// 4. HEALTH MONITORING
// ==========================================
#define MAX_NODES            8    // Máximo de nodos soportados por la red
// Timeout: si no hay heartbeat en 12 min, el nodo se considera caído
// (los nodos envían heartbeat cada 5 min: 10 ciclos × 30s)
#define HEARTBEAT_TIMEOUT_MS (12UL * 60 * 1000)

struct NodeHealth {
  unsigned long lastSeen;
  bool          alive;
  bool          fault;
  uint8_t       lastResetCause;
};

NodeHealth nodes[MAX_NODES + 1];        // Índices 1..MAX_NODES; índice 0 no se usa
uint32_t   chipIdMap[MAX_NODES + 1];    // Mapa nodeId → chipId (persistido en NVS)
uint8_t    registeredCount = 0;         // Número de nodos actualmente registrados

// ==========================================
// 5. MODBUS
// ==========================================
// Agrónic 2500 actúa como MASTER; gateway como SLAVE ID=1
#define SLAVE_ID 1

// Coils (read/write por Agrónic)
// TOPOLOGÍA ACTUAL: ambas válvulas en el MISMO nodo físico (nodo 1)
// Para nodos separados: cambiar el targetNode en sendLoRaCommand()
#define COIL_VALVE_1  0x00
#define COIL_VALVE_2  0x01

// Discrete Inputs (solo lectura por Agrónic – estado de salud de nodos)
#define ISTS_NODE1_ALIVE  0x00
#define ISTS_NODE2_ALIVE  0x01
#define ISTS_NODE1_FAULT  0x02
#define ISTS_NODE2_FAULT  0x03

// Discrete Inputs – estado VFD (solo lectura por Agrónic)
#define ISTS_VFD_RUNNING  0x04   // 1 = variador en marcha
#define ISTS_VFD_FAULT    0x05   // 1 = variador en fallo/trip

// Input Registers – medidas VFD (solo lectura por Agrónic, FC04)
#define IREG_VFD_FREQ     0x00   // Frecuencia real (0.01 Hz; ej: 5000 = 50.00 Hz)
#define IREG_VFD_CURR     0x01   // Corriente motor (0.1 A;  ej: 15   = 1.5 A)

// ==========================================
// 5b. VARIADOR ABB ACQ80-04 (Modbus MASTER)
// ==========================================
// Serial2 → MAX3485 → bus RS485 dedicado al variador
#define VFD_TX_PIN    19
#define VFD_RX_PIN    20
#define VFD_DE_PIN    15

#define VFD_SLAVE_ID  1     // Parámetro 58.03 del ACQ80
#define VFD_BAUD      9600  // Parámetro 58.01 del ACQ80

// Registros Holding del ACQ80 (perfil "ABB Drives", FC03, 0-indexed)
#define VFD_REG_SW    2     // Status Word    – bit2=RUN, bit3=FAULT
#define VFD_REG_FREQ  3     // Output Freq    – unidad según param 46.01
#define VFD_REG_CURR  4     // Motor Current  – unidad según param 46.02

// ==========================================
// 6. PARÁMETROS DE TRANSMISIÓN
// ==========================================
#define MAX_RETRIES    3
#define RETRY_DELAY_MS 1000
#define ACK_TIMEOUT_MS 4000

// ==========================================
// 7. OBJETOS GLOBALES
// ==========================================
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
ModbusRTU mb;
ModbusRTU vfd;
Preferences prefs;

uint32_t msgCounter = 0;
bool valve1_state = false;
bool valve2_state = false;
unsigned long lastDisplayUpdate  = 0;
unsigned long lastTimeoutCheck   = 0;

volatile bool gwReceivedFlag = false;

// Estado VFD (actualizado por callback Modbus master)
uint16_t      vfdRawData[3]    = {0};  // [SW, FREQ, CURR]
bool          vfdPollPending   = false;
bool          vfdRunning       = false;
bool          vfdFault         = false;
unsigned long lastVfdPoll      = 0;

// ==========================================
// 8. FUNCIONES AUXILIARES
// ==========================================

// ISR: el SX1262 avisa que recibió un paquete
IRAM_ATTR void gwSetFlag(void) {
  gwReceivedFlag = true;
}

// --- HMAC helpers ---

// HMAC para paquete de comando (Gateway → Nodo)
void computeHMAC(const LoRaPacket* p, uint8_t out[MAC_LEN]) {
  uint8_t full_mac[32];
  const uint8_t payload[8] = {
    p->pktType, p->targetNode, p->valve, p->command,
    (uint8_t)(p->messageId),       (uint8_t)(p->messageId >> 8),
    (uint8_t)(p->messageId >> 16), (uint8_t)(p->messageId >> 24)
  };
  mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                  HMAC_KEY, HMAC_KEY_LEN, payload, sizeof(payload), full_mac);
  memcpy(out, full_mac, MAC_LEN);
}

// HMAC para paquete de JOIN (Nodo → Gateway): cubre pktType + chipId
void computeHMACJoin(const LoRaJoin* j, uint8_t out[MAC_LEN]) {
  uint8_t full_mac[32];
  const uint8_t payload[5] = {
    j->pktType,
    (uint8_t)(j->chipId),       (uint8_t)(j->chipId >> 8),
    (uint8_t)(j->chipId >> 16), (uint8_t)(j->chipId >> 24)
  };
  mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                  HMAC_KEY, HMAC_KEY_LEN, payload, sizeof(payload), full_mac);
  memcpy(out, full_mac, MAC_LEN);
}

// HMAC para paquete de REGISTER (Gateway → Nodo): cubre pktType + chipId + assignedId
void computeHMACRegister(const LoRaRegister* r, uint8_t out[MAC_LEN]) {
  uint8_t full_mac[32];
  const uint8_t payload[6] = {
    r->pktType,
    (uint8_t)(r->chipId),       (uint8_t)(r->chipId >> 8),
    (uint8_t)(r->chipId >> 16), (uint8_t)(r->chipId >> 24),
    r->assignedId
  };
  mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                  HMAC_KEY, HMAC_KEY_LEN, payload, sizeof(payload), full_mac);
  memcpy(out, full_mac, MAC_LEN);
}

bool verifyHMACJoin(const LoRaJoin* j) {
  uint8_t expected[MAC_LEN];
  computeHMACJoin(j, expected);
  return (memcmp(expected, j->mac, MAC_LEN) == 0);
}

// --- Display ---

void updateDisplay(const String& status, const String& lastCmd) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.printf("GATEWAY  Reg:%d/%d", registeredCount, MAX_NODES);

  // Estado válvulas y salud de los dos primeros nodos
  display.setCursor(0, 12);
  display.printf("V1:%s%s  V2:%s%s",
    valve1_state ? "ABT" : "CRR",
    (registeredCount >= 1 && nodes[1].fault) ? "!" : " ",
    valve2_state ? "ABT" : "CRR",
    (registeredCount >= 2 && nodes[2].fault) ? "!" : " ");

  display.setCursor(0, 24);
  display.printf("N1:%s  N2:%s",
    (registeredCount >= 1) ? (nodes[1].alive ? "OK " : "OFF") : "---",
    (registeredCount >= 2) ? (nodes[2].alive ? "OK " : "OFF") : "---");

  display.setCursor(0, 36);
  display.printf("VFD:%s%s",
    vfdRunning ? "RUN " : "STOP",
    vfdFault   ? " FLT" : "    ");

  display.setCursor(0, 48);
  display.print(status);

  display.setCursor(0, 56);
  display.print(lastCmd);

  display.display();
}

// Sincroniza el estado de salud de nodos con los Discrete Inputs Modbus
void updateModbusHealth() {
  mb.Ists(ISTS_NODE1_ALIVE, (registeredCount >= 1) ? nodes[1].alive : false);
  mb.Ists(ISTS_NODE2_ALIVE, (registeredCount >= 2) ? nodes[2].alive : false);
  mb.Ists(ISTS_NODE1_FAULT, (registeredCount >= 1) ? nodes[1].fault : false);
  mb.Ists(ISTS_NODE2_FAULT, (registeredCount >= 2) ? nodes[2].fault : false);
}

// Callback Modbus master: se llama cuando llega la respuesta del variador
bool vfdReadCb(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  vfdPollPending = false;
  if (event != Modbus::EX_SUCCESS) {
    Serial.printf("[VFD] Error lectura Modbus: %d\n", event);
    return true;
  }
  uint16_t sw   = vfdRawData[0];
  uint16_t freq = vfdRawData[1];
  uint16_t curr = vfdRawData[2];

  vfdRunning = (sw >> 2) & 0x01;  // Status Word bit 2 = RUN
  vfdFault   = (sw >> 3) & 0x01;  // Status Word bit 3 = FAULT/TRIP

  mb.Ists(ISTS_VFD_RUNNING, vfdRunning);
  mb.Ists(ISTS_VFD_FAULT,   vfdFault);
  mb.Ireg(IREG_VFD_FREQ,    freq);
  mb.Ireg(IREG_VFD_CURR,    curr);

  Serial.printf("[VFD] SW=0x%04X | %s | %.2f Hz | %.1f A\n",
                sw,
                vfdRunning ? "RUN " : "STOP",
                freq / 100.0f,
                curr / 10.0f);
  return true;
}

// Procesa un paquete de estado (ACK / heartbeat / reset) recibido de un nodo
void processNodeStatus(const LoRaStatus* s) {
  uint8_t n = s->fromNode;
  if (n < 1 || n > registeredCount) return;

  bool wasAlive = nodes[n].alive;
  nodes[n].lastSeen = millis();
  nodes[n].alive    = true;

  switch (s->type) {
    case 0x00:  // ACK de comando
      Serial.printf("[ACK] Nodo %d confirmo V%d (ID=%d)\n", n, s->detail, s->messageId);
      break;
    case 0x01:  // Heartbeat
      Serial.printf("[HB] Nodo %d vivo. Ciclos: %d\n", n, s->messageId);
      if (!wasAlive) {
        nodes[n].fault = false;
        Serial.printf("[INFO] Nodo %d recuperado.\n", n);
        updateModbusHealth();
        updateDisplay("NODO RECUPERADO", "N" + String(n) + " vuelve online");
      }
      break;
    case 0x02:  // Boot reason (reset anormal)
      nodes[n].lastResetCause = s->detail;
      Serial.printf("[ALERTA] Nodo %d reinicio anormal! Causa: %d\n", n, s->detail);
      updateDisplay("ALERTA RESET", "N" + String(n) + " causa:" + String(s->detail));
      break;
  }
}

// Procesa una solicitud de registro de un nodo nuevo
// Protocolo: nodo envía PKT_JOIN con su chipId único; gateway asigna ID secuencial y responde
void processJoin(const LoRaJoin* j) {
  if (!verifyHMACJoin(j)) {
    Serial.println("[JOIN] HMAC invalido. Paquete descartado.");
    return;
  }

  // Idempotencia: si el chipId ya está registrado, reenviar el mismo ID
  for (uint8_t n = 1; n <= registeredCount; n++) {
    if (chipIdMap[n] == j->chipId) {
      Serial.printf("[JOIN] Nodo ya registrado (ChipId: 0x%08X → ID: %d). Reenviando REGISTER.\n",
                    j->chipId, n);
      LoRaRegister reg;
      reg.pktType    = PKT_REGISTER;
      reg.chipId     = j->chipId;
      reg.assignedId = n;
      computeHMACRegister(&reg, reg.mac);
      radio.transmit((uint8_t*)&reg, sizeof(LoRaRegister));
      radio.startReceive();
      updateDisplay("JOIN OK", "Re-Reg N" + String(n));
      return;
    }
  }

  // Comprobar límite de nodos
  if (registeredCount >= MAX_NODES) {
    Serial.println("[JOIN] Limite de nodos alcanzado. Solicitud rechazada.");
    return;
  }

  // Asignar nuevo ID secuencial
  registeredCount++;
  uint8_t newId = registeredCount;
  chipIdMap[newId] = j->chipId;

  // Persistir en NVS
  prefs.putUChar("regCount", registeredCount);
  char key[8];
  snprintf(key, sizeof(key), "chip%d", newId);
  prefs.putUInt(key, j->chipId);

  // Inicializar salud del nuevo nodo
  nodes[newId] = { millis(), true, false, 0 };
  updateModbusHealth();

  // Enviar respuesta con ID asignado
  LoRaRegister reg;
  reg.pktType    = PKT_REGISTER;
  reg.chipId     = j->chipId;
  reg.assignedId = newId;
  computeHMACRegister(&reg, reg.mac);
  radio.transmit((uint8_t*)&reg, sizeof(LoRaRegister));
  radio.startReceive();

  Serial.printf("[JOIN] Nodo registrado: ChipId 0x%08X → ID %d (Total: %d)\n",
                j->chipId, newId, registeredCount);
  updateDisplay("NUEVO NODO", "ID:" + String(newId) + " Reg:" + String(registeredCount));
}

// Comprueba si algún nodo supera el timeout de heartbeat
void checkNodeTimeouts() {
  bool changed = false;
  for (uint8_t n = 1; n <= registeredCount; n++) {
    if (nodes[n].alive &&
        (millis() - nodes[n].lastSeen) > HEARTBEAT_TIMEOUT_MS) {
      nodes[n].alive = false;
      nodes[n].fault = true;
      Serial.printf("[ALERTA] Nodo %d SIN RESPUESTA (timeout %lu min)\n",
                    n, HEARTBEAT_TIMEOUT_MS / 60000);
      changed = true;
    }
  }
  if (changed) {
    updateModbusHealth();
    updateDisplay("FALLO NODO", "Ver estado nodos");
  }
}

// Espera ACK del nodo con timeout, manteniendo Modbus activo durante la espera
bool waitForAck(uint8_t expectedNode, uint32_t expectedMsgId) {
  radio.startReceive();
  unsigned long t = millis();

  while (millis() - t < ACK_TIMEOUT_MS) {
    mb.task();  // Mantener Modbus respondiendo al Agrónic durante la espera

    if (gwReceivedFlag) {
      gwReceivedFlag = false;
      uint8_t buf[MAX_PKT_LEN] = {0};

      if (radio.readData(buf, sizeof(buf)) == RADIOLIB_ERR_NONE &&
          buf[0] == PKT_STATUS) {
        const LoRaStatus* s = (const LoRaStatus*)buf;
        processNodeStatus(s);

        if (s->fromNode   == expectedNode  &&
            s->type       == 0x00          &&
            s->messageId  == expectedMsgId) {
          return true;
        }
      }
      radio.startReceive();
    }
    delay(10);
  }
  return false;
}

// Envía un comando LoRa con reintentos y espera de ACK
bool sendLoRaCommand(uint8_t node, uint8_t valve, uint8_t cmd) {
  msgCounter++;
  prefs.putUInt("msgId", msgCounter);

  LoRaPacket packet;
  packet.pktType    = PKT_COMMAND;
  packet.targetNode = node;
  packet.valve      = valve;
  packet.command    = cmd;
  packet.messageId  = msgCounter;
  computeHMAC(&packet, packet.mac);

  for (uint8_t attempt = 0; attempt < MAX_RETRIES; attempt++) {
    Serial.printf("[TX] Intento %d/%d → Nodo:%d V:%d Cmd:%d ID:%d\n",
                  attempt + 1, MAX_RETRIES, node, valve, cmd, msgCounter);

    int state = radio.transmit((uint8_t*)&packet, sizeof(LoRaPacket));
    if (state != RADIOLIB_ERR_NONE) {
      Serial.printf("[TX] Error de transmision: %d\n", state);
      delay(RETRY_DELAY_MS);
      continue;
    }

    if (waitForAck(node, msgCounter)) {
      Serial.println("[TX] Comando confirmado.");
      String label = "N" + String(node) + " V" + String(valve) +
                     (cmd == 1 ? " ABRIO" : (cmd == 2 ? " CERRO" : " PING"));
      updateDisplay("ACK OK", label);
      if (node >= 1 && node <= registeredCount) {
        nodes[node].fault = false;
        updateModbusHealth();
      }
      return true;
    }

    Serial.printf("[TX] Sin ACK. Reintentando en %dms...\n", RETRY_DELAY_MS);
    delay(RETRY_DELAY_MS);
  }

  // Todos los intentos fallaron
  Serial.printf("[ERROR] Nodo %d no respondio tras %d intentos.\n", node, MAX_RETRIES);
  if (node >= 1 && node <= registeredCount) {
    nodes[node].fault = true;
    updateModbusHealth();
  }

  // Revertir el coil para que el Agrónic detecte el fallo en el próximo poll
  if (cmd == 1 || cmd == 2) {
    bool prevState = (cmd == 2);
    uint16_t coilAddr = (valve == 1) ? COIL_VALVE_1 : COIL_VALVE_2;
    mb.Coil(coilAddr, prevState);
    if (valve == 1) valve1_state = prevState;
    else            valve2_state = prevState;
  }

  updateDisplay("ERROR TX", "N" + String(node) + " sin ACK (V" + String(valve) + ")");
  return false;
}

// ==========================================
// 9. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- INICIANDO GATEWAY LORA ---");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Fallo OLED");
  }
  updateDisplay("Iniciando...", "Arrancando sistema");

  // SX1262 LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int state = radio.begin(868.0, 125.0, 9, 7, 18, 22, 8, 1.8, false);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ERROR] Fallo LoRa, codigo: %d\n", state);
    while (true);
  }
  radio.setDio2AsRfSwitch(true);
  radio.setDio1Action(gwSetFlag);
  radio.startReceive();
  Serial.println("✓ Radio LoRa SX1262 Iniciada");

  // Modbus RTU – Gateway como Esclavo (Agrónic es el Master)
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  mb.begin(&Serial1, DE_RE_PIN);
  mb.slave(SLAVE_ID);
  mb.addCoil(COIL_VALVE_1, false);
  mb.addCoil(COIL_VALVE_2, false);
  mb.addIsts(ISTS_NODE1_ALIVE,  false);
  mb.addIsts(ISTS_NODE2_ALIVE,  false);
  mb.addIsts(ISTS_NODE1_FAULT,  false);
  mb.addIsts(ISTS_NODE2_FAULT,  false);
  mb.addIsts(ISTS_VFD_RUNNING,  false);
  mb.addIsts(ISTS_VFD_FAULT,    false);
  mb.addIreg(IREG_VFD_FREQ,     0);
  mb.addIreg(IREG_VFD_CURR,     0);
  Serial.println("✓ Modbus RTU Esclavo Iniciado (ID=" + String(SLAVE_ID) + ")");

  // Variador ABB ACQ80-04 – Modbus master en Serial2
  Serial2.begin(VFD_BAUD, SERIAL_8N1, VFD_RX_PIN, VFD_TX_PIN);
  vfd.begin(&Serial2, VFD_DE_PIN);
  vfd.master();
  Serial.println("✓ Modbus RTU Master VFD Iniciado (Serial2, ID esclavo=" + String(VFD_SLAVE_ID) + ")");

  // NVS: recuperar msgCounter y tabla de registro de nodos
  prefs.begin("agro", false);
  msgCounter = prefs.getUInt("msgId", 0);
  Serial.printf("✓ Contador de mensajes recuperado: %d\n", msgCounter);

  registeredCount = prefs.getUChar("regCount", 0);
  for (uint8_t n = 1; n <= registeredCount; n++) {
    char key[8];
    snprintf(key, sizeof(key), "chip%d", n);
    chipIdMap[n] = prefs.getUInt(key, 0);
    nodes[n] = { millis(), true, false, 0 };  // Asumidos vivos al arrancar
    Serial.printf("  Nodo %d: ChipId 0x%08X\n", n, chipIdMap[n]);
  }
  updateModbusHealth();

  updateDisplay("LISTO", "Esperando ordenes");
  Serial.printf("--- SISTEMA LISTO (%d nodos registrados) ---\n", registeredCount);
  Serial.println("Comandos: A1 C1 A2 C2 PING1 PING2 NODES CLEAR_NODES");
}

// ==========================================
// 10. LOOP
// ==========================================
void loop() {
  // 1. MODBUS: atender al Agrónic + procesar respuestas del variador
  mb.task();
  vfd.task();

  // 1b. POLLING VFD: leer SW + FREQ + CURR cada 2 s (no bloqueante)
  if (!vfdPollPending && millis() - lastVfdPoll > 2000) {
    lastVfdPoll    = millis();
    vfdPollPending = true;
    vfd.readHregs(VFD_SLAVE_ID, VFD_REG_SW, 3, vfdRawData, vfdReadCb);
  }

  // 2. RECEPCIÓN LORA: heartbeats, ACKs y solicitudes de registro
  if (gwReceivedFlag) {
    gwReceivedFlag = false;
    uint8_t buf[MAX_PKT_LEN] = {0};

    if (radio.readData(buf, sizeof(buf)) == RADIOLIB_ERR_NONE) {
      switch (buf[0]) {
        case PKT_STATUS:
          processNodeStatus((const LoRaStatus*)buf);
          break;
        case PKT_JOIN:
          processJoin((const LoRaJoin*)buf);
          break;
        default:
          break;
      }
    }
    radio.startReceive();
  }

  // 3. CAMBIOS DE COILS MODBUS → COMANDOS LORA
  // Topología actual: ambas válvulas en el nodo físico 1
  // Para nodos separados: sendLoRaCommand(2, 1, cmd) para la segunda válvula
  bool mb_v1 = mb.Coil(COIL_VALVE_1);
  if (mb_v1 != valve1_state) {
    valve1_state = mb_v1;
    sendLoRaCommand(1, 1, valve1_state ? 1 : 2);
    radio.startReceive();
  }

  bool mb_v2 = mb.Coil(COIL_VALVE_2);
  if (mb_v2 != valve2_state) {
    valve2_state = mb_v2;
    sendLoRaCommand(1, 2, valve2_state ? 1 : 2);
    radio.startReceive();
  }

  // 4. CONTROL POR TERMINAL SERIE
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "A1") {
      valve1_state = true;  mb.Coil(COIL_VALVE_1, true);
      sendLoRaCommand(1, 1, 1); radio.startReceive();
    } else if (input == "C1") {
      valve1_state = false; mb.Coil(COIL_VALVE_1, false);
      sendLoRaCommand(1, 1, 2); radio.startReceive();
    } else if (input == "A2") {
      valve2_state = true;  mb.Coil(COIL_VALVE_2, true);
      sendLoRaCommand(1, 2, 1); radio.startReceive();
    } else if (input == "C2") {
      valve2_state = false; mb.Coil(COIL_VALVE_2, false);
      sendLoRaCommand(1, 2, 2); radio.startReceive();
    } else if (input == "PING1") {
      sendLoRaCommand(1, 0, 3); radio.startReceive();
    } else if (input == "PING2") {
      sendLoRaCommand(2, 0, 3); radio.startReceive();
    } else if (input == "NODES") {
      Serial.printf("Nodos registrados: %d/%d\n", registeredCount, MAX_NODES);
      for (uint8_t n = 1; n <= registeredCount; n++) {
        Serial.printf("  Nodo %d | ChipId: 0x%08X | %s | %s\n",
          n, chipIdMap[n],
          nodes[n].alive ? "VIVO " : "CAIDO",
          nodes[n].fault ? "FALLO" : "OK");
      }
    } else if (input == "CLEAR_NODES") {
      Serial.println("[CLEAR] Borrando tabla de registro...");
      prefs.putUChar("regCount", 0);
      for (uint8_t n = 1; n <= registeredCount; n++) {
        char key[8]; snprintf(key, sizeof(key), "chip%d", n);
        prefs.remove(key);
      }
      registeredCount = 0;
      memset(chipIdMap, 0, sizeof(chipIdMap));
      memset(nodes,     0, sizeof(nodes));
      updateModbusHealth();
      Serial.println("[CLEAR] Registro borrado. Los nodos deberan re-registrarse.");
      updateDisplay("REGISTRO BORRADO", "CLEAR_NODES OK");
    } else {
      Serial.println("Comandos: A1 C1 A2 C2 PING1 PING2 NODES CLEAR_NODES");
    }
  }

  // 5. BOTÓN FÍSICO (alterna Válvula 1)
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);
    if (digitalRead(BUTTON_PIN) == LOW) {
      Serial.println("BOTON: Alternando V1");
      valve1_state = !valve1_state;
      mb.Coil(COIL_VALVE_1, valve1_state);
      sendLoRaCommand(1, 1, valve1_state ? 1 : 2);
      radio.startReceive();
      while (digitalRead(BUTTON_PIN) == LOW) { delay(10); }
    }
  }

  // 6. COMPROBACIÓN DE TIMEOUTS DE NODOS (cada 30 segundos)
  if (millis() - lastTimeoutCheck > 30000) {
    lastTimeoutCheck = millis();
    checkNodeTimeouts();
  }

  // 7. REFRESCAR PANTALLA (cada 2 segundos)
  if (millis() - lastDisplayUpdate > 2000) {
    lastDisplayUpdate = millis();
    updateDisplay("ESCUCHANDO", "ID:" + String(msgCounter));
  }
}
