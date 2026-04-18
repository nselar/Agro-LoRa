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
// DRV8833 – STBY (controla ambos canales simultáneamente)
#define PIN_STBY      7

// ==========================================
// 2. CONFIGURACIÓN DEL NODO
// ==========================================
// El ID de nodo se asigna dinámicamente mediante el protocolo JOIN.
// No es necesario modificar este archivo para desplegar múltiples nodos;
// cada ESP32 obtiene su ID del gateway en el primer arranque.

#define MAX_NODES        8      // Límite de nodos por red (debe coincidir con gateway)
#define SLEEP_INTERVAL_S 30     // Segundos en deep sleep entre ciclos
#define LISTEN_WINDOW_MS 3000   // Ventana de escucha LoRa por ciclo (ms)
#define HEARTBEAT_EVERY  10     // Ciclos entre heartbeats (10 × 30s = 5 min)
#define WDT_TIMEOUT_S    10     // Watchdog: reset si activo más de 10s sin reset
#define JOIN_TIMEOUT_MS  8000   // Espera máxima de respuesta REGISTER tras enviar JOIN

// ==========================================
// 3. SEGURIDAD – PROTOCOLO Y CLAVES
// ==========================================
#define PKT_COMMAND  0xA1   // Gateway → Nodo: comando de válvula
#define PKT_STATUS   0xB2   // Nodo → Gateway: ACK / heartbeat / reset
#define PKT_JOIN     0xC3   // Nodo → Gateway: solicitud de registro
#define PKT_REGISTER 0xD4   // Gateway → Nodo: ID asignado

#define HMAC_KEY_LEN 16
#define MAC_LEN      4      // HMAC-SHA256 truncado a 4 bytes
#define MAX_PKT_LEN  12

#include "secrets.h"  // HMAC_KEY — ignorado por git, ver secrets.h.example

// ==========================================
// 4. ESTRUCTURAS DE PAQUETES
// ==========================================

// Gateway → Nodo (12 bytes)
struct __attribute__((packed)) LoRaPacket {
  uint8_t  pktType;
  uint8_t  targetNode;
  uint8_t  valve;         // 1 o 2 (canal del DRV8833)
  uint8_t  command;       // 1=ABRIR, 2=CERRAR, 3=PING
  uint32_t messageId;
  uint8_t  mac[MAC_LEN];
};

// Nodo → Gateway (8 bytes)
struct __attribute__((packed)) LoRaStatus {
  uint8_t  pktType;
  uint8_t  fromNode;
  uint8_t  type;          // 0x00=ACK, 0x01=HEARTBEAT, 0x02=BOOT_REASON
  uint8_t  detail;
  uint32_t messageId;
};

// Nodo → Gateway: solicitud de registro (9 bytes)
struct __attribute__((packed)) LoRaJoin {
  uint8_t  pktType;       // PKT_JOIN (0xC3)
  uint32_t chipId;        // ID único del chip (ESP.getEfuseMac() & 0xFFFFFFFF)
  uint8_t  mac[MAC_LEN];
};

// Gateway → Nodo: ID asignado (10 bytes)
struct __attribute__((packed)) LoRaRegister {
  uint8_t  pktType;       // PKT_REGISTER (0xD4)
  uint32_t chipId;        // Echo del chipId solicitante
  uint8_t  assignedId;    // ID asignado: 1..MAX_NODES
  uint8_t  mac[MAC_LEN];
};

// ==========================================
// 5. VARIABLES RTC (sobreviven al deep sleep)
// ==========================================
RTC_DATA_ATTR uint32_t lastMessageId = 0;
RTC_DATA_ATTR uint32_t wakeCount     = 0;

// ==========================================
// 6. OBJETOS GLOBALES
// ==========================================
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
Preferences prefs;

uint8_t  nodeId = 0;   // ID asignado dinámicamente; 0 = sin registrar
volatile bool receivedFlag = false;

// ==========================================
// 7. FUNCIONES AUXILIARES
// ==========================================

// ISR: el SX1262 avisa que recibió un paquete
#if defined(ESP8266) || defined(ESP32)
  IRAM_ATTR
#endif
void setFlag(void) {
  receivedFlag = true;
}

// --- HMAC helpers ---

// HMAC para verificar paquetes de comando entrantes (Gateway → Nodo)
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

bool verifyHMAC(const LoRaPacket* p) {
  uint8_t expected[MAC_LEN];
  computeHMAC(p, expected);
  return (memcmp(expected, p->mac, MAC_LEN) == 0);
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

// HMAC para verificar respuesta REGISTER del gateway: cubre pktType + chipId + assignedId
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

bool verifyHMACRegister(const LoRaRegister* r) {
  uint8_t expected[MAC_LEN];
  computeHMACRegister(r, expected);
  return (memcmp(expected, r->mac, MAC_LEN) == 0);
}

// --- Display ---

void updateDisplay(const String& status, const String& info) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  if (nodeId > 0) {
    display.printf("NODO #%d - 868MHz", nodeId);
  } else {
    display.print("NODO SIN ID - 868MHz");
  }

  display.setCursor(0, 18);
  display.print("Estado: ");
  display.print(status);

  display.setCursor(0, 36);
  display.print(info);

  display.setCursor(0, 52);
  display.printf("Ciclo: %d", wakeCount);

  display.display();
}

// Activa la válvula con el canal correcto del DRV8833
// valve=1 → Canal A (IN1/IN2), valve=2 → Canal B (IN3/IN4)
void accionarValvula(uint8_t valve, bool abrir) {
  uint8_t pinA = (valve == 1) ? PIN_IN1 : PIN_IN3;
  uint8_t pinB = (valve == 1) ? PIN_IN2 : PIN_IN4;

  Serial.printf(">>> V%d %s <<<\n", valve, abrir ? "APERTURA" : "CIERRE");

  digitalWrite(PIN_STBY, HIGH);  // Despertar driver DRV8833
  delay(1);                       // Wake-up time (~1ms, ver datasheet DRV8833)

  if (abrir) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  }

  delay(100);  // Pulso latch 100ms (estándar para solenoide 9V Baccara)

  digitalWrite(pinA, LOW);
  digitalWrite(pinB, LOW);
  delay(1);
  digitalWrite(PIN_STBY, LOW);   // Dormir driver tras completar el pulso

  Serial.println("--- Valvula en reposo (alta impedancia) ---");
}

// Envía un paquete de estado al gateway (ACK, heartbeat o boot reason)
void sendStatus(uint8_t type, uint8_t detail, uint32_t msgId) {
  LoRaStatus s;
  s.pktType   = PKT_STATUS;
  s.fromNode  = nodeId;
  s.type      = type;
  s.detail    = detail;
  s.messageId = msgId;
  radio.transmit((uint8_t*)&s, sizeof(LoRaStatus));
  // El radio queda en standby tras TX; el caller debe llamar startReceive() si procede
}

// Apaga periféricos y entra en deep sleep hasta el próximo ciclo
void goToSleep() {
  Serial.printf("[SLEEP] Durmiendo %ds. Proximo ciclo: %d\n",
                SLEEP_INTERVAL_S, wakeCount + 1);
  Serial.flush();

  radio.sleep();  // SX1262 a <1µA

  // Estado seguro de las válvulas antes de dormir
  digitalWrite(PIN_IN1,  LOW);
  digitalWrite(PIN_IN2,  LOW);
  digitalWrite(PIN_IN3,  LOW);
  digitalWrite(PIN_IN4,  LOW);
  digitalWrite(PIN_STBY, LOW);

  // Cortar VEXT: apaga OLED y TCXO del SX1262 (~1mA ahorrado durante el sleep)
  digitalWrite(VEXT, HIGH);

  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_INTERVAL_S * 1000000ULL);
  esp_deep_sleep_start();
  // El ESP32 se reinicia aquí; setup() se ejecuta en el próximo ciclo
}

// Solicita registro al gateway mediante el protocolo JOIN.
// Envía el chipId único del ESP32 y espera una respuesta REGISTER con el ID asignado.
// Retorna true si el registro fue exitoso, false si hubo timeout o error HMAC.
bool performJoin() {
  uint32_t myChipId = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFF);
  Serial.printf("[JOIN] Sin ID registrado. Solicitando registro (ChipId: 0x%08X)...\n", myChipId);
  updateDisplay("SIN REGISTRO", "Enviando JOIN...");

  // Construir y enviar paquete JOIN
  LoRaJoin joinPkt;
  joinPkt.pktType = PKT_JOIN;
  joinPkt.chipId  = myChipId;
  computeHMACJoin(&joinPkt, joinPkt.mac);

  int txState = radio.transmit((uint8_t*)&joinPkt, sizeof(LoRaJoin));
  if (txState != RADIOLIB_ERR_NONE) {
    Serial.printf("[JOIN] Error TX: %d\n", txState);
    return false;
  }

  // Escuchar la respuesta REGISTER del gateway
  receivedFlag = false;
  radio.setDio1Action(setFlag);
  radio.startReceive();

  unsigned long t = millis();
  while (millis() - t < JOIN_TIMEOUT_MS) {
    esp_task_wdt_reset();

    if (receivedFlag) {
      receivedFlag = false;
      uint8_t buf[MAX_PKT_LEN] = {0};
      int rxState = radio.readData(buf, sizeof(buf));

      if (rxState == RADIOLIB_ERR_NONE && buf[0] == PKT_REGISTER) {
        const LoRaRegister* reg = (const LoRaRegister*)buf;

        // Verificar que la respuesta es para este nodo
        if (reg->chipId != myChipId) {
          Serial.println("[JOIN] REGISTER ignorado: chipId no coincide.");
          radio.startReceive();
          continue;
        }
        if (!verifyHMACRegister(reg)) {
          Serial.println("[JOIN] REGISTER rechazado: HMAC invalido.");
          radio.startReceive();
          continue;
        }
        if (reg->assignedId < 1 || reg->assignedId > MAX_NODES) {
          Serial.printf("[JOIN] REGISTER rechazado: ID fuera de rango (%d).\n", reg->assignedId);
          radio.startReceive();
          continue;
        }

        // Registro exitoso: guardar ID en NVS
        nodeId = reg->assignedId;
        prefs.begin("node", false);
        prefs.putUChar("id", nodeId);
        prefs.end();

        Serial.printf("[JOIN] Registro exitoso! ID asignado: %d\n", nodeId);
        updateDisplay("REGISTRADO", "ID:" + String(nodeId));
        delay(1000);
        return true;
      }

      radio.startReceive();
    }
    delay(10);
  }

  Serial.println("[JOIN] Timeout. Reintentando en el proximo ciclo.");
  updateDisplay("JOIN TIMEOUT", "Reint. en " + String(SLEEP_INTERVAL_S) + "s");
  return false;
}

// ==========================================
// 8. SETUP (se ejecuta en cada ciclo de deep sleep)
// ==========================================
void setup() {
  Serial.begin(115200);

  esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
  bool isTimerWakeup = (wakeup_cause == ESP_SLEEP_WAKEUP_TIMER);

  if (!isTimerWakeup) {
    delay(2000);
    Serial.println("\n--- NODO CAMPO INICIANDO (Primer arranque) ---");
  }

  // Activar VEXT: alimenta OLED y TCXO del SX1262
  pinMode(VEXT, OUTPUT);
  digitalWrite(VEXT, LOW);
  delay(50);

  // DRV8833
  pinMode(PIN_IN1,  OUTPUT); digitalWrite(PIN_IN1,  LOW);
  pinMode(PIN_IN2,  OUTPUT); digitalWrite(PIN_IN2,  LOW);
  pinMode(PIN_IN3,  OUTPUT); digitalWrite(PIN_IN3,  LOW);
  pinMode(PIN_IN4,  OUTPUT); digitalWrite(PIN_IN4,  LOW);
  pinMode(PIN_STBY, OUTPUT); digitalWrite(PIN_STBY, LOW);

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[WARN] Fallo OLED");
  }
  updateDisplay("Iniciando...", "Configurando radio");

  // SX1262 LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int state = radio.begin(868.0, 125.0, 9, 7, 18, 22, 8, 1.8, false);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ERROR] Fallo LoRa, codigo: %d\n", state);
    updateDisplay("ERROR LORA", "Cod:" + String(state));
    delay(1000);
    goToSleep();
  }
  radio.setDio2AsRfSwitch(true);

  // Watchdog: reset si la tarea se cuelga más de WDT_TIMEOUT_S segundos
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  // Leer ID asignado desde NVS (persiste entre reinicios y deep sleeps)
  prefs.begin("node", true);   // true = solo lectura
  nodeId = prefs.getUChar("id", 0);
  prefs.end();

  if (nodeId == 0) {
    // Primer arranque sin ID: solicitar registro al gateway
    // El protocolo JOIN usa el chipId único del ESP32 como identificador
    if (!performJoin()) {
      // Si el JOIN falla (gateway apagado, timeout), dormir y reintentar
      goToSleep();
    }
  } else {
    if (!isTimerWakeup) {
      Serial.printf("✓ ID recuperado desde NVS: %d\n", nodeId);
    }
  }

  // Reportar reset anormal al gateway (pánico, watchdog, brownout)
  esp_reset_reason_t reset_cause = esp_reset_reason();
  if (reset_cause == ESP_RST_PANIC    ||
      reset_cause == ESP_RST_INT_WDT  ||
      reset_cause == ESP_RST_TASK_WDT ||
      reset_cause == ESP_RST_BROWNOUT) {
    Serial.printf("[ALERTA] Reset anormal. Causa: %d\n", (int)reset_cause);
    sendStatus(0x02, (uint8_t)reset_cause, 0);
    updateDisplay("RESET ANORMAL", "Causa:" + String((int)reset_cause));
    delay(500);
  }

  updateDisplay("ESCUCHANDO", "ID:" + String(lastMessageId));
  radio.setDio1Action(setFlag);
}

// ==========================================
// 9. LOOP (ejecutado UNA sola vez por ciclo; finaliza en goToSleep())
// ==========================================
void loop() {
  wakeCount++;
  esp_task_wdt_reset();

  // --- Heartbeat periódico: prueba de vida proactiva al gateway ---
  if (wakeCount % HEARTBEAT_EVERY == 0) {
    Serial.printf("[HB] Enviando heartbeat (ciclo %d)\n", wakeCount);
    sendStatus(0x01, 0, wakeCount);
  }

  // --- Ventana de escucha LoRa ---
  receivedFlag = false;
  radio.startReceive();
  unsigned long listenStart = millis();

  while (millis() - listenStart < LISTEN_WINDOW_MS) {
    esp_task_wdt_reset();

    if (receivedFlag) {
      receivedFlag = false;

      uint8_t buf[MAX_PKT_LEN] = {0};
      int rxState = radio.readData(buf, sizeof(buf));

      if (rxState == RADIOLIB_ERR_NONE && buf[0] == PKT_COMMAND) {
        const LoRaPacket* packet = (const LoRaPacket*)buf;

        Serial.printf("[LoRa] Pkt → Nodo:%d V:%d Cmd:%d ID:%d\n",
                      packet->targetNode, packet->valve,
                      packet->command, packet->messageId);

        if (packet->targetNode != nodeId) {
          Serial.println("[LoRa] Ignorado: no es para este nodo.");
        }
        else if (!verifyHMAC(packet)) {
          Serial.println("[SEGURIDAD] HMAC invalido. Paquete descartado.");
        }
        else if (packet->messageId <= lastMessageId) {
          Serial.printf("[SEGURIDAD] ID %d repetido (ultimo: %d). Descartado.\n",
                        packet->messageId, lastMessageId);
        }
        else {
          lastMessageId = packet->messageId;

          if (packet->command == 1 || packet->command == 2) {
            bool abrir = (packet->command == 1);
            String info = "V" + String(packet->valve) +
                          (abrir ? " ABRIENDO" : " CERRANDO");
            updateDisplay("ACTIVA", info);
            accionarValvula(packet->valve, abrir);
            sendStatus(0x00, packet->valve, packet->messageId);  // ACK
            delay(200);
            updateDisplay("ESCUCHANDO", "ID:" + String(lastMessageId));
          }
          else if (packet->command == 3) {
            Serial.println("[PING] Respondiendo al gateway.");
            sendStatus(0x00, 0, packet->messageId);
          }
        }
      }
      else if (rxState != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] Error de recepcion: %d\n", rxState);
      }

      radio.startReceive();  // Reanudar escucha dentro de la ventana
    }

    delay(10);
  }

  // Fin de ventana de escucha: dormir hasta el próximo ciclo
  goToSleep();
}
