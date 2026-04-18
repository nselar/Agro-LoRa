# Agro-LoRa: Sistema de Riego Automatizado por Radio

> Sistema open source de riego agrícola de largo alcance usando LoRa 868 MHz, ESP32 y Modbus RTU. Diseñado para integrarse con programadores comerciales (Agrónic 2500) y controlar válvulas solenoide tipo latch en campo abierto.

[![Platform](https://img.shields.io/badge/platform-ESP32--S3-blue)](https://www.espressif.com/en/products/socs/esp32-s3)
[![Framework](https://img.shields.io/badge/framework-PlatformIO%20%2F%20Arduino-orange)](https://platformio.org/)
[![Radio](https://img.shields.io/badge/radio-LoRa%20868%20MHz-green)](https://www.semtech.com/products/wireless-rf/lora-connect/sx1262)
[![License](https://img.shields.io/badge/license-MIT-lightgrey)](LICENSE)
[![Build](https://github.com/nselar/Agro-LoRa/actions/workflows/build.yml/badge.svg)](https://github.com/nselar/Agro-LoRa/actions/workflows/build.yml)

---

## Descripción general

Agro-LoRa resuelve el problema del cableado en sistemas de riego distribuidos. En lugar de tender cables desde el cuadro de control hasta cada electroválvula (costoso, frágil y difícil de mantener), este sistema transmite las órdenes de apertura y cierre por radio LoRa.

El proyecto soporta **dos cuadros de control independientes**, cada uno con su propio gateway ESP32, programador Agrónic y variador de frecuencia.

```
 ══════════════════════════════════════════════════════════════
  CUADRO 1 — Variador ABB ACQ80-04      (Nodo_Caseta)
 ══════════════════════════════════════════════════════════════

 ┌─────────────────────────────────┐
 │  Programador Agrónic 2500       │
 │  (Master Modbus RS485)          │
 └──────────────┬──────────────────┘
                │ Modbus RTU 9600 baud (Serial1)
                ▼
 ┌──────────────────────────────────┐
 │  NODO CASETA — Gateway ABB       │  )))  ───── LoRa SF9 ───── (((
 │  Heltec WiFi LoRa 32 V3          │
 │  • Modbus Slave → Agrónic        │         ▼              ▼
 │  • Modbus Master → VFD (Serial2) │  ┌──────────────┐  ┌──────────────┐
 └──────────────┬───────────────────┘  │ NODO SECTOR  │  │ NODO SECTOR  │
                │ Modbus RTU (RS485)    │    #1        │  │    #2 …8     │
                ▼                      └──────────────┘  └──────────────┘
 ┌──────────────────────────────────┐
 │  VFD ABB ACQ80-04                │
 │  Estado leído cada 2 s           │
 └──────────────────────────────────┘

 ══════════════════════════════════════════════════════════════
  CUADRO 2 — Variador Vacon 100X     (Nodo_Caseta_Vacon)
 ══════════════════════════════════════════════════════════════

 ┌─────────────────────────────────┐
 │  Programador Agrónic 2500       │
 │  (Master Modbus RS485)          │
 └──────────────┬──────────────────┘
                │ Modbus RTU 9600 baud (Serial1)
                ▼
 ┌──────────────────────────────────┐
 │  NODO CASETA — Gateway Vacon     │  )))  ───── LoRa SF9 ───── (((
 │  Heltec WiFi LoRa 32 V3          │
 │  • Modbus Slave → Agrónic        │         ▼              ▼
 │  • Modbus Master → VFD (Serial2) │  ┌──────────────┐  ┌──────────────┐
 └──────────────┬───────────────────┘  │ NODO SECTOR  │  │ NODO SECTOR  │
                │ Modbus RTU (RS485)    │    #1        │  │    #2 …8     │
                ▼                      └──────────────┘  └──────────────┘
 ┌──────────────────────────────────┐
 │  VFD Vacon 100X                  │
 │  Estado leído cada 2 s           │
 └──────────────────────────────────┘
```

---

## Características

- **Auto-registro de nodos**: Cada ESP32 de campo obtiene su ID automáticamente del gateway en el primer arranque, sin necesidad de recompilar el firmware.
- **Seguridad**: Autenticación HMAC-SHA256 truncada a 4 bytes en todos los paquetes + contador anti-replay persistente en NVS flash.
- **Bajo consumo**: Los nodos de campo operan en ciclos de 30 s con deep sleep; el SX1262 baja a <1 µA en sleep, ahorrando ~1 mA adicional al cortar VEXT (OLED + TCXO).
- **Fiabilidad**: Reintentos automáticos (hasta 3) + confirmación ACK por paquete + heartbeat cada 5 minutos para detectar nodos caídos.
- **Integración Modbus**: El gateway actúa como esclavo RTU que el Agrónic puede leer/escribir. Los estados de salud de los nodos se exponen como Discrete Inputs.
- **Hasta 8 nodos de campo** por gateway, expandible.
- **OLED en ambos nodos** para monitoreo local sin necesidad de ordenador.
- **Watchdog** en nodos de campo: reset automático si el firmware se cuelga más de 10 s.
- **Detección de resets anormales**: el nodo reporta al gateway cualquier pánico, watchdog o brownout.
- **Monitorización de variador de frecuencia**: el gateway actúa también como Modbus master sobre un segundo bus RS485 y lee estado (RUN/STOP/FALLO), frecuencia y corriente del variador cada 2 s, exponiéndolo al Agrónic como Discrete Inputs e Input Registers. Soporta **ABB ACQ80-04** (`Nodo_Caseta`) y **Vacon 100X** (`Nodo_Caseta_Vacon`) como firmwares independientes.
- **Clave HMAC fuera del repositorio**: `secrets.h` está en `.gitignore`; cada instalación usa su propia clave sin riesgo de publicarla accidentalmente.
- **CI con GitHub Actions**: cada push/PR compila los tres proyectos automáticamente.

---

## Hardware

### Nodo Caseta — Gateway ABB (`Nodo_Caseta`) — Cuadro 1

| Componente | Modelo | Función |
|---|---|---|
| Microcontrolador | Heltec WiFi LoRa 32 V3 | ESP32-S3 + SX1262 integrados |
| Transceptor LoRa | SX1262 (integrado) | 868 MHz, hasta 22 dBm |
| Interfaz RS485 #1 | MAX3485 o similar | Modbus RTU esclavo → Agrónic 2500 (Serial1, pines 47/48/45) |
| Interfaz RS485 #2 | MAX3485 o similar | Modbus RTU master → ABB ACQ80-04 (Serial2, pines 19/20/15) |
| Display OLED | SSD1306 128×64 (integrado) | Estado del sistema |
| Alimentación | 5V DC | Vía USB-C o regulador externo |

### Nodo Caseta — Gateway Vacon (`Nodo_Caseta_Vacon`) — Cuadro 2

Hardware idéntico al Gateway ABB. Solo difiere el firmware.

| Componente | Modelo | Función |
|---|---|---|
| Microcontrolador | Heltec WiFi LoRa 32 V3 | ESP32-S3 + SX1262 integrados |
| Transceptor LoRa | SX1262 (integrado) | 868 MHz, hasta 22 dBm |
| Interfaz RS485 #1 | MAX3485 o similar | Modbus RTU esclavo → Agrónic 2500 (Serial1, pines 47/48/45) |
| Interfaz RS485 #2 | MAX3485 o similar | Modbus RTU master → Vacon 100X (Serial2, pines 19/20/15) |
| Display OLED | SSD1306 128×64 (integrado) | Estado del sistema |
| Alimentación | 5V DC | Vía USB-C o regulador externo |

### Nodo Sector (Campo) — 1 unidad por arqueta

| Componente | Modelo | Función |
|---|---|---|
| Microcontrolador | Heltec WiFi LoRa 32 V3 | ESP32-S3 + SX1262 integrados |
| Transceptor LoRa | SX1262 (integrado) | 868 MHz |
| Driver de motor | DRV8833 (Texas Instruments) | Control de solenoides H-bridge dual |
| Elevador de tensión | Step-up DC-DC 9V | Alimentar pulso de activación |
| Válvulas | Baccara tipo latch | 2 válvulas por nodo (9V solenoide) |
| Display OLED | SSD1306 128×64 (integrado) | Estado del nodo |
| Batería / Fuente | LiPo 3.7V o 5V DC | Deep sleep <2 mA promedio |

> **Nota sobre las válvulas Baccara latch**: Son válvulas de solenoide de pulso (latching). Requieren un pulso breve (~100 ms) para abrirse y otro para cerrarse. No consumen corriente en reposo, lo que las hace ideales para sistemas con batería.

---

## Software

### Requisitos

- [PlatformIO](https://platformio.org/) (extensión VS Code recomendada)
- Python 3.x (incluido con PlatformIO)

### Integración continua (GitHub Actions)

En cada `push` o `pull_request`, el workflow `.github/workflows/build.yml` compila los tres proyectos automáticamente. El badge de estado aparece en la cabecera de este README.

Si el build falla, el PR no debe fusionarse hasta corregirlo.

### Librerías (se instalan automáticamente con PlatformIO)

| Librería | Versión | Uso |
|---|---|---|
| [RadioLib](https://github.com/jgromes/RadioLib) | ^6.3.0 | Control del SX1262 |
| [modbus-esp8266](https://github.com/emelianov/modbus-esp8266) | ^4.1.0 | Modbus RTU (solo Nodo Caseta) |
| [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306) | ^2.5.9 | Driver OLED |
| [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library) | ^1.11.9 | Gráficos OLED |
| mbedTLS | (ESP-IDF built-in) | HMAC-SHA256 |
| Preferences | (Arduino ESP32 built-in) | NVS flash persistente |
| ESP-IDF (esp_sleep, esp_task_wdt) | (built-in) | Deep sleep, watchdog |

---

## Inicio rápido

### 1. Clonar el repositorio

```bash
git clone https://github.com/nselar/Agro-LoRa.git
cd Agro-LoRa
```

### 2. Crear el archivo de clave HMAC (obligatorio antes del despliegue)

La clave **no está en el repositorio** — cada instalador genera la suya propia.

```bash
# Genera 16 bytes aleatorios
openssl rand -hex 16
# Ejemplo de salida: a3f82c1d9e4b7056f1c23a8d0e5b9472
```

Crea `secrets.h` a partir del ejemplo en **cada** proyecto:

```bash
cp Nodo_Caseta/src/secrets.h.example       Nodo_Caseta/src/secrets.h
cp Nodo_Caseta_Vacon/src/secrets.h.example Nodo_Caseta_Vacon/src/secrets.h
cp Nodo_Sector/src/secrets.h.example       Nodo_Sector/src/secrets.h
```

Edita cada `secrets.h` con los mismos bytes generados:

```c
static const uint8_t HMAC_KEY[16] = {
  0xA3, 0xF8, 0x2C, 0x1D, 0x9E, 0x4B, 0x70, 0x56,
  0xF1, 0xC2, 0x3A, 0x8D, 0x0E, 0x5B, 0x94, 0x72
};
```

> **Advertencia de seguridad**: `secrets.h` está en `.gitignore` y **nunca** se sube al repositorio. La clave debe ser idéntica en los tres firmwares (Nodo_Caseta, Nodo_Caseta_Vacon, Nodo_Sector) del mismo sistema. Un nodo con clave incorrecta no podrá comunicarse.

### 3. Flashear el Gateway

**Cuadro 1 — ABB ACQ80-04:**
```bash
cd Nodo_Caseta
pio run --target upload
pio device monitor   # Ver logs por puerto serie
```

**Cuadro 2 — Vacon 100X:**
```bash
cd Nodo_Caseta_Vacon
pio run --target upload
pio device monitor
```

### 4. Flashear los Nodos de Campo

Puedes usar **el mismo binario** para todos los nodos. Cada uno obtendrá su ID automáticamente:

```bash
cd Nodo_Sector
pio run --target upload
```

Repite este paso para cada Heltec de campo. No es necesario cambiar nada en el código entre nodo y nodo.

### 5. Primer arranque de un nodo de campo

Al encender por primera vez:

1. El nodo detecta que no tiene ID almacenado en flash.
2. Transmite un paquete `JOIN` con su ID de chip único (`ESP.getEfuseMac()`).
3. El gateway lo recibe, le asigna el siguiente ID libre (1, 2, 3…) y responde con `REGISTER`.
4. El nodo guarda su ID en NVS y entra en funcionamiento normal.
5. El OLED muestra `REGISTRADO | ID:1`.

Desde la consola del gateway puedes ver el proceso en tiempo real:
```
[JOIN] Nodo registrado: ChipId 0xA1B2C3D4 → ID 1 (Total: 1)
```

---

## Configuración

### Parámetros del Nodo Caseta — Gateway ABB (`Nodo_Caseta`)

| Parámetro | Valor por defecto | Descripción |
|---|---|---|
| `SLAVE_ID` | `1` | ID Modbus del gateway (debe coincidir con Agrónic) |
| `MAX_RETRIES` | `3` | Reintentos de comando LoRa |
| `ACK_TIMEOUT_MS` | `4000` | Timeout de espera de ACK (ms) |
| `HEARTBEAT_TIMEOUT_MS` | `720000` | Tiempo sin heartbeat para considerar nodo caído (12 min) |
| `MAX_NODES` | `8` | Máximo de nodos de campo |
| `VFD_SLAVE_ID` | `1` | ID Modbus del ABB ACQ80 (parámetro 58.03 en el drive) |
| `VFD_BAUD` | `9600` | Baud rate RS485 hacia el ABB (parámetro 58.01 en el drive) |

### Parámetros del Nodo Caseta — Gateway Vacon (`Nodo_Caseta_Vacon`)

Mismos parámetros de red/LoRa que el gateway ABB. Solo difieren los del variador:

| Parámetro | Valor por defecto | Descripción |
|---|---|---|
| `VFD_SLAVE_ID` | `1` | ID Modbus del Vacon 100X (parámetro P3.1 en el drive) |
| `VFD_BAUD` | `9600` | Baud rate RS485 hacia el Vacon (parámetro P3.2 en el drive) |

### Parámetros del Nodo Sector

| Parámetro | Archivo | Valor por defecto | Descripción |
|---|---|---|---|
| `SLEEP_INTERVAL_S` | `Nodo_Sector/src/main.cpp` | `30` | Duración del deep sleep entre ciclos (s) |
| `LISTEN_WINDOW_MS` | `Nodo_Sector/src/main.cpp` | `3000` | Duración de la ventana de escucha LoRa por ciclo (ms) |
| `HEARTBEAT_EVERY` | `Nodo_Sector/src/main.cpp` | `10` | Ciclos entre heartbeats (10 × 30s = 5 min) |
| `JOIN_TIMEOUT_MS` | `Nodo_Sector/src/main.cpp` | `8000` | Timeout esperando respuesta de registro del gateway (ms) |
| `WDT_TIMEOUT_S` | `Nodo_Sector/src/main.cpp` | `10` | Timeout del watchdog (s) |

### Ajustes Modbus para el Agrónic 2500

Verifica en la documentación de tu Agrónic los parámetros de comunicación:

| Parámetro | Valor por defecto en el proyecto | Cómo cambiar |
|---|---|---|
| Baud rate | 9600 | `Serial1.begin(9600, ...)` en `Nodo_Caseta/src/main.cpp` |
| Paridad | Ninguna (8N1) | Cambiar `SERIAL_8N1` a `SERIAL_8E1` si usa paridad par |
| Slave ID | 1 | `#define SLAVE_ID 1` |

---

## Nodos

### Nodo Caseta (Gateway) — ambas variantes

Actúa de puente entre el programador Agrónic y los nodos de campo.

**Funciones comunes:**
- Esclavo Modbus RTU que el Agrónic puede leer y escribir.
- Recibe órdenes de válvula vía Coils Modbus y las transmite por LoRa.
- Mantiene una tabla de registro de nodos (chipId → ID) persistida en NVS.
- Monitorea la salud de cada nodo vía heartbeats y expone el estado como Discrete Inputs.
- Master Modbus RTU hacia el variador: lee estado, frecuencia y corriente cada 2 s y los publica al Agrónic vía FC02 (Discrete Inputs) y FC04 (Input Registers).
- Botón físico (PRG, pin 0) para alternar manualmente la válvula 1.

**Diferencias entre variantes:**

| | `Nodo_Caseta` | `Nodo_Caseta_Vacon` |
|---|---|---|
| Variador | ABB ACQ80-04 | Vacon 100X |
| SW bit RUN | bit 2 | bit 1 |
| SW bit FAULT | bit 3 | bit 2 |
| Config drive | Param 58.01–58.04 | Param P3.1–P3.4 |
| Display OLED | `VFD:RUN/STOP FLT` | `VCN:RUN/STOP FLT` |

**Comandos por terminal serie (115200 baud):**

| Comando | Acción |
|---|---|
| `A1` | Abrir válvula 1 |
| `C1` | Cerrar válvula 1 |
| `A2` | Abrir válvula 2 |
| `C2` | Cerrar válvula 2 |
| `PING1` | Ping al nodo 1 (prueba de enlace) |
| `PING2` | Ping al nodo 2 |
| `NODES` | Listar nodos registrados con chipId y estado |
| `CLEAR_NODES` | Borrar tabla de registro (los nodos deberán re-registrarse) |

### Nodo Sector (Campo)

Opera en ciclos de bajo consumo.

**Ciclo de operación:**
```
┌─ Despertar (cada 30 s) ─────────────────────────────────────────┐
│                                                                   │
│  1. setup():                                                      │
│     • Encender VEXT (OLED + TCXO)                                │
│     • Inicializar SX1262, DRV8833                                │
│     • Leer ID desde NVS                                           │
│       → Si ID=0: enviar JOIN, esperar REGISTER (8 s)             │
│       → Si timeout: ir a sleep y reintentar                       │
│     • Si hubo reset anormal: reportar al gateway                  │
│                                                                   │
│  2. loop():                                                       │
│     • Cada 10 ciclos (5 min): enviar HEARTBEAT al gateway         │
│     • Abrir ventana de escucha LoRa (3 s):                       │
│       → Si llega PKT_COMMAND válido: actuar válvula + enviar ACK │
│     • Apagar periféricos + entrar en deep sleep (30 s)            │
└───────────────────────────────────────────────────────────────────┘
```

---

## Protocolo LoRa

Todos los paquetes usan 868 MHz, SF9, BW 125 kHz, CR 4/7, preámbulo 8, syncword 0x12.

### Tipos de paquete

| Tipo | Hex | Dirección | Tamaño |
|---|---|---|---|
| `PKT_COMMAND` | `0xA1` | Gateway → Nodo | 12 bytes |
| `PKT_STATUS` | `0xB2` | Nodo → Gateway | 8 bytes |
| `PKT_JOIN` | `0xC3` | Nodo → Gateway | 9 bytes |
| `PKT_REGISTER` | `0xD4` | Gateway → Nodo | 10 bytes |

### PKT_COMMAND (Gateway → Nodo) — 12 bytes

| Offset | Campo | Tipo | Descripción |
|---|---|---|---|
| 0 | `pktType` | `uint8` | `0xA1` |
| 1 | `targetNode` | `uint8` | ID del nodo destino |
| 2 | `valve` | `uint8` | Canal: 1 o 2 |
| 3 | `command` | `uint8` | 1=ABRIR, 2=CERRAR, 3=PING |
| 4–7 | `messageId` | `uint32` (LE) | Contador anti-replay |
| 8–11 | `mac` | `uint8[4]` | HMAC-SHA256 truncado |

### PKT_STATUS (Nodo → Gateway) — 8 bytes

| Offset | Campo | Tipo | Descripción |
|---|---|---|---|
| 0 | `pktType` | `uint8` | `0xB2` |
| 1 | `fromNode` | `uint8` | ID del nodo emisor |
| 2 | `type` | `uint8` | 0x00=ACK, 0x01=HB, 0x02=BOOT |
| 3 | `detail` | `uint8` | ACK: válvula; HB: 0; BOOT: causa reset |
| 4–7 | `messageId` | `uint32` (LE) | ACK: echo cmd ID; HB: wakeCount |

### PKT_JOIN (Nodo → Gateway) — 9 bytes

| Offset | Campo | Tipo | Descripción |
|---|---|---|---|
| 0 | `pktType` | `uint8` | `0xC3` |
| 1–4 | `chipId` | `uint32` (LE) | `ESP.getEfuseMac() & 0xFFFFFFFF` |
| 5–8 | `mac` | `uint8[4]` | HMAC-SHA256 truncado |

### PKT_REGISTER (Gateway → Nodo) — 10 bytes

| Offset | Campo | Tipo | Descripción |
|---|---|---|---|
| 0 | `pktType` | `uint8` | `0xD4` |
| 1–4 | `chipId` | `uint32` (LE) | Echo del chipId solicitante |
| 5 | `assignedId` | `uint8` | ID asignado (1–8) |
| 6–9 | `mac` | `uint8[4]` | HMAC-SHA256 truncado |

---

## Mapa de registros Modbus

El gateway actúa como esclavo Modbus RTU con ID=1.

### Coils (FC01 lectura / FC05 escritura)

| Dirección | Nombre | Descripción |
|---|---|---|
| `0x0000` | `COIL_VALVE_1` | 1=Abrir válvula 1, 0=Cerrar válvula 1 |
| `0x0001` | `COIL_VALVE_2` | 1=Abrir válvula 2, 0=Cerrar válvula 2 |

El Agrónic escribe el coil. El gateway detecta el cambio, envía el comando LoRa y espera el ACK. Si el nodo no responde tras 3 reintentos, el coil se revierte al estado anterior para señalizar el fallo.

### Discrete Inputs (FC02 lectura)

| Dirección | Nombre | Descripción |
|---|---|---|
| `0x0000` | `ISTS_NODE1_ALIVE` | 1=nodo 1 activo (heartbeat reciente), 0=sin respuesta |
| `0x0001` | `ISTS_NODE2_ALIVE` | 1=nodo 2 activo |
| `0x0002` | `ISTS_NODE1_FAULT` | 1=fallo activo en nodo 1 |
| `0x0003` | `ISTS_NODE2_FAULT` | 1=fallo activo en nodo 2 |
| `0x0004` | `ISTS_VFD_RUNNING` | 1=variador en marcha |
| `0x0005` | `ISTS_VFD_FAULT` | 1=fallo/trip activo en el variador |

El Agrónic puede leer estas entradas para disparar alarmas o detener el programa de riego.

### Input Registers (FC04 lectura)

| Dirección | Nombre | Escala | Descripción |
|---|---|---|---|
| `0x0000` | `IREG_VFD_FREQ` | ÷100 → Hz | Frecuencia de salida del variador (0.01 Hz/LSB) |
| `0x0001` | `IREG_VFD_CURR` | ÷10 → A | Corriente de salida del variador (0.1 A/LSB) |

Actualizado cada 2 s desde el variador vía Modbus master. Valor 0 si el variador no responde.

> **Nota**: El mapa de registros Modbus expuesto al Agrónic es **idéntico** en ambas variantes de firmware (ABB y Vacon). La diferencia está en cómo cada firmware interpreta internamente el Status Word del drive (ABB: bit2=RUN, bit3=FAULT; Vacon: bit1=RUN, bit2=FAULT).

---

## Pinout

### Nodo Caseta y Nodo Sector (Heltec WiFi LoRa 32 V3)

Los pines del SX1262 y el OLED son los mismos en ambas placas:

| Señal | Pin GPIO | Notas |
|---|---|---|
| LoRa NSS | 8 | SPI chip select |
| LoRa DIO1 | 14 | IRQ (interrupción de recepción) |
| LoRa NRST | 12 | Reset del SX1262 |
| LoRa BUSY | 13 | Señal de ocupado |
| SPI SCK | 9 | |
| SPI MISO | 11 | |
| SPI MOSI | 10 | |
| OLED SDA | 17 | I2C, dirección 0x3C |
| OLED SCL | 18 | |
| OLED RST | 21 | |

### Pines exclusivos del Nodo Caseta (ambas variantes — ABB y Vacon)

| Señal | Pin GPIO | Notas |
|---|---|---|
| RS485 #1 RX | 47 | UART1 — Agrónic 2500 |
| RS485 #1 TX | 48 | UART1 — Agrónic 2500 |
| RS485 #1 DE/RE | 45 | HIGH=TX, LOW=RX |
| RS485 #2 RX | 20 | UART2 — Variador (ABB ACQ80 o Vacon 100X) |
| RS485 #2 TX | 19 | UART2 — Variador (ABB ACQ80 o Vacon 100X) |
| RS485 #2 DE/RE | 15 | HIGH=TX, LOW=RX |
| Botón PRG | 0 | INPUT_PULLUP (LOW=pulsado) |

### Pines exclusivos del Nodo Sector

| Señal | Pin GPIO | Notas |
|---|---|---|
| VEXT | 36 | LOW=encendido, HIGH=apagado |
| DRV8833 IN1 | 4 | Canal A, válvula 1 (abrir) |
| DRV8833 IN2 | 5 | Canal A, válvula 1 (cerrar) |
| DRV8833 IN3 | 38 | Canal B, válvula 2 (abrir) |
| DRV8833 IN4 | 39 | Canal B, válvula 2 (cerrar) |
| DRV8833 STBY | 7 | LOW=sleep, HIGH=activo |

---

## Seguridad

### HMAC-SHA256

Todos los paquetes incluyen un HMAC-SHA256 de 4 bytes calculado sobre los campos de cabecera (excluyendo el propio `mac[]`). El nodo receptor recalcula el HMAC localmente y descarta el paquete si no coincide.

La clave compartida (`HMAC_KEY`, 16 bytes) debe estar previamente programada en el firmware de ambos dispositivos. No viaja por el aire.

### Anti-replay

Cada paquete de comando lleva un `messageId` monótonamente creciente, persistido en NVS (el gateway) y en RTC memory (el nodo de campo, sobrevive deep sleep). Un paquete con `messageId ≤ lastMessageId` se descarta.

### Gestión de la clave (secrets.h)

La clave HMAC vive en `src/secrets.h`, excluido de git via `.gitignore`. El repositorio incluye `secrets.h.example` con clave nula como plantilla.

**Flujo para nueva instalación:**
1. `openssl rand -hex 16` → genera 16 bytes únicos.
2. Copia `secrets.h.example` → `secrets.h` en los tres proyectos.
3. Rellena los bytes en cada `secrets.h` con los mismos valores.
4. Recompila y flashea todos los dispositivos.

**Nunca** commitear `secrets.h`. Si la clave de un nodo no coincide con la del gateway, sus paquetes serán silenciosamente descartados.

---

## Estructura del repositorio

```
Agro-LoRa/
├── .github/
│   └── workflows/
│       └── build.yml          # CI: compila los tres proyectos en cada push/PR
├── Nodo_Caseta/               # Gateway Cuadro 1 — variador ABB ACQ80-04
│   ├── src/
│   │   ├── main.cpp
│   │   ├── secrets.h          # ← NO en git (.gitignore). Crear desde el ejemplo
│   │   └── secrets.h.example  # Plantilla de clave (clave nula, sin datos reales)
│   └── platformio.ini
├── Nodo_Caseta_Vacon/         # Gateway Cuadro 2 — variador Vacon 100X
│   ├── src/
│   │   ├── main.cpp
│   │   ├── secrets.h          # ← NO en git
│   │   └── secrets.h.example
│   └── platformio.ini
├── Nodo_Sector/               # Firmware nodo de campo (mismo binario para todos los nodos)
│   ├── src/
│   │   ├── main.cpp
│   │   ├── secrets.h          # ← NO en git
│   │   └── secrets.h.example
│   └── platformio.ini
└── README.md
```

---

## Contribuir

¡Las contribuciones son bienvenidas! Por favor sigue este flujo:

1. Haz un fork del repositorio.
2. Crea una rama con un nombre descriptivo (`git checkout -b feature/nombre-descriptivo`).
3. Realiza tus cambios y asegúrate de que el proyecto compila sin errores con `pio run`.
4. Abre un Pull Request describiendo el problema que resuelve y los cambios realizados.

### Guías de estilo

- El código está escrito en C++ para Arduino/PlatformIO. Mantén el estilo existente.
- Los comentarios están en español para facilitar la comprensión a agricultores e instaladores locales.
- Evita dependencias externas innecesarias; las librerías del ESP32 SDK (mbedTLS, Preferences, deep sleep) se prefieren sobre librerías de terceros.

### Reportar errores

Abre un [issue](https://github.com/nselar/Agro-LoRa/issues) indicando:
- Hardware utilizado y revisión de placa.
- Versión de PlatformIO y de las librerías.
- Logs del monitor serie relevantes.
- Pasos para reproducir el problema.

---

## Licencia

Este proyecto está publicado bajo la [Licencia MIT](LICENSE).

```
MIT License

Copyright (c) 2025 Agro-LoRa Contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## Agradecimientos

- [RadioLib](https://github.com/jgromes/RadioLib) — Librería de radio embebida excepcional con soporte completo para SX1262.
- [modbus-esp8266](https://github.com/emelianov/modbus-esp8266) — Implementación Modbus RTU/TCP ligera para ESP32/ESP8266.
- [Heltec Automation](https://heltec.org/) — Placa de desarrollo compacta con SX1262 y OLED integrados.
- [Adafruit](https://www.adafruit.com/) — Librerías de display SSD1306 y GFX.
