# STM32 CAN ADC → LCD + Servo + Heartbeat (ThreadX)
**STM32F446RE (Transmitter/Monitor) ↔ STM32L476RG (Receiver/Actuator + ThreadX)**

This project demonstrates an end-to-end embedded CAN pipeline with real-time control and robustness features:

- **STM32F446RE (F4)** reads a potentiometer using ADC, scales it to 8-bit, and transmits the value over CAN (**StdID `0x446`**).
- **STM32L476RG (L4)** runs **ThreadX RTOS**, receives CAN frames via **ISR (FIFO1) → ThreadX Queue**, updates a **16×2 I2C LCD**, and drives a **servo via PWM (TIM2 CH1)** based on the received value.
- **Heartbeat / Link Health Check**: L4 periodically sends a heartbeat frame (**StdID `0x476`**) back to F4. F4 performs a **timeout-based heartbeat check** (e.g., no heartbeat within 300 ms → link/node fault) and indicates status via LED/UART logs.

---

## Key Features
- CAN communication (F4 → L4 control frames, L4 → F4 heartbeat)
- ADC sampling and scaling (F4)
- ThreadX ISR-to-Queue message passing (L4)
- Live 16×2 I2C LCD display (PCF8574 backpack; common addr `0x27` / `0x3F`)
- Servo control with calibrated PWM pulse range (L4: TIM2 PWM)
- Heartbeat monitoring with timeout detection (industry-style link supervision)
- UART debug counters (RX count, ISR hits, queue full, I2C errors)

---

## Hardware
- STM32F446RE (Tx + heartbeat monitor)
- STM32L476RG (Rx + ThreadX + LCD + Servo)
- 2× CAN transceivers (TJA1050 / SN65HVD230 / MCP2551, etc.)
- Potentiometer (e.g., 10k)
- SG90 (or similar) servo (external 5V recommended)
- 16×2 I2C LCD with PCF8574 backpack (example: **GJD1602IIC**)
- 2× 120Ω termination resistors (recommended)

---

## CAN Protocol
### 1) Potentiometer value (F4 → L4)
- **StdID:** `0x446`
- **DLC:** 8
- **Payload (example used):**
  - `TxData[7] = scaled` where `scaled = adc_raw >> 4` (0–255)

### 2) Heartbeat (L4 → F4)
- **StdID:** `0x476`
- **DLC:** 2 (or 8)
- **Payload suggestion:**
  - `d[0] = hb_seq` (increment each heartbeat)
  - `d[1] = last_scaled_received` (optional)

---

## ThreadX Architecture (L4)
### Why ISR → Queue?
CAN RX happens in interrupt context; the ISR stays short and safe:
- ISR reads CAN FIFO (FIFO1) and pushes a small message into a ThreadX queue
- A ThreadX thread blocks on the queue, then updates:
  - LCD
  - Servo PWM
  - Optional debug logs/counters

---

## Servo Control (L4)
- **Timer:** TIM2
- **Channel:** CH1 (example pin: PA0)
- Map `scaled (0–255)` → `angle (0–180)` → `pulse_us` → PWM compare ticks
- Practical calibration: many SG90 clones need wider range than 1000–2000 µs
  - Typical working range: **500–2500 µs**
  - Neutral: **1500 µs**

LCD line example:
- Line 1: `Pot->Servo`
- Line 2: `S:188 A:135`

---

## Heartbeat Check (F4)
Heartbeat provides an “industry-style” link supervision mechanism:
- L4 sends heartbeat periodically (e.g., every **100 ms**)
- F4 records the last heartbeat timestamp
- F4 declares a fault if no heartbeat arrives within a timeout window
  - Example: heartbeat period = 100 ms → timeout = **300 ms**
- On fault, F4 can:
  - set LED to solid ON / slow blink
  - print a “LINK LOST” log
  - (optional) stop transmitting control or enter safe mode

**Test:** unplug L4 power → F4 should time out and show fault state after the configured timeout.

---

## Debug Tips
### LCD not updating
- Confirm I2C address (`0x27` vs `0x3F`)
- Track I2C error counter from `HAL_I2C_Master_Transmit` return status
- If `i2c_err` increases, it’s an address/wiring/power issue







