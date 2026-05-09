# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

ESP32-S3 Stratum-1 NTP server: W5500 Ethernet (SPI) + u-blox NEO-M8N GPS (UART) + PPS signal → serves NTP on port 123/UDP, publishes status via MQTT every 30 s.

## Build & Flash

This is an Arduino sketch — there is no CLI build tool. Use **Arduino IDE ≥ 2.x**:

1. Board: `ESP32S3 Dev Module` (Espressif ESP32 board package required)
2. Libraries (Arduino Library Manager): `Ethernet` (Arduino), `TinyGPS++` (Mikal Hart), `PubSubClient` (Nick O'Leary)
3. Copy `config.h.example` → `config.h` and fill in network/MQTT values (`config.h` is gitignored)
4. Configure the GPS module once before first flash (see below)
5. Compile & upload via Arduino IDE

## GPS Module Setup (one-time)

```bash
pip install pyserial
python tools/ublox_config.py          # auto-detects port and baud
python tools/ublox_config.py --port /dev/ttyUSB0
python tools/ublox_config.py --no-save   # dry run
```

Sets dynModel=Stationary, all GNSS systems on, PPS 1 Hz UTC-aligned, UART1 → 115200 baud, saves to flash. Run this before flashing the sketch — the sketch expects 115200 baud. Generates `tools/gpsconfig.md` (gitignored) with the applied settings.

## Architecture

**Timing chain** (the core of the design):

```
PPS rising edge  → ISR (ppsHandler):   lastPPSus = esp_timer_get_time()
NMEA sentence    → GPS task (Core 0):  ntpEpochAtLastPPS = mktime(...) + 2208988800UL
W5500 INT falls  → ISR (ethIntHandler): rxTimerUs = esp_timer_get_time()
loop() (Core 1) sees parsePacket() ≥ 48:
  T2: rxTimerUs (ISR-captured, pre-SPI-overhead)
  T3: esp_timer_get_time() just before send
  Δt = Tx - lastPPSus → sec + fraction
```

Key invariants:
- `ntpEpochAtLastPPS` and `lastPPSus` are always a matched pair (same PPS edge). NMEA latency (100–400 ms) doesn't matter because the sentence always carries the time of the *last* PPS second.
- `rxTimerUs` is written by ISR on W5500 INT falling edge — before `parsePacket()` SPI overhead (~100–500 µs). Gives T2 accuracy of ~2–7 µs.
- GPS parsing runs on Core 0 (`gpsTask`), NTP loop runs on Core 1 (`loop()`). No GPS jitter on the NTP path. `ntpEpochAtLastPPS`, `gpsSats`, `gpsHdop` are `volatile` for cross-core visibility (32-bit aligned → atomically read/written on ESP32).
- W5500 interrupt configuration (`Sn_IMR`, `IMR`) must happen **after** `ntpUDP.begin(123)`.
- Board: **Waveshare ESP32-S3-ETH** — W5500 INT is hardwired to GPIO 10, no extra cable needed.

**Pin mapping** (defined as constants in `GPStoNTP.ino`):

| Signal | Pin |
|---|---|
| W5500 SCLK/MISO/MOSI/CS/RST | 13/12/11/14/9 |
| W5500 INT | 10 (`ETH_INT_PIN`, auf Waveshare ESP32-S3-ETH fest verdrahtet) |
| GPS RX/TX | 1/2 |
| PPS | 3 |

**Config** (`config.h`, never committed): network (`CFG_MAC`, `CFG_IP`, `CFG_GATEWAY`, `CFG_SUBNET`, `CFG_DNS`) and MQTT (`CFG_MQTT_SERVER/USER/PASS/TOPIC`).
