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
PPS rising edge → ISR: lastPPSus = esp_timer_get_time()   (64-bit µs, IRAM)
NMEA sentence (arrives 100–400 ms after PPS, carries time of that second)
  → ntpEpochAtLastPPS = mktime(gps.time/date) + 2208988800UL
NTP request arrives → rxTimerUs = esp_timer_get_time()
  → Δt = rxTimerUs - lastPPSus
  → sec  = ntpEpochAtLastPPS + Δt / 1e6
  → frac = (Δt % 1e6) × 2³² / 1e6
```

Key invariant: `ntpEpochAtLastPPS` and `lastPPSus` are always a matched pair — both represent the same PPS edge. The NMEA latency (100–400 ms) doesn't introduce error because the sentence always carries the time of the *last* PPS second.

**Pin mapping** (defined as constants in `GPStoNTP.ino`):

| Signal | Pin |
|---|---|
| W5500 SCLK/MISO/MOSI/CS/RST | 13/12/11/14/9 |
| GPS RX/TX | 1/2 |
| PPS | 4 |

**Config** (`config.h`, never committed): network (`CFG_MAC`, `CFG_IP`, `CFG_GATEWAY`, `CFG_SUBNET`, `CFG_DNS`) and MQTT (`CFG_MQTT_SERVER/USER/PASS/TOPIC`).
