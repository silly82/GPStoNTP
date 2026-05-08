# GPStoNTP — Stratum-1 NTP Server

GPS-disziplinierter NTP-Zeitserver (Stratum 1) mit PPS-Signal. Zwei Hardware-Varianten:

| Branch | Hardware | Genauigkeit |
|---|---|---|
| `main` | ESP32-S3 + W5500 (stabil, produktiv) | ±200–400 µs |
| `fast` | ESP32-S3 + W5500, INT-ISR + Core-0-GPS (nicht hardware-getestet) | ±50–200 µs |
| `concept/w6300-pico2` | RP2350 + W6300 QSPI, PIO-Timestamps (nicht hardware-getestet) | ±50–100 µs |

Status wird per MQTT publiziert.

## Genauigkeit

| Fehlerquelle | ESP32-S3 + W5500 | RP2350 + W6300 (`concept/w6300-pico2`) |
|---|---|---|
| GPS/PPS (NEO-M8N) | ±200 ns | ±200 ns |
| PPS-ISR-Jitter | ~1–5 µs | **~150 ns** (PIO + M33 IRQ) |
| T2-Timestamp | ~1–5 µs | **~150 ns** (PIO) |
| T3 vor TX | ~75 µs systematisch | **~2 µs** (QSPI 37.5 MHz, Split-Stamp) |
| Netzwerk-Jitter | ±50–150 µs | ±50–150 µs |
| **Gesamt (konservativ)** | **±200–400 µs** | **±50–100 µs** |

Der Flaschenhals ist das Netzwerk. Die GPS/PPS-Genauigkeit des NEO-M8N ist in beiden Varianten mehr als ausreichend.

## Hardware

### ESP32-S3 + W5500 (`main` / `fast`)

| Komponente | Modell |
|---|---|
| Mikrocontroller | Waveshare ESP32-S3-ETH |
| Ethernet | W5500 (onboard, SPI) |
| GPS | u-blox NEO-M8N (NMEA, 115200 Baud nach Konfiguration) |
| PPS | GPS-Modul PPS-Ausgang |

| Signal | GPIO |
|---|---|
| ETH SCLK/MISO/MOSI | 13/12/11 |
| ETH CS | 14 |
| ETH RST | 9 |
| ETH INT | 10 |
| GPS RX/TX | 1/2 |
| PPS | 4 |

### RP2350 + W6300 (`concept/w6300-pico2`)

| Komponente | Modell |
|---|---|
| Mikrocontroller | WIZnet W6300-EVB-Pico2 |
| Ethernet | W6300 (onboard, QSPI 4-bit) |
| GPS | u-blox NEO-M8N (NMEA, 115200 Baud) |
| PPS | GPS-Modul PPS-Ausgang |

| Signal | GPIO |
|---|---|
| W6300 INTn | 15 |
| W6300 CSn | 16 |
| W6300 SCLK/IO0-IO3 | 17–21 |
| W6300 RSTn | 22 |
| GPS RX/TX | 5/4 |
| PPS | 3 |

## Netzwerkkonfiguration

| Parameter | Wert |
|---|---|
| IP-Adresse | 192.168.x.x |
| Gateway | 192.168.x.x |
| Subnetzmaske | 255.255.255.0 |
| NTP-Port | 123/UDP |

## MQTT

Alle 30 Sekunden wird ein JSON-Payload auf `ntp/status` veröffentlicht:

```json
{
  "sats": 8,
  "hdop": 1.1,
  "pps": 3600,
  "served": 42
}
```

| Feld | Bedeutung |
|---|---|
| `sats` | Anzahl empfangener GPS-Satelliten |
| `hdop` | Horizontale Genauigkeit (HDOP-Wert) |
| `pps` | Gezählte PPS-Flanken seit Boot |
| `served` | Beantwortete NTP-Anfragen seit Boot |

## Arduino-Bibliotheken

| Bibliothek | Autor | Variante |
|---|---|---|
| Ethernet | Arduino | ESP32-S3 + W5500 |
| Ethernet_Generic | Khoi Hoang | RP2350 + W6300 |
| TinyGPS++ | Mikal Hart | beide |
| PubSubClient | Nick O'Leary | beide |

Alle über den Arduino Library Manager installierbar.

## Funktionsweise

### ESP32-S3 + W5500

```
GPS-Modul
  │  NMEA (115200 Baud) ──► TinyGPS++ (Core 0) ──► ntpEpochAtLastPPS
  │                        (kommt 100–400 ms nach PPS,
  │                         trägt die Zeit des letzten PPS)
  │
  └─ PPS (RISING) ──► ISR: lastPPSus = esp_timer_get_time()

W5500 INT (FALLING) ──► ISR: rxTimerUs = esp_timer_get_time()  ← T2
                                │
                         NTP-Anfrage eingehend (Core 1)
                    ┌───────────┴────────────┐
                 T2 (rxTimerUs)          T3 (vor endPacket)
```

### RP2350 + W6300 (`concept/w6300-pico2`)

```
GPS-Modul
  │  NMEA (UART1) ──────────────► Core 1: TinyGPS++ → ntpEpochAtLastPPS
  │
  └─ PPS (RISING) ─► PIO0-SM0 ─► IRQ0 ─► Core 0: lastPPSus = time_us_64()

W6300 INTn (FALLING) ─► PIO0-SM1 ─► IRQ1 ─► Core 0: rxTimerUs = time_us_64()
                                                       │
                                              Core 0: handleNTPRequest()
                                                QSPI (PIO1, 37.5 MHz) → W6300
```

PIO-Timestamps ersetzen Software-ISRs: Flanke → IRQ in ~150 ns (statt ~1–5 µs).  
QSPI überträgt 48-Byte NTP-Reply in ~6 µs (statt ~100 µs bei 1-bit SPI).

### NTP-Antwort (RFC 4330, Stratum 1, Referenz `GPS `)

| Feld | Bytes | Inhalt |
|---|---|---|
| Reference Timestamp | 16–23 | Zeitpunkt des letzten GPS/PPS-Sync |
| Origin Timestamp | 24–31 | T1 des Clients (unverändert zurück) |
| Receive Timestamp | 32–39 | T2: Hardware-Timer beim Paketempfang |
| Transmit Timestamp | 40–47 | T3: Hardware-Timer kurz vor/nach dem Senden |

**MQTT**: Alle 30 s wird ein Statuspaket an den konfigurierten Broker gesendet.

## Tools

### `tools/ublox_config.py` — GPS-Modul konfigurieren

Python-Script zur einmaligen Erstkonfiguration des u-blox GPS-Moduls über USB-Serial.

**Voraussetzung:**
```bash
pip install pyserial
```

**Verwendung:**
```bash
# Auto-Erkennung (Port + Baudrate werden automatisch gefunden)
python tools/ublox_config.py

# Expliziter Port
python tools/ublox_config.py --port /dev/ttyUSB0
python tools/ublox_config.py --port COM3

# Aktuelle Baudrate angeben (falls Auto-Erkennung fehlschlägt)
python tools/ublox_config.py --port /dev/ttyUSB0 --baud 9600

# Trocken-Lauf: konfigurieren ohne dauerhaft zu speichern
python tools/ublox_config.py --no-save
```

**Was das Script macht:**

| Schritt | UBX-Befehl | Beschreibung |
|---|---|---|
| Chip-Erkennung | `MON-VER` | Modell und Firmware-Version auslesen |
| Stationärer Modus | `CFG-NAV5` | dynModel=1, optimiert für festen Standort |
| GNSS maximieren | `CFG-GNSS` | Alle Systeme aktivieren, Kanal-Limit aufheben (M8+) |
| PPS-Timepuls | `CFG-TP5` | 1 Hz, UTC-ausgerichtet, nur aktiv wenn gelockt |
| Baudrate | `CFG-PRT` | UART1 auf 115200 Baud |
| Speichern | `CFG-CFG` | Dauerhaft in BBR + Flash |

Nach jedem Lauf wird `tools/gpsconfig.md` mit den tatsächlich angewendeten Einstellungen neu generiert (nicht im Repository, da automatisch erstellt).

> **Reihenfolge:** Script ausführen, dann erst den Arduino-Sketch flashen — der Sketch erwartet 115200 Baud vom GPS-Modul.

## Kompilieren & Flashen

### ESP32-S3 + W5500 (`main` / `fast`)

1. Arduino IDE ≥ 2.x, Espressif ESP32-Boardpaket installieren.
2. Board: **Waveshare ESP32-S3-ETH** (oder ESP32S3 Dev Module).
3. Bibliotheken: `Ethernet`, `TinyGPS++`, `PubSubClient` installieren.
4. `config.h.example` → `config.h` kopieren und Werte eintragen.
5. `tools/ublox_config.py` ausführen (GPS-Modul konfigurieren).
6. Kompilieren & Hochladen.

### RP2350 + W6300 (`concept/w6300-pico2`)

1. Arduino IDE ≥ 2.x, [arduino-pico Core](https://github.com/earlephilhower/arduino-pico) installieren.
2. Board: **WIZnet W6300-EVB-Pico2**.
3. Bibliotheken: `Ethernet_Generic`, `TinyGPS++`, `PubSubClient` installieren.
4. `W6300NTP/config.h.example` → `W6300NTP/config.h` kopieren und Werte eintragen.
5. `tools/ublox_config.py` ausführen.
6. Sketch `W6300NTP/W6300NTP.ino` kompilieren & hochladen.

## Konfiguration anpassen

Alle konfigurierbaren Werte stehen in `config.h` (aus `config.h.example` kopieren):

```cpp
/* --- NETZWERK --- */
#define CFG_MAC      { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }
#define CFG_IP       192, 168, x,   x
#define CFG_GATEWAY  192, 168, x,   1
#define CFG_SUBNET   255, 255, 255, 0
#define CFG_DNS      192, 168, x,   1

/* --- MQTT --- */
#define CFG_MQTT_SERVER  "192.168.x.x"
#define CFG_MQTT_USER    "your_user"
#define CFG_MQTT_PASS    "your_password"
#define CFG_MQTT_TOPIC   "ntp/status"
```

`config.h` ist in `.gitignore` eingetragen und wird nie commitet.

## Lizenz

MIT
