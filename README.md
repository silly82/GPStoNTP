# GPStoNTP — Stratum-1 NTP Server

ESP32-basierter NTP-Zeitserver (Stratum 1), der GPS-Zeit über Ethernet im lokalen Netzwerk bereitstellt. Ein PPS-Signal (Pulse Per Second) sorgt für präzise Zeitstempelung. Der Status wird per MQTT publiziert.

## Genauigkeit

| Ebene | Typischer Wert |
|---|---|
| GPS-Timing (NEO-M8N, PPS) | 50–200 ns |
| ESP32 Interrupt-Jitter (`esp_timer`) | ~1–5 µs |
| NTP über Ethernet (LAN) | 100–500 µs |

Der Flaschenhals ist das Netzwerk — die GPS/PPS-Genauigkeit des NEO-M8N ist für einen Stratum-1 Server im LAN mehr als ausreichend. Survey-in Timing Mode (NEO-M8T) würde die GPS-Ebene auf ~10–50 ns verbessern, ändert aber nichts an der NTP-Netzwerkgenauigkeit.

## Hardware

| Komponente | Modell |
|---|---|
| Mikrocontroller + Ethernet | [Waveshare ESP32-S3-ETH](https://www.waveshare.com/wiki/ESP32-S3-ETH) (ESP32-S3 + W5500 onboard) |
| GPS | u-blox NEO-M8N (NMEA, 115200 Baud nach Konfiguration) |
| PPS | GPS-Modul PPS-Ausgang |

Das Waveshare ESP32-S3-ETH hat den W5500 bereits onboard verdrahtet — kein separates SPI-Modul nötig.

### Pinbelegung

| Signal | ESP32-Pin | Hinweis |
|---|---|---|
| ETH SCLK | 13 | onboard W5500 |
| ETH MISO | 12 | onboard W5500 |
| ETH MOSI | 11 | onboard W5500 |
| ETH CS | 14 | onboard W5500 |
| ETH RST | 9 | onboard W5500 |
| ETH INT | 10 | onboard W5500, für ISR-Zeitstempel |
| GPS RX (ESP empfängt) | 1 | |
| GPS TX (ESP sendet) | 2 | |
| PPS | 4 | |

## Netzwerkkonfiguration

| Parameter | Wert |
|---|---|
| IP-Adresse | 192.168.x.x (in `config.h`) |
| Gateway | 192.168.x.x (in `config.h`) |
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

Alle über den Arduino Library Manager installierbar:

| Bibliothek | Autor |
|---|---|
| Ethernet | Arduino |
| TinyGPS++ | Mikal Hart |
| PubSubClient | Nick O'Leary |

## Funktionsweise

```
GPS-Modul
  │  NMEA (115200 Baud) ──► GPS-Task (Core 0) ──► ntpEpochAtLastPPS
  │                        (kommt 100–400 ms nach PPS,
  │                         trägt die Zeit des letzten PPS)
  │
  └─ PPS (1 Hz, RISING) ──► ISR: lastPPSus = esp_timer_get_time()

W5500 INT (FALLING) ──► ISR: rxTimerUs = esp_timer_get_time()  ← T2
                               │
                        NTP-Anfrage (loop, Core 1)
                               │
                   ┌───────────┴────────────┐
                T2 (rxTimerUs)         T3 (Transmit)
          ISR beim Paketeingang    esp_timer vor dem Senden
                   │
         Seconds  = ntpEpochAtLastPPS + Δt / 1e6
         Fraction = (Δt % 1e6) × 2³² / 1e6
```

1. **GPS-Task (Core 0)**: TinyGPS++ liest NMEA aus `HardwareSerial(1)`. Setzt `ntpEpochAtLastPPS` und cached Satelliten/HDOP für MQTT.
2. **PPS-ISR**: Bei jeder steigenden Flanke wird `esp_timer_get_time()` (64-bit Hardware-Timer) direkt gespeichert.
3. **W5500-INT-ISR**: Bei fallender Flanke des W5500-INT-Pins wird `rxTimerUs` gestempelt — vor dem `parsePacket()`-SPI-Overhead. Reduziert T2-Fehler von ~100–500 µs auf ~2–7 µs.
4. **NTP-Loop (Core 1)**: Nur NTP und MQTT, kein GPS-Parsing. Kein Jitter durch NMEA-Verarbeitung.
5. **NTP-Antwort** (RFC 4330, Stratum 1, Referenz `GPS `):

   | Feld | Bytes | Inhalt |
   |---|---|---|
   | Reference Timestamp | 16–23 | Zeitpunkt des letzten GPS/PPS-Sync |
   | Origin Timestamp | 24–31 | T1 des Clients (unverändert zurück) |
   | Receive Timestamp | 32–39 | T2: ISR-Stempel beim Paketeingang |
   | Transmit Timestamp | 40–47 | T3: Hardware-Timer kurz vor dem Senden |

6. **MQTT**: Alle 30 s wird ein Statuspaket an den konfigurierten Broker gesendet.

## Branches

| Branch | Status | Beschreibung |
|---|---|---|
| `main` | stabil | Bewährter Stand, polling-basiertes T2 |
| `fast` | in Erprobung | W5500-INT-ISR, GPS auf Core 0, SPI 20 MHz |

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

1. Arduino IDE ≥ 2.x öffnen, ESP32-Boardpaket installiert (Espressif).
2. Board: **ESP32S3 Dev Module** (oder passendes ESP32-S3-Board).
3. Obige Bibliotheken installieren.
4. `tools/ublox_config.py` ausführen um das GPS-Modul zu konfigurieren.
5. `config.h.example` → `config.h` kopieren und Werte eintragen.
6. Kompilieren & Hochladen.

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
