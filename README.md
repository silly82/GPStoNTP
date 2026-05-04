# GPStoNTP — Stratum-1 NTP Server

ESP32-basierter NTP-Zeitserver (Stratum 1), der GPS-Zeit über ein W5500-Ethernet-Modul im lokalen Netzwerk bereitstellt. Ein PPS-Signal (Pulse Per Second) sorgt für präzise Zeitstempelung. Der Status wird per MQTT publiziert.

## Genauigkeit

| Ebene | Typischer Wert |
|---|---|
| GPS-Timing (NEO-M8N, PPS) | 50–200 ns |
| ESP32 Interrupt-Jitter (`esp_timer`) | ~1–5 µs |
| NTP über Ethernet (LAN) | 100–500 µs |

Der Flaschenhals ist das Netzwerk — die GPS/PPS-Genauigkeit des NEO-M8N ist für einen Stratum-1 Server im LAN mehr als ausreichend. Survey-in Timing Mode (NEO-M8T) würde die GPS-Ebene auf ~10–50 ns verbessern, ändert aber nichts an der NTP-Netzwerkgenauigkeit.

## Hardware

| Komponente | Modell (Beispiel) |
|---|---|
| Mikrocontroller | ESP32-S3 DevKit |
| Ethernet | W5500 SPI-Modul |
| GPS | u-blox NEO-M8N (NMEA, 115200 Baud nach Konfiguration) |
| PPS | GPS-Modul PPS-Ausgang |

### Pinbelegung

| Signal | ESP32-Pin |
|---|---|
| ETH SCLK | 13 |
| ETH MISO | 12 |
| ETH MOSI | 11 |
| ETH CS | 14 |
| ETH RST | 9 |
| GPS RX (ESP empfängt) | 1 |
| GPS TX (ESP sendet) | 2 |
| PPS | 4 |

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

Alle über den Arduino Library Manager installierbar:

| Bibliothek | Autor |
|---|---|
| Ethernet | Arduino |
| TinyGPS++ | Mikal Hart |
| PubSubClient | Nick O'Leary |

## Funktionsweise

```
GPS-Modul
  │  NMEA (115200 Baud) ──► TinyGPS++ ──► ntpEpochAtLastPPS
  │                        (kommt 100–400 ms nach PPS,
  │                         trägt die Zeit des letzten PPS)
  │
  └─ PPS (1 Hz, RISING) ──► ISR: lastPPSus = esp_timer_get_time()
                                │
                         NTP-Anfrage eingehend
                                │
                    ┌───────────┴────────────┐
                 T2 (Receive)           T3 (Transmit)
          esp_timer beim Empfang    esp_timer vor dem Senden
                    │
          Seconds  = ntpEpochAtLastPPS + Δt / 1e6
          Fraction = (Δt % 1e6) × 2³² / 1e6
```

1. **GPS-Parsing**: TinyGPS++ liest NMEA-Sätze aus `HardwareSerial(1)`. Die Sentence trifft 100–400 ms nach dem PPS-Puls ein und enthält die Zeit der gerade gestarteten Sekunde — also genau die Sekunde, die `lastPPSus` markiert.
2. **PPS/NMEA-Sync**: `ntpEpochAtLastPPS` wird beim NMEA-Parse gesetzt und ist damit korrekt an den Hardware-Timer-Wert `lastPPSus` gebunden.
3. **PPS-ISR**: Bei jeder steigenden PPS-Flanke wird `esp_timer_get_time()` (64-bit Hardware-Timer, kein Arduino-HAL-Overhead) direkt im Interrupt gespeichert.
4. **NTP-Antwort** (RFC 4330, Stratum 1, Referenz `GPS `):

   | Feld | Bytes | Inhalt |
   |---|---|---|
   | Reference Timestamp | 16–23 | Zeitpunkt des letzten GPS/PPS-Sync |
   | Origin Timestamp | 24–31 | T1 des Clients (unverändert zurück) |
   | Receive Timestamp | 32–39 | T2: Hardware-Timer beim Paketempfang |
   | Transmit Timestamp | 40–47 | T3: Hardware-Timer kurz vor dem Senden |

5. **MQTT**: Alle 30 s wird ein Statuspaket an den konfigurierten Broker gesendet.

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
2. Board: **ESP32S3 Dev Module** (oder passendes ESP32-Board).
3. Obige Bibliotheken installieren.
4. `GPStoNTP.ino` öffnen, Netzwerk- und MQTT-Parameter in den `#define`/`const`-Blöcken anpassen.
5. Kompilieren & Hochladen.

## Konfiguration anpassen

Alle konfigurierbaren Werte stehen am Anfang der Datei `GPStoNTP.ino`:

```cpp
// Netzwerk
byte mac[]       = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip     (192, 168, x, x);
...

// MQTT
const char* mqtt_server = "192.168.x.x";
const char* mqtt_user   = "your_user";
const char* mqtt_pass   = "your_password";
```

## Lizenz

MIT
