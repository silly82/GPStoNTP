# GPStoNTP — Stratum-1 NTP Server

ESP32-basierter NTP-Zeitserver (Stratum 1), der GPS-Zeit über ein W5500-Ethernet-Modul im lokalen Netzwerk bereitstellt. Ein PPS-Signal (Pulse Per Second) sorgt für Sub-Millisekunden-Genauigkeit. Der Status wird per MQTT publiziert.

## Hardware

| Komponente | Modell (Beispiel) |
|---|---|
| Mikrocontroller | ESP32-S3 DevKit |
| Ethernet | W5500 SPI-Modul |
| GPS | u-blox NEO-6M / NEO-8M (NMEA, 9600 Baud) |
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
  │  NMEA (9600 Baud) ──► TinyGPS++ ──► ntpEpochAtLastPPS
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
