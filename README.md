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
| PPS | 3 |

## Netzwerkkonfiguration

| Parameter | Wert |
|---|---|
| IP-Adresse | 192.168.188.2 |
| Gateway | 192.168.188.1 |
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
  │  NMEA (9600 Baud) ──► TinyGPS++ ──► UTC-Zeit → NTP-Epoch
  │
  └─ PPS (1 Hz, RISING) ──► ISR speichert micros()
                                │
                         NTP-Anfrage eingehend
                                │
                         Transmit-Timestamp =
                           GPS-Epoch + (micros() – lastPPS) / 1e6
```

1. **GPS-Parsing**: TinyGPS++ liest NMEA-Sätze aus `HardwareSerial(1)` und liefert validierte UTC-Zeit und Datum.
2. **NTP-Epoch**: GPS-UTC wird in einen NTP-Timestamp (Sekunden seit 1.1.1900) umgerechnet.
3. **PPS-ISR**: Beim jedem sekundengenauem PPS-Puls wird `micros()` gespeichert. Damit lässt sich der Sub-Sekunden-Anteil für den NTP-Transmit-Timestamp berechnen.
4. **NTP-Antwort**: Eingehende UDP-Pakete auf Port 123 werden nach RFC 4330 beantwortet (Stratum 1, Referenz "GPS ").
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
IPAddress ip     (192, 168, 188, 2);
...

// MQTT
const char* mqtt_server = "192.168.188.62";
const char* mqtt_user   = "silly82";
const char* mqtt_pass   = "...";
```

## Lizenz

MIT
