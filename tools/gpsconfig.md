# ublox_config.py — GPS Konfigurationsdokumentation

Konfigurationsskript für u-blox GPS-Module. Verbindet sich per USB-Serial,
erkennt das Modell automatisch und optimiert alle Einstellungen für
stationären NTP-Betrieb.

## Voraussetzungen

```bash
pip install pyserial
```

## Verwendung

```bash
# Auto-Erkennung (Port + Baudrate werden automatisch gefunden)
python tools/ublox_config.py

# Expliziter Port
python tools/ublox_config.py --port /dev/ttyUSB0
python tools/ublox_config.py --port COM3

# Port + aktuelle Baudrate angeben (falls Auto-Erkennung fehlschlägt)
python tools/ublox_config.py --port /dev/ttyUSB0 --baud 9600

# Trocken-Lauf: Einstellungen senden, aber NICHT dauerhaft speichern
python tools/ublox_config.py --no-save
```

## Ablauf

```
1. Alle verfügbaren Serial-Ports scannen
2. Auf jedem Port UBX-MON-VER senden (9600 / 115200 / 38400 / ...)
3. Ersten antwortenden u-blox Chip verwenden
4. Chip-Generation erkennen (NEO-6 / NEO-7 / NEO-M8)
5. Konfigurationsschritte ausführen (siehe unten)
6. Baudrate auf 115200 umstellen
7. Verbindung bei neuer Baudrate verifizieren
8. Alle Einstellungen dauerhaft speichern
```

## Konfigurierte Einstellungen

### Stationärer Modus — `UBX-CFG-NAV5`

| Parameter | Wert | Bedeutung |
|---|---|---|
| `dynModel` | 1 | Stationary — optimiert Filterung für festes GPS |
| `fixMode` | 3 | Auto 2D/3D |

> Warum: Im Stationary-Modus geht der Empfänger davon aus, dass er sich
> nicht bewegt. Die Positionsfilterung wird aggressiver, Zeitgenauigkeit
> verbessert sich deutlich gegenüber dem Standard-Modus (Portable).

---

### GNSS Multi-System + Kanal-Limit aufheben — `UBX-CFG-GNSS` *(M8+ only)*

| Parameter | Wert | Bedeutung |
|---|---|---|
| `numTrkChUse` | 0xFF | Alle verfügbaren Hardware-Kanäle nutzen |
| Alle GNSS-Blöcke | enabled | GPS, SBAS, Galileo, BeiDou, QZSS, GLONASS |
| `maxTrkCh` | Hardware-Maximum | Pro System werden alle verfügbaren Kanäle freigegeben |

> Warum: Standardmässig ist oft nur GPS aktiv oder die Kanalzahl auf 12
> begrenzt. Mit GLONASS + Galileo sind gleichzeitig 20–30+ Satelliten
> sichtbar, was die Zeitlösung robuster und schneller macht.
>
> NEO-6M: Nur GPS + SBAS unterstützt, dieser Schritt wird übersprungen.

---

### PPS-Timepuls — `UBX-CFG-TP5`

| Parameter | Wert | Bedeutung |
|---|---|---|
| Frequenz | 1 Hz | Ein Puls pro Sekunde |
| Pulsbreite | 100 ms | Steigende Flanke = Sekundenanfang |
| Ausrichtung | UTC | Puls ist zur UTC-Sekunde ausgerichtet (nicht GPS-Zeit) |
| Aktivierung | Nur wenn gelockt | Kein Puls bei fehlendem GNSS-Fix |
| `antCableDelay` | 0 ns | Antennenkabelverzögerung — bei Bedarf anpassen |
| `userConfigDelay` | 0 ns | Zusätzlicher Software-Delay |

**Flags-Bitmuster:**

| Bit | Name | Wert | Bedeutung |
|---|---|---|---|
| 0 | active | 1 | Timepuls aktiv |
| 1 | lockGnssFreq | 1 | GNSS-Frequenz verwenden |
| 2 | lockedOtherSet | 1 | Lock-Parametersatz aktiv wenn gesynct |
| 3 | isFreq | 1 | `freqPeriod` = Hz (nicht µs) |
| 4 | isLength | 1 | `pulseLenRatio` = µs (nicht Duty-Cycle) |
| 5 | alignToTow | 1 | Puls am Time-of-Week ausrichten |
| 6 | polarity | 1 | Steigende Flanke = Sekundenanfang |
| 7–10 | gridUtcGnss | 0 | UTC-Raster (nicht GPS) |

> Tipp: Wenn die Antenne eine lange Zuleitung hat (>1 m), kann
> `antCableDelay` auf ~5 ns/m gesetzt werden. Direkt im Script unter
> `cfg_timepulse()` anpassen.

---

### Baudrate — `UBX-CFG-PRT`

| Parameter | Wert |
|---|---|
| UART1 Baudrate | 115200 |
| Protokoll In | UBX + NMEA |
| Protokoll Out | UBX + NMEA |

> Nach der Baudrate-Änderung muss im Arduino-Sketch angepasst werden:
> ```cpp
> GPS_Serial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
> ```

---

### Dauerhaft speichern — `UBX-CFG-CFG`

| Ziel | Aktiviert |
|---|---|
| BBR (Battery-Backed RAM) | ja |
| Flash | ja |
| EEPROM | ja (falls vorhanden) |

> Einstellungen bleiben nach Stromverlust erhalten, solange die
> Backup-Batterie des GPS-Moduls nicht leer ist.

---

## Nach der Konfiguration

### Arduino-Sketch anpassen

In `config.h` (oder direkt in `GPStoNTP.ino`) die GPS-Baudrate ändern:

```cpp
// GPS-UART (nach ublox_config.py: 115200 Baud)
GPS_Serial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
```

### Konfiguration prüfen

Mit einem Serial-Monitor (z.B. `screen /dev/ttyUSB0 115200`) sollten
NMEA-Sätze mit Zeitstempel sichtbar sein:

```
$GPRMC,123456.00,A,4728.12345,N,00832.12345,E,0.000,,040526,,,D*XX
```

---

## Kompatibilität

| Modul | Stationär | GNSS Multi | PPS-Config | Baudrate |
|---|---|---|---|---|
| NEO-6M | ✓ | — | ✓ | ✓ |
| NEO-7M | ✓ | — | ✓ | ✓ |
| NEO-M8N | ✓ | ✓ | ✓ | ✓ |
| NEO-M8T | ✓ | ✓ | ✓ | ✓ |
| ZED-F9P | ✓ | ✓ | ✓ | ✓ |
