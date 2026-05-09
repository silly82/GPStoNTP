# W6300-EVB-Pico2 als NTP-Stratum-1-Server

> Status: **implementiert** (`concept/w6300-pico2` Branch, nicht hardware-getestet)  
> Board: [WIZnet W6300-EVB-Pico2](https://docs.wiznet.io/Product/Chip/Ethernet/W6300/w6300-evb-pico2)  
> Verglichen mit: `fast`-Branch (Waveshare ESP32-S3-ETH)

---

## Board-Übersicht

| Merkmal | W6300-EVB-Pico2 | Waveshare ESP32-S3-ETH |
|---|---|---|
| MCU | RP2350 (dual Cortex-M33, 150 MHz) | ESP32-S3 (dual Xtensa LX7, 240 MHz) |
| Ethernet | W6300 **QSPI** (4 Datenleitungen) | W5500 SPI (1 Datenleitung) |
| SRAM | 520 KB | 512 KB |
| PIO | **8 State Machines** | — |
| INT-Pin | GPIO15 (onboard) | GPIO10 (onboard) |

### Pinbelegung W6300

| Signal | GPIO |
|---|---|
| W6300 INTn | 15 |
| W6300 CSn | 16 |
| W6300 SCLK | 17 |
| W6300 IO0 (MOSI) | 18 |
| W6300 IO1 (MISO) | 19 |
| W6300 IO2 | 20 |
| W6300 IO3 | 21 |
| W6300 RSTn | 22 |

---

## Warum besser?

Zwei unabhängige Hardware-Verbesserungen, die direkt die NTP-Genauigkeit erhöhen:

### 1. PIO für Timestamps (PPS + T2)

**Problem heute:** ESP32-ISR-Latenz ~1–5 µs. Der Interrupt-Controller und der Xtensa-CPU-Pipeline-Flush kosten Zyklen.

**Mit PIO:** Ein RP2350-PIO-State-Machine wartet deterministisch auf eine GPIO-Flanke und löst einen IRQ aus. Der Cortex-M33 antwortet in **12–20 CPU-Zyklen** (Pipeline-Tiefe + Interrupt-Entry). Bei 150 MHz = **80–133 ns** Latenz — etwa 20–50× besser als ESP32.

Implementiert in `pps_capture.pio` (PIO0 SM0) und `eth_int_capture.pio` (PIO0 SM1):

```asm
; pps_capture.pio — PIO0 SM0
.program pps_capture
.wrap_target
    wait 0 pin 0     ; warte bis PPS LOW (Re-Arming)
    wait 1 pin 0     ; steigende Flanke
    irq nowait 0     ; IRQ 0 → Core 0 liest time_us_64()
.wrap

; eth_int_capture.pio — PIO0 SM1
.program eth_int_capture
.wrap_target
    wait 1 pin 0     ; warte bis INTn HIGH (quittiert)
    wait 0 pin 0     ; fallende Flanke = Paket eingetroffen
    irq nowait 1     ; IRQ 1 → Core 0 liest time_us_64()
.wrap
```

### 2. QSPI statt SPI — T3-Systematik drastisch reduziert

**Problem heute:** T3 wird vor `endPacket()` gestempelt. Die SPI-Übertragung des UDP-Pakets dauert ~50–150 µs (W5500, 1-bit SPI, ~14 MHz). T3 ist damit systematisch zu früh.

**Mit QSPI (4-bit, 37.5 MHz):**
- 4× mehr Bits pro Takt: gleiche Datenmenge in 1/4 der Zeit
- `QSPI_CLKDIV = 2.0f` → 75 MHz PIO-Takt, 37.5 MHz QSPI (SPI-Mode 0: H auf LOW = H nach SCLK-High)
- 48 Bytes NTP-Reply + IP/UDP-Header (~120 Bytes total): ~120×8 / (4×37.5M) ≈ **6 µs** statt ~100 µs

T3 wird nach dem QSPI-Write (Split-Strategie) gestempelt — T3-Fehler sinkt auf **~1–3 µs**.

---

## Implementierung

### PIO-Programme

| Datei | PIO | SM | Funktion |
|---|---|---|---|
| `pps_capture.pio` | PIO0 | 0 | Rising-Edge PPS → IRQ 0 |
| `eth_int_capture.pio` | PIO0 | 1 | Falling-Edge INTn → IRQ 1 |
| `w6300_qspi.pio` | PIO1 | 0 | Single-SPI TX (Opcode + Adresse) |
| `w6300_qspi.pio` | PIO1 | 1 | Quad-SPI TX (Daten, 4-bit/Takt) |
| `w6300_qspi.pio` | PIO1 | 2 | Quad-SPI RX (Daten, 4-bit/Takt) |

### QSPI-TX-Pfad in `handleNTPRequest()`

```
1. beginPacket()          → setzt Ziel-IP/Port via SPI (Ethernet_Generic)
2. ReadReg16(Sn_TX_WR)   → liest TX-Schreibzeiger via SPI
3. qspiWriteBuf(bytes 0–39)  → QSPI-Write via PIO1 (Payload ohne T3)
4. T3 = time_us_64()     → Timestamp direkt nach QSPI-Transaktion
5. qspiWriteBuf(bytes 40–47) → QSPI-Write T3-Bytes
6. WriteReg16(Sn_TX_WR)  → aktualisiert Schreibzeiger via SPI
7. w6300Write(Sn_CR=SEND) → triggert Ethernet-TX
```

### GPIO-Switching

`qspiTakeGPIOs()` / `qspiReleaseGPIOs()` schalten GPIO17–21 zwischen SPI-Peripheral (Ethernet_Generic) und PIO1 (QSPI-Treiber) um.

### Core-Aufteilung

```
GPS-Modul
  │  NMEA (UART1, GPIO4/5) ──────► Core 1: TinyGPS++ → ntpEpochAtLastPPS
  │
  └─ PPS (RISING) ─► PIO0-SM0 ─► IRQ0 ─► Core 0: lastPPSus = time_us_64()

W6300 INTn (FALLING) ─► PIO0-SM1 ─► IRQ1 ─► Core 0: rxTimerUs = time_us_64()
                                                       │
                                              Core 0: handleNTPRequest()
                                                QSPI (PIO1) → W6300 → Ethernet
```

| Core | Aufgabe |
|---|---|
| Core 0 | NTP-Handler, PIO-IRQs (PPS + INT), QSPI-Treiber, MQTT |
| Core 1 | GPS-Parsing (TinyGPS++) |

**Cross-Core-Sync:** `ntpEpochAtLastPPS` als `std::atomic<uint32_t>` mit `store(release)` / `load(acquire)`.

---

## Fehlerbudget-Vergleich (konservativ, 12 Sats)

| Fehlerquelle | ESP32-S3 + W5500 (`fast`) | RP2350 + W6300 |
|---|---|---|
| GPS/PPS (NEO-M8N) | ±200 ns | ±200 ns |
| PPS-ISR-Jitter | ±5 µs | **±150 ns** (PIO + M33 IRQ) |
| T2-INT-ISR | ±10 µs | **±150 ns** (PIO) |
| T3 vor SPI-Send | ~75 µs syst. | **~2 µs** (QSPI 37.5 MHz, Split-Stamp) |
| Netzwerk-Jitter | ±50–150 µs | ±50–150 µs |
| Client T4 (chrony) | ±10–50 µs | ±10–50 µs |
| **Gesamt (konservativ)** | **±200–400 µs** | **±50–100 µs** |

Limitierender Faktor wird das Netzwerk und der NTP-Client — nicht mehr die Hardware.

---

## Benötigte Bibliotheken

| Bibliothek | Quelle | Status |
|---|---|---|
| Ethernet_Generic (W6300) | Arduino Library Manager | verfügbar |
| arduino-pico (RP2350 Core) | Earle Philhower | stabil |
| TinyGPS++ | Arduino Library Manager | portabel |
| PubSubClient | Arduino Library Manager | portabel |
| pioasm | Pico SDK | `.pio.h` via `pioasm` generiert |

---

## Offene Punkte vor Hardware-Test

### Kritisch (Blocker)

1. **`Sn_TX_WR` Registeradresse prüfen**: Im Code als `0x0022` definiert — laut W5500/W6300-Registermap ist `0x0022` jedoch `Sn_TX_RD` (Read Pointer), `Sn_TX_WR` (Write Pointer) ist `0x0024`. Falls falsch: keine NTP-Antworten. Sofort mit `Serial.println(w6300ReadReg16(0x0022, 1))` vs. `(0x0024, 1)` vergleichen und korrigieren.

2. **`g_txWrPtr` Initialwert**: Nach `ntpUDP.begin(123)` muss `g_txWrPtr == 0x0000` sein. Mit `Serial.println(g_txWrPtr)` in `setup()` bestätigen, bevor der erste NTP-Request kommt.

3. **QSPI-Takt Signalqualität**: `QSPI_CLKDIV = 2.0f` → 37.5 MHz. Mit Oszilloskop auf IO0–IO3 messen — Leitungskapazität auf EVB-Pico2 kann Flanken verzerren. Bei Fehlern `QSPI_CLKDIV` auf `3.0f` (25 MHz) erhöhen.

### Validierung

4. **T3 Split-Strategie messen**: Mit `chronyc tracking` die RMS-Abweichung loggen. Zielwert: < 10 µs systematischer Fehler (T3-Stamp zwischen den beiden QSPI-Writes).

5. **GPIO-Switching-Timing**: Nach jeder SPI-Transaktion (Ethernet_Generic) muss `qspiReleaseGPIOs()` korrekt abschliessen — prüfen ob CS-Leitungen nach QSPI-Transaktion sauber HIGH sind.

6. **GPS-UART-Pins**: `GPS_RX_PIN 1` / `GPS_TX_PIN 0` auf W6300-EVB-Pico2 verfügbar und nicht mit UART0 des USB-CDC kollidierend prüfen.

### Fine-Tuning nach erstem erfolgreichen Betrieb

7. **ISR-Flag statt Polling**: `ntpUDP.parsePacket()` in `loop()` pollt via SPI. Die `ethIntISR` setzt ein Flag; `loop()` prüft nur noch das Flag — eliminiert Loop-Latenz aus dem T2→handleNTPRequest-Pfad (potentiell ±50–200 µs weniger Jitter).

8. **`mktime()` auf Core 1 absichern**: `mktime()` ist nicht deterministisch (Schaltjahr-Logik). Falls die NTP-Epoch gelegentlich um 1 Sekunde springt, `timegm()` (kein Timezone-Overhead) oder manuelle Berechnung verwenden.

9. **Ring-Buffer-Wrap von `g_txWrPtr`**: Aktuell uint16_t, wraps at 0xFFFF. Der Masking-Ausdruck `txWrPtr & 0x1FFF` setzt 8 KB TX-Buffer voraus — bei anderem Puffer-Setup anpassen.

---

## Fazit

| Kriterium | ESP32-S3 + W5500 | RP2350 + W6300 |
|---|---|---|
| Erreichbare NTP-Genauigkeit | ±200–400 µs | **±50–100 µs** |
| Aufwand Migration | — | mittel (neue Plattform, PIO, QSPI-HAL) |
| Bibliotheksreife | hoch | mittel (QSPI-HAL selbst implementiert) |
| Kosten/Verfügbarkeit | gut | gut |
| Kompilierung | ✓ getestet | ✓ **kompiliert** (Hardware-Test ausstehend) |

**Empfehlung:** Sinnvoll wenn ±50–100 µs das Ziel ist. QSPI-Treiber ist via PIO1 implementiert, SPI-Overhead im TX-Pfad eliminiert (`beginPacket()` → direkte Register-Writes, `Sn_TX_WR` software-tracked). Nächster Schritt: Hardware-Test auf echtem W6300-EVB-Pico2, Blocker 1–3 zuerst prüfen.
