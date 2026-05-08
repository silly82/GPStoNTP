# Konzept: W6300-EVB-Pico2 als NTP-Stratum-1-Server

> Machbarkeitsstudie — noch nicht implementiert.  
> Board: [WIZnet W6300-EVB-Pico2](https://docs.wiznet.io/Product/Chip/Ethernet/W6300/w6300-evb-pico2)  
> Verglichen mit: aktuellem Stand (Waveshare ESP32-S3-ETH, `fast`-Branch)

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

PIO-Programm für PPS-Capture (konzeptionell):

```asm
; pps_capture.pio
.program pps_capture
    wait 0 gpio PPS_PIN   ; warte auf LOW (Entprellung)
    wait 1 gpio PPS_PIN   ; steigende Flanke
    irq set 0             ; IRQ 0 auslösen → CPU liest Timer
```

Der IRQ-Handler auf Core 0 liest sofort `timer_hw->timerawl` (untere 32 Bit, keine Latch-Latenz nötig für µs-Timer):

```cpp
void __isr ppsIRQ() {
  pio_interrupt_clear(pio0, 0);
  lastPPSus = time_us_64();   // 64-bit µs, atomisch auf RP2350
  ppsCount++;
}
```

Gleiches Prinzip für W6300-INT → T2-Timestamp.

### 2. QSPI statt SPI — T3-Systematik drastisch reduziert

**Problem heute:** T3 wird vor `endPacket()` gestempelt. Die SPI-Übertragung des UDP-Pakets dauert ~50–150 µs (W5500, 1-bit SPI, ~14 MHz). T3 ist damit systematisch zu früh.

**Mit QSPI (4-bit, bis 50 MHz):**
- 4× mehr Bits pro Takt: gleiche Datenmenge in 1/4 der Zeit
- Höherer Takt möglich (W6300 QSPI bis 50 MHz, RP2350 SPI bis 62.5 MHz)
- 48 Bytes NTP-Reply + IP/UDP-Header (~120 Bytes total): ~120×8 / (4×50M) ≈ **5 µs** statt ~100 µs

T3-Fehler sinkt von **~75 µs systematisch** auf **~3–5 µs** — über 15× besser.

---

## Fehlerbudget-Vergleich (konservativ, 12 Sats)

| Fehlerquelle | ESP32-S3 + W5500 (`fast`) | RP2350 + W6300 (Konzept) |
|---|---|---|
| GPS/PPS (NEO-M8N) | ±200 ns | ±200 ns |
| PPS-ISR-Jitter | ±5 µs | **±150 ns** (PIO + M33 IRQ) |
| T2-INT-ISR | ±10 µs | **±150 ns** (PIO) |
| T3 vor SPI-Send | ~75 µs syst. | **~3 µs** (QSPI 50 MHz) |
| Netzwerk-Jitter | ±50–150 µs | ±50–150 µs |
| Client T4 (chrony) | ±10–50 µs | ±10–50 µs |
| **Gesamt (konservativ)** | **±200–400 µs** | **±50–100 µs** |

Limitierender Faktor wird das Netzwerk und der NTP-Client — nicht mehr die Hardware.

---

## Architektur

```
GPS-Modul
  │  NMEA (UART) ─────────────► Core 1: TinyGPS++ → ntpEpochAtLastPPS
  │
  └─ PPS (RISING) ─► PIO-SM0 ─► IRQ ─► Core 0: lastPPSus = time_us_64()

W6300 INTn (FALLING) ─► PIO-SM1 ─► IRQ ─► Core 0: rxTimerUs = time_us_64()
                                                    │
                                           Core 0: handleNTPRequest()
                                             QSPI → W6300 → Ethernet
```

### Core-Aufteilung

| Core | Aufgabe |
|---|---|
| Core 0 | NTP-Handler, PIO-IRQs (PPS + INT), QSPI-Treiber |
| Core 1 | GPS-Parsing (TinyGPS++), MQTT-Status |

### Dualcore-Sync

`ntpEpochAtLastPPS` als `std::atomic<uint32_t>` mit `store(release)` / `load(acquire)` — identisch zum `fast`-Branch.

---

## Benötigte Bibliotheken

| Bibliothek | Quelle | Status |
|---|---|---|
| ioLibrary_Driver (W6300) | [WIZnet GitHub](https://github.com/Wiznet/ioLibrary_Driver) | verfügbar, C |
| arduino-pico (RP2350 Core) | Earle Philhower | stabil, aktiv |
| TinyGPS++ | Arduino Library Manager | portabel |
| PubSubClient | Arduino Library Manager | portabel |
| PIO-Assembler | pioasm (SDK) | im SDK enthalten |

**Hinweis:** Die WIZnet ioLibrary benötigt einen SPI-HAL-Wrapper für den RP2350 QSPI. WIZnet bietet RP2040-Beispiele auf GitHub (`W6300-EVB-Pico2` Repo) — diese müssen für Arduino-Pico adaptiert werden.

---

## Offene Punkte vor Implementierung

1. **ioLibrary QSPI-HAL für Arduino-Pico**: WIZnet SDK-Beispiele existieren für den Pico SDK. Arduino-Pico-Wrapper muss geprüft/geschrieben werden.
2. **PIO-IRQ-Konflikt**: Beide PIO-State-Machines (PPS + INT) müssen auf unterschiedliche IRQ-Nummern gemappt werden.
3. **`time_us_64()` thread-safety auf RP2350**: Pico-SDK garantiert atomic 64-bit read via TIMELR-Latch. Verifizieren ob unter Arduino-Pico verfügbar.
4. **GPS-UART-Pin**: Freie UART-Pins auf dem W6300-EVB-Pico2 identifizieren (GPIO0/1 oder GPIO4/5).
5. **T3 nach QSPI-Send stempeln**: Mit W6300 `SEND_OK`-Interrupt (Sn_IR Bit 4) wäre T3 nach der Übertragung präzise fassbar — ~1 µs Restfehler.

---

## Fazit

| Kriterium | ESP32-S3 + W5500 | RP2350 + W6300 |
|---|---|---|
| Erreichbare NTP-Genauigkeit | ±200–400 µs | **±50–100 µs** |
| Aufwand Migration | — | mittel (neue Plattform, PIO) |
| Bibliotheksreife | hoch | mittel (W6300-HAL muss angepasst werden) |
| Kosten/Verfügbarkeit | gut | gut |

**Empfehlung:** Sinnvoll wenn ±50–100 µs das Ziel ist. Die GPS/PPS-Quelle ist in beiden Fällen identisch — der Gewinn kommt rein aus besserer Timestamp-Hardware (PIO) und schnellerem SPI (QSPI). Migration ist machbar aber nicht trivial wegen des W6300-HALs.
