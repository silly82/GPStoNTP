#!/usr/bin/env python3
"""
ublox_config.py — u-blox GPS Konfiguration für stationären NTP-Betrieb

Erkennt den angeschlossenen u-blox Chip automatisch und konfiguriert:
  - Stationärer Modus (dynModel=1)
  - Alle verfügbaren GNSS-Systeme aktivieren (M8+ only)
  - Maximale Satelliten-Kanalzahl (Limit aufheben)
  - Baudrate auf 115200
  - PPS-Timepuls: 1 Hz, UTC-ausgerichtet, aktiv nur wenn gelockt
  - Konfiguration dauerhaft in Flash/BBR speichern

Abhängigkeiten:
  pip install pyserial

Verwendung:
  python ublox_config.py                  # Auto-Erkennung
  python ublox_config.py --port /dev/ttyUSB0
  python ublox_config.py --port COM3 --baud 115200
  python ublox_config.py --no-save        # Nicht in Flash schreiben
"""

import argparse
import struct
import sys
import time

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
# Konstanten
# ---------------------------------------------------------------------------

TARGET_BAUD  = 115200
PROBE_BAUDS  = [9600, 115200, 38400, 57600, 4800]

GNSS_NAMES = {0: "GPS", 1: "SBAS", 2: "Galileo", 3: "BeiDou",
              4: "IMES", 5: "QZSS", 6: "GLONASS"}

# ---------------------------------------------------------------------------
# UBX Protokoll-Hilfsfunktionen
# ---------------------------------------------------------------------------

def _checksum(data: bytes) -> bytes:
    a = b = 0
    for byte in data:
        a = (a + byte) & 0xFF
        b = (b + a) & 0xFF
    return bytes([a, b])


def _build(cls: int, id_: int, payload: bytes = b"") -> bytes:
    hdr = bytes([cls, id_]) + struct.pack("<H", len(payload)) + payload
    return b"\xb5\x62" + hdr + _checksum(hdr)


def _send(ser: serial.Serial, cls: int, id_: int, payload: bytes = b"") -> None:
    ser.write(_build(cls, id_, payload))
    ser.flush()


def _read_ubx(ser: serial.Serial, timeout: float = 2.0):
    """Liest eine UBX-Nachricht; überspringt NMEA-Zeilen. Gibt (cls, id, payload) zurück."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        ser.timeout = min(remaining, 0.05)

        b = ser.read(1)
        if not b or b[0] != 0xB5:
            continue
        b = ser.read(1)
        if not b or b[0] != 0x62:
            continue

        hdr = ser.read(4)
        if len(hdr) < 4:
            continue
        cls, id_ = hdr[0], hdr[1]
        length = struct.unpack("<H", hdr[2:4])[0]

        body = ser.read(length + 2)
        if len(body) < length + 2:
            continue

        payload   = body[:length]
        checksum  = body[length:]
        expected  = _checksum(bytes([cls, id_]) + hdr[2:4] + payload)
        if bytes(checksum) != expected:
            continue

        return cls, id_, bytes(payload)
    return None


def _wait_ack(ser: serial.Serial, cls: int, id_: int, timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = _read_ubx(ser, timeout=deadline - time.monotonic())
        if msg is None:
            break
        r_cls, r_id, r_payload = msg
        if r_cls == 0x05 and len(r_payload) >= 2:
            if r_payload[0] == cls and r_payload[1] == id_:
                return r_id == 0x01  # 0x01=ACK, 0x00=NAK
    return False


def _send_ack(ser: serial.Serial, cls: int, id_: int,
              payload: bytes = b"", label: str = "") -> bool:
    _send(ser, cls, id_, payload)
    ok = _wait_ack(ser, cls, id_)
    print(f"  {'OK  ' if ok else 'FAIL'} {label}")
    return ok


def _poll(ser: serial.Serial, cls: int, id_: int,
          poll_payload: bytes = b"", timeout: float = 2.0):
    """Sendet einen Poll-Request und wartet auf die Antwort gleicher Klasse/ID."""
    _send(ser, cls, id_, poll_payload)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = _read_ubx(ser, timeout=deadline - time.monotonic())
        if msg and msg[0] == cls and msg[1] == id_:
            return msg[2]
    return None

# ---------------------------------------------------------------------------
# Port-Erkennung
# ---------------------------------------------------------------------------

def find_ublox() -> tuple:
    """Scannt alle Ports; gibt (port, baud) des ersten u-blox Moduls zurück."""
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        return None, None
    print(f"Verfügbare Ports: {', '.join(ports)}")
    for port in ports:
        for baud in PROBE_BAUDS:
            try:
                with serial.Serial(port, baud, timeout=0.5) as ser:
                    ser.reset_input_buffer()
                    _send(ser, 0x0A, 0x04)           # MON-VER Poll
                    msg = _read_ubx(ser, timeout=1.2)
                    if msg and msg[0] == 0x0A and msg[1] == 0x04:
                        print(f"u-blox gefunden auf {port} @ {baud} Baud")
                        return port, baud
            except (serial.SerialException, OSError):
                pass
    return None, None

# ---------------------------------------------------------------------------
# Chip-Identifikation
# ---------------------------------------------------------------------------

def get_version(ser: serial.Serial) -> dict:
    payload = _poll(ser, 0x0A, 0x04)
    if not payload or len(payload) < 40:
        return {}
    sw  = payload[0:30].rstrip(b"\x00").decode("ascii", errors="replace")
    hw  = payload[30:40].rstrip(b"\x00").decode("ascii", errors="replace")
    ext = [payload[i:i+30].rstrip(b"\x00").decode("ascii", errors="replace")
           for i in range(40, len(payload), 30)]
    return {"sw": sw, "hw": hw, "ext": [e for e in ext if e]}


def detect_generation(ver: dict) -> int:
    hw = ver.get("hw", "")
    for marker, gen in [("00080000", 8), ("M8", 8), ("00070000", 7), ("M7", 7)]:
        if marker in hw:
            return gen
    return 6

# ---------------------------------------------------------------------------
# Konfigurationsschritte
# ---------------------------------------------------------------------------

def cfg_nav5_stationary(ser: serial.Serial) -> bool:
    """Stationärer Modus (dynModel=1, Auto-Fix 2D/3D)."""
    payload = _poll(ser, 0x06, 0x24)
    p = bytearray(payload) if payload and len(payload) >= 36 else bytearray(36)
    struct.pack_into("<H", p, 0, 0x0005)  # mask: dynModel + fixMode
    p[2] = 1   # dynModel = 1 (Stationary)
    p[3] = 3   # fixMode  = 3 (Auto 2D/3D)
    return _send_ack(ser, 0x06, 0x24, bytes(p),
                     "Stationärer Modus (dynModel=1, fixMode=3)")


def cfg_gnss_all(ser: serial.Serial) -> bool:
    """Alle GNSS-Systeme aktivieren, Kanal-Limit aufheben (M8+ only)."""
    payload = _poll(ser, 0x06, 0x3E)
    if not payload or len(payload) < 4:
        print("  SKIP GNSS-Konfiguration nicht verfügbar (NEO-6/7)")
        return False

    p = bytearray(payload)
    num_blocks = p[3]
    p[1] = 0xFF  # numTrkChUse = alle verfügbaren Hardware-Kanäle

    for i in range(num_blocks):
        off = 4 + i * 8
        if off + 8 > len(p):
            break
        gnss_id  = p[off]
        max_trk  = p[off + 2]
        flags    = struct.unpack_from("<I", p, off + 4)[0]
        flags   |= 0x01              # enable
        struct.pack_into("<I", p, off + 4, flags)
        name = GNSS_NAMES.get(gnss_id, f"GNSS-{gnss_id}")
        print(f"       {name}: aktiviert, max {max_trk} Kanäle")

    return _send_ack(ser, 0x06, 0x3E, bytes(p),
                     "GNSS Multi-System + Kanal-Limit aufgehoben")


def cfg_timepulse(ser: serial.Serial) -> bool:
    """PPS: 1 Hz, UTC-ausgerichtet, 100 ms Puls, nur aktiv wenn gelockt."""
    flags = (
        (1 << 0) |  # active
        (1 << 1) |  # lockGnssFreq
        (1 << 2) |  # lockedOtherSet (Lock-Werte wenn gesynct)
        (1 << 3) |  # isFreq (freqPeriod in Hz)
        (1 << 4) |  # isLength (pulseLenRatio in µs)
        (1 << 5) |  # alignToTow
        (1 << 6)    # polarity: steigende Flanke = Sekundenanfang
        # Bits 7-10 gridUtcGnss = 0 → UTC
    )
    payload = (
        struct.pack("<BB",  0, 0)        +  # tpIdx=0, version=0
        b"\x00\x00"                      +  # reserved
        struct.pack("<hh",  0, 0)        +  # antCableDelay, rfGroupDelay (ns)
        struct.pack("<II",  1, 1)        +  # freqPeriod, freqPeriodLock (Hz)
        struct.pack("<II",  100000, 100000) +  # pulseLenRatio, pulseLenRatioLock (µs)
        struct.pack("<i",   0)           +  # userConfigDelay (ns)
        struct.pack("<I",   flags)          # flags
    )
    assert len(payload) == 32
    return _send_ack(ser, 0x06, 0x31, payload,
                     "PPS: 1 Hz, UTC-ausgerichtet, 100 ms Puls")


def cfg_baud(ser: serial.Serial, new_baud: int) -> None:
    """Ändert die UART1-Baudrate. ACK kommt bei altem Baud, danach sofort neuer."""
    payload = _poll(ser, 0x06, 0x00, poll_payload=b"\x01", timeout=1.0)
    if not payload or len(payload) < 20:
        # Fallback: Standard 8N1, UBX+NMEA in/out
        payload = struct.pack("<BBHIIHHHxx",
            0x01, 0x00, 0x0000,
            0x000008D0,   # 8N1
            9600,
            0x0003,       # inProtoMask: UBX + NMEA
            0x0003,       # outProtoMask: UBX + NMEA
            0x0000)
    p = bytearray(payload)
    struct.pack_into("<I", p, 8, new_baud)
    _send(ser, 0x06, 0x00, bytes(p))
    # ACK kommt noch bei alter Baudrate
    _wait_ack(ser, 0x06, 0x00, timeout=0.3)
    print(f"  OK   Baudrate → {new_baud} Baud")


def cfg_save(ser: serial.Serial) -> bool:
    """Speichert alle Einstellungen in BBR und Flash."""
    payload = struct.pack("<IIIB",
        0x00000000,   # clearMask
        0x0000FFFF,   # saveMask (alles)
        0x00000000,   # loadMask
        0x17)         # deviceMask: BBR + Flash + EEPROM
    return _send_ack(ser, 0x06, 0x09, payload,
                     "Konfiguration gespeichert (BBR + Flash)")

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="u-blox GPS Konfiguration für stationären NTP-Betrieb")
    parser.add_argument("--port",    help="Serieller Port (z.B. /dev/ttyUSB0, COM3)")
    parser.add_argument("--baud",    type=int, default=0,
                        help="Aktuelle Baudrate (0 = auto-detect)")
    parser.add_argument("--no-save", action="store_true",
                        help="Nicht dauerhaft in Flash speichern")
    args = parser.parse_args()

    # --- Port ermitteln ---
    if args.port:
        port        = args.port
        current_baud = args.baud if args.baud else 9600
        print(f"Port: {port} @ {current_baud} Baud")
    else:
        print("Suche u-blox GPS Modul...")
        port, current_baud = find_ublox()
        if port is None:
            print("FEHLER: Kein u-blox Modul gefunden. "
                  "Port mit --port angeben oder Modul anschliessen.")
            sys.exit(1)

    ser = serial.Serial(port, current_baud, timeout=1.0)
    ser.reset_input_buffer()
    time.sleep(0.3)

    # --- Chip identifizieren ---
    print("\n── Chip-Erkennung ──────────────────────────────")
    ver = get_version(ser)
    if ver:
        print(f"  SW : {ver['sw']}")
        print(f"  HW : {ver['hw']}")
        for e in ver["ext"]:
            print(f"  EXT: {e}")
        gen = detect_generation(ver)
        print(f"  Generation: NEO-{gen}x")
    else:
        print("  WARNUNG: Chip-Version nicht auslesbar")
        gen = 6

    # --- Konfiguration ---
    print("\n── Konfiguration ───────────────────────────────")
    cfg_nav5_stationary(ser)

    if gen >= 8:
        cfg_gnss_all(ser)
    else:
        print("  SKIP GNSS Multi-System (nur M8+)")

    cfg_timepulse(ser)

    # --- Baudrate ändern ---
    if current_baud != TARGET_BAUD:
        print(f"\n── Baudrate {current_baud} → {TARGET_BAUD} ──────────────────────")
        cfg_baud(ser, TARGET_BAUD)
        time.sleep(0.3)
        ser.close()
        ser = serial.Serial(port, TARGET_BAUD, timeout=1.0)
        ser.reset_input_buffer()
        time.sleep(0.3)
        # Verbindung verifizieren
        ver2 = get_version(ser)
        if ver2:
            print(f"  OK   Verbindung bei {TARGET_BAUD} Baud bestätigt")
        else:
            print(f"  WARNUNG: Keine Antwort bei {TARGET_BAUD} Baud — "
                  "Baudrate im Modul ggf. bereits anders")
    else:
        print(f"\n  Baudrate bereits {TARGET_BAUD} Baud")

    # --- Speichern ---
    if not args.no_save:
        print("\n── Speichern ───────────────────────────────────")
        cfg_save(ser)

    ser.close()

    print("\n── Fertig ──────────────────────────────────────")
    print("GPS-Modul für NTP-Betrieb konfiguriert.")
    if current_baud != TARGET_BAUD:
        print(f"\nArduino-Sketch anpassen (config.h oder GPStoNTP.ino):")
        print(f"  GPS_Serial.begin({TARGET_BAUD}, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);")


if __name__ == "__main__":
    main()
