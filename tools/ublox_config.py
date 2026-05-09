#!/usr/bin/env python3
"""
ublox_config.py — u-blox GPS Konfiguration für stationären NTP-Betrieb

Erkennt den angeschlossenen u-blox Chip automatisch, konfiguriert ihn und
schreibt danach gpsconfig.md mit den tatsächlich angewendeten Einstellungen.

Konfiguriert:
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
import os
import struct
import sys
import time
from datetime import datetime

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
# Konstanten
# ---------------------------------------------------------------------------

TARGET_BAUD = 115200
PROBE_BAUDS = [9600, 115200, 38400, 57600, 4800]

GNSS_NAMES = {0: "GPS", 1: "SBAS", 2: "Galileo", 3: "BeiDou",
              4: "IMES", 5: "QZSS",  6: "GLONASS"}

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MD_PATH    = os.path.join(SCRIPT_DIR, "gpsconfig.md")

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
    """Liest eine UBX-Nachricht; überspringt NMEA-Zeilen."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ser.timeout = min(deadline - time.monotonic(), 0.05)
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
        payload  = body[:length]
        checksum = body[length:]
        if bytes(checksum) != _checksum(bytes([cls, id_]) + hdr[2:4] + payload):
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
                return r_id == 0x01
    return False


def _send_ack(ser: serial.Serial, cls: int, id_: int,
              payload: bytes = b"", label: str = "") -> bool:
    _send(ser, cls, id_, payload)
    ok = _wait_ack(ser, cls, id_)
    print(f"  {'OK  ' if ok else 'FAIL'} {label}")
    return ok


def _poll(ser: serial.Serial, cls: int, id_: int,
          poll_payload: bytes = b"", timeout: float = 2.0):
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

def _probe_port(port: str, baud: int) -> bool:
    """Gibt True zurück wenn auf diesem Port/Baud ein u-blox antwortet."""
    try:
        with serial.Serial(port, baud, timeout=0.5) as ser:
            ser.reset_input_buffer()
            for _ in range(2):
                _send(ser, 0x0A, 0x04)
            msg = _read_ubx(ser, timeout=2.0)
            return msg is not None and msg[0] == 0x0A and msg[1] == 0x04
    except (serial.SerialException, OSError):
        return False


def find_ublox() -> tuple:
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        return None, None
    print(f"Verfügbare Ports: {', '.join(ports)}")
    for port in ports:
        for baud in PROBE_BAUDS:
            if _probe_port(port, baud):
                print(f"u-blox gefunden auf {port} @ {baud} Baud")
                return port, baud
    return None, None


def find_baud_on_port(port: str) -> int:
    """Findet die aktuelle Baudrate eines bekannten Ports. Gibt 0 zurück wenn nicht gefunden."""
    for baud in PROBE_BAUDS:
        if _probe_port(port, baud):
            print(f"u-blox antwortet auf {port} @ {baud} Baud")
            return baud
    return 0

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
# Konfigurationsschritte — geben jeweils ein dict mit den Ist-Werten zurück
# ---------------------------------------------------------------------------

def cfg_nav5_stationary(ser: serial.Serial) -> dict:
    """Stationärer Modus. Gibt {"dynModel", "fixMode", "ok"} zurück."""
    payload = _poll(ser, 0x06, 0x24)
    p = bytearray(payload) if payload and len(payload) >= 36 else bytearray(36)
    struct.pack_into("<H", p, 0, 0x0005)
    p[2] = 1
    p[3] = 3
    ok = _send_ack(ser, 0x06, 0x24, bytes(p),
                   "Stationärer Modus (dynModel=1, fixMode=3)")
    return {"dynModel": 1, "fixMode": 3, "ok": ok}


def cfg_gnss_all(ser: serial.Serial) -> dict:
    """Alle GNSS-Systeme aktivieren. Gibt {"systems": [...], "ok"} zurück."""
    payload = _poll(ser, 0x06, 0x3E)
    if not payload or len(payload) < 4:
        print("  SKIP GNSS-Konfiguration nicht verfügbar (NEO-6/7)")
        return {"systems": [], "ok": False, "skipped": True}

    p = bytearray(payload)
    num_blocks = p[3]
    p[1] = 0xFF

    systems = []
    for i in range(num_blocks):
        off = 4 + i * 8
        if off + 8 > len(p):
            break
        gnss_id = p[off]
        max_trk = p[off + 2]
        flags   = struct.unpack_from("<I", p, off + 4)[0]
        flags  |= 0x01
        struct.pack_into("<I", p, off + 4, flags)
        name = GNSS_NAMES.get(gnss_id, f"GNSS-{gnss_id}")
        print(f"       {name}: aktiviert, max {max_trk} Kanäle")
        systems.append({"name": name, "max_trk": max_trk, "gnss_id": gnss_id})

    ok = _send_ack(ser, 0x06, 0x3E, bytes(p),
                   "GNSS Multi-System + Kanal-Limit aufgehoben")
    return {"systems": systems, "ok": ok, "skipped": False}


def cfg_timepulse(ser: serial.Serial) -> dict:
    """PPS konfigurieren. Gibt die tatsächlichen Werte als dict zurück."""
    freq_hz    = 1
    pulse_us   = 100_000
    cable_ns   = 0
    user_ns    = 0
    flags = (
        (1 << 0) |  # active
        (1 << 1) |  # lockGnssFreq
        (1 << 2) |  # lockedOtherSet
        (1 << 3) |  # isFreq
        (1 << 4) |  # isLength
        (1 << 5) |  # alignToTow
        (1 << 6)    # polarity: steigende Flanke = Sekundenanfang
    )
    payload = (
        struct.pack("<BB",  0, 0)              +
        b"\x00\x00"                            +
        struct.pack("<hh",  cable_ns, 0)       +
        struct.pack("<II",  freq_hz, freq_hz)  +
        struct.pack("<II",  pulse_us, pulse_us)+
        struct.pack("<i",   user_ns)           +
        struct.pack("<I",   flags)
    )
    assert len(payload) == 32
    ok = _send_ack(ser, 0x06, 0x31, payload,
                   f"PPS: {freq_hz} Hz, UTC-ausgerichtet, {pulse_us//1000} ms Puls")
    return {"freq_hz": freq_hz, "pulse_us": pulse_us,
            "cable_ns": cable_ns, "user_ns": user_ns,
            "flags": flags, "ok": ok}


def cfg_baud(ser: serial.Serial, new_baud: int) -> bool:
    payload = _poll(ser, 0x06, 0x00, poll_payload=b"\x01", timeout=1.0)
    if not payload or len(payload) < 20:
        payload = struct.pack("<BBHIIHHHxx",
            0x01, 0x00, 0x0000, 0x000008D0, 9600,
            0x0003, 0x0003, 0x0000)
    p = bytearray(payload)
    struct.pack_into("<I", p, 8, new_baud)
    _send(ser, 0x06, 0x00, bytes(p))
    _wait_ack(ser, 0x06, 0x00, timeout=0.3)
    print(f"  OK   Baudrate → {new_baud} Baud")
    return True


def cfg_save(ser: serial.Serial) -> bool:
    payload = struct.pack("<IIIB",
        0x00000000, 0x0000FFFF, 0x00000000, 0x17)
    return _send_ack(ser, 0x06, 0x09, payload,
                     "Konfiguration gespeichert (BBR + Flash)")

# ---------------------------------------------------------------------------
# gpsconfig.md generieren
# ---------------------------------------------------------------------------

def write_gpsconfig_md(log: dict) -> None:
    chip    = log["chip"]
    nav5    = log["nav5"]
    gnss    = log["gnss"]
    tp      = log["timepulse"]
    saved   = log["saved"]
    ts      = log["timestamp"]

    dyn_names = {1: "Stationary", 2: "Pedestrian", 3: "Automotive",
                 0: "Portable",   4: "Sea"}
    fix_names = {1: "2D only", 2: "3D only", 3: "Auto 2D/3D"}

    flag_bits = [
        (0, "active",         "Timepuls aktiv"),
        (1, "lockGnssFreq",   "GNSS-Frequenz verwenden"),
        (2, "lockedOtherSet", "Lock-Parametersatz aktiv wenn gesynct"),
        (3, "isFreq",         "freqPeriod in Hz (nicht µs)"),
        (4, "isLength",       "pulseLenRatio in µs (nicht Duty-Cycle)"),
        (5, "alignToTow",     "Puls am Time-of-Week ausrichten"),
        (6, "polarity",       "Steigende Flanke = Sekundenanfang"),
    ]

    lines = []
    w = lines.append

    w(f"# gpsconfig.md — Letzte GPS-Konfiguration")
    w(f"")
    w(f"> Automatisch generiert von `ublox_config.py` am {ts}  ")
    w(f"> **Nicht manuell bearbeiten** — wird beim nächsten Script-Lauf überschrieben.")
    w(f"")

    # Chip
    w(f"## Chip")
    w(f"")
    w(f"| Parameter | Wert |")
    w(f"|---|---|")
    w(f"| SW Version | `{chip.get('sw', 'unbekannt')}` |")
    w(f"| HW Version | `{chip.get('hw', 'unbekannt')}` |")
    for e in chip.get("ext", []):
        w(f"| Extension  | `{e}` |")
    w(f"| Generation | NEO-{chip.get('gen', '?')}x |")
    w(f"| Port       | `{log['port']}` |")
    w(f"| Baudrate   | {log['baud_final']} Baud |")
    w(f"")

    # NAV5
    status = "OK" if nav5["ok"] else "FEHLER"
    w(f"## Stationärer Modus — `UBX-CFG-NAV5` ({status})")
    w(f"")
    w(f"| Parameter | Wert | Bedeutung |")
    w(f"|---|---|---|")
    w(f"| `dynModel` | {nav5['dynModel']} | {dyn_names.get(nav5['dynModel'], '?')} |")
    w(f"| `fixMode`  | {nav5['fixMode']} | {fix_names.get(nav5['fixMode'], '?')} |")
    w(f"")

    # GNSS
    if gnss.get("skipped"):
        w(f"## GNSS Multi-System — `UBX-CFG-GNSS` (übersprungen, NEO-6/7)")
        w(f"")
        w(f"Nicht verfügbar auf dieser Chip-Generation.")
    else:
        status = "OK" if gnss["ok"] else "FEHLER"
        w(f"## GNSS Multi-System — `UBX-CFG-GNSS` ({status})")
        w(f"")
        w(f"| System | Max Kanäle | Aktiviert |")
        w(f"|---|---|---|")
        for s in gnss["systems"]:
            w(f"| {s['name']} | {s['max_trk']} | ja |")
        w(f"")
        w(f"`numTrkChUse` = 0xFF (alle verfügbaren Hardware-Kanäle)")
    w(f"")

    # Timepulse
    status = "OK" if tp["ok"] else "FEHLER"
    w(f"## PPS-Timepuls — `UBX-CFG-TP5` ({status})")
    w(f"")
    w(f"| Parameter | Wert | Bedeutung |")
    w(f"|---|---|---|")
    w(f"| Frequenz         | {tp['freq_hz']} Hz | Ein Puls pro Sekunde |")
    w(f"| Pulsbreite       | {tp['pulse_us'] // 1000} ms | Länge des HIGH-Signals |")
    w(f"| Ausrichtung      | UTC | Puls zur UTC-Sekunde ausgerichtet |")
    w(f"| Aktivierung      | Nur wenn gelockt | Kein Puls ohne GNSS-Fix |")
    w(f"| `antCableDelay`  | {tp['cable_ns']} ns | Antennenkabelverzögerung |")
    w(f"| `userConfigDelay`| {tp['user_ns']} ns | Zusätzlicher Software-Delay |")
    w(f"| `flags`          | 0x{tp['flags']:04X} | Siehe Tabelle unten |")
    w(f"")
    w(f"| Bit | Name | Gesetzt | Bedeutung |")
    w(f"|---|---|---|---|")
    for bit, name, desc in flag_bits:
        val = "ja" if (tp["flags"] >> bit) & 1 else "nein"
        w(f"| {bit} | `{name}` | {val} | {desc} |")
    w(f"")

    # Baudrate
    w(f"## Baudrate — `UBX-CFG-PRT`")
    w(f"")
    w(f"| Parameter | Wert |")
    w(f"|---|---|")
    w(f"| Baudrate vorher | {log['baud_initial']} Baud |")
    w(f"| Baudrate nachher | {log['baud_final']} Baud |")
    w(f"| Protokoll In | UBX + NMEA |")
    w(f"| Protokoll Out | UBX + NMEA |")
    w(f"")
    w(f"Arduino-Sketch anpassen:")
    w(f"```cpp")
    w(f"GPS_Serial.begin({log['baud_final']}, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);")
    w(f"```")
    w(f"")

    # Speichern
    w(f"## Gespeichert — `UBX-CFG-CFG`")
    w(f"")
    if saved:
        w(f"Konfiguration dauerhaft gespeichert in BBR + Flash + EEPROM.")
    else:
        w(f"**Nicht gespeichert** (`--no-save` gesetzt). "
          f"Einstellungen gehen beim Neustart verloren.")
    w(f"")

    with open(MD_PATH, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"\n  gpsconfig.md aktualisiert: {MD_PATH}")

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
        port = args.port
        if args.baud:
            current_baud = args.baud
            print(f"Port: {port} @ {current_baud} Baud")
        else:
            print(f"Suche Baudrate auf {port}...")
            current_baud = find_baud_on_port(port)
            if current_baud == 0:
                print(f"FEHLER: Kein u-blox Modul auf {port} gefunden "
                      f"(getestet: {PROBE_BAUDS}).")
                sys.exit(1)
    else:
        print("Suche u-blox GPS Modul...")
        port, current_baud = find_ublox()
        if port is None:
            print("FEHLER: Kein u-blox Modul gefunden.")
            sys.exit(1)

    log = {
        "timestamp":    datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "port":         port,
        "baud_initial": current_baud,
        "baud_final":   current_baud,
        "chip":         {},
        "nav5":         {},
        "gnss":         {},
        "timepulse":    {},
        "saved":        False,
    }

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
        log["chip"] = {**ver, "gen": gen}
    else:
        print("  WARNUNG: Chip-Version nicht auslesbar")
        gen = 6
        log["chip"] = {"gen": gen}

    # --- Konfiguration ---
    print("\n── Konfiguration ───────────────────────────────")
    log["nav5"] = cfg_nav5_stationary(ser)

    if gen >= 8:
        log["gnss"] = cfg_gnss_all(ser)
    else:
        print("  SKIP GNSS Multi-System (nur M8+)")
        log["gnss"] = {"systems": [], "ok": False, "skipped": True}

    log["timepulse"] = cfg_timepulse(ser)

    # --- Baudrate ändern ---
    if current_baud != TARGET_BAUD:
        print(f"\n── Baudrate {current_baud} → {TARGET_BAUD} ──────────────────────")
        cfg_baud(ser, TARGET_BAUD)
        time.sleep(0.3)
        ser.close()
        ser = serial.Serial(port, TARGET_BAUD, timeout=1.0)
        ser.reset_input_buffer()
        time.sleep(0.3)
        ver2 = get_version(ser)
        if ver2:
            print(f"  OK   Verbindung bei {TARGET_BAUD} Baud bestätigt")
            log["baud_final"] = TARGET_BAUD
        else:
            print(f"  WARNUNG: Keine Antwort bei {TARGET_BAUD} Baud")
    else:
        print(f"\n  Baudrate bereits {TARGET_BAUD} Baud")
        log["baud_final"] = TARGET_BAUD

    # --- Speichern ---
    if not args.no_save:
        print("\n── Speichern ───────────────────────────────────")
        log["saved"] = cfg_save(ser)

    ser.close()

    # --- gpsconfig.md schreiben ---
    print("\n── Dokumentation ───────────────────────────────")
    write_gpsconfig_md(log)

    print("\n── Fertig ──────────────────────────────────────")
    print("GPS-Modul für NTP-Betrieb konfiguriert.")


if __name__ == "__main__":
    main()
