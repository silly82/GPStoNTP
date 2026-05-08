/*
 * W6300NTP — Stratum-1 NTP Server auf RP2350 + W6300
 *
 * Hardware:
 *   - WIZnet W6300-EVB-Pico2 (RP2350 + W6300 onboard, QSPI)
 *   - GPS-Modul mit NMEA-Ausgang (UART, 115200 Baud)
 *   - PPS-Ausgang des GPS-Moduls
 *
 * Verbesserungen gegenüber ESP32-S3 + W5500:
 *   - PPS-Timestamp per PIO-IRQ: ~80–150 ns Latenz (statt 1–5 µs)
 *   - T2-Timestamp per PIO-IRQ: ~80–150 ns Latenz (statt 1–5 µs)
 *   - W6300 QSPI: T3-Unsicherheit ~3–5 µs (statt ~75 µs)
 *   - RP2350 Dual-Core: Core 0 = NTP, Core 1 = GPS + MQTT
 *
 * Board:      W6300-EVB-Pico2 (Arduino-Pico Core, Earle Philhower)
 * Abhängigkeiten (Arduino Library Manager):
 *   - Ethernet_Generic   (Khoi Hoang, unterstützt W5500 + W6300)
 *   - TinyGPS++          (Mikal Hart)
 *   - PubSubClient       (Nick O'Leary)
 *
 * HINWEIS QSPI: Der W6300 wird hier im Standard-SPI-Modus betrieben
 * (4-wire, GPIO 17–19). Ein PIO-QSPI-Treiber würde T3 von ~5 µs auf
 * ~1 µs verbessern, ist aber noch nicht implementiert (Phase 2).
 */

#include <SPI.h>
#include <Ethernet_Generic.h>
#include <EthernetUdp.h>
#include <TinyGPS++.h>
#include <PubSubClient.h>
#include <atomic>
#include <time.h>

// Pico SDK
#include <hardware/pio.h>
#include <hardware/irq.h>
#include <pico/time.h>

// Generierte PIO-Header (automatisch durch Arduino-Pico-Build aus .pio)
#include "pps_capture.pio.h"
#include "eth_int_capture.pio.h"

#include "config.h"

/* --- HARDWARE MAPPING (W6300-EVB-Pico2) --- */
#define ETH_MISO_PIN  19  // W6300 IO1
#define ETH_MOSI_PIN  18  // W6300 IO0
#define ETH_SCLK_PIN  17
#define ETH_CS_PIN    16
#define ETH_RST_PIN   22
#define ETH_INT_PIN   15  // W6300 INTn (onboard)
#define GPS_TX_PIN    0   // RP2350 UART0 TX → GPS RX
#define GPS_RX_PIN    1   // RP2350 UART0 RX ← GPS TX
#define PPS_PIN       2   // GPS PPS

/* --- NETZWERK-KONFIGURATION (aus config.h) --- */
byte mac[]       = CFG_MAC;
IPAddress ip     (CFG_IP);
IPAddress gateway(CFG_GATEWAY);
IPAddress subnet (CFG_SUBNET);
IPAddress dnsServer(CFG_DNS);

/* --- MQTT-KONFIGURATION (aus config.h) --- */
const char* mqtt_server = CFG_MQTT_SERVER;
const char* mqtt_user   = CFG_MQTT_USER;
const char* mqtt_pass   = CFG_MQTT_PASS;
const char* mqtt_topic  = CFG_MQTT_TOPIC;

/* --- GLOBALE OBJEKTE --- */
EthernetUDP    ntpUDP;
EthernetClient ethClient;
PubSubClient   mqttClient(ethClient);
TinyGPSPlus    gps;          // nur Core 1

/* PPS — von PIO-IRQ auf Core 0 beschrieben */
volatile uint64_t lastPPSus = 0; // 64-bit, time_us_64() — auf RP2350 atomar lesbar
volatile uint32_t ppsCount  = 0;

/* W6300 INT → T2-Stempel, von PIO-IRQ auf Core 0 beschrieben */
volatile uint64_t rxTimerUs = 0;

/* NTP-Zustand — atomic: Core 1 schreibt, Core 0 liest */
std::atomic<uint32_t> ntpEpochAtLastPPS{0};
uint32_t ntpRequestsServed = 0;

/* GPS-Statuscache für MQTT — Core 1 schreibt, Core 0 liest */
volatile uint8_t gpsSats = 0;
volatile float   gpsHdop = 99.0f;

/* ------------------------------------------------------------------ */
/*  W6300 Direktzugriff für INT-Quittierung (Standard-SPI-Modus)      */
/*  bsb: 0 = Common, 1 = Socket-0-Register                            */
/* ------------------------------------------------------------------ */
static void w6300Write(uint16_t addr, uint8_t bsb, uint8_t data) {
  uint8_t cb = (bsb << 3) | 0x04;
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(ETH_CS_PIN, LOW);
  SPI.transfer(addr >> 8);
  SPI.transfer(addr & 0xFF);
  SPI.transfer(cb);
  SPI.transfer(data);
  digitalWrite(ETH_CS_PIN, HIGH);
  SPI.endTransaction();
}

static void w6300ClearRecvInt() {
  w6300Write(0x0002, 1, 0x04); // Sn_IR: RECV-Bit löschen (W1C)
}

/* ------------------------------------------------------------------ */
/*  PIO-ISRs auf Core 0                                               */
/* ------------------------------------------------------------------ */
void __isr __time_critical_func(ppsISR)() {
  pio_interrupt_clear(pio0, 0);
  lastPPSus = time_us_64();
  ppsCount++;
}

void __isr __time_critical_func(ethIntISR)() {
  pio_interrupt_clear(pio0, 1);
  rxTimerUs = time_us_64();
}

/* ------------------------------------------------------------------ */
/*  PIO initialisieren (Core 0, in setup())                           */
/* ------------------------------------------------------------------ */
static void setupPIO() {
  // SM 0: PPS rising-edge
  uint offsetPPS = pio_add_program(pio0, &pps_capture_program);
  pps_capture_program_init(pio0, 0, offsetPPS, PPS_PIN);
  pio_set_irq0_source_enabled(pio0, pis_interrupt0, true);
  irq_set_exclusive_handler(PIO0_IRQ_0, ppsISR);
  irq_set_enabled(PIO0_IRQ_0, true);

  // SM 1: W6300 INTn falling-edge
  uint offsetINT = pio_add_program(pio0, &eth_int_capture_program);
  eth_int_capture_program_init(pio0, 1, offsetINT, ETH_INT_PIN);
  // IRQ 1 des PIO wird auf denselben CPU-IRQ-Eingang geleitet wie IRQ 0,
  // aber über pis_interrupt1 → eigener Handler über PIO0_IRQ_1
  pio_set_irq1_source_enabled(pio0, pis_interrupt1, true);
  irq_set_exclusive_handler(PIO0_IRQ_1, ethIntISR);
  irq_set_enabled(PIO0_IRQ_1, true);
}

/* ------------------------------------------------------------------ */
/*  setup — Core 0 (NTP)                                              */
/* ------------------------------------------------------------------ */
void setup() {
  Serial.begin(115200);

  // W6300 Hardware-Reset
  pinMode(ETH_RST_PIN, OUTPUT);
  digitalWrite(ETH_RST_PIN, LOW);  delay(200);
  digitalWrite(ETH_RST_PIN, HIGH); delay(500);

  // SPI initialisieren (Standard 4-wire, 50 MHz)
  SPI.setRX(ETH_MISO_PIN);
  SPI.setTX(ETH_MOSI_PIN);
  SPI.setSCK(ETH_SCLK_PIN);
  SPI.setCS(ETH_CS_PIN);
  SPI.begin();

  Ethernet.init(ETH_CS_PIN);
  Ethernet.begin(mac, ip, dnsServer, gateway, subnet);
  ntpUDP.begin(123);

  // W6300 Socket-0 RECV-Interrupt nach ntpUDP.begin()
  w6300Write(0x002C, 1, 0x04); // Sn_IMR: RECV-Bit
  w6300Write(0x0016, 0, 0x01); // IMR: Socket-0 freigeben
  w6300ClearRecvInt();

  // MQTT
  mqttClient.setServer(mqtt_server, 1883);

  // PIO State Machines + ISRs
  setupPIO();

  Serial.println("W6300NTP bereit (RP2350 + W6300, PIO-Timestamps).");
}

/* ------------------------------------------------------------------ */
/*  loop — Core 0, nur NTP + MQTT                                     */
/* ------------------------------------------------------------------ */
void loop() {
  int packetSize = ntpUDP.parsePacket();
  if (packetSize >= 48) {
    handleNTPRequest();
    w6300ClearRecvInt();
    ntpRequestsServed++;
  }

  static uint32_t lastMQTT = 0;
  if (millis() - lastMQTT > 30000) {
    sendMQTTStatus();
    lastMQTT = millis();
  }
  mqttClient.loop();
}

/* ------------------------------------------------------------------ */
/*  setup1 + loop1 — Core 1 (GPS + MQTT-Daten)                       */
/* ------------------------------------------------------------------ */
void setup1() {
  Serial1.setTX(GPS_TX_PIN);
  Serial1.setRX(GPS_RX_PIN);
  Serial1.begin(115200);
}

void loop1() {
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }
  if (gps.time.isUpdated() && gps.date.isValid()) {
    struct tm t;
    t.tm_year  = gps.date.year() - 1900;
    t.tm_mon   = gps.date.month() - 1;
    t.tm_mday  = gps.date.day();
    t.tm_hour  = gps.time.hour();
    t.tm_min   = gps.time.minute();
    t.tm_sec   = gps.time.second();
    t.tm_isdst = 0;
    ntpEpochAtLastPPS.store((uint32_t)mktime(&t) + 2208988800UL,
                            std::memory_order_release);
  }
  gpsSats = (uint8_t)gps.satellites.value();
  gpsHdop = gps.hdop.hdop();
  delay(1);
}

/* ------------------------------------------------------------------ */
/*  Hilfsfunktion: µs-seit-PPS → NTP-Seconds + Fraction              */
/*  lastPPSus ist 64-bit: auf RP2350 via TIMELR-Latch atomar lesbar,  */
/*  kein Interrupt-Disable nötig (M33 liest 64-bit in zwei Loads,     */
/*  aber PIO-ISR läuft auf demselben Core → präemptiv korrekt).       */
/* ------------------------------------------------------------------ */
static inline void usToNTP(uint64_t timerUs, uint32_t &sec, uint32_t &frac) {
  int64_t delta = (int64_t)(timerUs - lastPPSus);
  if (delta < 0 || delta >= 2000000) delta = 0;
  sec  = ntpEpochAtLastPPS.load(std::memory_order_acquire)
         + (uint32_t)(delta / 1000000);
  frac = (uint32_t)(((uint64_t)(delta % 1000000) * 4294967296ULL) / 1000000ULL);
}

/* ------------------------------------------------------------------ */
/*  NTP-Antwort senden (RFC 4330)                                     */
/* ------------------------------------------------------------------ */
void handleNTPRequest() {
  // T2: PIO-ISR gestempelt bei W6300-INTn-Flanke
  uint64_t t2 = rxTimerUs;

  byte packetBuffer[48];
  ntpUDP.read(packetBuffer, 48);

  uint32_t refSec, refFrac, rxSec, rxFrac, txSec, txFrac;

  refSec  = ntpEpochAtLastPPS.load(std::memory_order_acquire);
  refFrac = 0;

  usToNTP(t2, rxSec, rxFrac);

  // T3: nach reply-Aufbau, unmittelbar vor beginPacket()
  // Mit QSPI (Phase 2) käme T3 nach endPacket() via SEND_OK-IRQ → ~1 µs Fehler
  usToNTP(time_us_64(), txSec, txFrac);

  byte reply[48];
  memset(reply, 0, 48);

  reply[0] = 0b00100100; reply[1] = 1; reply[2] = 0; reply[3] = (byte)-15;
  memcpy(&reply[12], "GPS ", 4);

  reply[16] = (refSec  >> 24) & 0xFF; reply[17] = (refSec  >> 16) & 0xFF;
  reply[18] = (refSec  >>  8) & 0xFF; reply[19] =  refSec         & 0xFF;
  reply[20] = (refFrac >> 24) & 0xFF; reply[21] = (refFrac >> 16) & 0xFF;
  reply[22] = (refFrac >>  8) & 0xFF; reply[23] =  refFrac        & 0xFF;

  memcpy(&reply[24], &packetBuffer[40], 8);

  reply[32] = (rxSec  >> 24) & 0xFF; reply[33] = (rxSec  >> 16) & 0xFF;
  reply[34] = (rxSec  >>  8) & 0xFF; reply[35] =  rxSec         & 0xFF;
  reply[36] = (rxFrac >> 24) & 0xFF; reply[37] = (rxFrac >> 16) & 0xFF;
  reply[38] = (rxFrac >>  8) & 0xFF; reply[39] =  rxFrac        & 0xFF;

  reply[40] = (txSec  >> 24) & 0xFF; reply[41] = (txSec  >> 16) & 0xFF;
  reply[42] = (txSec  >>  8) & 0xFF; reply[43] =  txSec         & 0xFF;
  reply[44] = (txFrac >> 24) & 0xFF; reply[45] = (txFrac >> 16) & 0xFF;
  reply[46] = (txFrac >>  8) & 0xFF; reply[47] =  txFrac        & 0xFF;

  ntpUDP.beginPacket(ntpUDP.remoteIP(), ntpUDP.remotePort());
  ntpUDP.write(reply, 48);
  ntpUDP.endPacket();
}

/* ------------------------------------------------------------------ */
/*  MQTT-Status senden                                                 */
/* ------------------------------------------------------------------ */
void sendMQTTStatus() {
  if (!mqttClient.connected()) {
    mqttClient.connect("RP2350-NTP", mqtt_user, mqtt_pass);
  }
  if (mqttClient.connected()) {
    String payload = "{\"sats\":"   + String(gpsSats)           +
                     ",\"hdop\":"   + String(gpsHdop)           +
                     ",\"pps\":"    + String(ppsCount)          +
                     ",\"served\":" + String(ntpRequestsServed) + "}";
    mqttClient.publish(mqtt_topic, payload.c_str());
  }
}
