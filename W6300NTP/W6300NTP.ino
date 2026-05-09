/*
 * W6300NTP — Stratum-1 NTP Server auf RP2350 + W6300
 *
 * Hardware:
 *   - WIZnet W6300-EVB-Pico2 (RP2350 + W6300 onboard, QSPI)
 *   - GPS-Modul mit NMEA-Ausgang (UART, 115200 Baud)
 *   - PPS-Ausgang des GPS-Moduls
 *
 * Verbesserungen gegenüber ESP32-S3 + W5500 (fast-Branch):
 *   PIO-Timestamps   PPS + T2 per PIO-IRQ: ~80–150 ns Latenz (statt 1–5 µs)
 *   QSPI-TX          T3-Unsicherheit ~3 µs (statt ~75 µs, 25× besser)
 *   Dual-Core        Core 0 = NTP-Loop, Core 1 = GPS + MQTT (keine Jitter-Kopplung)
 *   std::atomic      Cross-Core-Epoch mit release/acquire-Ordering
 *
 * QSPI-TX-Pfad (handleNTPRequest):
 *   1. ntpUDP.beginPacket() setzt Ziel-IP/Port via SPI (Ethernet_Generic)
 *   2. w6300ReadReg16(Sn_TX_WR) liest TX-Schreibzeiger via SPI
 *   3. qspiWriteBuf() schreibt 48-Byte-NTP-Reply via PIO1-QSPI in TX-Buffer
 *   4. T3 = time_us_64() — direkt nach QSPI-Write, ~1 µs vor SEND
 *   5. w6300WriteReg16(Sn_TX_WR) aktualisiert Schreibzeiger via SPI
 *   6. w6300Write(Sn_CR = SEND) triggert Ethernet-TX
 *
 * GPIO-Switching: SPI-Peripheral (GPIO 17–19) ↔ PIO1 (GPIO 17–21)
 *   qspiTakeGPIOs()    : übergibt GPIO 17–21 an PIO1 für QSPI-Transaktion
 *   qspiReleaseGPIOs() : gibt GPIO 17–19 an SPI0 zurück, GPIO 20–21 als SIO
 *
 * Board:      W6300-EVB-Pico2 (Arduino-Pico Core, Earle Philhower)
 * Bibliotheken (Arduino Library Manager):
 *   Ethernet_Generic   (Khoi Hoang — W5500/W6300-Unterstützung)
 *   TinyGPS++          (Mikal Hart)
 *   PubSubClient       (Nick O'Leary)
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
#include <hardware/gpio.h>
#include <hardware/dma.h>
#include <pico/time.h>

// PIO-Header (aus .pio-Dateien, von pioasm generiert)
#include "pps_capture.pio.h"
#include "eth_int_capture.pio.h"
#include "w6300_qspi.pio.h"

#include "config.h"

/* --- HARDWARE MAPPING (W6300-EVB-Pico2) --- */
#define ETH_CS_PIN    16
#define ETH_SCLK_PIN  17
#define ETH_IO0_PIN   18  // MOSI (SPI) / IO0 (QSPI)
#define ETH_IO1_PIN   19  // MISO (SPI) / IO1 (QSPI)
#define ETH_IO2_PIN   20  // QSPI only
#define ETH_IO3_PIN   21  // QSPI only
#define ETH_RST_PIN   22
#define ETH_INT_PIN   15  // W6300 INTn (onboard, fest verdrahtet)
#define GPS_TX_PIN    0   // UART0 TX → GPS RX
#define GPS_RX_PIN    1   // UART0 RX ← GPS TX
#define PPS_PIN       2   // GPS PPS (steigende Flanke)

/* W6300 Socket-0 Register-Adressen (BSB=1) */
#define Sn_CR       0x0001  // Command Register
#define Sn_TX_WR    0x0022  // TX Write Pointer (16-bit)

/* W6300 QSPI-Opcodes (Flash-kompatibel) */
#define W6300_OP_WRITE      0x02  // Single-SPI Write (Command+Addr single, Daten single)
#define W6300_OP_QUAD_WRITE 0x32  // Quad-SPI Write  (Command+Addr single, Daten quad)

/* --- NETZWERK-KONFIGURATION (aus config.h) --- */
byte mac[]           = CFG_MAC;
IPAddress ip         (CFG_IP);
IPAddress gateway    (CFG_GATEWAY);
IPAddress subnet     (CFG_SUBNET);
IPAddress dnsServer  (CFG_DNS);

/* --- MQTT-KONFIGURATION (aus config.h) --- */
const char* mqtt_server = CFG_MQTT_SERVER;
const char* mqtt_user   = CFG_MQTT_USER;
const char* mqtt_pass   = CFG_MQTT_PASS;
const char* mqtt_topic  = CFG_MQTT_TOPIC;

/* --- GLOBALE OBJEKTE --- */
EthernetUDP    ntpUDP;
EthernetClient ethClient;
PubSubClient   mqttClient(ethClient);
TinyGPSPlus    gps;          // exklusiv Core 1

/* PPS — von PIO0-ISR auf Core 0 beschrieben */
volatile uint64_t lastPPSus = 0;
volatile uint32_t ppsCount  = 0;

/* W6300 INTn → T2-Stempel, von PIO0-ISR auf Core 0 beschrieben */
volatile uint64_t rxTimerUs = 0;

/* NTP-Epoch — Core 1 schreibt (release), Core 0 liest (acquire) */
std::atomic<uint32_t> ntpEpochAtLastPPS{0};
uint32_t ntpRequestsServed = 0;

/* GPS-Statuscache — Core 1 schreibt, Core 0 liest (MQTT) */
volatile uint8_t gpsSats = 0;
volatile float   gpsHdop = 99.0f;

/* PIO1 SM-Offsets (QSPI), nach qspiInit() gültig */
static uint qspiOffsetCmd   = 0;
static uint qspiOffsetWrite = 0;
static uint qspiOffsetRead  = 0;

/* DMA-Kanal + vorgepackter TX-Puffer (MSB-first 32-Bit-Wörter) */
static int      qspiDmaCh  = -1;
static uint32_t qspiDmaBuf[12];  // 48 Byte max = 12 Wörter

/* ================================================================== */
/*  W6300 SPI-Zugriff (Standard 4-wire SPI via Ethernet_Generic)      */
/*  Verwendet für Register-Zugriffe außerhalb des NTP-TX-Pfads.        */
/* ================================================================== */

static void w6300Write(uint16_t addr, uint8_t bsb, uint8_t data) {
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(ETH_CS_PIN, LOW);
  SPI.transfer(addr >> 8);
  SPI.transfer(addr & 0xFF);
  SPI.transfer((bsb << 3) | 0x04);
  SPI.transfer(data);
  digitalWrite(ETH_CS_PIN, HIGH);
  SPI.endTransaction();
}

static uint8_t w6300ReadReg8(uint16_t addr, uint8_t bsb) {
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(ETH_CS_PIN, LOW);
  SPI.transfer(addr >> 8);
  SPI.transfer(addr & 0xFF);
  SPI.transfer((bsb << 3) | 0x00);  // read
  uint8_t v = SPI.transfer(0);
  digitalWrite(ETH_CS_PIN, HIGH);
  SPI.endTransaction();
  return v;
}

static uint16_t w6300ReadReg16(uint16_t addr, uint8_t bsb) {
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(ETH_CS_PIN, LOW);
  SPI.transfer(addr >> 8);
  SPI.transfer(addr & 0xFF);
  SPI.transfer((bsb << 3) | 0x00);
  uint16_t v = ((uint16_t)SPI.transfer(0) << 8) | SPI.transfer(0);
  digitalWrite(ETH_CS_PIN, HIGH);
  SPI.endTransaction();
  return v;
}

static void w6300WriteReg16(uint16_t addr, uint8_t bsb, uint16_t val) {
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(ETH_CS_PIN, LOW);
  SPI.transfer(addr >> 8);
  SPI.transfer(addr & 0xFF);
  SPI.transfer((bsb << 3) | 0x04);
  SPI.transfer(val >> 8);
  SPI.transfer(val & 0xFF);
  digitalWrite(ETH_CS_PIN, HIGH);
  SPI.endTransaction();
}

static void w6300ClearRecvInt() {
  w6300Write(0x0002, 1, 0x04);  // Sn_IR: RECV-Bit löschen (W1C)
}

/* ================================================================== */
/*  QSPI-Treiber (PIO1)                                                */
/*                                                                      */
/*  Drei State Machines auf PIO1:                                       */
/*    SM0  spi_cmd    Single-SPI TX (Command + 3-Byte-Adresse)          */
/*    SM1  qspi_write Quad-SPI TX  (Datenbytes, 4-Bit pro SCLK)         */
/*    SM2  qspi_read  Quad-SPI RX  (Datenbytes, 4-Bit pro SCLK)         */
/*                                                                      */
/*  SCLK-Frequenz: 150 MHz / clkdiv / 2 Zyklen-pro-Bit                 */
/*    clkdiv=2.0 → 37.5 MHz QSPI (W6300-max ~50 MHz)                   */
/*                                                                      */
/*  GPIO-Sharing mit SPI-Peripheral:                                    */
/*    qspiTakeGPIOs()    übergibt GPIO17–21 an PIO1                     */
/*    qspiReleaseGPIOs() gibt GPIO17–19 an SPI0, GPIO20–21 als SIO      */
/* ================================================================== */

#define QSPI_CLKDIV  2.0f

static void qspiTakeGPIOs() {
  // GPIO17–21 → PIO1 (überschreibt SPI0-Funktionszuweisung)
  pio_gpio_init(pio1, ETH_SCLK_PIN);
  pio_gpio_init(pio1, ETH_IO0_PIN);
  pio_gpio_init(pio1, ETH_IO1_PIN);
  pio_gpio_init(pio1, ETH_IO2_PIN);
  pio_gpio_init(pio1, ETH_IO3_PIN);
}

static void qspiReleaseGPIOs() {
  // GPIO17–19 → SPI0 zurückgeben
  gpio_set_function(ETH_SCLK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(ETH_IO0_PIN,  GPIO_FUNC_SPI);
  gpio_set_function(ETH_IO1_PIN,  GPIO_FUNC_SPI);
  // GPIO20–21 nur für QSPI genutzt → als einfaches SIO idle-HIGH
  gpio_set_function(ETH_IO2_PIN,  GPIO_FUNC_SIO);
  gpio_set_function(ETH_IO3_PIN,  GPIO_FUNC_SIO);
  gpio_set_dir(ETH_IO2_PIN, GPIO_OUT); gpio_put(ETH_IO2_PIN, 1);
  gpio_set_dir(ETH_IO3_PIN, GPIO_OUT); gpio_put(ETH_IO3_PIN, 1);
}

static void qspiInit() {
  qspiOffsetCmd   = pio_add_program(pio1, &spi_cmd_program);
  qspiOffsetWrite = pio_add_program(pio1, &qspi_write_program);
  qspiOffsetRead  = pio_add_program(pio1, &qspi_read_program);
  gpio_init(ETH_IO2_PIN); gpio_set_dir(ETH_IO2_PIN, GPIO_OUT); gpio_put(ETH_IO2_PIN, 1);
  gpio_init(ETH_IO3_PIN); gpio_set_dir(ETH_IO3_PIN, GPIO_OUT); gpio_put(ETH_IO3_PIN, 1);
  qspiDmaCh = dma_claim_unused_channel(true);
}

/*
 * qspiWriteAsync — startet eine QSPI-Schreibtransaktion, kehrt nach DMA-Start zurück.
 *
 *   Phase 1 (cmd+addr, SM0): blockierend, nur 4 Bytes — Overhead vernachlässigbar.
 *   Phase 2 (daten, SM1):    DMA übergibt Bytes an PIO-FIFO; CPU frei während
 *                             ~2 µs Taktzeit (40 Byte bei 37.5 MHz QSPI).
 *
 * CS bleibt LOW; GPIOs gehören PIO1. Muss von qspiWriteWait() abgeschlossen werden.
 */
static void qspiWriteAsync(uint16_t bufAddr, uint8_t bsb,
                           const uint8_t *data, uint16_t len) {
  uint8_t addrH = bufAddr >> 8;
  uint8_t addrL = bufAddr & 0xFF;
  uint8_t ctrl  = (bsb << 3) | 0x04;

  // Daten als MSB-first 32-Bit-Wörter in DMA-Puffer packen
  uint16_t nw = (len + 3) >> 2;
  for (uint16_t i = 0; i < nw; i++) {
    uint32_t w = 0;
    for (int j = 0; j < 4 && (i * 4 + j) < len; j++)
      w |= (uint32_t)data[i * 4 + j] << (24 - j * 8);
    qspiDmaBuf[i] = w;
  }

  qspiTakeGPIOs();

  // Phase 1: opcode + Adresse (Single-SPI, SM0, blockierend)
  spi_cmd_init(pio1, 0, qspiOffsetCmd, ETH_IO0_PIN, ETH_SCLK_PIN, QSPI_CLKDIV);
  gpio_put(ETH_CS_PIN, 0);
  pio_sm_put_blocking(pio1, 0, (uint32_t)W6300_OP_QUAD_WRITE << 24);
  pio_sm_put_blocking(pio1, 0, (uint32_t)addrH << 24);
  pio_sm_put_blocking(pio1, 0, (uint32_t)addrL << 24);
  pio_sm_put_blocking(pio1, 0, (uint32_t)ctrl  << 24);
  while (!pio_sm_is_tx_fifo_empty(pio1, 0)) tight_loop_contents();
  pio_sm_exec(pio1, 0, pio_encode_nop() | pio_encode_sideset(1, 0));
  pio_sm_exec(pio1, 0, pio_encode_nop() | pio_encode_sideset(1, 0));
  pio_sm_set_enabled(pio1, 0, false);

  // Phase 2: Daten (Quad-SPI, SM1, DMA — nicht-blockierend)
  pio_sm_set_consecutive_pindirs(pio1, 1, ETH_IO0_PIN, 4, true);
  qspi_write_init(pio1, 1, qspiOffsetWrite, ETH_IO0_PIN, ETH_SCLK_PIN, QSPI_CLKDIV);

  dma_channel_config dc = dma_channel_get_default_config(qspiDmaCh);
  channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
  channel_config_set_read_increment(&dc, true);
  channel_config_set_write_increment(&dc, false);
  channel_config_set_dreq(&dc, pio_get_dreq(pio1, 1, true));
  dma_channel_configure(qspiDmaCh, &dc, &pio1->txf[1], qspiDmaBuf, nw, true);
  // DMA läuft; CPU kehrt zurück
}

/*
 * qspiWriteWait — wartet auf DMA-Abschluss, leert PIO-FIFO, hebt CS und gibt GPIOs frei.
 * Muss nach qspiWriteAsync() aufgerufen werden.
 */
static void qspiWriteWait() {
  dma_channel_wait_for_finish_blocking(qspiDmaCh);
  while (!pio_sm_is_tx_fifo_empty(pio1, 1)) tight_loop_contents();
  pio_sm_exec(pio1, 1, pio_encode_nop() | pio_encode_sideset(1, 0));
  pio_sm_exec(pio1, 1, pio_encode_nop() | pio_encode_sideset(1, 0));
  pio_sm_set_enabled(pio1, 1, false);
  gpio_put(ETH_CS_PIN, 1);
  qspiReleaseGPIOs();
}

/* qspiWriteBuf — synchrone Variante (async + wait); für kurze Tail-Writes. */
static void qspiWriteBuf(uint16_t bufAddr, uint8_t bsb,
                         const uint8_t *data, uint16_t len) {
  qspiWriteAsync(bufAddr, bsb, data, len);
  qspiWriteWait();
}

/* ================================================================== */
/*  PIO0-ISRs (Core 0): PPS + W6300-INT Timestamp                     */
/* ================================================================== */

void __isr __time_critical_func(ppsISR)() {
  pio_interrupt_clear(pio0, 0);
  lastPPSus = time_us_64();  // Cortex-M33 Latenz: ~80–150 ns
  ppsCount++;
}

void __isr __time_critical_func(ethIntISR)() {
  pio_interrupt_clear(pio0, 1);
  rxTimerUs = time_us_64();  // T2: W6300 INT → ISR → Timer-Read
}

static void setupPIO0() {
  uint offsetPPS = pio_add_program(pio0, &pps_capture_program);
  pps_capture_program_init(pio0, 0, offsetPPS, PPS_PIN);
  pio_set_irq0_source_enabled(pio0, pis_interrupt0, true);
  irq_set_exclusive_handler(PIO0_IRQ_0, ppsISR);
  irq_set_enabled(PIO0_IRQ_0, true);

  uint offsetINT = pio_add_program(pio0, &eth_int_capture_program);
  eth_int_capture_program_init(pio0, 1, offsetINT, ETH_INT_PIN);
  pio_set_irq1_source_enabled(pio0, pis_interrupt1, true);
  irq_set_exclusive_handler(PIO0_IRQ_1, ethIntISR);
  irq_set_enabled(PIO0_IRQ_1, true);
}

/* ================================================================== */
/*  setup / loop — Core 0 (NTP)                                       */
/* ================================================================== */

void setup() {
  Serial.begin(115200);

  // W6300 Hardware-Reset
  pinMode(ETH_RST_PIN, OUTPUT);
  digitalWrite(ETH_RST_PIN, LOW);  delay(200);
  digitalWrite(ETH_RST_PIN, HIGH); delay(500);

  // Standard-SPI (GPIO17–19) für Ethernet_Generic
  SPI.setRX(ETH_IO1_PIN);
  SPI.setTX(ETH_IO0_PIN);
  SPI.setSCK(ETH_SCLK_PIN);
  SPI.setCS(ETH_CS_PIN);
  SPI.begin();

  Ethernet.init(ETH_CS_PIN);
  Ethernet.begin(mac, ip, dnsServer, gateway, subnet);
  ntpUDP.begin(123);

  // W6300 Socket-0 RECV-Interrupt aktivieren (nach ntpUDP.begin())
  w6300Write(0x002C, 1, 0x04);  // Sn_IMR: RECV-Bit
  w6300Write(0x0016, 0, 0x01);  // IMR: Socket-0-Interrupt freigeben
  w6300ClearRecvInt();

  // MQTT
  mqttClient.setServer(mqtt_server, 1883);

  // PIO0: Timestamp-ISRs (PPS + INT)
  setupPIO0();

  // PIO1: QSPI-Treiber vorbereiten (Programme laden, Offsets speichern)
  qspiInit();

  Serial.println("W6300NTP bereit (PIO-Timestamps + QSPI-TX).");
}

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

/* ================================================================== */
/*  setup1 / loop1 — Core 1 (GPS + MQTT)                             */
/* ================================================================== */

void setup1() {
  Serial1.setTX(GPS_TX_PIN);
  Serial1.setRX(GPS_RX_PIN);
  Serial1.begin(115200);
}

void loop1() {
  while (Serial1.available()) gps.encode(Serial1.read());
  if (gps.time.isUpdated() && gps.date.isValid()) {
    struct tm t = {};
    t.tm_year  = gps.date.year() - 1900;
    t.tm_mon   = gps.date.month() - 1;
    t.tm_mday  = gps.date.day();
    t.tm_hour  = gps.time.hour();
    t.tm_min   = gps.time.minute();
    t.tm_sec   = gps.time.second();
    ntpEpochAtLastPPS.store((uint32_t)mktime(&t) + 2208988800UL,
                            std::memory_order_release);
  }
  gpsSats = (uint8_t)gps.satellites.value();
  gpsHdop = gps.hdop.hdop();
  delay(1);
}

/* ================================================================== */
/*  usToNTP: µs-Delta seit PPS → NTP seconds + fraction              */
/* ================================================================== */

static inline void usToNTP(uint64_t timerUs, uint32_t &sec, uint32_t &frac) {
  int64_t delta = (int64_t)(timerUs - lastPPSus);
  if (delta < 0 || delta >= 2000000) delta = 0;
  sec  = ntpEpochAtLastPPS.load(std::memory_order_acquire)
         + (uint32_t)(delta / 1000000);
  frac = (uint32_t)(((uint64_t)(delta % 1000000) * 4294967296ULL) / 1000000ULL);
}

/* ================================================================== */
/*  handleNTPRequest — RFC 4330, Stratum 1                            */
/*                                                                      */
/*  TX-Pfad mit DMA-QSPI:                                               */
/*    ntpUDP.beginPacket()    → Ziel-IP/Port (SPI)                      */
/*    w6300ReadReg16(TX_WR)   → TX-Schreibzeiger (SPI)                  */
/*    qspiWriteAsync(0–39 B)  → DMA startet; CPU frei (~2 µs)           */
/*    qspiWriteWait()         → DMA abwarten, CS↑                       */
/*    T3 = time_us_64()       → deterministisch zwischen den Writes      */
/*    qspiWriteBuf(40–47 B)   → T3-Bytes (8 Byte, synchron)             */
/*    TX_WR += 48; Sn_CR=SEND → Ethernet-TX auslösen                    */
/* ================================================================== */

void handleNTPRequest() {
  uint64_t t2 = rxTimerUs;

  byte packetBuffer[48];
  ntpUDP.read(packetBuffer, 48);

  uint32_t refSec = ntpEpochAtLastPPS.load(std::memory_order_acquire);
  uint32_t rxSec, rxFrac;
  usToNTP(t2, rxSec, rxFrac);

  byte reply[48];
  memset(reply, 0, 48);
  reply[0] = 0b00100100; reply[1] = 1; reply[2] = 0; reply[3] = (byte)-15;
  memcpy(&reply[12], "GPS ", 4);

  reply[16]=(refSec >>24)&0xFF; reply[17]=(refSec >>16)&0xFF;
  reply[18]=(refSec >> 8)&0xFF; reply[19]= refSec      &0xFF;

  memcpy(&reply[24], &packetBuffer[40], 8);

  reply[32]=(rxSec >>24)&0xFF; reply[33]=(rxSec >>16)&0xFF;
  reply[34]=(rxSec >> 8)&0xFF; reply[35]= rxSec      &0xFF;
  reply[36]=(rxFrac>>24)&0xFF; reply[37]=(rxFrac>>16)&0xFF;
  reply[38]=(rxFrac>> 8)&0xFF; reply[39]= rxFrac      &0xFF;

  ntpUDP.beginPacket(ntpUDP.remoteIP(), ntpUDP.remotePort());
  uint16_t txWrPtr = w6300ReadReg16(Sn_TX_WR, 1);

  // Bytes 0–39 via DMA — CPU blockiert nicht während ~2 µs Taktzeit
  qspiWriteAsync(txWrPtr & 0x1FFF, 2, reply, 40);
  qspiWriteWait();

  // T3 zwischen den beiden QSPI-Writes: deterministischer Abstand ~2 µs vor SEND
  uint32_t txSec, txFrac;
  usToNTP(time_us_64(), txSec, txFrac);

  reply[40]=(txSec >>24)&0xFF; reply[41]=(txSec >>16)&0xFF;
  reply[42]=(txSec >> 8)&0xFF; reply[43]= txSec      &0xFF;
  reply[44]=(txFrac>>24)&0xFF; reply[45]=(txFrac>>16)&0xFF;
  reply[46]=(txFrac>> 8)&0xFF; reply[47]= txFrac      &0xFF;

  // Bytes 40–47 (T3) synchron — 8 Byte, DMA-Overhead würde nicht lohnen
  qspiWriteBuf((txWrPtr + 40) & 0x1FFF, 2, &reply[40], 8);

  w6300WriteReg16(Sn_TX_WR, 1, txWrPtr + 48);
  w6300Write(Sn_CR, 1, 0x20);
}

/* ================================================================== */
/*  sendMQTTStatus                                                     */
/* ================================================================== */

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
