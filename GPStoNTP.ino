/*
 * GPStoNTP — Stratum-1 NTP Server auf ESP32 + W5500 + GPS
 *
 * Hardware:
 *   - ESP32 (z.B. S3 DevKit)
 *   - W5500 Ethernet-Modul (SPI)
 *   - GPS-Modul mit NMEA-Ausgang (UART, 115200 Baud)
 *   - PPS-Ausgang des GPS-Moduls für präzisen 1-Hz-Takt
 *
 * Abhängigkeiten (Arduino Library Manager):
 *   - Ethernet          (Arduino)
 *   - TinyGPS++         (Mikal Hart)
 *   - PubSubClient      (Nick O'Leary)
 */

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <TinyGPS++.h>
#include <PubSubClient.h>
#include <time.h>
#include "esp_timer.h"
#include "config.h"

/* --- HARDWARE MAPPING --- */
#define ETH_SCLK_PIN 13
#define ETH_MISO_PIN 12
#define ETH_MOSI_PIN 11
#define ETH_CS_PIN   14
#define ETH_RST_PIN  9
#define GPS_RX_PIN   1
#define GPS_TX_PIN   2
#define PPS_PIN      4

/* --- NETZWERK-KONFIGURATION (aus config.h) --- */
byte mac[]       = CFG_MAC;
IPAddress ip     (CFG_IP);
IPAddress gateway(CFG_GATEWAY);
IPAddress subnet (CFG_SUBNET);
IPAddress dns    (CFG_DNS);

/* --- MQTT-KONFIGURATION (aus config.h) --- */
const char* mqtt_server = CFG_MQTT_SERVER;
const char* mqtt_user   = CFG_MQTT_USER;
const char* mqtt_pass   = CFG_MQTT_PASS;
const char* mqtt_topic  = CFG_MQTT_TOPIC;

/* --- GLOBALE OBJEKTE --- */
EthernetUDP    ntpUDP;
EthernetClient ethClient;
PubSubClient   mqttClient(ethClient);
TinyGPSPlus    gps;
HardwareSerial GPS_Serial(1);

/* PPS-Zeitstempel (IRAM, da ISR) — esp_timer: 64-bit, µs, kein Überlauf */
volatile int64_t  lastPPSus = 0;
volatile uint32_t ppsCount  = 0;

/* NTP-Zustand */
uint32_t ntpEpochAtLastPPS = 0; // NTP-Epoch der Sekunde, die der letzte PPS markierte
uint32_t ntpRequestsServed = 0;

/* ------------------------------------------------------------------ */
/*  Interrupt: PPS-Flanke per Hardware-Timer latchen                    */
/* ------------------------------------------------------------------ */
void IRAM_ATTR ppsHandler() {
  lastPPSus = esp_timer_get_time(); // Hardware-Timer, direkt beim Edge gelesen
  ppsCount++;
}

/* ------------------------------------------------------------------ */
/*  setup                                                               */
/* ------------------------------------------------------------------ */
void setup() {
  Serial.begin(115200);

  // W5500 Hardware-Reset
  pinMode(ETH_RST_PIN, OUTPUT);
  digitalWrite(ETH_RST_PIN, LOW);  delay(200);
  digitalWrite(ETH_RST_PIN, HIGH); delay(1000);

  // SPI + Ethernet initialisieren
  SPI.begin(ETH_SCLK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN, ETH_CS_PIN);
  Ethernet.init(ETH_CS_PIN);
  Ethernet.begin(mac, ip, dns, gateway, subnet);
  ntpUDP.begin(123);

  // MQTT
  mqttClient.setServer(mqtt_server, 1883);

  // GPS-UART (115200 Baud, konfiguriert via ublox_config.py)
  GPS_Serial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // PPS-Interrupt auf steigende Flanke
  pinMode(PPS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsHandler, RISING);

  Serial.println("Stratum-1 Server bereit.");
}

/* ------------------------------------------------------------------ */
/*  loop                                                                */
/* ------------------------------------------------------------------ */
void loop() {
  // GPS-Daten einlesen und parsen
  while (GPS_Serial.available()) {
    gps.encode(GPS_Serial.read());
  }

  // GPS-Zeit → NTP-Epoch auf letzten PPS-Puls einrasten
  // Die NMEA-Sentence kommt 100–400 ms nach dem PPS-Puls und gibt die Zeit
  // der gerade gestarteten Sekunde an — also exakt die Sekunde des letzten PPS.
  if (gps.time.isUpdated() && gps.date.isValid()) {
    struct tm t;
    t.tm_year  = gps.date.year() - 1900;
    t.tm_mon   = gps.date.month() - 1;
    t.tm_mday  = gps.date.day();
    t.tm_hour  = gps.time.hour();
    t.tm_min   = gps.time.minute();
    t.tm_sec   = gps.time.second();
    t.tm_isdst = 0;
    ntpEpochAtLastPPS = (uint32_t)mktime(&t) + 2208988800UL;
  }

  // Eingehende NTP-Anfragen beantworten
  int packetSize = ntpUDP.parsePacket();
  if (packetSize >= 48) {
    handleNTPRequest();
    ntpRequestsServed++;
  }

  // MQTT-Status alle 30 s senden
  static uint32_t lastMQTT = 0;
  if (millis() - lastMQTT > 30000) {
    sendMQTTStatus();
    lastMQTT = millis();
  }
  mqttClient.loop();
}

/* ------------------------------------------------------------------ */
/*  Hilfsfunktion: µs-seit-PPS → NTP-Seconds + Fraction                */
/* ------------------------------------------------------------------ */
static inline void usToNTP(int64_t timerUs, uint32_t &sec, uint32_t &frac) {
  int64_t delta = timerUs - lastPPSus;
  if (delta < 0 || delta >= 2000000) delta = 0; // ungültig → 0
  sec  = ntpEpochAtLastPPS + (uint32_t)(delta / 1000000);
  frac = (uint32_t)(((uint64_t)(delta % 1000000) * 4294967296ULL) / 1000000ULL);
}

/* ------------------------------------------------------------------ */
/*  NTP-Antwort senden (RFC 4330)                                       */
/* ------------------------------------------------------------------ */
void handleNTPRequest() {
  // Empfangszeitpunkt so früh wie möglich stempeln (T2)
  int64_t rxTimerUs = esp_timer_get_time();

  byte packetBuffer[48];
  ntpUDP.read(packetBuffer, 48);

  uint32_t refSec, refFrac, rxSec, rxFrac, txSec, txFrac;

  // Reference Timestamp: Zeitpunkt des letzten GPS/PPS-Sync (Fraction = 0,
  // da PPS per Definition die exakte Sekundengrenze markiert)
  refSec  = ntpEpochAtLastPPS;
  refFrac = 0;

  // Receive Timestamp (T2): wann traf das Paket ein
  usToNTP(rxTimerUs, rxSec, rxFrac);

  // Transmit Timestamp (T3): unmittelbar vor dem Senden
  usToNTP(esp_timer_get_time(), txSec, txFrac);

  byte reply[48];
  memset(reply, 0, 48);

  // Byte 0: LI=0, VN=4, Mode=4 (server)  Byte 1: Stratum=1
  // Byte 2: Poll=0                        Byte 3: Precision=-15
  reply[0] = 0b00100100; reply[1] = 1; reply[2] = 0; reply[3] = (byte)-15;

  // Reference Identifier (bytes 12–15): "GPS "
  memcpy(&reply[12], "GPS ", 4);

  // Reference Timestamp (bytes 16–23)
  reply[16] = (refSec  >> 24) & 0xFF; reply[17] = (refSec  >> 16) & 0xFF;
  reply[18] = (refSec  >>  8) & 0xFF; reply[19] =  refSec         & 0xFF;
  reply[20] = (refFrac >> 24) & 0xFF; reply[21] = (refFrac >> 16) & 0xFF;
  reply[22] = (refFrac >>  8) & 0xFF; reply[23] =  refFrac        & 0xFF;

  // Origin Timestamp (bytes 24–31): T1 des Clients unverändert zurück
  memcpy(&reply[24], &packetBuffer[40], 8);

  // Receive Timestamp (bytes 32–39): T2
  reply[32] = (rxSec  >> 24) & 0xFF; reply[33] = (rxSec  >> 16) & 0xFF;
  reply[34] = (rxSec  >>  8) & 0xFF; reply[35] =  rxSec         & 0xFF;
  reply[36] = (rxFrac >> 24) & 0xFF; reply[37] = (rxFrac >> 16) & 0xFF;
  reply[38] = (rxFrac >>  8) & 0xFF; reply[39] =  rxFrac        & 0xFF;

  // Transmit Timestamp (bytes 40–47): T3
  reply[40] = (txSec  >> 24) & 0xFF; reply[41] = (txSec  >> 16) & 0xFF;
  reply[42] = (txSec  >>  8) & 0xFF; reply[43] =  txSec         & 0xFF;
  reply[44] = (txFrac >> 24) & 0xFF; reply[45] = (txFrac >> 16) & 0xFF;
  reply[46] = (txFrac >>  8) & 0xFF; reply[47] =  txFrac        & 0xFF;

  ntpUDP.beginPacket(ntpUDP.remoteIP(), ntpUDP.remotePort());
  ntpUDP.write(reply, 48);
  ntpUDP.endPacket();
}

/* ------------------------------------------------------------------ */
/*  MQTT-Status senden                                                  */
/* ------------------------------------------------------------------ */
void sendMQTTStatus() {
  if (!mqttClient.connected()) {
    mqttClient.connect("ESP32-NTP-S3", mqtt_user, mqtt_pass);
  }
  if (mqttClient.connected()) {
    String payload = "{\"sats\":"   + String(gps.satellites.value()) +
                     ",\"hdop\":"   + String(gps.hdop.hdop())        +
                     ",\"pps\":"    + String(ppsCount)               +
                     ",\"served\":" + String(ntpRequestsServed)      + "}";
    mqttClient.publish(mqtt_topic, payload.c_str());
  }
}
