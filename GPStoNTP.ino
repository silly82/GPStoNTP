/*
 * GPStoNTP — Stratum-1 NTP Server auf ESP32 + W5500 + GPS
 *
 * Hardware:
 *   - ESP32 (z.B. S3 DevKit)
 *   - W5500 Ethernet-Modul (SPI)
 *   - GPS-Modul mit NMEA-Ausgang (UART, 9600 Baud)
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
/*  Interrupt: PPS-Flanke per Hardware-Timer laten                      */
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

  // GPS-UART (9600 Baud, sicher für alle gängigen Module)
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // PPS-Interrupt auf steigende Flanke
  pinMode(PPS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsHandler, RISING);

  Serial.println("Stratum-1 Server bereit. IP: 192.168.188.2");
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
/*  NTP-Antwort senden (RFC 4330)                                       */
/* ------------------------------------------------------------------ */
void handleNTPRequest() {
  byte packetBuffer[48];
  ntpUDP.read(packetBuffer, 48);

  // Verstrichene µs seit letztem PPS — Hardware-Timer, kein Float
  int64_t  usSincePPS64 = esp_timer_get_time() - lastPPSus;
  if (usSincePPS64 < 0 || usSincePPS64 >= 1000000) usSincePPS64 = 0;
  uint32_t usSincePPS   = (uint32_t)usSincePPS64;

  // Aktuelle Sekunde = PPS-Epoch + ggf. überrollte Sekunde (sollte 0 sein)
  uint32_t txSeconds  = ntpEpochAtLastPPS + (uint32_t)(usSincePPS / 1000000);
  // µs → NTP-Fraction: (µs * 2^32) / 1e6, reine Integer-Arithmetik
  uint32_t txFraction = (uint32_t)(((uint64_t)usSincePPS * 4294967296ULL) / 1000000ULL);

  byte reply[48];
  memset(reply, 0, 48);

  // LI=0, VN=4, Mode=4 (server), Stratum=1, Poll=0, Precision=-15
  reply[0] = 0b00100100;
  reply[1] = 1;
  reply[2] = 0;
  reply[3] = (byte)-15;

  // Reference Identifier: "GPS "
  memcpy(&reply[12], "GPS ", 4);

  // Transmit Timestamp (Seconds + Fraction)
  reply[40] = (txSeconds  >> 24) & 0xFF;
  reply[41] = (txSeconds  >> 16) & 0xFF;
  reply[42] = (txSeconds  >>  8) & 0xFF;
  reply[43] =  txSeconds         & 0xFF;
  reply[44] = (txFraction >> 24) & 0xFF;
  reply[45] = (txFraction >> 16) & 0xFF;
  reply[46] = (txFraction >>  8) & 0xFF;
  reply[47] =  txFraction        & 0xFF;

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
