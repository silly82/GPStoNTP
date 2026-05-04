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

/* PPS-Zeitstempel (IRAM, da ISR) */
volatile uint32_t lastPPSMicros = 0;
volatile uint32_t ppsCount      = 0;

/* NTP-Zustand */
uint32_t ntpEpoch          = 0; // NTP-Timestamp (Sekunden seit 1.1.1900)
uint32_t ntpRequestsServed = 0;

/* ------------------------------------------------------------------ */
/*  Interrupt: PPS-Flanke merken                                        */
/* ------------------------------------------------------------------ */
void IRAM_ATTR ppsHandler() {
  lastPPSMicros = micros();
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

  // GPS-Zeit → NTP-Epoch aktualisieren
  if (gps.time.isUpdated() && gps.date.isValid()) {
    struct tm t;
    t.tm_year  = gps.date.year() - 1900;
    t.tm_mon   = gps.date.month() - 1;
    t.tm_mday  = gps.date.day();
    t.tm_hour  = gps.time.hour();
    t.tm_min   = gps.time.minute();
    t.tm_sec   = gps.time.second();
    t.tm_isdst = 0;
    // Unix-Zeit + Offset 1900→1970 ergibt NTP-Timestamp
    ntpEpoch = (uint32_t)mktime(&t) + 2208988800UL;
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

  // Sub-Sekunden-Anteil aus PPS-Abstand berechnen
  uint32_t usSincePPS = micros() - lastPPSMicros;
  if (usSincePPS > 1000000) usSincePPS = 0; // PPS älter als 1 s → verwerfen
  // µs → NTP-Fraction (2^32 / 1e6 ≈ 4294.967296)
  uint32_t fraction = (uint32_t)(usSincePPS * 4294.967296);

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
  reply[40] = (ntpEpoch >> 24) & 0xFF;
  reply[41] = (ntpEpoch >> 16) & 0xFF;
  reply[42] = (ntpEpoch >>  8) & 0xFF;
  reply[43] =  ntpEpoch        & 0xFF;
  reply[44] = (fraction >> 24) & 0xFF;
  reply[45] = (fraction >> 16) & 0xFF;
  reply[46] = (fraction >>  8) & 0xFF;
  reply[47] =  fraction        & 0xFF;

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
