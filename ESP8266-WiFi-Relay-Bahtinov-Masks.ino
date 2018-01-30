/*
 * ESP8266-WiFi-Relay-Bahtinov-Masks.ino
 * ----------------------------------------------------------------------------
 * simple sketch of using ESP8266WebServer to switch relays on GPIO pins
 * it serves a simple website with toggle buttons for each relay
 * additional it handles up to 3 servo motors
 * ----------------------------------------------------------------------------
 * Source:     https://github.com/Jorgen-VikingGod/ESP8266-WiFi-Relay-Bahtinov-Masks
 * Copyright:  Copyright (c) 2017 Juergen Skrotzky
 * Author:     Juergen Skrotzky <JorgenVikingGod@gmail.com>
 * License:    MIT License
 * Created on: 29.Sep. 2017
 * ----------------------------------------------------------------------------
 */

extern "C" {
#include "user_interface.h"
}

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Hash.h>
#include <FS.h>
#include "ArduinoJson.h"                  //https://github.com/bblanchon/ArduinoJson
#include "webserver.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// declare and initial list of default servo settings
sServo servo[3] = {sServo(13, 0, 180), sServo(14, 180, 0), sServo(15, 0, 90)};
uint8_t servoCount = sizeof(servo) / sizeof(sServo);

// declare and initial list of default relay settings
sRelay relay[5] = {sRelay(D4), sRelay(D5), sRelay(D6), sRelay(D7), sRelay(D3)};
uint8_t relayCount = sizeof(relay) / sizeof(sRelay);

// declare WiFiMulti
ESP8266WiFiMulti WiFiMulti;

/*
 * setup function
 * ----------------------------------------------------------------------------
 */
void setup() {
  // serial interface for debug messages
  if (_debug) {
    Serial.begin(115200);
  }
  pinMode(D0, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  pinMode(D8, OUTPUT);
  digitalWrite(D0, LOW);
  digitalWrite(D4, LOW);
  digitalWrite(D5, LOW);
  digitalWrite(D6, LOW);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  yield();
  DEBUG_PRINT("\n");
  // use EEPROM
  EEPROM.begin(512);
  // use SPIFFS
  SPIFFS.begin();
  // handle requests
  server.on("/list", HTTP_GET, handleFileList);
  server.on("/edit", HTTP_GET, handleGetEditor);
  server.on("/edit", HTTP_PUT, handleFileCreate);
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  server.on("/edit", HTTP_POST, []() {
    server.send(200, "text/plain", "");
  }, handleFileUpload);
  server.onNotFound([]() {
    if (!handleFileRead(server.uri()))
      server.send(404, "text/plain", "FileNotFound");
  });
  server.on("/all", HTTP_GET, handleGetAll);
  server.on("/current", HTTP_GET, sendCurrentStates);
  server.on("/relay", HTTP_GET, handleGetRelay);
  server.on("/servo", HTTP_GET, handleGetServo);
  server.on("/toggle", HTTP_POST, handlePostToggle);
  server.on("/settings/status", HTTP_GET, handleGetStatus);
  server.on("/settings/configfile", HTTP_GET, handleGetConfigfile);
  server.on("/settings/configfile", HTTP_POST, handlePostConfigfile);
  server.begin();
  // Add service to MDNS
  MDNS.addService("http", "tcp", 80);
  // load config.json and connect to WiFI
  if (!loadConfiguration()) {
    // if no configuration found,
    // try to connect hard coded multi APs
    connectSTA(nullptr, nullptr, "wifi-relay");
  }
  // load last states from EEPROM
  loadSettings();
}

unsigned long previousMillis = 0;
int interval = 250;

/*
 * main loop function
 * ----------------------------------------------------------------------------
 */  
void loop() {
  // only drive servos to new position each 250ms
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last update time
    previousMillis = currentMillis;
    // loop all servos and sweep to new position
    for (uint8_t idx = 0; idx < servoCount; idx++) {
      if (servo[idx].sweep) {
        servo[idx].sweep = false;
        pwm.setPWM(servo[idx].pin, 0, servo[idx].currentPulseLength());
      }
    }
  }
  // handle http clients
  server.handleClient();
}

/*
 * load stored settings from EEPROM
 * ----------------------------------------------------------------------------
 */
void loadSettings() {
  // load relay states
  relay[0].state = EEPROM.read(0);
  relay[1].state = EEPROM.read(1);
  relay[2].state = EEPROM.read(2);
  relay[3].state = EEPROM.read(3);
  relay[4].state = EEPROM.read(4);
  // load servo states
  servo[0].state = EEPROM.read(5);
  servo[1].state = EEPROM.read(6);
  servo[2].state = EEPROM.read(7);
  DEBUG_PRINTF("loadSettings: relay1: %d, relay2: %d, relay3: %d, relay4: %d, relay5: %d, servo1: %d, servo2: %d, servo3: %d\n", relay[0].state, relay[1].state, relay[2].state, relay[3].state, relay[4].state, servo[0].state, servo[1].state, servo[2].state);
  digitalWrite(D0, HIGH);
  // initialize IO pins for relays
  pinMode(relay[0].pin, OUTPUT);
  pinMode(relay[1].pin, OUTPUT);
  pinMode(relay[2].pin, OUTPUT);
  pinMode(relay[3].pin, OUTPUT);
  pinMode(relay[4].pin, OUTPUT);
  // set relay states
  setRelay(0, relay[0].state);
  setRelay(1, relay[1].state);
  setRelay(2, relay[2].state);
  setRelay(3, relay[3].state);
  setRelay(4, relay[4].state);
  // set sweep flags to force servo motors to drive on correct state (open or close)
  servo[0].sweep = true;
  servo[1].sweep = true;
  servo[2].sweep = true;
  delay(500);
}

/*
 * load config.json file and try to connect
 * ----------------------------------------------------------------------------
 */
bool loadConfiguration() {
  // try to open config.json file
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    DEBUG_PRINTLN(F("[WARN] Failed to open config file"));
    return false;
  }
  size_t size = configFile.size();
  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);
  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(buf.get());
  if (!json.success()) {
    DEBUG_PRINTLN(F("[WARN] Failed to parse config file"));
    return false;
  }
  DEBUG_PRINTLN(F("[INFO] Config file found"));
  if (_debug) {
    json.prettyPrintTo(Serial);
  }
  // get relay settings
  relay[0].type = json["relay1"]["type"];
  relay[0].pin = json["relay1"]["pin"];
  relay[1].type = json["relay2"]["type"];
  relay[1].pin = json["relay2"]["pin"];
  relay[2].type = json["relay3"]["type"];
  relay[2].pin = json["relay3"]["pin"];
  relay[3].type = json["relay4"]["type"];
  relay[3].pin = json["relay4"]["pin"];
  relay[4].type = json["relay5"]["type"];
  relay[4].pin = json["relay5"]["pin"];
  // get servo settings
  servo[0].open = json["servo1"]["open"];
  servo[0].close = json["servo1"]["close"];
  servo[0].pulseMin = json["servo1"]["pulsemin"];
  servo[0].pulseMax = json["servo1"]["pulsemax"];
  servo[0].pin = json["servo1"]["pin"];
  servo[1].open = json["servo2"]["open"];
  servo[1].close = json["servo2"]["close"];
  servo[1].pulseMin = json["servo2"]["pulsemin"];
  servo[1].pulseMax = json["servo2"]["pulsemax"];
  servo[1].pin = json["servo2"]["pin"];
  servo[2].open = json["servo3"]["open"];
  servo[2].close = json["servo3"]["close"];
  servo[2].pulseMin = json["servo3"]["pulsemin"];
  servo[2].pulseMax = json["servo3"]["pulsemax"];
  servo[2].pin = json["servo3"]["pin"];
  // get stored wifi settings
  const char * ssid = json["ssid"];
  const char * password = json["wifipwd"];
  const char * hostname = json["hostname"];
  // try to connect with stored settings and hard coded ones
  if (!connectSTA(ssid, password, hostname)) {
    return false;
  }
  return true;
}

/*
 * try to connect WiFi
 * ----------------------------------------------------------------------------
 */
bool connectSTA(const char* ssid, const char* password, const char * hostname) {
  WiFi.mode(WIFI_STA);
  // add here your hard coded backfall wifi ssid and password
  //WiFiMulti.addAP("<YOUR-SSID-1>", "<YOUR-WIFI-PASS-1>");
  //WiFiMulti.addAP("<YOUR-SSID-2>", "<YOUR-WIFI-PASS-2>");
  if (ssid && password) {
    WiFiMulti.addAP(ssid, password);
  }
  // We try it for 30 seconds and give up on if we can't connect
  unsigned long now = millis();
  uint8_t timeout = 30; // define when to time out in seconds
  DEBUG_PRINT(F("[INFO] Trying to connect WiFi: "));
  while (WiFiMulti.run() != WL_CONNECTED) {
    if (millis() - now < timeout * 1000) {
      delay(200);
      DEBUG_PRINT(F("."));
    } else {
      DEBUG_PRINTLN("");
      DEBUG_PRINTLN(F("[WARN] Couldn't connect in time"));
      return false;
    }
  }
  if (hostname) {
    if (MDNS.begin(hostname)) {
      DEBUG_PRINTLN(F("MDNS responder started"));
    }
  }
  DEBUG_PRINTLN("");
  DEBUG_PRINT(F("[INFO] Client IP address: ")); // Great, we connected, inform
  DEBUG_PRINTLN(WiFi.localIP());
  return true;
}

/*
 * send ESP8266 status
 * ----------------------------------------------------------------------------
 */
void sendStatus() {
  DEBUG_PRINTLN("sendStatus()");
  struct ip_info info;
  FSInfo fsinfo;
  if (!SPIFFS.info(fsinfo)) {
    DEBUG_PRINTLN(F("[ WARN ] Error getting info on SPIFFS"));
  }
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["command"] = "status";
  root["heap"] = ESP.getFreeHeap();
  root["chipid"] = String(ESP.getChipId(), HEX);
  root["cpu"] = ESP.getCpuFreqMHz();
  root["availsize"] = ESP.getFreeSketchSpace();
  root["availspiffs"] = fsinfo.totalBytes - fsinfo.usedBytes;
  root["spiffssize"] = fsinfo.totalBytes;
  wifi_get_ip_info(STATION_IF, &info);
  struct station_config conf;
  wifi_station_get_config(&conf);
  root["ssid"] = String(reinterpret_cast<char*>(conf.ssid));
  root["dns"] = printIP(WiFi.dnsIP());
  root["mac"] = WiFi.macAddress();
  IPAddress ipaddr = IPAddress(info.ip.addr);
  IPAddress gwaddr = IPAddress(info.gw.addr);
  IPAddress nmaddr = IPAddress(info.netmask.addr);
  root["ip"] = printIP(ipaddr);
  root["gateway"] = printIP(gwaddr);
  root["netmask"] = printIP(nmaddr);
  String json;
  root.printTo(json);
  DEBUG_PRINTLN(json);
  server.setContentLength(root.measureLength());
  server.send(200, "application/json", json);
}

/*
 * send whole configfile and status
 * ----------------------------------------------------------------------------
 */
void sendConfigfile() {
  // try to open config.json file
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    DEBUG_PRINTLN(F("[WARN] Failed to open config file"));
  }
  size_t size = configFile.size();
  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);
  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(buf.get());
  if (!root.success()) {
    DEBUG_PRINTLN(F("[WARN] Failed to parse config file"));
  }
  DEBUG_PRINTLN(F("[INFO] Config file found"));
  struct ip_info info;
  FSInfo fsinfo;
  if (!SPIFFS.info(fsinfo)) {
    DEBUG_PRINTLN(F("[ WARN ] Error getting info on SPIFFS"));
  }
  root["heap"] = ESP.getFreeHeap();
  root["chipid"] = String(ESP.getChipId(), HEX);
  root["cpu"] = ESP.getCpuFreqMHz();
  root["availsize"] = ESP.getFreeSketchSpace();
  root["availspiffs"] = fsinfo.totalBytes - fsinfo.usedBytes;
  root["spiffssize"] = fsinfo.totalBytes;
  wifi_get_ip_info(STATION_IF, &info);
  struct station_config conf;
  wifi_station_get_config(&conf);
  root["ssid"] = String(reinterpret_cast<char*>(conf.ssid));
  root["dns"] = printIP(WiFi.dnsIP());
  root["mac"] = WiFi.macAddress();
  IPAddress ipaddr = IPAddress(info.ip.addr);
  IPAddress gwaddr = IPAddress(info.gw.addr);
  IPAddress nmaddr = IPAddress(info.netmask.addr);
  root["ip"] = printIP(ipaddr);
  root["gateway"] = printIP(gwaddr);
  root["netmask"] = printIP(nmaddr);
  String json;
  root.printTo(json);
  DEBUG_PRINTLN(json);
  server.setContentLength(root.measureLength());
  server.send(200, "application/json", json);
}

/*
 * send all states
 * ----------------------------------------------------------------------------
 */
void sendAll() {
  DEBUG_PRINTLN("sendAll()");
  // try to open config.json file
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    DEBUG_PRINTLN(F("[WARN] Failed to open config file"));
  }
  size_t size = configFile.size();
  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);
  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(buf.get());
  if (!json.success()) {
    DEBUG_PRINTLN(F("[WARN] Failed to parse config file"));
  }
  DynamicJsonBuffer jsonBuffer2;
  JsonObject &root = jsonBuffer2.createObject();
  // create relay json objects
  JsonObject& relay1 = jsonBuffer2.createObject();
  relay1["name"] = json["relay1"]["name"];
  relay1["state"] = relay[0].state;
  root.set("relay1", relay1);
  JsonObject& relay2 = jsonBuffer2.createObject();
  relay2["name"] = json["relay2"]["name"];
  relay2["state"] = relay[1].state;
  root.set("relay2", relay2);
  JsonObject& relay3 = jsonBuffer2.createObject();
  relay3["name"] = json["relay3"]["name"];
  relay3["state"] = relay[2].state;
  root.set("relay3", relay3);
  JsonObject& relay4 = jsonBuffer2.createObject();
  relay4["name"] = json["relay4"]["name"];
  relay4["state"] = relay[3].state;
  root.set("relay4", relay4);
  JsonObject& relay5 = jsonBuffer2.createObject();
  relay5["name"] = json["relay5"]["name"];
  relay5["state"] = relay[4].state;
  root.set("relay5", relay5);
  // create servo json objects
  JsonObject& servo1 = jsonBuffer2.createObject();
  servo1["name"] = json["servo1"]["name"];
  servo1["state"] = servo[0].state;
  root.set("servo1", servo1);
  JsonObject& servo2 = jsonBuffer2.createObject();
  servo2["name"] = json["servo2"]["name"];
  servo2["state"] = servo[1].state;
  root.set("servo2", servo2);
  JsonObject& servo3 = jsonBuffer2.createObject();
  servo3["name"] = json["servo3"]["name"];
  servo3["state"] = servo[2].state;
  root.set("servo3", servo3);
  String jsonStr;
  root.printTo(jsonStr);
  DEBUG_PRINTLN(jsonStr);
  server.setContentLength(root.measureLength());
  server.send(200, "application/json", jsonStr);
}

/*
 * send current states
 * ----------------------------------------------------------------------------
 */
void sendCurrentStates() {
  DEBUG_PRINTLN("sendCurrentStates()");
  DynamicJsonBuffer jsonBuffer;
  JsonObject &root = jsonBuffer.createObject();
  // create relay json values
  JsonObject& relay1 = jsonBuffer.createObject();
  relay1["state"] = relay[0].state;
  root.set("relay1", relay1);
  JsonObject& relay2 = jsonBuffer.createObject();
  relay2["state"] = relay[1].state;
  root.set("relay2", relay2);
  JsonObject& relay3 = jsonBuffer.createObject();
  relay3["state"] = relay[2].state;
  root.set("relay3", relay3);
  JsonObject& relay4 = jsonBuffer.createObject();
  relay4["state"] = relay[3].state;
  root.set("relay4", relay4);
  JsonObject& relay5 = jsonBuffer.createObject();
  relay5["state"] = relay[4].state;
  root.set("relay5", relay5);
  // create servo json values
  JsonObject& servo1 = jsonBuffer.createObject();
  servo1["state"] = servo[0].state;
  root.set("servo1", servo1);
  JsonObject& servo2 = jsonBuffer.createObject();
  servo2["state"] = servo[1].state;
  root.set("servo2", servo2);
  JsonObject& servo3 = jsonBuffer.createObject();
  servo3["state"] = servo[2].state;
  root.set("servo3", servo3);
  String jsonStr;
  root.printTo(jsonStr);
  DEBUG_PRINTLN(jsonStr);
  server.setContentLength(root.measureLength());
  server.send(200, "application/json", jsonStr);
}

/*
 * set current relay state and store it in EEPROM
 * ----------------------------------------------------------------------------
 */
void setRelay(uint8_t idx, uint8_t value) {
  DEBUG_PRINTF("setRelay(%d, %d)\n", idx, value);
  uint8_t lowValue  = (relay[idx].type == 0 ? HIGH : LOW);
  uint8_t highValue = (relay[idx].type == 0 ? LOW  : HIGH);
  relay[idx].state = (value == 0 ? LOW : HIGH);
  digitalWrite(relay[idx].pin, (relay[idx].state == LOW ? lowValue : highValue));
  EEPROM.write(idx, relay[idx].state);
  EEPROM.commit();
}

/*
 * send relay state back to website (idx = index of relay)
 * ----------------------------------------------------------------------------
 */
void sendRelay(uint8_t idx) {
  DEBUG_PRINTF("sendRelay(%d)\n", idx);
  DynamicJsonBuffer jsonBuffer;
  JsonObject &root = jsonBuffer.createObject();
  root["relay" + String(idx + 1)] = relay[idx].state;
  String json;
  root.printTo(json);
  DEBUG_PRINTLN(json);
  server.setContentLength(root.measureLength());
  server.send(200, "application/json", json);
}

/*
 * set current servo state and store it in EEPROM
 * ----------------------------------------------------------------------------
 */
void setServo(uint8_t idx, uint8_t value) {
  DEBUG_PRINTF("setServo(%d, %d)\n", idx, value);
  servo[idx].state = value;
  // set sweep flags to force servo motors to drive on correct state (open or close)
  servo[idx].sweep = true;
  EEPROM.write(5 + idx, servo[idx].state);
  EEPROM.commit();
}

/*
 * send servo state back to website (idx = index of servo)
 * ----------------------------------------------------------------------------
 */
void sendServo(uint8_t idx) {
  DEBUG_PRINTF("sendServo(%d)\n", idx);
  DynamicJsonBuffer jsonBuffer;
  JsonObject &root = jsonBuffer.createObject();
  root["servo" + String(idx + 1)] = servo[idx].state;
  String json;
  root.printTo(json);
  DEBUG_PRINTLN(json);
  server.setContentLength(root.measureLength());
  server.send(200, "application/json", json);
}

