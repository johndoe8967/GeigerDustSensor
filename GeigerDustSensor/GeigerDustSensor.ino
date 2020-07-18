
/*
   File: GeigerDustSensor
   Original Author: https://github.com/johndoe8967

   an environmental sensor application with IoT connection to ThingSpeak and Radmon

   detection of radioactive decay events on interrupt pin 0
   2 modes: stationary (wifi client) mobile (wifi SoftAP) depending on pin 2

   measure air quality (particle matter

   basic settings are changeable as telnet commands

   Created on June 6, 2020
*/
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <ESP8266HTTPClient.h>
#include <SdsDustSensor.h>
#include <DateTime.h>
#include "EspMQTTClient.h"
#include <ArduinoJson.h>
#include "RemoteDebugCfg.h"
#include <RemoteDebug.h>
#include "CredentialSetting.h"

#define SWVersion "GeigerDust V1.0"

RemoteDebug Debug;

#define INT_PIN 0   // GPIO0 to detect radiation (has to be true at boot)
#define MODE_PIN 2  // GPIO2 to change mode (has to be true at boot)

SdsDustSensor sds(Serial); // passing HardwareSerial& as parameter
enum {sleeping = 0, wakeup, heating, measure} sdsStates;

String RadmonHost = "http://radmon.org";     // no need to change this
String ThingSpeakHost = "http://api.thingspeak.com";
HTTPClient thingSpeakDustClient;
HTTPClient thingSpeakGeigerClient;
String deviceName = "GeigerDust";

#define geiger
#ifdef geiger
//Geiger Counter Variables
uint32 event_counter;
uint32 events = 0;
uint32 measureInterval = 0;
uint32 actMeasureInterval = 0;        // last measure intervall in us
uint32 setMeasureInterval = 60000000;   // set value for measure intervall in us
float doseRatio;
os_timer_t measureTimer;
bool tickOccured;

// start of timerCallback
void taskMeasure(void *pArg) {            // cyclic measurement task 100ms
  uint32 actMicros = micros();
  auto actInterval = actMicros - actMeasureInterval;
  bool stopMeasure = false;

  if (setMeasureInterval == 0) {      // if no measure interval is set then stop after 100 events or 15 seconds
    if ((event_counter >= 100) && (actInterval > 15000000)) {
      stopMeasure = true;
    }
  } else {
    stopMeasure = true;
  }

  if (stopMeasure) {
    // send Measurement
    debugV("Events: %ld ", event_counter);
    debugV("Interval: %ld\r\n", actInterval);

    storeGeigerData(event_counter, actInterval);
    event_counter = 0;
    actMeasureInterval = actMicros;

    if (setMeasureInterval == 0) {  // is no measure interval is set check periodically with 100ms
      debugV("100ms measureinterval");
      os_timer_arm(&measureTimer, 100, false);
    } else {
      debugV("set next measureinterval");
      os_timer_arm(&measureTimer, (setMeasureInterval - (micros() - actMicros)) / 1000, false);
    }
  }
} // End of timerCallback


void ICACHE_RAM_ATTR interruptHandler()
{
  event_counter++;            // count radiation event
}
#endif

bool connected = false;
unsigned long actMillis = 0;
unsigned long sdsTimer = 0;
unsigned long sdsUpdateIntervall = 600000;
String message;
bool timeValid = false;
bool lastTimeValid = false;


// MQTT client settings
EspMQTTClient MQTTClient(
  MQTTHostname,       // MQTT Broker server ip
  MQTTPort,           // The MQTT port, default to 1883. this line can be omitted
  MQTTUser,           // Can be omitted if not needed
  MQTTPassword,       // Can be omitted if not needed
  deviceName.c_str()  // Client name that uniquely identify your device
);

// This function is called once everything is connected (Wifi and MQTT)
void onConnectionEstablished()
{
  MQTTClient.publish("device/online", deviceName.c_str());
  MQTTClient.subscribe(deviceName.c_str(), [](const String & payload) {
    debugD("Received message: %s", payload.c_str());

    if (payload == "getCommands") {
      debugD("Received get commands");
      String commands = "{\"commands\":[{\"cmd\":\"Intervall\",\"type\":\"integer\",\"min\":60000,\"max\":3600000,\"value\":";
      commands += sdsUpdateIntervall;
      commands += "},{\"cmd\":\"Debug\",\"type\":\"bool\"}]}";
      MQTTClient.publish((deviceName + "/commands").c_str(), commands);
    } else if (payload == "info") {
      debugD("Received info command");
      String info = "{\"version\":\"";
      info += SWVersion;
      info += "\"}";
      MQTTClient.publish("CO2Sensor/info", info);
    } else {
      const int capacity = JSON_OBJECT_SIZE(3) + 2 * JSON_OBJECT_SIZE(1);
      StaticJsonDocument<capacity> doc;
      DeserializationError err = deserializeJson(doc, payload);
      if (err) {
        debugE("deserializeJson() failed with code %s", err.c_str());
      }

      if (doc.containsKey("Intervall")) {
        auto receivedVal = doc["Intervall"].as<unsigned long>();
        debugD("Received Intervall: %lu", receivedVal);
        if ((receivedVal >= 60000) && (receivedVal < 3600000)) {
          sdsUpdateIntervall = receivedVal;
        }
      }
      if (doc.containsKey("Debug")) {
        debugD("Received Debug");
      }
    }
  });

  MQTTClient.subscribe("device", [](const String & payload) {
    if (payload == "scan") {
      debugD("Received device scan");
      MQTTClient.publish("device/scan", deviceName.c_str());
    }
  });


  DateTime.setTimeZone(DateTimeClass::TIMEZONE_UTC);
  DateTime.setServer("0.at.pool.ntp.org");
  // this method config ntp and wait for time sync
  // default timeout is 10 seconds
  DateTime.begin(/* timeout param */);
  if (!DateTime.isTimeValid()) {
    debugE("Failed to get time from server.");
  }

  sds.begin(); // this line will begin Serial1 with given baud rate (9600 by default)
  debugD("SDS Version: %s", sds.queryFirmwareVersion().toString().c_str()); // prints firmware version
  debugD("SDS Mode: %s", sds.setQueryReportingMode().toString().c_str()); // ensures sensor is in 'query' reporting mode


#ifdef geiger
  attachInterrupt(INT_PIN, interruptHandler, RISING);

  os_timer_setfn(&measureTimer, taskMeasure, NULL);
  os_timer_arm(&measureTimer, 100, true);
#endif

  connected = true;
}


void setup() {
  int cnt = 0;

  // set for STA mode
  WiFi.mode(WIFI_STA);

  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Booting");

  Debug.begin("GeigerDust.local");

  //configure pin0
  pinMode(MODE_PIN, INPUT_PULLUP);

  // deplay for 2 sec for smartConfig
//  Serial.println("2 sec before clear SmartConfig");
  delay(2000);

  // read pullup
  int isSmartConfig = digitalRead(MODE_PIN);
  if (isSmartConfig == 0) {
//    Serial.println("clear config");
    // reset default config
    WiFi.disconnect();

  }

  // if wifi cannot connect start smartconfig
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
//    Serial.print(".");
    if (cnt++ >= 15) {
      WiFi.beginSmartConfig();
      while (1) {
        delay(500);
        if (WiFi.smartConfigDone()) {
//          Serial.println("SmartConfig Success");
          break;
        }
      }
    }
  }

//  Serial.println("");

  WiFi.printDiag(Serial);

  // Print the IP address
//  Serial.println(WiFi.localIP());
}

bool sendGeigerValid = false;
void storeGeigerData(uint32 event_counter, uint32 actInterval) {
  events = event_counter;
  measureInterval = actInterval;
  sendGeigerValid = true;
}

void loop() {
  Debug.handle();
  MQTTClient.loop();

  if (!DateTime.isTimeValid()) {
    debugE("Failed to get time from server.");
    DateTime.begin(/* timeout param */);
  }


  actMillis = millis();

  if (sendGeigerValid) {
    sendGeiger(events, measureInterval);
    sendGeigerValid = false;
  }

  switch (sdsStates) {
    case sleeping:
      if (actMillis - sdsTimer >= sdsUpdateIntervall) {
        sdsStates = wakeup;
        debugV("SDS Wakeup");
      }
      break;
    case wakeup:
      sds.wakeup();
      sdsStates = heating;
      debugV("SDS Heating");
      break;
    case heating:
      if (actMillis - (sdsTimer + 30000) >= sdsUpdateIntervall) {
        sdsStates = measure;
        debugV("SDS Measure");
      }
      break;
    case measure:
      PmResult pm = sds.queryPm();
      if (pm.isOk()) {
        debugV("SDS Send");
        sendDust(pm.pm25, pm.pm10);
        debugD("PM2.5 = %f", pm.pm25);
        debugD("PM1.0 = %f", pm.pm10);

      } else {
        debugE("SDS Error: %s", pm.statusToString().c_str());
      }
      WorkingStateResult state = sds.sleep();
      if (state.isWorking()) {
        debugE("SDS Error: sleeping");
      } else {
        debugV("SDS Sleep");
        sdsTimer = actMillis;
        sdsStates = sleeping;
      }
      break;
  }
}


void sendHTTPRequest (String requestUrl) {
  WiFiClient client;
  debugD("HTTPReq: %s", requestUrl.c_str());
  if (thingSpeakDustClient.begin(client, requestUrl.c_str())) {  // HTTP
    int httpCode = thingSpeakDustClient.GET();        // start connection and send HTTP header
    if (httpCode > 0) { // httpCode is positiv on success
      // HTTP header has been send and Server response header has been handled
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
        String payload = thingSpeakDustClient.getString();
        debugD("%s", payload.c_str());
      } else {
        debugD("[HTTP] Return: %u",httpCode);
      }
    } else {
      debugE("[HTTP] GET... failed, error: %s\n", thingSpeakDustClient.errorToString(httpCode).c_str());
    }
    thingSpeakDustClient.end();
  } else {
    debugE("[HTTP] Unable to connect\n");
  }
}

String getStringTimeWithMS() {
  String strTime = "";
  strTime += DateTime.now();
  strTime += "000";
  return strTime;
}

void sendDust(float p25, float p10) {
  if (!DateTime.isTimeValid()) {
    return;
  }

  String requestUrl;
  requestUrl = ThingSpeakHost;
  requestUrl += "/update?key=";
  requestUrl += thingSpeakAPIDust;
  requestUrl += "&field1=";
  requestUrl += p25;
  requestUrl += "&field2=";
  requestUrl += p10;
  requestUrl += "&created_at=";
  requestUrl += DateTime.toISOString();
  sendHTTPRequest(requestUrl);

  // Publish a message to "mytopic/test"
  message = "{\"name\":\"GeigerDust\",\"field\":\"Dust\",\"p25\":";
  message += p25;
  message += ",\"p10\":";
  message += p10;
  message += ",\"time\":";
  message += getStringTimeWithMS();
  message += "}";
  debugD("MQTT Publish: %s", message.c_str());
  MQTTClient.publish("sensors", message); // You can activate the retain flag by setting the third parameter to true

}

#define doseRatio 120.0
void sendGeiger(uint32 events, uint32 intervall) {
  String requestUrl;
  float cpm = float(events) / (float(intervall) / 60000000.0);;
  float dose = cpm / doseRatio;

  Debug.printf ("CPM: %f Dose: %f Time: %s\r\n", cpm, dose, DateTime.toISOString().c_str());

  requestUrl = RadmonHost;
  requestUrl += "/radmon.php?function=submit&user=";
  requestUrl += RadMonUser;
  requestUrl += "&password=";
  requestUrl += RadMonPwd;
  requestUrl += "&value=";
  requestUrl += (int)cpm;
  requestUrl += "&unit=CPM";
  sendHTTPRequest(requestUrl);

  requestUrl = ThingSpeakHost;
  requestUrl += "/update?key=";
  requestUrl += thingSpeakAPIGeiger;
  requestUrl += "&field1=";
  requestUrl += cpm;
  requestUrl += "&field2=";
  requestUrl += dose;
  requestUrl += "&field3=";
  requestUrl += WiFi.RSSI();
  requestUrl += "&created_at=";
  requestUrl += DateTime.toISOString();
  sendHTTPRequest(requestUrl);


  // Publish a message to "mytopic/test"
  message = "{\"name\":\"GeigerDust\",\"field\":\"Geiger\",\"dose\":";
  message += dose;
  message += ",\"CPM\":";
  message += cpm;
  message += ",\"time\":";
  message += getStringTimeWithMS();
  message += "}";
  debugD("MQTT Publish: %s", message.c_str());
  MQTTClient.publish("sensors", message); // You can activate the retain flag by setting the third parameter to true

}
