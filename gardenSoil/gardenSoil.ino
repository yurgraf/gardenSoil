/* This is an initial sketch to be used as a "blueprint" to create apps which can be used with IOTappstory.com infrastructure
  Your code can be filled wherever it is marked.


  Copyright (c) [2016] [Andreas Spiess]

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*/

#define SKETCH "gardenSoil "
#define VERSION "V0.2"
#define FIRMWARE SKETCH VERSION

#define SERIALDEBUG         // Serial is used to present debugging messages 
#define REMOTEDEBUGGING     // UDP is used to transfer debug messages

#define LEDS_INVERSE   // LEDS on = GND

#define MQTT_CLIENT   //add mqtt connectivity
#define SLEEP_MODE    //add sleep mode

#include <credentials.h>
#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <WiFiManager.h>        //https://github.com/kentaylor/WiFiManager
#include <Ticker.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <FS.h>

#ifdef REMOTEDEBUGGING
#include <WiFiUDP.h>
#endif

extern "C" {
#include "user_interface.h" // this is for the RTC memory read/write functions
}

//--------  Sketch Specific -------
#ifdef MQTT_CLIENT
#include <PubSubClient.h>
#endif


// -------- PIN DEFINITIONS ------------------
#ifdef ARDUINO_ESP8266_ESP01           // Generic ESP's 
#define MODEBUTTON 0
#define LEDgreen 13
//#define LEDred 12

#define D0 16 
#define D1 5 
#define D2 4  
#define D4 2
#define D5 14 
#define D6 12 
#define D7 13 
#define D8 15 

#else
#define MODEBUTTON D3
#define LEDgreen D7
//#define LEDred D6
#endif

// --- Sketch Specific -----



//---------- DEFINES for SKETCH ----------
#define STRUCT_CHAR_ARRAY_SIZE 50  // length of config variables
#define MAX_WIFI_RETRIES 50
#define RTCMEMBEGIN 68
#define MAGICBYTE 85

// --- Sketch Specific -----
// #define SERVICENAME "VIRGIN"  // name of the MDNS service used in this group of ESPs


//-------- SERVICES --------------


// --- Sketch Specific -----



//--------- ENUMS AND STRUCTURES  -------------------

typedef struct {
  char ssid[STRUCT_CHAR_ARRAY_SIZE];
  char password[STRUCT_CHAR_ARRAY_SIZE];
  char boardName[STRUCT_CHAR_ARRAY_SIZE];
  char IOTappStory1[STRUCT_CHAR_ARRAY_SIZE];
  char IOTappStoryPHP1[STRUCT_CHAR_ARRAY_SIZE];
  char IOTappStory2[STRUCT_CHAR_ARRAY_SIZE];
  char IOTappStoryPHP2[STRUCT_CHAR_ARRAY_SIZE];
  char automaticUpdate[2];   // right after boot
  // insert NEW CONSTANTS according boardname example HERE!

  char mqtt_server[STRUCT_CHAR_ARRAY_SIZE];
  char mqtt_password[STRUCT_CHAR_ARRAY_SIZE];
  char mqtt_statusTopic[STRUCT_CHAR_ARRAY_SIZE];
  char mqtt_outTopic[STRUCT_CHAR_ARRAY_SIZE];
  char mqtt_inTopic[STRUCT_CHAR_ARRAY_SIZE];

  char sleepMinutes[STRUCT_CHAR_ARRAY_SIZE];

  char magicBytes[4];

} strConfig;

strConfig config = {
  "",
  "",
  "gardenSoil",
  "iotappstory.com",
  "/ota/esp8266-v1.php",
  "iotappstory.com",
  "/ota/esp8266-v1.php",
  "0",
  
  "broker.mqtt-dashboard.com",  //MQTT server
  "",                           //MQTT password
  "garden/soil/status",         //statusTopic
  "garden/soil/out",            //outTopic
  "garden/soil/in",             //inTopic

  "5",
  
  "CFG"  // Magic Bytes
};

// --- Sketch Specific -----



//---------- VARIABLES ----------

unsigned long debugEntry;

String sysMessage;

//++++++++++ SLEEP_MODE varibles ++++++++++
#ifdef SLEEP_MODE
int sleepMin = 5;     //sleep for this number of minutes
long sleepAfter = 30;           //in seconds
long lastSleep = 0;
#endif

// --- Sketch Specific -----
// String xx; // add NEW CONSTANTS for WiFiManager according the variable "boardname"
byte LEDpin;
unsigned long blinkEntry;

//++++++++++ MQTT ++++++++++
#ifdef MQTT_CLIENT
const char* mqtt_server = "broker.mqtt-dashboard.com";

const char* statusTopic = "sensors/plant/test_status";
const char* outTopic = "sensors/plant/test_out";
const char* inTopic = "sensors/plant/test_in";

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
long lastReconnectAttempt = 0;

char msg[50];
int value = 0;
#endif

//---------- FUNCTIONS ----------
// to help the compiler, sometimes, functions have  to be declared here
void loopWiFiManager(void);
void readFullConfiguration(void);
bool readRTCmem(void);
void printRTCmem(void);
void initialize(void);
void sendDebugMessage(void);

void callback(char* topic, byte* payload, unsigned int length);
boolean reconnect();
void read_sensors();
void going2sleep();


//---------- OTHER .H FILES ----------
#include <ESP_Helpers.h>           // General helpers for all IOTappStory sketches
#include "IOTappStoryHelpers.h"    // Sketch specific helpers for all IOTappStory sketches




// ================================== SETUP ================================================

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 5; i++) DEBUG_PRINTLN("");
  DEBUG_PRINTLN("Start "FIRMWARE);


  // ----------- PINS ----------------
  pinMode(MODEBUTTON, INPUT_PULLUP);  // MODEBUTTON as input for Config mode selection

#ifdef LEDgreen
  pinMode(LEDgreen, OUTPUT);
  digitalWrite(LEDgreen, LEDOFF);
#endif
#ifdef LEDred
  pinMode(LEDred, OUTPUT);
  digitalWrite(LEDred, LEDOFF);
#endif

  // --- Sketch Specific -----



  // ------------- INTERRUPTS ----------------------------
  attachInterrupt(MODEBUTTON, ISRbuttonStateChanged, CHANGE);
  blink.detach();

  #ifdef SLEEP_MODE
  // Connect D0 to RST to wake up
    pinMode(D0, WAKEUP_PULLUP);
  #endif



  //------------- LED and DISPLAYS ------------------------
  LEDswitch(GreenBlink);
  
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH


  // --------- BOOT STATISTICS ------------------------
  // read and increase boot statistics (optional)
  readRTCmem();
  rtcMem.bootTimes++;
  writeRTCmem();
  printRTCmem();


  //---------- SELECT BOARD MODE -----------------------------

  system_rtc_mem_read(RTCMEMBEGIN + 100, &boardMode, 1);   // Read the "boardMode" flag RTC memory to decide, if to go to config
  if (boardMode == 'C') configESP();

  readFullConfiguration();

  // --------- START WIFI --------------------------

  connectNetwork();

  sendSysLogMessage(2, 1, config.boardName, FIRMWARE, 10, counter++, "------------- Normal Mode -------------------");

  if (atoi(config.automaticUpdate) == 1) IOTappStory(false);  // replace false with true if you want tu update the SPIFFS, too

  // ---------- START MQTT -----------------------
  
  #ifdef MQTT_CLIENT
    client.setServer(config.mqtt_server, 1883);
    client.setCallback(callback);
  #endif

  // ----------- SPECIFIC SETUP CODE ----------------------------

  // ----------- SPECIFIC SETUP CODE ----------------------------

  String sleepMinutes(config.sleepMinutes);
  sleepMin = sleepMinutes.toInt();
  Serial.print("Slipping time: ");
  Serial.println(sleepMin);


  // ----------- END SPECIFIC SETUP CODE ----------------------------

  LEDswitch(None);
  pinMode(MODEBUTTON, INPUT_PULLUP);  // MODEBUTTON as input for Config mode selection

 sendSysLogMessage(7, 1, config.boardName, FIRMWARE, 10, counter++, "Setup done");
}




//======================= MY FUNCTIONS =======================

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }
}

boolean reconnect() {
  if (client.connect("arduinoClient")) {
    // Once connected, publish an announcement...
    client.publish(config.mqtt_statusTopic, "Connected");
    // ... and resubscribe
    client.subscribe(config.mqtt_inTopic);
  }
  return client.connected();
}

void read_sensors(){
  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    snprintf (msg, 50, "#%ld", value);
    client.publish(config.mqtt_outTopic, msg);
  }
}

void going2sleep(){
  long now = millis();
  if (now > (sleepAfter * 1000) ) {
    lastSleep = now;
    // convert to microseconds
    client.publish(config.mqtt_statusTopic, "Sleeping");
    client.loop();
     Serial.print("Going to sleep for ");
     Serial.print(sleepMin);
     Serial.println(" minutes\n  . . .\n   . .\n    .\n");
    ESP.deepSleep(sleepMin *60 * 1000000);
  }
}



//======================= LOOP =========================
void loop() {
  //-------- IOTappStory Block ---------------
  yield();
  handleModeButton();   // this routine handles the reaction of the Flash button. If short press: update of skethc, long press: Configuration

  // Normal blind (1 sec): Connecting to network
  // fast blink: Configuration mode. Please connect to ESP network
  // Slow Blink: IOTappStore Update in progress

  if (millis() - debugEntry > 5000) { // Non-Blocking second counter
    debugEntry = millis();
    sendDebugMessage();
  }


  //-------- Your Sketch ---------------


  //++++++++++ MQTT ++++++++++
  
#ifdef MQTT_CLIENT
    if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
        Serial.println("MQTT not connected, try again in 5 seconds");
      }
    }
  } else {
    // Client connected
    read_sensors();
    client.loop();
  }
#endif

//++++++++++ Sleep mode ++++++++++

#ifdef SLEEP_MODE
  going2sleep();
#endif

}
//------------------------- END LOOP --------------------------------------------

void sendDebugMessage() {
  // ------- Syslog Message --------

  /* severity: 2 critical, 6 info, 7 debug
    facility: 1 user level
    String hostName: Board Name
    app: FIRMWARE
    procID: unddefined
    msgID: counter
    message: Your message
  */

  sysMessage = "";
  long h1 = ESP.getFreeHeap();
  sysMessage += " Heap ";
  sysMessage += h1;
  sendSysLogMessage(6, 1, config.boardName, FIRMWARE, 10, counter++, sysMessage);
}


bool readRTCmem() {
  bool ret = true;
  system_rtc_mem_read(RTCMEMBEGIN, &rtcMem, sizeof(rtcMem));
  if (rtcMem.markerFlag != MAGICBYTE) {
    rtcMem.markerFlag = MAGICBYTE;
    rtcMem.bootTimes = 0;
    system_rtc_mem_write(RTCMEMBEGIN, &rtcMem, sizeof(rtcMem));
    ret = false;
  }
  return ret;
}

void printRTCmem() {
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("rtcMem ");
  DEBUG_PRINT("markerFlag ");
  DEBUG_PRINTLN(rtcMem.markerFlag);
  DEBUG_PRINT("bootTimes ");
  DEBUG_PRINTLN(rtcMem.bootTimes);
}


