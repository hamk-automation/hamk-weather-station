#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADS1015.h>
#include <TSL2561.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoOTA.h>
#include <Ticker.h>

#define SERIAL_DEBUG 0
#define SERIAL_SPEED 115200 // Baud rate of Serial communication
#define DAQ_INTERVAL 10

Adafruit_BME280 bme; // Barometer
TSL2561 tsl(TSL2561_ADDR_FLOAT); // Light sensor
Adafruit_ADS1115 ads; // ADC for windspeed sensor
WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);

float vMeas = 0;

uint16_t dirValue;
float rValues [] = {
  688,
  891,
  1000,
  1410,
  2200,
  3140,
  3900,
  6570,
  8200,
  14120,
  16000,
  21880,
  33000,
  42120,
  64900,
  120000
};
float dValues [] = {
  112.5,
  67.5,
  90,
  157.5,
  135,
  202.5,
  180,
  22.5,
  45,
  247.5,
  225,
  337.5,
  0,
  292.5,
  315,
  270
};

float vDir[16];
volatile unsigned long windCount = 0, rainCount = 0;
float windspeed = 0.0;
float rain = 0.0;


// Read sensor values
float p, t, h, l, lum_v, lum_ir, lum_full;

uint32_t lum;
uint16_t ir, full;

int16_t adc0;

// Wifi
const char* wifi_ssid = "KonttiGateway";
const char* wifi_password = "DankAssWeed";
const char* mqtt_server = "192.168.0.20"; // MQTT broker's IP address
const char* clientId = "ESP8266_HAMK_WeatherStation";

// Ticker scheduler
Ticker DAQTimer, rainTimer;
bool daq_flag = true;

void setup() {
  initialize();
  runDAQ();
  DAQTimer.attach(DAQ_INTERVAL,setDAQ);
  rainTimer.attach(86400,resetRainValue);
#if SERIAL_DEBUG
  //Serial.println("Entering sleep mode");
  //Serial.println(micros());
#endif
  //if (micros() > DAQ_INTERVAL * 1000000) ESP.deepSleep(1000);
  //else ESP.deepSleep((DAQ_INTERVAL * 1000000) - micros());
}

void loop() {
  if (daq_flag) runDAQ();
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(wifi_ssid, wifi_password);
    delay(100);
  }
  ArduinoOTA.handle();
}

void initialize()
{
  bme.begin(0x77);
  delay(20);
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::FILTER_OFF,
                  Adafruit_BME280::STANDBY_MS_1000);
  tsl.begin();
  tsl.setGain(TSL2561_GAIN_0X);
  tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);
  ads.begin();
  ads.setGain(GAIN_ONE); // 2x gain: Voltage range: +/- 2.048V;  Voltage resolution: 1 bit = 0.0625mV
#if SERIAL_DEBUG
  Serial.begin(SERIAL_SPEED);
#endif
  twi_setClock(600000);
  initWifi();
  mqtt_client.setServer(mqtt_server, 1883); // Connect to broker at port 1883
  mqtt_client.connect(clientId);
  #if SERIAL_DEBUG
  Serial.println("values");
  #endif
  for (int i = 0; i < 16 ; i++) {
    vDir[i] = (3300 / 0.125) * rValues[i] / (rValues[i] + 10000.0);
    #if SERIAL_DEBUG
    Serial.println(vDir[i]);
    #endif

  }
  ArduinoOTA.onStart([]() {
    detachInterrupt(D6);
    detachInterrupt(D7);
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    #if SERIAL_DEBUG
    Serial.println("Start updating " + type);
    #endif
  });
  ArduinoOTA.onEnd([]() {
    #if SERIAL_DEBUG
    Serial.println("\nEnd");
    #endif
    attachInterrupt(D6, windspeedInterrupt, FALLING);
    attachInterrupt(D7, rainInterrupt, FALLING);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    #if SERIAL_DEBUG
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    #endif

  });
  ArduinoOTA.onError([](ota_error_t error) {
    #if SERIAL_DEBUG
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    #endif
    attachInterrupt(D6, windspeedInterrupt, FALLING);
    attachInterrupt(D7, rainInterrupt, FALLING);
  });
  ArduinoOTA.begin();
  digitalWrite(BUILTIN_LED, LOW);
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(D6, INPUT_PULLUP);
  pinMode(D7, INPUT_PULLUP);
  attachInterrupt(D6, windspeedInterrupt, FALLING);
  attachInterrupt(D7, rainInterrupt, FALLING);

}

// Connect to wifi
void initWifi() {
#if SERIAL_DEBUG
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);
#endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    yield();
#if SERIAL_DEBUG
    Serial.print(WiFi.status());
    Serial.print(".");
    Serial.println("");
#endif
  }
#if SERIAL_DEBUG
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
#endif
}

void runDAQ()
{
  windspeed = 2 * windCount / (3.0*DAQ_INTERVAL); //km/h
  windCount = 0;
  rain = rainCount*0.2794;
  readWindDir();
  // Read sensor values
  p = bme.readPressure(); // bar
  t = bme.readTemperature();
  h = bme.readHumidity();
  getTSLValue();
  pushMqtt();
  daq_flag = false;
}

inline void pushMqtt() {
  long rssi = WiFi.RSSI();
  static char result_str[128] = "";
  sprintf(result_str,
    "{\"p\":%4.2f,"
    "\"t\":%4.2f,"
    "\"rh\":%4.2f,"
    "\"lux\":%4.2f,"
    "\"irc\":%u,"
    "\"vic\":%u,"
    "\"wm2\":%4.2f,"
    "\"wspeed\":%4.2f,"
    "\"dir\":%4.1f,"
    "\"rain\":%4.2f,"
    "\"rssi\":%ld"
  "}", p, t, h, l, ir, full - ir, lum_full, windspeed, dValues[dirValue], rain, rssi);
#if SERIAL_DEBUG
  Serial.println(result_str);
#endif
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(wifi_ssid, wifi_password);
    delay(100);
  }
    // Connect to MQTT
    while (mqtt_client.state()) {
      //while (0) {
#if SERIAL_DEBUG
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      mqtt_client.connect(clientId);
#endif
#if SERIAL_DEBUG
      Serial.print("Attempting MQTT connection...");
#endif
    }
#if SERIAL_DEBUG
    Serial.println("connected");
#endif
    mqtt_client.publish("weather", result_str);
    mqtt_client.loop();
}

inline void getTSLValue() {
  lum = tsl.getFullLuminosity();
  ir = lum >> 16;
  full = lum & 0xFFFF;
  lum_full = full/10000.0;
  l = tsl.calculateLux(full, ir);
}

void windspeedInterrupt() {
  windCount++;
}

void rainInterrupt() {
  rainCount++; // mm
}

void readWindDir() {
  int cmp;
  adc0 = ads.readADC_SingleEnded(0);
  vMeas = adc0 * 0.125;
  cmp = adc0 / 10;
  if ( cmp < 192) dirValue = 0;
  else if (cmp < 227) dirValue = 1;
  else if (cmp < 282) dirValue = 2;
  else if (cmp < 400) dirValue = 3;
  else if (cmp < 552) dirValue = 4;
  else if (cmp < 685) dirValue = 5;
  else if (cmp < 893) dirValue = 6;
  else if (cmp < 1117) dirValue = 7;
  else if (cmp < 1367) dirValue = 8;
  else if (cmp < 1585) dirValue = 9;
  else if (cmp < 1717) dirValue = 10;
  else if (cmp < 1918) dirValue = 11;
  else if (cmp < 2079) dirValue = 12;
  else if (cmp < 2210) dirValue = 13;
  else if (cmp < 2361) dirValue = 14;
  else dirValue = 15;
}

void resetRainValue() {
  rainCount = 0;
}

void setDAQ() {
  daq_flag = true;
}
