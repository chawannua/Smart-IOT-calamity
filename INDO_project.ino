#include <Wire.h>
#include "LiquidCrystal_I2C.h"
#include <SPI.h>
#include <LoRa.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

/*************************************************************

  This example shows how value can be pushed from Arduino to
  the Blynk App.

  WARNING :
  For this example you'll need Adafruit DHT sensor libraries:
    https://github.com/adafruit/Adafruit_Sensor
    https://github.com/adafruit/DHT-sensor-library

  App project setup:
    Value Display widget attached to V5
    Value Display widget attached to V6
 *************************************************************/
#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <WiFiClient.h>
//#include <TridentTD_LineNotify.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <Wire.h>
SoftwareSerial mySerial(16, 17);  // RX, TX
unsigned int pm1 = 0;
unsigned int pm2_5 = 0;
unsigned int pm10 = 0;
float t = 0;
float h = 0;


#define RXD2 16
#define TXD2 17


// ESP32 --> Pantower PMS7003
// 26    --> RX
// 25    --> TX
// GND   -->

// Template ID, Device Name and Auth Token are provided by the Blynk.Cloud
// See the Device Info tab, or Template settings
//#define BLYNK_TEMPLATE_ID           "TMPLxxxxxx"
//#define BLYNK_DEVICE_NAME           "Device"
#define BLYNK_AUTH_TOKEN "ugC8Vv96e6aL_eln-X4zSRnP4_Lw19t8"
// Comment this out to disable prints and save space
char auth[] = BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.
#define ssid "Space AC"    // ชื่อ Wifi
#define pass "0632317613"  // รหัส Wifi

#define DHTPIN 26  // What digital pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11     // DHT 11
#define DHTTYPE DHT22  // DHT 22, AM2302, AM2321
//#define DHTTYPE DHT21   // DHT 21, AM2301

DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;

// This function sends Arduino's up time every second to Virtual Pin (5).
// In the app, Widget's reading frequency should be set to PUSH. This means
// that you define how often to send data to Blynk App.

//**************************************************
void sendSensor1() {
  int index = 0;
  char value;
  char previousValue;

  while (mySerial.available()) {
    value = mySerial.read();
    if ((index == 0 && value != 0x42) || (index == 1 && value != 0x4d)) {
      Serial.println("Cannot find the data header.");
      break;
    }

    if (index == 4 || index == 6 || index == 8 || index == 10 || index == 12 || index == 14) {
      previousValue = value;
    } else if (index == 5) {
      pm1 = 256 * previousValue + value;
      Serial.print("{ ");
      Serial.print("\"pm1\": ");
      Serial.print(pm1);
      Serial.print(" ug/m3");
      Serial.print(", ");
    } else if (index == 7) {
      pm2_5 = 256 * previousValue + value;
      Serial.print("\"pm2_5\": ");
      Serial.print(pm2_5);
      Serial.print(" ug/m3");
      Serial.print(", ");
    } else if (index == 9) {
      pm10 = 256 * previousValue + value;
      Serial.print("\"pm10\": ");
      Serial.print(pm10);
      Serial.print(" ug/m3");
    } else if (index > 15) {
      break;
    }
    index++;
  }
  while (mySerial.available()) mySerial.read();
  Serial.println(" }");
  Blynk.virtualWrite(V0, pm1);    // ทำ virtual ที่ v5 โดยรับค่าความชืน
  Blynk.virtualWrite(V1, pm2_5);  // ทำ virtual ที่ v6  โดยรับค่าความอุณหภูมิ
  Blynk.virtualWrite(V2, pm10);
  delay(1000);
  //*********************************
  //***********************************
  h = dht.readTemperature();
  t = dht.readTemperature();  // or dht.readTemperature(true) for Fahrenheit

  if (isnan(h) || isnan(t)) {
    // Serial.println("Failed to read from DHT sensor!");
    return;
  }
  lcd.setCursor(3, 0);
  lcd.print(h);
  lcd.setCursor(1, 0);
  lcd.print("H:");
  lcd.setCursor(8, 0);
  lcd.print("%");
  //////////////////////////////////////////////////
  lcd.setCursor(12, 0);
  lcd.print(t);
  lcd.setCursor(10, 0);
  lcd.print("T:");
  lcd.setCursor(17, 0);
  lcd.print("C");
  //////////////////////////////////////////////////
  lcd.setCursor(6, 1);
  lcd.print(pm1);
  lcd.setCursor(2, 1);
  lcd.print("pm1=");
  //////////////////////////////////////////////////
  lcd.setCursor(16, 1);
  lcd.print(pm2_5);
  lcd.setCursor(10, 1);
  lcd.print("pm2.5=");
  //////////////////////////////////////////////////
  lcd.setCursor(8, 2);
  lcd.print(pm10);
  lcd.setCursor(3, 2);
  lcd.print("pm10=");
  //////////////////////////////////////////////////

  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V5, h);
  Blynk.virtualWrite(V6, t);
  //***************************************************
  //**********************************************
}
//******************************************
int vibration;
void sendSensor2() {
  vibration = analogRead(35);
  delay(100);
  Blynk.virtualWrite(V7, vibration);
  Serial.println(vibration);
}

//*********************************************
int carbon;
void sendSensor3() {
  carbon = analogRead(34);
  delay(50);
  Blynk.virtualWrite(V8, carbon);
  Serial.println(carbon);
}
//*********************************************8



////////////////////////////////////////////////////////////////////////////

#include <TridentTD_LineNotify.h>
#define LINE_TOKEN "EPGdguCojJ8ma0uGF3DIQOAUFuRfDTM5UcwkLVHMEUc"
#define TX 1
#define RX 3


void setup() {
  //initialize Serial Monitor
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("LoRa Receiver");

  //setup LoRa transceiver module
  LoRa.setPins(TX,RX);

  //replace the LoRa.begin(---E-) argument with your location's frequency
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(433E6)) {
    Serial.println(".");
    delay(500);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
  
// Debug console
  {
  Serial.begin(9600);
  while (!Serial)
    ;
  mySerial.begin(9600);

  //  Serial.begin(9600);
  //  SerialPMS.begin(9600, SERIAL_8N1, RXD2, TXD2);
  //  pms.passiveMode();
  // Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  // Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  // Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
  Blynk.begin(auth, ssid, pass, "blynk.iot-cm.com", 8080);
  lcd.begin();
  lcd.backlight();
  dht.begin();

  // Setup a function to be called every second
  timer.setInterval(1000L, sendSensor1);
  timer.setInterval(1500L, sendSensor2);
  timer.setInterval(2000L, sendSensor3);
  }

//  timer.setInterval(4000L, sendSensor4);
/////////////////////////////////////////////////////////////////////////////////////////
 {
  Serial.begin(115200);
  Serial.println(LINE.getVersion());
  WiFi.begin(ssid, pass);
  Serial.printf("WiFi connecting ", ssid);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.printf("\nWiFi connected\nIP : ");
  Serial.println(WiFi.localIP());
  LINE.setToken(LINE_TOKEN);
 }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      Serial.print(LoRaData);
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
  if (t > 40) {
    LINE.notify("Caution: Over Temperature ");
    LINE.notifySticker(4, 274);
    Serial.print("Temperature Warning");
    delay(30000);
  }

  if (vibration > 2) {
    LINE.notify("Warning: Earthquake");
    Serial.print("Earthquake Warning");
    delay(30000);
  }
  if (carbon > 1500) {
    LINE.notify("Caution: Over Carbon Dioxide ");
    Serial.print("Carbon Dioxide Warning");
    delay(30000);
  }
  if (pm2_5 > 100) {
    LINE.notify("Caution: High dust");
    Serial.print("High dust Warning");
    delay(30000);
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
  Blynk.run();
  timer.run();
  LINE.notify("\nTemperature : " + String(t) + "°C\n" + "Humidity : " + String(h) + "%\n" + "Vibration : " + String(vibration) + " richter\n" + "Carbon Dioxide : " + String(carbon) + " ppm\n" + "PM2.5 : " + String(pm2_5) + " AQI");
  Serial.print("Send normal data");
  delay(60000);
}