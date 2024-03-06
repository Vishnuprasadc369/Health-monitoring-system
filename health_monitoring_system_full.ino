/***************************************************
  Adafruit MQTT Library ESP8266 Example

  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino

  Works great with Adafruit's Huzzah ESP board & Feather
  ----> https://www.adafruit.com/product/2471
  ----> https://www.adafruit.com/products/2821

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <DHT11.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

MAX30105 particleSensor;
LiquidCrystal_I2C lcd(0x27,16,2);  
static const int RXPin = D7, TXPin = D6;
const int button=D5;
const int BUZZER =D0;
static const uint32_t GPSBaud = 9600;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
byte netflag=0;

TinyGPSPlus gps;
WiFiManager wm;
/************************* WiFi Access Point *********************************/
SoftwareSerial softSerial(RXPin, TXPin);
// #define WLAN_SSID       "hidden_network"
// #define WLAN_PASS       "itsnotzigbee"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME  "IamDheeraj"
#define AIO_KEY       "aio_ZQvD1777iabqkuq80OaJ57BRmjkn"


/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// GPIO where the DS18B20 is connected to
const int oneWireBus = D4;  
DHT11 dht11(D3);//pin for dht11  
OneWire oneWire(oneWireBus); 
DallasTemperature sensors(&oneWire);
// or... use WiFiClientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temp");
Adafruit_MQTT_Publish HUM = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/HUM");
Adafruit_MQTT_Publish BPM = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/BPM");
Adafruit_MQTT_Publish GPSLocation = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/GPSLocation/csv");
// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/onoff");
float speed_mph = 0;
float alltitude = 0;
float lati;     //Storing the Latitude
float longi;    //Storing the Longitude

char gpsdata[120];
/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();

void ICACHE_RAM_ATTR isr() {

 lcd.clear();
    if (!GPSLocation.publish(gpsdata)) {                     //Publish to Adafruit
      Serial.println(F("Failed"));
            lcd.setCursor(3, 0);
      lcd.print("GPS Failed!");
                lcd.setCursor(4, 1);
      lcd.print("offline !");
    } 
    else {
      lcd.setCursor(3, 0);
      lcd.print("GPS Sent!");
      Serial.println(gpsdata);
      Serial.println(F("Sent!"));
      lcd.setCursor(0, 1);
      lcd.print(gpsdata);
  
    }
   
    
    //while (digitalRead(button)==0) {}
  
   
}

void setup() {

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED


  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER,LOW);
  Serial.begin(115200);
    softSerial.begin(GPSBaud);
  delay(10);
    sensors.begin();

  Serial.println(F("Adafruit MQTT demo"));
  lcd.init();   
  lcd.clear();
  lcd.backlight();
  pinMode(button, INPUT);
    lcd.setCursor(0, 0);
  lcd.print("Button:offline");
      lcd.setCursor(0, 1);
  lcd.print("Default:online");

delay(1000);
if(digitalRead(button)==0){
netflag=1;
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("offline mode!");
}
delay(1000);
  lcd.clear();
     
  
if(netflag==0){
      WiFiManager wm;
  // Connect to WiFi access point.
 lcd.setCursor(1, 1);
  lcd.print("IP:"); 
    lcd.print("192.168.4.1");
    bool res;
    // res = wm.autoConnect(); // auto generated AP name from chipid
    // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
    res = wm.autoConnect("AutoConnectAP","password"); // password protected ap
   

    if(!res) {
        Serial.println("Failed to connect");
      
           lcd.setCursor(0,0);
  lcd.print("Failed to connect");

    lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("IP:"); 
    lcd.print(WiFi.localIP()); 
delay(5000);
   //  ESP.restart();
  
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected...yeey :)");
    }
    Serial.println(wm.getWiFiSSID());
    Serial.println(wm.getWiFiPass());
   

  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wm.getWiFiSSID());
  lcd.setCursor(1, 0);
  lcd.print("Connecting to");
  lcd.setCursor(0, 1);
  lcd.print(wm.getWiFiSSID());


  WiFi.begin(wm.getWiFiSSID(),wm.getWiFiPass());
  while (WiFi.status() != WL_CONNECTED) {
 
    Serial.print(".");
    lcd.setCursor(14, 1);
    digitalWrite(BUZZER, HIGH);
    lcd.print("> ");
    delay(500);
    lcd.setCursor(14, 1);
    digitalWrite(BUZZER,LOW);
    lcd.print(" >");
    delay(500);
  }
  lcd.clear();
   lcd.setCursor(5, 0);
  lcd.print("wait");
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); 
  Serial.println(WiFi.localIP());
  connect();

}
  // Setup MQTT subscription for onoff feed.
 attachInterrupt(button, isr,FALLING);
}
void connect() {
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }
 
    if(ret >= 0)
      mqtt.disconnect();
 
    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
  lcd.clear();
}
int print_count=0;
uint32_t x=0;

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  if(netflag==0)
  MQTT_connect();
    getCoordinates();
   int humidity = dht11.readHumidity();

 long irValue ;
  long delta;
  int count=0;
   for (byte x = 0 ; x < RATE_SIZE ; x++)
        rates[x]=0;
 while(count<200){
    
  delta=0;
   irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    
     
    
    }

  }
  count++;
}



  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  // Adafruit_MQTT_Subscribe *subscription;
  // while ((subscription = mqtt.readSubscription(5000))) {
  //   if (subscription == &onoffbutton) {
  //     Serial.print(F("Got: "));
  //     Serial.println((char *)onoffbutton.lastread);
  //   }
  // }
   sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);

  // Now we can publish stuff!
  Serial.print(F("\nSending photocell val "));
  Serial.print(temperatureC);
  Serial.print("...");
  HUM.publish(humidity);
  Serial.print(humidity);
    Serial.println(" %");
 print_count++;
 if(print_count>100)
 print_count=0;
switch (print_count%2) {
case 0:

  lcd.setCursor(0,0);
   lcd.print("temp=");
   lcd.print(temperatureF);
      lcd.print("F");
     lcd.setCursor(0,1);
      lcd.print("humidity=");
   lcd.print(humidity);
   lcd.print("%");

   break;
   case 1:

  lcd.clear();
  lcd.setCursor(0,0);
   lcd.print(" BPM=");
   lcd.print(beatsPerMinute);
     lcd.setCursor(0,1);
   lcd.print(" Avg BPM=");
   lcd.print(beatAvg);
    BPM.publish(beatsPerMinute);

   break;

}







  if (! temp.publish(temperatureF)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!")); 
  }
    delay(2000);
    Serial.print("Lati = ");
  Serial.print(lati,6);
  Serial.print("\tLongi = ");
  Serial.println(longi,6);
  
     
     delay(1000);
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000)
    Serial.print(" No finger?");
beatAvg=0;
beatsPerMinute=0;
  Serial.println();
  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  /*
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
  */
 delay(3000);

}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
void getCoordinates()
{
   readGPSData();
  char *p = gpsdata;
  // add speed value
  dtostrf(speed_mph, 2, 6, p);
  p += strlen(p);
  p[0] = ','; p++;

  // concat latitude
  dtostrf(lati, 2, 6, p);
  p += strlen(p);
  p[0] = ','; p++;

  // concat longitude
  dtostrf(longi, 3, 6, p);
  p += strlen(p);
  p[0] = ','; p++;

  // concat altitude
  dtostrf(alltitude, 2, 6, p);
  p += strlen(p);

  // null terminate
  p[0] = 0;

}

void readGPSData()
{
  if(gps.location.isValid()){
    lati = gps.location.lat();
    longi = gps.location.lng();
    Serial.print("Lati: ");
    Serial.print(lati,6);
    Serial.print("\tLongi: ");
    Serial.println(longi,6);
  }
  waitGPS(1000);
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println("Waiting for data...");
}

static void waitGPS(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (softSerial.available())
      gps.encode(softSerial.read());
  } while (millis() - start < ms);
}


