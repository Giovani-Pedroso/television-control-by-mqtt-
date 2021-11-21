//addicion of the libraries
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <Arduino.h>
#include <IRrecv.h>
#include <IRac.h>
#include <IRutils.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

//Wifi conffiguration
#define WLAN_SSID       "" //Wifi name
#define WLAN_PASS       "" //wifi password

//MQTT configuration
#define AIO_SERVER      "io.adafruit.command"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    ""//
#define AIO_KEY         ""

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Subscribe A = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME"/feeds/devicerelho"); // FeedName
Adafruit_MQTT_Subscribe C = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/commandando");
Adafruit_MQTT_Subscribe P = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/programar");




long lastMsg = 0;
const uint16_t kIrLed = 12;
char msg[50];
int value = 0;
int device = 0 ;
unsigned long int command = 1;
bool conf = 0, lock = 0;
int green = 5, red = 4, blue = 16, ent = 0;
unsigned long int data;
const uint16_t kRecvPin = 14;
const uint32_t kBaudRate = 115200;
const uint16_t kCaptureBufferSize = 1024;
#if DECODE_AC
const uint8_t kTimeout = 50;
#else   // DECODE_AC
const uint8_t kTimeout = 15;
#endif  // DECODE_AC

IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);
decode_results results;
IRsend irsend(12);


void EEPROMWritelong(int address, long value);
long int EEPROMReadlong(long address);

void sendCommand(int command, int device);
void recordCommand(int command, int device);

void MQTT_connect();




void setup() {

  //peripherals initialization
  EEPROM.begin(4096);
  Serial.begin(115200);
  irsend.begin();

  int i = 1;
  int j = 5;

  //Debug leds configuration
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(red, OUTPUT);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  digitalWrite(blue, 1);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  digitalWrite(blue, 0);
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&A);
  mqtt.subscribe(&C);
  mqtt.subscribe(&P);




  //*****************************************************************************
#if defined(ESP8266)
  Serial.begin(kBaudRate, SERIAL_8N1, SERIAL_TX_ONLY);
#else  // ESP8266
  Serial.begin(kBaudRate, SERIAL_8N1);
#endif  // ESP8266
  while (!Serial)  // Wait for the serial connection to be establised.
    delay(50);
  Serial.printf("\nIRrecvDumpV2 is now running and waiting for IR input on Pin "
                "%d\n", kRecvPin);
#if DECODE_HASH
#endif                  // DECODE_HASH
  irrecv.enableIRIn();  // Start the receiver

  //**********************************************************************************
#if ESP8266

  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
#else  // ESP8266
  Serial.begin(115200, SERIAL_8N1);
#endif  // ESP8266
}


void loop() {

  
  MQTT_connect();


  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(20000))) {


    
    if (subscription == &A) {
      Serial.print(F("Got: "));
      Serial.println((char *)A.lastread);
      device = atoi((char *)A.lastread);
      device = device * 19;
    }
    
    if (subscription == &C) {
      Serial.print(F("Got: "));
      Serial.println((char *)C.lastread);
      command = atoi((char *)C.lastread);
      if (conf == 0)sendCommand(command, device);
      if (conf == 1)recordCommand(command, device);
    }

    if (subscription == &P) {
      Serial.print(F("Got: "));
      Serial.println((char *)P.lastread);
      conf = atoi((char *)P.lastread);


    }
  }


}

void MQTT_connect() {
  int8_t ret;

  // Stop if is already connected.
  if (mqtt.connected()) {
    return;
  }
  
  digitalWrite(red, 1);
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
  digitalWrite(red, 0);

}


//-------------------------------------------Functions to manipulate the eeprom memory
void EEPROMWritelong(int address, long value)
{
  Serial.println("Write");
  byte four = (value & 0xFF);

  byte three = ((value >> 8) & 0xFF);

  byte two = ((value >> 16) & 0xFF);

  byte one = ((value >> 24) & 0xFF);


  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
  EEPROM.commit();
}


long int EEPROMReadlong(long address)
{ //Read the 4 bytes from the eeprom memory.
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);

  //Return the recommandposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

//-------------------------------------------Functions to send and record the NEC commandmands

void sendCommand(int command, int device) {
  command = command--;
  command = command * 5;
  device = device--;
  device = device * 56;
  data = EEPROMReadlong(command + device);

  Serial.print("enviando data =");
  Serial.println(data, HEX);
  irsend.sendNEC(data);
  digitalWrite(green, 1);
  delay(100);
  digitalWrite(green, 0);
  digitalWrite(blue, 1);
  delay(100);
  digitalWrite(blue, 0);
}

void recordCommand(int command, int device) {

  Serial.println("gravando");
  command = command--;
  command = command * 5;
  device = device--;
  device = device * 56;
  lock = 1;
  digitalWrite(green, 1);
  while (lock == 1) {
    if (irrecv.decode(&results) ) {

      Serial.println("recebeu");
      data = results.value;
      Serial.println("datas = ");
      Serial.println(data, HEX);
      lock = 0;
      Serial.print("lock =");
      Serial.println(lock);
      delay(50);


    }
    yield();

  }
  digitalWrite(green, 0);
  EEPROMWritelong(command + device, data);
  long int qw =  EEPROMReadlong(command + device);
  Serial.print("na memoria");
  Serial.println(qw, HEX);
}
