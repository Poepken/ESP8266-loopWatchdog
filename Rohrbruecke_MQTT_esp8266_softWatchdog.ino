
/*************************************************
 * This drives a temperature monitor with 3 Sensors on an ESP8266, sending data to MQTT
 * * 2x Temperature Sensor DS18B20 - libs: OneWire and DallasTemperature
 * * 1x DHT22 Temp / Humidity - libs: DHTesp
 * * MQTT libs: PubSubClient
 * Settings for the IDE (Amica 2.0 Board):  NodeMCU 1.0 (ESP12E Module)
 * To enable debug messages to serial, uncomment line after end of includes
 ************************************************/
//for the ds18b20
#include <OneWire.h>
#include <DallasTemperature.h>

//for WiFi
#include <ESP8266WiFi.h>

//for DHT Sensor
#include <DHTesp.h>

//for MQTT
#include <PubSubClient.h>
const char* topicRohrInnen = "/rohrbruecke/temperatur/innen";
const char* topicRohrAussen = "/rohrbruecke/temperatur/aussen";
const char* topicBalkonTemp = "/balkon/temperatur";
const char* topicBalkonHumid = "/balkon/luftfeuchte";
const char* topicDebug = "/balkon/debug";
const char* MQTT_name = "ESP8266-Rohrbruecke"; // this name must be unique, otherwise connection fails

// for loop watchdog timer, see https://github.com/esp8266/Arduino/issues/1532
#include <Ticker.h>
Ticker tickerOSWatch;

#include "private.h"
// includes ssid, password for wifi as well as MQTT_BROKER, the IP of the broker, all as const char*
// include in "" makes the IDE look in the local folder first, and only secondly in the library folder

/* end of includes */

//#define DEBUG_SERIAL // uncomment for serial debug output. Comment out for production use as serial connection eats resources!

/* Set Up One Wire */
// Data Line of the DS18B20 is connected to nodemcu GPIO 5 or D1
#define ONE_WIRE_BUS 5

// Setting a one wire instance
OneWire oneWire(ONE_WIRE_BUS);
// Passing onewire instance to Dallas Temperature sensor library
DallasTemperature sensors(&oneWire);

// fixed one wire addresses - much faster than probing them by Index!
DeviceAddress insideT = {0x28, 0xF6, 0xB4, 0xC6, 0x1B, 0x13, 0x01, 0xD2}; // sensor 1
DeviceAddress outsideT = {0x28, 0xF7, 0x0F, 0xCA, 0x1B, 0x13, 0x01, 0xC8}; // sensor 2
// OneWire temperature storage
float Celsius1=-100;
float Celsius2=-100;


/*  Set up DHT22 Sensor*/
#define DHTPIN 4     // what digital pin the DHT22 is conected to
DHTesp dht;
// variables to store the temperatures
float CelsiusDHT22=-100;
float HumidityDHT22=0;
// Reading temperature and humidity takes about 250 milliseconds!
// Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
TempAndHumidity lastValues;


/* timing variables */
// for software watchdog timer
#define OSWATCH_RESET_TIME 60 // if stuck in loop for 60 seconds, trigger reset
static unsigned long last_loop;
// for reading the temperature
long interval = 29292; // interval to update temperatures in milliseconds
unsigned long previousMillis = 0; // 
// for updating MQTT (longer than before = multiple attempts possible)
long intervalMQTT = 120000; // interval to update MQTT in milliseconds
unsigned long previousMillisMQTT = 0; // 

/* WiFi variables */
int connectionAttemptCount = 0;

// WiFi Client for MQTT
WiFiClient espClient;
PubSubClient mqtt_client(espClient);


/* this routine serves to connect / re-connect to the WiFi. 
 *  The connection sometimes drops!
 */
  
void connect_WiFi() {
  connectionAttemptCount++;
#ifdef DEBUG_SERIAL
  Serial.print("WiFi:Connecting to ");
  Serial.print(ssid);
  Serial.print(" - connection attempt #");
  Serial.println(connectionAttemptCount);
#endif
  WiFi.begin(ssid, password);
  delay(100);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
#ifdef DEBUG_SERIAL
    Serial.print(".");
#endif
  }
#ifdef DEBUG_SERIAL
  Serial.println("");
  Serial.println("WiFi connected");

  // Print the IP address on serial monitor
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
#endif
}

/* this routine serves to connect / re-connect to the MQTT Server. 
 *  The connection sometimes drops!
 */
void connect_MQTT() {
    while (!mqtt_client.connected()) {
        yield(); // testing if this helps preventing random connection drops
#ifdef DEBUG_SERIAL
        Serial.print("Connecting to MQTT...");
#endif
        if (!mqtt_client.connect(MQTT_name)) { 
#ifdef DEBUG_SERIAL
            Serial.print("failed, rc=");
            Serial.print(mqtt_client.state());
            Serial.println(" retrying in 3 seconds");
#endif
            delay(3000);
        }
#ifdef DEBUG_SERIAL
        else Serial.println(" success!");
#endif
    }
}


/* callback function for timer checking whether the ESP is stuck somewhere (loop watchdog timer)
 * 
 */
void ICACHE_RAM_ATTR osWatch(void) {
    unsigned long t = millis();
    unsigned long last_run = abs(t - last_loop);
    if(last_run >= (OSWATCH_RESET_TIME * 1000)) {
      // save the hit here to eeprom or to rtc memory if needed
        ESP.restart();  // normal reboot -- does not delete variable data like connection cou
        //ESP.reset();  // hard reset
    }
}

/*******
 * Reading the Temperature Sensors and storing in global variables
 */

void updateTemperatures() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) { //only in certain intervals
    previousMillis = currentMillis;  // Remember the time

#ifdef DEBUG_SERIAL
    Serial.print(" Updating Temperatures ....");
#endif

    //OneWire -- no connection gives -127 deg C
    sensors.requestTemperatures(); //Call all sensors on one wire to start calculating the temperature readings
    // fast with direct addresses stored in insideT and outside T
    float oneWireT;
    oneWireT = sensors.getTempC(insideT);
    if (oneWireT > -100) {
      Celsius1=oneWireT;
    }
    oneWireT = sensors.getTempC(outsideT);
    if (oneWireT > -100) {
      Celsius2=oneWireT;
    }

#ifdef DEBUG_SERIAL
    Serial.print(" One Wire done....");
#endif

    //DHT22
    lastValues = dht.getTempAndHumidity();
    CelsiusDHT22=lastValues.temperature;
    HumidityDHT22=lastValues.humidity;
#ifdef DEBUG_SERIAL
    Serial.println(" DTH done.");
#endif
    
  }
}

/*******
 * Sending the data to the MQTT Server
 */

void mqtt_send_data() {
  unsigned long currentMillis = millis();
  char msg[10];
  if (currentMillis - previousMillisMQTT >= intervalMQTT) { //only in certain intervals
    previousMillisMQTT = currentMillis;  // Remember the time
    //send all data to MQTT server
#ifdef DEBUG_SERIAL
    Serial.println("... publishing data to MQTT:");
#endif
    sprintf (msg, "%5d", connectionAttemptCount);
      mqtt_client.publish(topicDebug, msg);
#ifdef DEBUG_SERIAL
      Serial.print("Connection Attempts:   ");
      Serial.println(msg);
#endif    
    if (Celsius1 > -99) {
      sprintf (msg, "%5.2f", Celsius1);
      mqtt_client.publish(topicRohrInnen, msg);
#ifdef DEBUG_SERIAL
      Serial.print("Temp. innen:   ");
      Serial.println(msg);
#endif
    } else {
#ifdef DEBUG_SERIAL
      Serial.println("Celsius 1 below range. Likely disconnected.");
#endif
    }
    if (Celsius2 > -99){
      sprintf (msg, "%5.2f", Celsius2);
      mqtt_client.publish(topicRohrAussen, msg);
#ifdef DEBUG_SERIAL
      Serial.print("Temp. aussen:  ");
      Serial.println(msg);
#endif
    } else {
#ifdef DEBUG_SERIAL
      Serial.println("Celsius 2 below range. Likely disconnected.");
#endif
    }
    sprintf (msg, "%5.2f", CelsiusDHT22);
    mqtt_client.publish(topicBalkonTemp, msg);
#ifdef DEBUG_SERIAL
    Serial.print("Temp. Balkon:  ");
    Serial.println(msg);
#endif
    sprintf (msg, "%5.2f", HumidityDHT22);
    mqtt_client.publish(topicBalkonHumid, msg);
#ifdef DEBUG_SERIAL
    Serial.print("Feuchte Balkon:");
    Serial.println(msg);
#endif  
  }
}

/***************************************************************************
 * SETUP
 **************************************************************************/

void setup(){ 

  // for loop watchdog timer
  last_loop = millis();
  tickerOSWatch.attach_ms(((OSWATCH_RESET_TIME / 3) * 1000), osWatch);



#ifdef DEBUG_SERIAL
  Serial.begin(115200);           // start up serial communication
  delay(10);
#endif

  dht.setup(DHTPIN, DHTesp::DHT22); // start DHT22
  delay(10);
  
  sensors.begin();                // init the DallasTemperature sensors
  delay(10);
#ifdef DEBUG_SERIAL
  Serial.println("Startup: Sensors connected.");
  Serial.println("Startup: Connecting WiFi now.");
#endif
  // connect to WiFi
  connect_WiFi();  

  //set MQTT server
  mqtt_client.setServer(MQTT_BROKER, 1883);
  
}



/***************************************************************************
 * L O O P
 **************************************************************************/
void loop(){ 

  // feed the loop watchdog timer
  last_loop = millis();
  
  // collect sensor data
  updateTemperatures();

  // check if WiFi still connected
  if (WiFi.status() != WL_CONNECTED) {
    delay(10);
    connect_WiFi();  
    return;
  }

  if (!mqtt_client.connected()) {
      connect_MQTT();
  }
  mqtt_client.loop();
 
  mqtt_send_data();

}
