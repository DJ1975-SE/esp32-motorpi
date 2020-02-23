/*
 * Connect to Wifi and set time
 * 
 * display time and rssi on SSD106 I2C display
 * 
 * Read temperature from DS18B20 via w1
 * Read temperature from MAX6675 via SPI
 * 
 * Write data back when wlan available
 * 
 * Scanning...
 * I2C device found at address 0x3C - display
 * I2C device found at address 0x68 - gyrometer MPU6050
 * I2C device found at address 0x76 - bme280
 * 
 * 
 * 
 * 
 * 
 * ToDo: 
 * Break out sensor reading in own thread
 * Write multiple
 */

#define DEVICE "ESP32"

#include <WiFi.h>
#include "time.h"
#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"
#include <InfluxDbClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <LinkedList.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>

// should be adjustable with a potentiometer
const int   loopWait = 1000;

static const int spiClk = 1000000; // 1 MHz

struct Measurement {
  int rssi;
  unsigned int freeheap;
  unsigned int listlength;
  float w1_tempC;
  float spi_tempC;
  unsigned int rpm;
  float bme280_airpressure;
  float bme280_humidity;
  float bme280_temperature;
  float bme280_altitude;
  float mpu6050_accel_x;
  float mpu6050_accel_y;
  float mpu6050_accel_z;
  float mpu6050_gyro_x;
  float mpu6050_gyro_y;
  float mpu6050_gyro_z;  
  float mpu6050_temperature;
  unsigned int analog0;
  // We assume 0-5 volt
  float afrvoltage0;
  unsigned int sensorreadtime;
  wl_status_t wlanstatus;
  time_t timestamp;
};

// We simulate data for RPM
// Simulation of AFR by means of outputting analogue

#define RPM_MAX 12000
#define RPM_MIN 1000

#define ANALOG_MAX 255
#define ANALOG_MIN 1

// We generate a delta between -DELTA and DELTA and divide by 100
#define RPMDELTA 1000
#define ANALOGDELTA 10

float curRPM=RPM_MIN;
int curANALOG=ANALOG_MAX/2;

// BME280 Calibration
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
unsigned bme280status;

// MPU6050
Adafruit_MPU6050 mpu;

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;

// Data wire is connected to GPIO15
#define ONE_WIRE_BUS 15
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
//DeviceAddress sensor1 = { 0x28, 0xD7, 0x96, 0xE4, 0x07, 0x00, 0x00, 0x49 };

//MCP3008 SPI AD Converter
#define MCP3008_SPI_MAX_5V 3600000         ///< SPI MAX Value on 5V pin
#define MCP3008_SPI_MAX_3V 1350000         ///< SPI MAX Value on 3V pin
#define MCP3008_SPI_MAX MCP3008_SPI_MAX_5V ///< SPI MAX Value
#define MCP3008_SPI_ORDER MSBFIRST         ///<  SPI ORDER
#define MCP3008_SPI_MODE SPI_MODE0         ///< SPI MODE
#define MCP3008_VREF 5.1

const char* ssid     = "your-ssid";
const char* password = "your-password-here";
char timestring[50];

// Static IP of ESP32
IPAddress local_IP(192, 168, 20, 100);
IPAddress gateway(192, 168, 20, 10);
IPAddress subnet(255, 255, 255, 0);
IPAddress pingTarget(192, 168, 20, 10);
const char* ntpServer = "192.168.20.10";

const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;

#define INFLUXDB_URL "http://192.168.20.10:8086/"
#define INFLUXDB_USER "writer"
#define INFLUXDB_PASS "writerpassword"
#define INFLUXDB_DATABASE "sensordata"
#define INFLUXDB_WRITE_PRECISION WritePrecision::S

// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h

// Create the influxdb object
InfluxDBClient influxclient(INFLUXDB_URL, INFLUXDB_DATABASE);

// This is where we save data
LinkedList<Measurement> myMeasurements = LinkedList<Measurement>();
Measurement M;
Measurement curM;

// amateurish way to figure out if we have an ntp sync
# define FEBRUARY2020 1581627570
unsigned int timeiteration = 0;
time_t setuptime;

// to catch quick errors with the influx connection
bool hadinfluxerrors = 0;

void setup() {
    Serial.begin(115200);
    delay(1);
    time(&setuptime);
    // Initialising the UI will init the display too.
    Serial.print(String(setuptime));
    Serial.println(" Initializing display");
    display.init();
    display.flipScreenVertically();
    printStringOnDisplay("Connecting to Wifi");
    // Assign static IP address prior to connect
    if (!WiFi.config(local_IP, gateway, subnet)) {
      Serial.println("STA Failed to configure");
    }
    // connect to the wifi network

    Serial.println();
    time(&setuptime);
    Serial.print(String(setuptime));
    Serial.print(" Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    printStringOnDisplay("Wifi connected");
    Serial.println("");
    time(&setuptime);
    Serial.print(String(setuptime));
    Serial.println(" WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("ESP Mac Address: ");
    Serial.println(WiFi.macAddress());

    // there is no point in writing data unless we dont have a proper time.
    // halt until good time
    printStringOnDisplay("Setting time ");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    time(&setuptime);
    while (setuptime < FEBRUARY2020) {
      timeiteration++;
      display.clear();
      display.drawString(0, 10,String(ssid) + ": " + String(WiFi.RSSI()) + "dBm");
      display.drawString(0, 20,"Heap : " + String(ESP.getFreeHeap()));
      display.drawString(0, 30,"Time Sync attempt " + String(timeiteration) + ".");
      display.drawString(0, 40,"Epoch : " + String(setuptime));
      display.display();
      Serial.println("Time attempt " + String(timeiteration) + " " + String(setuptime));
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      time(&setuptime);
      delay(1000);
    }    
    //configure the influxdb connection
    printStringOnDisplay("connect to influx, ");
    influxclient.setConnectionParamsV1(INFLUXDB_URL, INFLUXDB_DATABASE, INFLUXDB_USER, INFLUXDB_PASS);
    influxclient.setWriteOptions(INFLUXDB_WRITE_PRECISION);
    // initialize w1 bus
    printStringOnDisplay("init w1 bus, ");
    sensors.begin();
    printStringOnDisplay("init BME280 i2c, 0x76, ");
    // default address for the BME280 is in this library 0x60
    bme280status = bme.begin(0x76);
    if (!bme280status) {
      printStringOnDisplay("BME280 failed, ");
    }
    // initialize MPU6050
    printStringOnDisplay("init MPU6050 i2c, 0x68");
    if (!mpu.begin()) {
      printStringOnDisplay("MPU6050 failed");
    }
    MPU6050calibration();
    //initialise instance of the SPIClass attached to VSPI (perhaps we need HSPI later)
    printStringOnDisplay("init SPI bus, ");
    vspi = new SPIClass(VSPI);
    //initialise vspi with default pins
    //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
    vspi->begin();
    //set up slave select pins as outputs as the Arduino API
    //doesn't handle automatically pulling SS low
    pinMode(5, OUTPUT); //VSPI SS for first SPI device
    pinMode(2, OUTPUT); //VSPI SS for second SPI device
//    digitalWrite(2, HIGH); //avoid second SPI for now
    
    printStringOnDisplay("setup finished");
}

void loop() {
  // clear the display
  display.clear();

  //analogue output random value
  int deltaAnalog=random(-ANALOGDELTA,ANALOGDELTA);
  limitedaddint(&curANALOG,deltaAnalog,ANALOG_MIN,ANALOG_MAX);
  dacWrite(25,curANALOG);
  
  readAllData(&M);
  myMeasurements.add(M);
//  printAllInfo(&M);
  printOnDisplay(&M);
  if (M.wlanstatus != WL_CONNECTED && M.w1_tempC != DEVICE_DISCONNECTED_C) {
    Serial.print("Wifi or w1 sensor not connected, WiFi.status = ");
    Serial.println(wl_status_to_string(M.wlanstatus));
  }
  else 
  {
    // We have connectivity, sync time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    // pop all measurements and write them to the db, verify that we have connectivity while we do it
    // since we pop first we lose a measurement if the wifi is lost while emptying the list
    hadinfluxerrors = 0;
    while (WiFi.status() == WL_CONNECTED && myMeasurements.size() > 0 && hadinfluxerrors == 0) {
      // curM is pulled from the list
      curM = myMeasurements.pop();
      // Write to the DB
      Point row("motorpitest1");
      row.clearFields();
      row.addField("rssi", curM.rssi);
      // optional
      row.addField("freeheap", curM.freeheap);
      row.addField("listlength", curM.listlength);
      row.addField("w1_tempC", curM.w1_tempC);
      row.addField("spi_tempC", curM.spi_tempC);
      row.addField("wlstatus", curM.wlanstatus);
      // this is something we can drop for performance, it is simply to have a readable
      // time in the database
//      row.addValueString("humantime",localTimeString(curM.timestamp));
      row.addField("rpm", curM.rpm);
      row.addField("afrvoltage0",curM.afrvoltage0);
      row.addField("analog0",curM.analog0);
      row.addField("bme280_temperature",curM.bme280_temperature);
      row.addField("bme280_humidity",curM.bme280_humidity);
      row.addField("bme280_airpressure",curM.bme280_airpressure);
      row.addField("mpu6050_accel_x",curM.mpu6050_accel_x);
      row.addField("mpu6050_accel_y",curM.mpu6050_accel_y);
      row.addField("mpu6050_accel_z",curM.mpu6050_accel_z);
      row.addField("mpu6050_gyro_x",curM.mpu6050_gyro_x);
      row.addField("mpu6050_gyro_y",curM.mpu6050_gyro_y);
      row.addField("mpu6050_gyro_z",curM.mpu6050_gyro_z);
      row.addField("mpu6050_temperature",curM.mpu6050_temperature);
      row.addField("sensorreadtime",curM.sensorreadtime);
      row.addTag("device", "ESP32");
      // old client wrote 1581717283000000000 as timestamp
//      row.setTime(curM.timestamp * 1000000000);
      row.setTime(curM.timestamp);
      if(!influxclient.writePoint(row)) {
        Serial.println("Failed to write to InfluxDB");
        // We failed for some reason, put the Measurement back in the list
        myMeasurements.add(curM);
        hadinfluxerrors = 1;
      }
    }
  }
  delay (loopWait);
}
