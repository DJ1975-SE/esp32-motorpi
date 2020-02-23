# Read values with an ESP32 and store them in an influxdb

This is code in development which reads various sensors connected to an ESP32 and stores them locally, and whenever there is Wifi connectivity, it dumps it off in an influxdb.

The code is poor in the following ways:

* no peer review
* mostly assuming sensors are always OK
* superhappy fun time with global variables, break statements etc
* designed during creation
* possibly buggy, no real testing yet

## Data Storage

A Raspberry Pi or some other linuxish system is used to store the data/run an influxdb, present it using grafana, and run the Wifi accespoint.
Steps done on the Pi:

* Credentials in influxdb
* raspbian' influxdb is too old, have to block and install from influx (apt commands?)
* some settings to improve association speed, lower bandwidth on AP (hostapd conf file)

## To-do list

* Add interrupt code for counting digital input
* Split code on different cores, perhaps breaking out the sensor reading on the free core
* Fix issue when recovery from missing connectivity, until local buffer has been written away no new values are being read
* A ton of other things
* List the dependencies' repos / URLs / details (Arduino libraries)
* be more efficient with RAM (datatypes in struct)
* credentials into own .conf file or something
* describe the pi setup better

## Pinout / Wiring

* ESP GPIO18 = SPI CLK (MCP3008, MAX6675)
* ESP GPIO16 = SPI Data out
* ESP GPIO23 = SPI Data in
* ESP GPIO5  = SPI CS for MAX6675
* ESP GPIO2  = SPI CS for MCP3008
* ESP GPIO15 = Dallas 1Wire Data for DS18B20
* ESP GPIO22 = I2C CLK (MPU6050, DME280, SSD1306)
* ESP GPIO21 = I2C SDA

The MCP3008 have a +5 Vin (Vin cannot be lower than Vref), the rest is running on 3.3 V

