# Read values with an ESP32 and store them in an influxdb

This is code in development which reads various sensors connected to an ESP32 and stores them locally, and whenever there is Wifi connectivity, it dumps it off in an influxdb.

The code is poor in the following ways:

* no peer review
* mostly assuming sensors are always OK
* superhappy fun time with global variables, break statements etc
* designed during creation
* possibly buggy, no real testing yet

## To-do list

* Add interrupt code for counting digital input
* Split code on different cores, perhaps breaking out the sensor reading on the free core
* Fix issue when recovery from missing connectivity, until local buffer has been written away no new values are being read
* A ton of other things
