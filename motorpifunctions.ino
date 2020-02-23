/*
 * Functions for the sensor ESP 
 * most taken from examples / stanzas
 * 
 * 
 */

void MPU6050calibration() {
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  Serial.println("MPU6050 calibrated");
  delay(1000);  
}

void readAllData(Measurement *Meas) 
{   
  // telemetry
  unsigned long startMillis=millis();
  //Timestamp of the measurement
  time(&Meas->timestamp);
  // read the temperature from the buses
  sensors.requestTemperatures();
  Meas->w1_tempC = sensors.getTempCByIndex(0);
  Meas->spi_tempC = readSPImax6675();
  // for statistics/debugging, can be removed
  Meas->freeheap = ESP.getFreeHeap();
  Meas->listlength = myMeasurements.size();
  Meas->wlanstatus = WiFi.status();
  // Assume connectivity is lost
  Meas->rssi = -120;
  if (Meas->wlanstatus == WL_CONNECTED) {
    Meas->rssi = WiFi.RSSI();
  }
  // Simulate RPM, Voltage 
  float deltaRPM=random(-RPMDELTA,RPMDELTA)/100;
  limitedadd(&curRPM,deltaRPM,RPM_MIN,RPM_MAX);
  Meas->rpm = int(curRPM);

  //MCP3008 0-5 Volts
  Meas->analog0 = readSPImcp3008(0);
  //The value is 0-1023 in reference to Vref, which is 5.1 Volt
  Meas->afrvoltage0 = ((Meas->analog0 * MCP3008_VREF) / 1024);
  
  //BME280
  Meas->bme280_temperature = bme.readTemperature();
  Meas->bme280_humidity = bme.readHumidity();
  Meas->bme280_airpressure = bme.readPressure() / 100.0F;
  Meas->bme280_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  //MPU6050
  sensors_event_t a, g, mputemp;

  // acceleration and gyro register mismatch?
  mpu.getEvent(&g, &a, &mputemp);
  
  Meas->mpu6050_accel_x = a.acceleration.x;
  Meas->mpu6050_accel_y = a.acceleration.y;
  Meas->mpu6050_accel_z = a.acceleration.z;

  Meas->mpu6050_gyro_x = g.gyro.x;
  Meas->mpu6050_gyro_y = g.gyro.y;
  Meas->mpu6050_gyro_z = g.gyro.z;

  Meas->mpu6050_temperature = mputemp.temperature;
  Meas->sensorreadtime = millis() - startMillis;
  return;
}

// Add the delta, reverse sign if outside of range

void limitedaddint(int *curval, int delta, int min, int max) {
  if (*curval < min) {
      *curval=*curval + abs(delta);  
  }
  else if (*curval > max) {
      *curval=*curval - abs(delta);          
  }
  else {
      *curval=*curval + delta;    
  }
  return;
}
void limitedadd(float *curval, float delta, int min, int max) {
  if (*curval < min) {
      *curval=*curval + abs(delta);  
  }
  else if (*curval > max) {
      *curval=*curval - abs(delta);          
  }
  else {
      *curval=*curval + delta;    
  }
  return;
}

float readSPImax6675() {
  uint16_t temp_reading;
  float celsius;
  //use it as you would the regular arduino SPI API
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(5, LOW); //pull SS slow to prep other end for transfer
  // Read and Write is done simultaneously, we write 0xff because we just want to read
  // Exact format and bit shiftery is 12 bits from a MAX6675, copied from arduino class
  temp_reading = vspi->transfer(0xff);
  temp_reading <<= 8;
  temp_reading |= vspi->transfer(0xff);
  if (temp_reading & 0x4) {
    // uh oh, no thermocouple attached!
    return NAN; 
    //return -100;
  }
  temp_reading >>= 3;

  celsius = temp_reading * 0.25;

  digitalWrite(5, HIGH); //pull ss high to signify end of data transfer
  vspi->endTransaction();
  return celsius;
}


// taken from adafruit mcp3008, use hw SPI
// retval is 0-1023 (depending on Vref)
int readSPImcp3008(uint8_t channel) {
  byte command;
  command = ((0x01 << 7) |             // start bit
             (0x00 << 6) |          // single read
             ((channel & 0x07) << 3)); // channel number

  byte b0, b1, b2;

  vspi->beginTransaction(SPISettings(MCP3008_SPI_MAX, MCP3008_SPI_ORDER, MCP3008_SPI_MODE));
  digitalWrite(2, LOW);

  b0 = vspi->transfer(command);
  b1 = vspi->transfer(0x00);
  b2 = vspi->transfer(0x00);

  digitalWrite(2, HIGH);
  vspi->endTransaction();

//  Serial.println("Finished transaction - b0: " + String(b0) + " b1: " + String(b1) + " b2: " + String(b2));

  return 0x3FF & ((b0 & 0x01) << 9 | (b1 & 0xFF) << 1 | (b2 & 0x80) >> 7);
}

char* wl_status_to_string(wl_status_t status) {
  switch (status) {
    case WL_NO_SHIELD: return "WL_NO_SHIELD";
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED: return "WL_DISCONNECTED";
  }
}

void printAllInfo(Measurement *Meas)
{
    time_t now;
    Serial.println("\n-------------------------");
    Serial.println(String(localTimeString(time(&now))));
    Serial.println("-------------------------\n");
    Serial.println("W1 TempC    : " + String(Meas->w1_tempC) + " C");
    Serial.println("SPI TempC   : " + String(Meas->spi_tempC) + " C");
    Serial.println("Chan0 Input : " + String(Meas->analog0));
    Serial.println("Chan0 Volt  : " + String(Meas->afrvoltage0));
    Serial.println("RSSI        : " + String(Meas->rssi) + " dBm");
    Serial.println("Wifi Status : " + String(wl_status_to_string(Meas->wlanstatus)));
    Serial.println("Free Heap   : " + String(ESP.getFreeHeap()));
    Serial.println("List size   : " + String(myMeasurements.size()));
    Serial.println("Timestamp   : " + String(Meas->timestamp));
    Serial.println("-------------------------\n");
}

char* localTimeString(time_t timestamp)
{
  struct tm * timeinfo;
  timeinfo = gmtime(&timestamp);
  strftime(timestring, sizeof(timestring), "%Y-%m-%d %H:%M:%S", timeinfo);
  return timestring;
}

void printOnDisplay(Measurement *Meas) {
  struct tm timeinfo;
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  if(!getLocalTime(&timeinfo)){
    display.drawString(0, 0, "Failed to obtain Time");
    return;
  }
  strftime(timestring, sizeof(timestring), "%Y-%m-%d %H:%M:%S", &timeinfo);
  display.drawString(0, 0, timestring);
  display.drawString(0, 10,"Heap " + String(ESP.getFreeHeap()) + ", Len " + String(Meas->listlength));
  // strength, formula stolen
  unsigned int strength = 120 - ((5/3) * abs(Meas->rssi));
  // draw the percentage as String
  display.drawProgressBar(0, 22, 120, 10, strength);
//  display.setTextAlignment(TEXT_ALIGN_CENTER);
//  display.drawString(64,42, "RSSI: " + String(Meas->rssi));
//  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 34,"x:" + String(Meas->mpu6050_gyro_x) + " y:" + String(Meas->mpu6050_gyro_y) + " z:" + String(Meas->mpu6050_gyro_z));
  display.drawString(0, 44,String(Meas->w1_tempC) + "C " + String(Meas->spi_tempC) + "C " + String(Meas->mpu6050_temperature) + "C");
  display.drawString(0, 54,String(Meas->bme280_temperature) + "C " + String(Meas->bme280_humidity) + " rel%");
  display.display();
}

void printStringOnDisplay(char* inputstring) {
  struct tm timeinfo;
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  if(!getLocalTime(&timeinfo)){
    display.drawString(0, 0, "Failed to obtain Time");
    return;
  }
  strftime(timestring, sizeof(timestring), "%Y-%m-%d %H:%M:%S", &timeinfo);
  display.drawString(0, 0, timestring);
  display.drawString(0, 20, inputstring);
  Serial.println(inputstring);
  display.display();
}
