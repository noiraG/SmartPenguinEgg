/*************************************************** 
  This is a library example for the MLX90614 Temp Sensor

  Designed specifically to work with the MLX90614 sensors in the
  adafruit shop
  ----> https://www.adafruit.com/products/1747 3V version
  ----> https://www.adafruit.com/products/1748 5V version

  These sensors use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
//temp library
#include <Adafruit_MLX90614.h>
//gyro-accel library
#include <Adafruit_LSM9DS1.h>
//humidity library
#include "Adafruit_Si7021.h"

//initialize sensors
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Si7021 sensor = Adafruit_Si7021();

//declare files
File humidityFile;
File IrTemperatureFile;
File SensorTemperatureFile;
File GyroX;
File GyroY;
File GyroZ;
File AccelX;
File AccelY;
File AccelZ;
File MagX;
File MagY;
File MagZ;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  lsm.begin();
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  mlx.begin();
  Serial.print("Initializing SD card...");
  pinMode(10, OUTPUT);
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
}

void loop() {
  
  delay(2000);
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }   
  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC()); Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

  Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");
  Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
  Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
  Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");
  Serial.print("Humidity:    ");
  Serial.print(sensor.readHumidity(), 2);
  Serial.print("\tTemperature: ");
  Serial.println(sensor.readTemperature(), 2);
  Serial.println();
  
  humidityFile = SD.open("humidity.csv", FILE_WRITE);
  if (humidityFile) {
    Serial.print("Writing to humidity.csv..."); 
    humidityFile.print(sensor.readHumidity(), 2);
    humidityFile.print(", ");
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening humidity.csv");
  }
  humidityFile.close();

  IrTemperatureFile = SD.open("irtemp.csv", FILE_WRITE);
  if (IrTemperatureFile) {
    Serial.print("Writing to ir.csv..."); 
    IrTemperatureFile.print(mlx.readObjectTempC(), 2);
    IrTemperatureFile.print(", ");
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening irtemp.csv");
  }
  IrTemperatureFile.close();

  SensorTemperatureFile = SD.open("sentemp.csv", FILE_WRITE);
  if (SensorTemperatureFile) {
    Serial.print("Writing to sentemp.csv..."); 
    SensorTemperatureFile.print(mlx.readAmbientTempC(), 2);
    SensorTemperatureFile.print(", ");
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening sentemp.csv");
  }
  SensorTemperatureFile.close();

  GyroX = SD.open("gyrox.csv", FILE_WRITE);
  if (GyroX) {
    Serial.print("Writing to gyrox.csv..."); 
    GyroX.print(g.gyro.x, 2);
    GyroX.print(", ");
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening gyrox.csv");
  }
  GyroX.close();

  GyroY = SD.open("gyroy.csv", FILE_WRITE);
  if (GyroY) {
    Serial.print("Writing to gyroy.csv..."); 
    GyroY.print(g.gyro.y, 2);
    GyroY.print(", ");
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening gyroy.csv");
  }
  GyroY.close();

  GyroZ = SD.open("gyroz.csv", FILE_WRITE);
  if (GyroZ) {
    Serial.print("Writing to gyroz.csv..."); 
    GyroZ.print(g.gyro.z, 2);
    GyroZ.print(", ");
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening gyroz.csv");
  }
  GyroZ.close();

  AccelX = SD.open("accelx.csv", FILE_WRITE);
  if (AccelX) {
    Serial.print("Writing to accelx.csv..."); 
    AccelX.print(a.acceleration.x, 2);
    AccelX.print(", ");
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening accelx.csv");
  }
  AccelX.close();

  AccelY = SD.open("accely.csv", FILE_WRITE);
  if (AccelY) {
    Serial.print("Writing to accely.csv..."); 
    AccelY.print(a.acceleration.y, 2);
    AccelY.print(", ");
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening accely.csv");
  }
  AccelY.close();

  AccelZ = SD.open("accelz.csv", FILE_WRITE);
  if (AccelZ) {
    Serial.print("Writing to accelz.csv..."); 
    AccelZ.print(a.acceleration.z, 2);
    AccelZ.print(", ");
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening accelz.csv");
  }
  AccelZ.close();

  MagX = SD.open("magx.csv", FILE_WRITE);
  if (MagX) {
    Serial.print("Writing to magx.csv..."); 
    MagX.print(m.magnetic.x, 2);
    MagX.print(", ");
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening magx.csv");
  }
  MagX.close();

  MagY = SD.open("magy.csv", FILE_WRITE);
  if (MagY) {
    Serial.print("Writing to magy.csv..."); 
    MagY.print(m.magnetic.y, 2);
    MagY.print(", ");
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening magy.csv");
  }
  MagY.close();

  MagZ = SD.open("magz.csv", FILE_WRITE);
  if (MagZ) {
    Serial.print("Writing to magz.csv..."); 
    MagZ.print(m.magnetic.z, 2);
    MagZ.print(", ");
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening magz.csv");
  }
  MagZ.close();
}

