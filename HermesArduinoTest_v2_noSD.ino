//Hermes Test Code For Arduino
//Components: 
//MPU6050 - Accelerometer/Gyroscope
//Adafruit Micro SD Card
//BMP280 - Altimeter/Barometer
//Adafruit Ultimate Breakout GPS
//Xbee G Radio Trasmitter


#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <XBee.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <basicMPU6050.h> 
#include <Adafruit_GPS.h>


//BMP 280 Sensor
Adafruit_BMP280 bmp; //Communicates via I2C
#define SEALEVELPRESSURE_HPA (1013.25)

//MPU6050 Sensor
basicMPU6050<> imu; //Communicates via I2C

//Adafruit GPS
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

void setupBMP() {
  unsigned status;
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor"));
    while (1);
  }

  Serial.println("BMP initialized successfully");
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

String readBMP() {
 float temp = bmp.readTemperature();
 float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
 float pressure = bmp.readPressure();

 return ("Temperature = " + String(temp) + " *C\n" + "Altitude = "
         + String(altitude) + " m\n" + "Pressure = " + String(pressure) 
         + " Pa");
}

void setupMPU() {
  imu.setup();

  Serial.println("MPU6050 Initiallized successfully");

  //Callibration
  imu.setBias();

}

void setupGPS() {
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
}

String readMPU() {
  
  //Accelerometer
  float accelx = imu.ax();
  float accely = imu.ay();
  float accelz = imu.az();

  //Gyroscope
  float gyrox = imu.gx();
  float gyroy = imu.gy();
  float gyroz = imu.gz();

  return ("Acceleration\n X:" + String(accelx) + " Y:" + String(accely)
          + " Z:" + String(accelz) + " m/s^2\n" + "Gyroscope Rotation\n X:" 
          + String(gyrox) + " Y:" + String(gyroy) + "Z: " 
          + String(gyroz) + " radians/s");
}

String  readGPS() {
  int fix = int(GPS.fix);
  float latitude = GPS.latitude;
  float longitude = GPS.longitude;
  char lat = GPS.lat;
  char lon = GPS.lon;

  return ("Fix: " + String(fix) + "\n" + "Latitude: " + String(latitude) + lat + "\n" + "Longitude: " + String(longitude) + lon + "\n");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    }
  setupBMP();
  setupMPU();
  setupGPS();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(readBMP());
  delay(1000);
  Serial.println(readMPU());
  delay(2000);
  Serial.println(readGPS());
  delay(500);
}
