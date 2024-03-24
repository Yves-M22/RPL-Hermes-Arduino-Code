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

//Adafruit Breakout GPS
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#define PMTK_Q_RELEASE "$PMTK605*31"

//SD Card Reader
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
File myFile;
const int CS = 10;

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

void setupSD() {
  Serial.print("Initializing SD card...");

  if (!SD.begin(CS)) {
    Serial.println("initialization failed!");
  }
  Serial.println("initialization done."); 
}

void writeFile(String fileName, String contents){
    myFile = SD.open(fileName, FILE_WRITE);
  if (myFile) {
    Serial.print("Writing to " + fileName);

    myFile.println(contents);
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
  }
}

void setupGPS() {
  mySerial.begin(9600);
  mySerial.println(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  mySerial.println(PMTK_Q_RELEASE);
}

char readGPS() {
  if (Serial.available()) {
   char c = Serial.read();
   Serial.write(c);
   mySerial.write(c);
   return c;
  }
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
  setupSD();
}

void loop() {
  // put your main code here, to run repeatedly:
  writeFile("HermesBMP.txt", readBMP());
  Serial.println("BMP Data");
  Serial.println(readBMP());
  delay(500);
  writeFile("HermesMPU.txt", readMPU());
  Serial.println("MPU Data");
  Serial.println(readMPU());
  delay(500);

}
