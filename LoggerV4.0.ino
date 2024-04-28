//Libraries. This flight computer uses a BMP280 and MPU over I2C and an SD reader over SPI.
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_MPU6050.h>

//Define constants
#define chargepin 6 //change to whatever pin you're using to trigger deployment
#define buzzer 5 //change to whatever pin you're using for buzzer and/or LED
#define buzztime 3000 //interval for tone to indicate logging is happening-
#define flushtime 2000 //interval for clearing for flushing to SD card
#define armcheck 10 //<--This is the minimum height in meters that the deployment charge can go off. Change this for whatever needed.
#define logtime 15 //set logging delay. the logging loop takes approximately 10ms, this is additional time added to the loop

//Variables
float pres, alti, altioffset, correctedalt, ax, ay, az, gx, gy, gz;
float maxalt = 0; 
float alticheck = 0;
byte logNumber = 0;
byte deploycount = 0;
byte logtimeout = 0;
unsigned long timer, buzzclock, flushclock;
bool armed = false;
bool fired = false;

//BMP, ADXL and SD things
Adafruit_BMP280 bmp; //May be changed to suit BMP390
Adafruit_MPU6050 mpu; //May be changed to 9-axis sensor
Adafruit_Sensor *mpu_accel, *mpu_gyro;
SdFat sd;
SdFile logfile;

void setup() {
//set the charge pin mode safely
  pinMode(chargepin, OUTPUT);
  digitalWrite(chargepin, LOW);

//setup begin tone
  pinMode(buzzer, OUTPUT);
  tone(buzzer, 4500, 100);
  delay(260);
  tone(buzzer, 4500, 100);
  delay(150);
  tone(buzzer, 4500, 100);
  delay(150);
  tone(buzzer, 5800, 500);

//setup BMP, MPU and SD
  #define COMMA logfile.print(",");

  bmp.begin(0x76); //<--I2C address is needed apparently otherwise baro just spits out garbage.
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X4,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X4,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);   /* Standby time. */
  
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  Serial.println("2");

  if (sd.begin(10, SD_SCK_MHZ(8))){ 
     while (sd.exists("FltLog" + String(logNumber) + ".csv")) {
     logNumber++;
     }
     char filename[20];
     snprintf(filename, sizeof(filename), "FltLog%d.csv", logNumber);
     logfile.open(filename, O_WRITE | O_CREAT);
     delay(2500); //this might be necessary because the SD card initialises so fast now
     logfile.sync();
     logfile.println("Logger v4");
     logfile.println();
     logfile.println("ms,pres,alt,rel. alt,ax,ay,az,gx,gy,gz");
     logfile.sync();    
  }

  else {
       //Serial.println("SD card initialisation failed");
       Serial.println("3");
       tone(buzzer, 5600, 200);
       delay(250);
       tone(buzzer, 3800, 500);
       while (1);
       }

//Figure out an offest for height above ground. There's a better way to do this with the BMP probably.
  Wire.begin();
  altioffset = bmp.readAltitude();

//Double tone to indicate setup is done. 
  tone(buzzer, 3700, 220);
  delay(200);
  tone(buzzer, 5700, 120);
  delay(5000);
}

//Read stuff from the BMP sensor, store it in variables
void readsensors (void) {
  Wire.begin(0x76);
  pres = bmp.readPressure() / 100.0;
  alti = 44330.0 * (1.0 - pow(pres / 1013.25, 0.1903));
  correctedalt = alti - altioffset;
  Wire.endTransmission();

//Read stuff from the accelerometer and gyro, store it in variables
  Wire.begin();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax = a.acceleration.x - 0.53;
  ay = a.acceleration.y + 0.60;
  az = a.acceleration.z - 15.36;
  gx = g.gyro.x - 0.44;
  gy = g.gyro.y - 0.04;
  gz = g.gyro.z - 0.01;
  Wire.endTransmission();
}

//Run arming check. Altitude above ground must be above armcheck before the deployment check runs.
//Disables deployment charge for safety if altitude falls below armcheck.
void arming (void) {
  if (armed && !fired){
     deployment();
     }
  else {
       if (correctedalt > armcheck && !fired) {
          armed = true;
          logfile.print("~ARM~");
          }
       if (correctedalt < armcheck && armed) {
          armed = false;
          deploycount = 0;
          digitalWrite(chargepin, LOW);
          logfile.print("~DISABLE~");
          }
       }
}

//Run the deployment checks and fires deployment charge if condition is met. 
//Apogee is detected when there have been at least 3 consecutive reductions in altitude. 
void deployment (void) {
  if (correctedalt <= alticheck && !fired) {
     deploycount = deploycount + 1;
     alticheck = correctedalt;
     }
  else {
       deploycount = 0;
       alticheck = correctedalt;
       }
  if (deploycount > 2) {
     digitalWrite(chargepin, HIGH);
     fired = true;
     logfile.print("~FIRE~");
     }
}

//wrap up the log and record a brief flight summary.
void endlog (void) {
  logfile.println();
  logfile.println("~END~");
  logfile.println();
  logfile.close();
  while(1) {
            tone(buzzer, 4700, 300);
            delay(500);
            tone(buzzer, 4700, 300);
            delay(3000);
           }
}

void loop() {
//Timestamp
  timer = millis();
  while (millis() - timer < logtime) {  
  }
//Tone indicating logging is happening
  if (timer - buzzclock >= buzztime) {
      tone(buzzer, 4700, 300);
      buzzclock = timer;
      }

//run through all the reads and checks
  readsensors();
  arming();

//log the readings to file
  logfile.println();
  logfile.print(timer); COMMA;
  logfile.print(pres); COMMA;
  logfile.print(alti); COMMA;
  logfile.print(correctedalt); COMMA;
  logfile.print(ax); COMMA;
  logfile.print(ay); COMMA;
  logfile.print(az); COMMA;
  logfile.print(gx); COMMA;
  logfile.print(gy); COMMA;
  logfile.print(gz); COMMA;

//logging timeout check
  if (fired && correctedalt < 5) {
     logtimeout = logtimeout + 1;
     }
  if (logtimeout >= 50) {
     endlog();
  } 

//Required to write to the text file properly, otherwise the file comes back empty.
//Loop runs quicker this way compared to opening and then closing the file every loop, allowing more frequent logging.
   if ((timer - flushclock >= flushtime))
      {
       logfile.sync();
       flushclock = timer;
      } 
}
