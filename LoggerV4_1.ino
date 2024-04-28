//Libraries. This flight computer uses a BMP280 and MPU over I2C and an SD reader over SPI.
#include <BMP280_DEV.h> //This one is a bit nicer compared to the Adafruit one
#include <SdFat.h>
#include <Adafruit_MPU6050.h>
#include "Buzzer.h"


//Define constants
#define chargepin 6
#define buzztime 3000
#define flushtime 2000
#define armcheck 10 //<--This is the minimum height in meters that the deployment charge can go off. Change this for whatever needed.
#define logtime 4 //set logging delay. the logging loop takes approximately 10ms, this is additional time added to the loop

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
char sep = ',';

//BMP, MPU and SD things
BMP280_DEV bmp; 
Adafruit_MPU6050 mpu; //May be changed to 9-axis sensor later
Adafruit_Sensor *mpu_accel, *mpu_gyro;
SdFat sd;
SdFile logfile;
Buzzer Buzz(5);

void setup() {
//set the charge pin mode safely
  Serial.begin(9600);
  digitalWrite(chargepin, LOW);
  pinMode(chargepin, OUTPUT);
  digitalWrite(chargepin, LOW);

//setup begin tone
  Buzz.begin();
  Buzz.startup();
  Serial.println("setup");

//setup BMP, MPU and SD
  #define COMMA logfile.print(",");
  #define COMMAx4 logfile.print(",,,,");

  bmp.begin(NORMAL_MODE, BMP280_I2C_ALT_ADDR); //<--alternate address is needed apparently otherwise baro just spits out garbage.
  bmp.setPresOversampling(OVERSAMPLING_X8);
  bmp.setTempOversampling(OVERSAMPLING_X2);
  bmp.setIIRFilter(IIR_FILTER_4);
  bmp.setTimeStandby(TIME_STANDBY_05MS); 
  bmp.startNormalConversion();

  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

  if (sd.begin(10, SD_SCK_MHZ(8))){ 
     while (sd.exists("FltLog" + String(logNumber) + ".csv")) {
     logNumber++;
     }
     char filename[20];
     snprintf(filename, sizeof(filename), "FltLog%d.csv", logNumber);
     logfile.open(filename, O_WRITE | O_CREAT);
     delay(2500); //this might be necessary because the SD card initialises so fast now
     logfile.sync();
     logfile.println("v4.1");
     logfile.println();
     logfile.println("ms,pres,alt,rel. alt,ax,ay,az,gx,gy,gz");
     logfile.sync();  
  }

  else {
       //Serial.println("SD card initialisation failed");
       Buzz.error();
       while (1);
       }


//Figure out an offest for height above ground. There's a better way to do this with the BMP probably.
  Wire.begin();
  bmp.getCurrentPressure(pres);
  altioffset = 44330.0 * (1.0 - pow(pres / 1013.25, 0.1903));

//Double tone to indicate setup is done. 
  Buzz.success();
  delay(5000);
}


void readBMP (void) {
  //Read stuff from the BMP sensor, store it in variables
  bmp.getCurrentPressure(pres);
  alti = 44330.0 * (1.0 - pow(pres / 1013.25, 0.1903));
  correctedalt = alti - altioffset;
}

void readIMU (void) {
  //Read stuff from the accelerometer and gyro, store it in variables  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax = a.acceleration.x - 0.53;
  ay = a.acceleration.y + 0.60;
  az = a.acceleration.z - 15.36;
  gx = g.gyro.x - 0.44;
  gy = g.gyro.y - 0.04;
  gz = g.gyro.z - 0.01;
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
           Buzz.ended();
           }
}

void loop() {
//Timestamp
  timer = millis();
  //only read and log the IMU because BMP data won't be ready yet
  for (int i=1; i<=4; i=i+1) {
      timer = millis();
      readIMU();
      //log only the IMU readings to file
      logfile.println();
      logfile.print(timer); COMMAx4;
      logfile.print(ax); COMMA;
      logfile.print(ay); COMMA;
      logfile.print(az); COMMA;
      logfile.print(gx); COMMA;
      logfile.print(gy); COMMA;
      logfile.print(gz);
      delay(logtime);
      }

  //run through all sensors and checks once the extra IMU logs are done
  //if (millis() - timer >= logtime) {
     timer = millis();
     readBMP();
     readIMU();
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
    // }

  //Tone indicating logging is happening
  if (timer - buzzclock >= buzztime) {
      Buzz.running();
      buzzclock = timer;
      }

//logging timeout check
  if (fired && correctedalt < 5) {
     logtimeout = logtimeout + 1;
     }
  if (logtimeout >= 100) {
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