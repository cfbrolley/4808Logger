This code is for an early version of a rocketry flight computer based on an Arduino Nano. 
It uses a BMP280 module and MPU6050 module over I2C, a micro SD card reader over SPI, features one ignition channel and audio tones.

Logging:
Logging is performed by writing to a.csv file on the micro SD card. data poings include elapsed time in ms, barometric pressure and altitude, altitude above starting level, accelerometer and gryo readings (x, y and z)
rate of data logging can be increased or decreased by changing the value defined for logtime. value of maxes out the rate at around 100hz but this would exceed the data output rate of the BMP280 if any filtering is enabled.
 
Altitude sensing:
BMP280 sensor needs filtering and oversampling to be reliable enough to be used for ignition channel trigger.
The current logic states that if the current altitude is read as being above arming height and is <= the previous reading 3 times consecutively, it will fire.
This can lead to a false trigger if there is too much noise or variance not filtered out.
Increasing filtering and/or oversampling does increase the amount of time needed between readings to prevent duplicate values.

Accelerometer/Gyro
MPU6050 can reach much higher output data rates than the BMP280 can but this logic has not been worked in yet due to the limited memore of the Arduino Nano
offsets are hard-coded so will neet to be changed for each logger build unfortunately.
Again, no more room to add calibration.

Ignition channel logic
ignition channel remains disarmed and no apogee checks are performed when rocket is below a set altitude above ground level
Channel is fired when apogee detection parameters are met. Parameters are: 
-Current altitude is above arming altitude
-Current altitude is equal to or below previous altitude 3 times consecutively

SD card:
Uses SDFat library to keep up with logging rates.
A separate new log file is created during setup and is given a numerical number in the file name based on whether the next number in order is already present on the card.
A timer is used to flush the buffer at interval set by flushtime (in ms)
File is automatically closed and logging stopped after 100 loops when apogee is detected and altitude falls below 5
variables are written straight to a .csv file on the SD card in rows. I don't like the way it's currently written though.
each row contains (in order):
-time
-pressure
-barometric altitude
-altitude above ground level
-accelerometer readings (x, y, z)
-gyro readings (x, y, z)

Settings:
#define buzztime > interval in ms between buzzer tones while logging
#define flushtime > interval in ms between flushing the SD card buffer
#define armcheck 10 > This is the minimum altitude above starting ground level in meters that the deployment charge can be triggered. Apogee check will not be called if altitude above starting ground level is below this height.
#define logtime 15 > extra delay added to the loop to set the logging rate. the loop itself takes approximately 10ms, so setting this to 0 results in the loop running at 100hz.
