#include <arduino.h>
#include "Buzzer.h"

#define dly150 delay(150);
#define dly200 delay(200);
#define dly250 delay(250);
#define dly500 delay(500);
#define dly1000 delay(1000);
#define dly3000 delay(3000);


Buzzer::Buzzer(int pin)
{
  _pin = pin;
}

void Buzzer::begin()
{
  pinMode(_pin, OUTPUT);
}

void Buzzer::startup()
{
  tone(_pin, 4500, 160); dly250
  tone(_pin, 4500, 100); dly150
  tone(_pin, 4500, 100); dly150
  tone(_pin, 5800, 500); dly1000
}

void Buzzer::error()
{
  tone(_pin, 5600, 200); dly250
  tone(_pin, 3800, 500);
}

void Buzzer::success()
{
  tone(_pin, 3700, 220); dly200
  tone(_pin, 5700, 120);
}

void Buzzer::running()
{
 tone(_pin, 5700, 300);
}

void Buzzer::ended()
{
 tone(_pin, 4700, 300); dly500
 tone(_pin, 4700, 300); dly3000
}