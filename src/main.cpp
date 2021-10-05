#include <Arduino.h>
#include <Wire.h>
#include <si5351.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

void doEncoderA();
void doEncoderB();

#define SCREEN_ADDRESS 0x3C
SSD1306AsciiWire oled;

static boolean rotating = false; // debounce management
volatile unsigned long lastFrequency = 1;
volatile unsigned long frequency = 7032000;          // This will be the frequency it always starts on.
long freqstep[] = {10, 50, 100, 500, 1000, 5000, 10000}; // set this to your wanted tuning rate in Hz.
long corr = -181000;                                       // this is the correction factor for the Si5351, use calibration sketch to find value.
unsigned int freqsteps = 2;

#define arraylength (sizeof(freqstep) / sizeof(freqstep[0]))

// interrupt service routine vars
boolean A_set = false;
boolean B_set = false;

int encoderPinA = 2; // rigth
int encoderPinB = 3; // left
int stepbutton = 14;

Si5351 si5351(0x60);

void setup()
{
  Serial.begin(9600);

  Serial.println("Starting");

  bool i2c_found;
  i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, corr);
  if (!i2c_found)
  {
    Serial.println("Device not found on I2C bus!");
  }

  oled.begin(&Adafruit128x64, SCREEN_ADDRESS);
  Serial.println("DISPLAY initted");
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.set2X();
  oled.print("AF7QD VFO");

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(stepbutton, INPUT_PULLUP);

  attachInterrupt(0, doEncoderA, CHANGE);
  attachInterrupt(1, doEncoderB, CHANGE);

  // Set CLK0 to output 14 MHz
  si5351.set_freq(frequency * 100ULL, SI5351_CLK0);
}

void loop()
{  
  if (lastFrequency != frequency)
  {
    lastFrequency = frequency;

    oled.setRow(3);
    oled.setCol(0);
    oled.clearToEOL();
    oled.print((long)frequency);

    Serial.print("Setting FREQ ");
    Serial.print(frequency);
    si5351.set_freq((frequency * 100ULL), SI5351_CLK0);
    Serial.println(" Done");
  }
  delay(50);

  if (digitalRead(stepbutton) == LOW)
  {
    delay(50); // delay to debounce
    if (digitalRead(stepbutton) == LOW)
    {
      freqsteps = freqsteps + 1;      
      if (freqsteps >= arraylength)
      {
        freqsteps = 0;
      }
      Serial.print("Step ");
      Serial.println(freqstep[freqsteps]);
      delay(150); //delay to avoid many steps at one
    }
  }

  rotating = true; // reset the debouncer
}

// Interrupt on A changing state
void doEncoderA()
{
  if (rotating)
    delay(1); // wait a little until the bouncing is done
  // Test transition, did things really change?
  if (digitalRead(encoderPinA) != A_set)
  { // debounce once more
    A_set = !A_set;
    // adjust counter + if A leads B
    if (A_set && !B_set)
    {
      Serial.println("Freq++");
      //if (!tx) {
      frequency += freqstep[freqsteps]; // here is the amount to increase the freq
      //}
      rotating = false; // no more debouncing until loop() hits again
    }
  }
}

// Interrupt on B changing state, same as A above
void doEncoderB()
{
  if (rotating)
    delay(1);
  if (digitalRead(encoderPinB) != B_set)
  {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if (B_set && !A_set)
    {
      Serial.println("Freq--");
      //if (!tx) {
      frequency -= freqstep[freqsteps]; // here is the amount to decrease the freq
      //}
      rotating = false;
    }
  }
}
