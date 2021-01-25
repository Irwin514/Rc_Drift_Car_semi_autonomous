
#include <SoftwareSerial.h>
#include <PS2X_lib.h>  
#include <Servo.h>

#include <Arduino.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#define BIT(a) (1 << (a))

PS2X ps2x; // create PS2 Controller Object
Servo servo1; 
Servo esc;

int LeftStick = 128;
int RightStick = 128;
int PadLeft;
int Psb_l1_button;
int PadRight;
int angle  ;                  //sets the steering servo to mid point i.e. straight ahead
int throttle = 1500;          // 1500 microseconds is 0 m/s speed
int calib = 1500;             // used for calibrations of the steering, initialized at 1500 i.e. straight forawrd
int cruise = 1500;            // For cruise control
int a, b, x, n = 5, i = 0;

void setup() {
 
}

void loop() {
  // put your main code here, to run repeatedly:

}
