/*
PS2Controlv0.ino
21 September 2015
Hamish Trolove - www.techmonkeybusiness.com

This sketch illustrates the use of a Playstation 2 Controller to
actuate a series of servos and an Electronic Speed Controller (ESC)
as you might do with a vehicle.  An LED light is used to illustrate
the use of the Playstation 2 Buttons.

Pin assignments are:

3.3V output to PS2 red pin
5V output to 1Kohm pull up resistors for PS2.
Pin D02 to PS2 brown pin (data)
Pin D03 to PS2 yellow pin (attention)
Pin D04 to PS2 orange pin (command)
Pin D05 to PS2 blue pin (clock)

Pin D06 to ESC Signal Pin  
Pin D07 to Steering Servo Signal pin
Pin D08 to Servo 1 Signal pin
Pin D09 to Servo 2 Signal pin

Pin D13 to LED Transistor Base

The ESC servo connection supplies 5.6V to the servos.

The coding pulls on the PS2X library developed by Bill Porter.
See www.billporter.info for the latest from Bill Porter and to
download the library.

The controls used for this sketch are;
Right Stick - X-axis = Steering Servo left/right, Y-axis = ESC forward/backward
Left Stick - X-axis = Servo 2 left/right, Y-axis = Servo 1 left/right

Triangle = Toggle the LED
*/


#include <Servo.h>  //For driving the ESCs and Servos
#include <PS2X_lib.h> // Bill Porter's PS2 Library

PS2X ps2x;  //The PS2 Controller Class
Servo SteeringServo;  //Create servo object representing SteeringServo
Servo ServoN1;  //Create servo object representing Servo 1
Servo ServoN2;  //Create servo object representing Servo 2
Servo ESCcontrol;  //Create servo object representing ESC

const int LEDpin = 1;  //green LED is on Digital pin 13

volatile boolean LEDHdlts; //LED headlights on/off toggle

int PlyStnRStickUpDn = 0;  //Value read off the PS2 Right Stick up/down.
int PlyStnRStickLtRt = 0;  //Value read off the PS2 Right Stick left/right
int PlyStnLStickUpDn = 0; //Value read off the PS2 Left Stick up/down
int PlyStnLStickLtRt = 0; // Value read off the PS2 Left Stick left/right

int ESCSetting = 90; //Setting for the ESC (degrees).
int StrServoSetting = 90; //Setting for the Steering Servo
int ServoN1Setting = 90; //Setting for the Servo 1
int ServoN2Setting = 90; //Setting for the Servo 2


void setup()
          { 
            ps2x.config_gamepad(12,10,9,11, false, false);
            //ps2x.config_gamepad(5,4,3,2, false, false);
            //setup pins and settings: GamePad(clock, command, attention, data, Pressures, Rumble)
            //We have disabled the pressure sensitivity and rumble in this instance.
            pinMode(LEDpin, OUTPUT);  //Sets the LEDpin to output
          
            LEDHdlts = false;  //Sets the Headlights to off
            SteeringServo.attach(2);// attaches the Steering Servo to pin 7
            ServoN1.attach(3);// attaches the Servo 1 to pin 8
            ServoN2.attach(4);// attaches the Servo 2 to pin 9
            ESCcontrol.attach(6,0,2250);// attaches the ESC to pin 6
            //The ESC attachment command above also includes the signal settings
            //for the maximum and minimum that the ESC will recognise.  This
            //varies for different ESCs.
            
            //Set all ESCs and Servos to a neutral 90 degree position
            //this avoids the ESC trying to calibrate.
            ESCcontrol.write(90);
            SteeringServo.write(90);
            ServoN1.write(90);
            ServoN2.write(90);
            delay(5000); //Five second delay to allow ESC and controller to
                         // fully initialise.
            
          }

void loop()
{
  ps2x.read_gamepad(); //This needs to be called at least once a second
                        // to get data from the controller.

  if(ps2x.ButtonPressed(PSB_GREEN))  //Triangle pressed
  {
    LEDHdlts = !LEDHdlts; //Toggle the LED light flag
  }


//Analogue Stick readings
  PlyStnRStickUpDn = ps2x.Analog(PSS_RY); //Right Stick Up and Down
  PlyStnRStickLtRt = ps2x.Analog(PSS_RX); //Right Stick Left and Right
  PlyStnLStickUpDn = ps2x.Analog(PSS_LY); //left Stick Up and Down
  PlyStnLStickLtRt = ps2x.Analog(PSS_LX); //Left Stick Left and Right
  
//Readings from PS2 Controller Sticks are from 0 to 255
//with the neutral being 128.  The zero positions are to
//the left for X-axis movements and up for Y-axis movements.

//Variables to carry the settings for the ESCs and Servos
//The values from the PS2 Sticks are mapped to 0 to 180 degrees 

  ESCSetting = map(PlyStnRStickUpDn,-256,256,0,179);
  StrServoSetting = map(PlyStnRStickLtRt,-256,256,0,179);
  ServoN1Setting = map(PlyStnLStickUpDn,-256,256,0,179);
  ServoN2Setting = map(PlyStnLStickLtRt,-256,256,0,179);

 
//Write it to the Servos or ESCs 
  
  ESCcontrol.write(ESCSetting);
  SteeringServo.write(StrServoSetting);
  ServoN1.write(ServoN1Setting);
  ServoN2.write(ServoN2Setting);


  digitalWrite(LEDpin,LEDHdlts); //Light the LED based on headlights status flag
  delay(15);
}
