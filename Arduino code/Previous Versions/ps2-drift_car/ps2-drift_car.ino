#include <PS2X_lib.h>  
#include <Servo.h>

PS2X ps2x; // create PS2 Controller Class
Servo servo1;
Servo esc;

int LeftStick = 0;
int RightStick = 0;
int angle =0 ;            //sets the steering servo to mid point i.e. straight ahead
int throttle = 90 ;       //sets esc to 90 which is 0 speed

void setup()
{
 
  ps2x.config_gamepad(12,10,9,11, false, false);      //GamePad(clock, command, attention, data, Pressures?, Rumble?)
  servo1.attach(6);
  esc.attach(3);
 Serial.begin(9600);
 delay(2000);

}

void loop(){
 
   ps2x.read_gamepad();                 //reads input from ps2 controller
   
   if(ps2x.Button(PSB_R1))              //R1 is the brake button
    esc.write(90);                      //esc at 90 should set the speed of motor to 0, needs actual testing
    
   else
   {
   LeftStick = ps2x.Analog(PSS_LX);     //left stick horizontal controls steering
   RightStick = ps2x.Analog(PSS_RY);    //right stick vertical controls speed
   Serial.print("LeftStick: ");
   Serial.print(LeftStick);
   Serial.print(", RightStick: ");
   Serial.print(RightStick);
   angle = map(LeftStick, 0, 255, 0, 179);
   throttle = map(RightStick,255, 0, 75, 105);    //below 90 is reverse, and above is forward. 
                                                  //note: anything above 110 causes the tires come off the rear wheels
                                                  //we should be using PWM and esc.writeMicroseconds() to control speed 
                                                  //however,our arduino nano does not seem to support PWM
                                                  //also need to test if our controller sticks need a dead zone to avoid small twitches.
   Serial.print(", angle: ");
   Serial.print(angle);
   Serial.print(", throttle: ");
   Serial.print(throttle);

   servo1.write(angle);
   esc.write(throttle);
   }

 
delay(15);
     
}
