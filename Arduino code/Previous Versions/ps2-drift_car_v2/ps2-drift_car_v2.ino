//v2 changes: 
//-removed break button since it didnt work
//-addded tentative straight accelelration control
//-added deadzone to steering
//-reduced steering angle to 35 deg on either sides
//-added a honking function so we can tell the other teams to get out of our way lol


#include <PS2X_lib.h>  
#include <Servo.h>

PS2X ps2x; // create PS2 Controller Class
Servo servo1;
Servo esc;

int LeftStick = 128;
int RightStick = 128;
int angle = 90 ;            //sets the steering servo to mid point i.e. straight ahead
int throttle = 90 ;         //sets esc to 90 which is 0 speed

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
   LeftStick = ps2x.Analog(PSS_LX);     //left stick horizontal controls steering
   RightStick = ps2x.Analog(PSS_RY);    //right stick vertical controls speed
    Serial.print("LeftStick: ");
    Serial.print(LeftStick);
    Serial.print(", RightStick: ");
    Serial.print(RightStick);
     
   if(LeftStick <=138 && LeftStick >=118)    //this serves both to create a small deadzone in the steering as well as constant straight-steering control, needs testing         
      servo1.write(90); 

   else
   {
   angle = map(LeftStick, 255, 0, 55, 125); //this allows for 35 deg of turnning on either side, further testing needed
    Serial.print(", angle: ");
    Serial.print(angle);
   servo1.write(angle);
   }
   
   throttle = map(RightStick, 255, 0, 75, 105); //note: anything above 110 causes the tires come off the rear wheels
    Serial.print(", throttle: ");
    Serial.print(throttle);
   esc.write(throttle); //we should be using PWM and esc.writeMicroseconds() to control speed however,our arduino nano does not seem to support PWM



  if(ps2x.Button(PSB_L1)) //if L1 is pressed, honk horn
   {
    tone(4, 294);
   }
  
delay(15);
     
}
