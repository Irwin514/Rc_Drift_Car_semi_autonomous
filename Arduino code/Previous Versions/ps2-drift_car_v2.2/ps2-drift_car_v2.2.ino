//v2.2 changes: 
//removed straightening button (L1)and replaced it with a break button


#include <PS2X_lib.h>  
#include <Servo.h>

PS2X ps2x; // create PS2 Controller Class
Servo servo1;
Servo esc;

int LeftStick = 128;
int RightStick = 128;
float angle = 90.0 ;            //sets the steering servo to mid point i.e. straight ahead
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
    
    
  if(LeftStick <=138 && LeftStick >=118)    //this serves both to create a small deadzone in the steering as well as constant straight-steering control, needs testing         
      servo1.write(89.5);                     //offset 5 deg to the right
  
   else
   {
   angle = map(LeftStick, 255, 0, 53.5, 123.5); //this allows for 35 deg of turnning on either side, further testing needed
    
   servo1.write(angle);
   }

 if (ps2x.Button(PSB_R1))
    { 
      throttle = map(RightStick, 255, 0, 80, 100);
      esc.write(throttle);
    }
    
else{
   throttle = map(RightStick, 255, 0, 75, 105); //note: anything above 110 causes the tires come off the rear wheels
    
   esc.write(throttle); //we should be using PWM and esc.writeMicroseconds() to control speed however,our arduino nano does not seem to support PWM
}

 if (ps2x.Button(PSB_L1)) //break button
   {
    float to = millis();
    float dt = 0.0;
    while(dt<=200)
    {
      esc.write(80);
      dt = millis()-to;
    }
   }
  
delay(15);
     
}
