//see changelog
//NOTE: NEEDS TESTING 

#include <PS2X_lib.h>  
#include <Servo.h>


PS2X ps2x; // create PS2 Controller Class
Servo servo1;
Servo esc;

int LeftStick = 128;
int RightStick = 128;
int PadLeft;
int PadRight;
int angle  ;                  //sets the steering servo to mid point i.e. straight ahead
int throttle = 1500;          // assuming that 1500 microseconds is 0 speed
int calib = 1500;             //used for calibrations of the steering, initialized at 1500 i.e. straight forawrd
int a;
int b;
int n = 1;
int x;

void setup()
{
 ps2x.config_gamepad(12,10,9,11, false, false);      //GamePad(clock, command, attention, data, Pressures?, Rumble?)
 servo1.attach(6);
 esc.attach(3);
  Serial.begin(9600);
  delay(2000);
}

void loop()
{
   ps2x.read_gamepad();                 //reads input from ps2 controller
   LeftStick = ps2x.Analog(PSS_LX);     //left stick horizontal controls steering
   RightStick = ps2x.Analog(PSS_RY);    //right stick vertical controls speed
   PadLeft =  ps2x.Button(PSB_PAD_LEFT); // left d-pad button to 
   PadRight = ps2x.Button(PSB_PAD_RIGHT);

  if (ps2x.Button(PSB_PAD_LEFT)); //decrements of 10 microseconds to adjust the steering
   calib -= 10;

  if (ps2x.Button(PSB_PAD_RIGHT)); //increments of 10 microseconds to adjust the steering
   calib += 10;

   a = calib - 200;               //a = newly calibrated "straight" position - ~35 deg to the left
   b = calib + 200;               //b = newly calibrated "straight" position + ~35 deg to the right

  Serial.println("calib: ");
  Serial.print(calib);
  Serial.println("a: ");
  Serial.print(a);
  Serial.println("b: ");
  Serial.print(b);
     
  if(LeftStick <=138 && LeftStick >=118)    //this serves both to create a small deadzone in the steering as well as constant straight-steering control, needs testing         
      servo1.writeMicroseconds(calib);                     //standard servos use 2000 as max and 1000 as min, so 1500 is the middle
  
   else
   {
   angle = map(LeftStick, 255, 0, a, b); //this allows for approx. 35 deg of turnning on either side, further testing needed
   servo1.writeMicroseconds(angle);
   }

if (ps2x.Button(PSB_R1))            //counts as launch control?
    { 
      if (throttle <= 1780)
      {
        n++;
        x = n^2;
        throttle = 1500 + x;
        esc.writeMicroseconds(throttle);
      }
      else{
        esc.writeMicroseconds(1800);
      } 
    }  
else
   {
   throttle = 1500;
   n = 1; 
   esc.writeMicroseconds(throttle); 
   }

if (ps2x.Button(PSB_L1)) //break button
   {
    float to = millis();
    float dt = 0.0;
    while(dt<=200)
    {
      esc.writeMicroseconds(1350);
      dt = millis()-to;
    }
   }
  
delay(15);
     
}
