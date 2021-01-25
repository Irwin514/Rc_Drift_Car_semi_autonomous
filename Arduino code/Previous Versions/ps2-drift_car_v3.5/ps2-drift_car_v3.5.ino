//see changelog
//NOTE: NEEDS TESTING 

#include <PS2X_lib.h>  
#include <Servo.h>

#include <Arduino.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#define BIT(a) (1 << (a))

PS2X ps2x; // create PS2 Controller Class
Servo steering;
Servo esc;

int LeftStick = 128;
int RightStick = 128;
int PadLeft;
int PadRight;
int pwm  ;                  //sets the steering servo to mid point i.e. straight ahead
int throttle = 1500;          // assuming that 1500 microseconds is 0 speed
int calib = 1500;  
int cruise = 1500; 
int a, b, x, n = 5, i = 0;


void setup_ADC();
void traction();


void setup()
{
  TCNT1=1;
 ps2x.config_gamepad(12,10,9,11, false, false);      //GamePad(clock, command, attention, data, Pressures?, Rumble?)
 steering.attach(6);
 esc.attach(3);
  Serial.begin(9600);
  cli();  // disable interrupts
  //setup_ADC();
  
  sei(); // enable interrupts
  delay(2000);
}

void loop()
{
   ps2x.read_gamepad();                 //reads input from ps2 controller
   LeftStick = ps2x.Analog(PSS_LX);     //left stick horizontal controls steering
   RightStick = ps2x.Analog(PSS_RY);    //right stick vertical controls speed
   PadLeft =  ps2x.Button(PSB_PAD_LEFT); // left d-pad button to 
   PadRight = ps2x.Button(PSB_PAD_RIGHT);
  

  if (ps2x.Button(PSB_PAD_LEFT)) //decrements of 10 microseconds to adjust the steering
   calib -= 10;

  if (ps2x.Button(PSB_PAD_RIGHT)) //increments of 10 microseconds to adjust the steering
   calib += 10;

   a = calib - 200;               //a = newly calibrated "straight" position - ~35 deg to the left
   b = calib + 200;               //b = newly calibrated "straight" position + ~35 deg to the right

 
     
  if(LeftStick <=145 && LeftStick >=111)    //this serves both to create a small deadzone in the steering as well as constant straight-steering control, needs testing         
      steering.writeMicroseconds(calib);                     //standard servos use 2000 as max and 1000 as min, so 1500 is the middle
  
   else
   {
   pwm = map(LeftStick, 255, 0, a, b); //this allows for approx. 35 deg of turnning on either side, further testing needed
   steering.writeMicroseconds(pwm);
   }
   
 //if (ps2x.ButtonPressed(PSB_RED))
 //{
  //traction();
  // }

if (ps2x.Button(PSB_R1))            //counts as launch control?
    { 
      if (throttle <= 1740)
        {
          n++;
          x = n^2;
          throttle = 1550 + x;
          esc.writeMicroseconds(throttle);
        }

      //if (ps2x.ButtonPressed(PSB_GREEN)) //cruise control option 1
        //{ 
         // if(i==1) i=0;
         // else i=1;
         // while (i==1 && !ps2x.Button(PSB_L1) && !ps2x.Button(PSB_R1))
         // esc.writeMicroseconds(throttle);
        //}
        
        else
        esc.writeMicroseconds(1750);

    }
      
      
 else
   {
   throttle = 1500;
   n = 5; 
   esc.writeMicroseconds(throttle); 
   }


 

if (ps2x.Button(PSB_L1)) //break button
   {
    TCNT1=40000;
   }
   
while (cruise <= 1700 && cruise >=1300)
 {
  if (ps2x.Button(PSB_PAD_UP)) 
   cruise += 20;

  if (ps2x.Button(PSB_PAD_DOWN)) 
   cruise -= 20;

  if (ps2x.ButtonPressed(PSB_GREEN) && !ps2x.Button(PSB_R1) && !ps2x.Button(PSB_L1)) //cruise control option 2

    { 
       if(i==1) i=0;
       else i=1;
       while (i==1)
       esc.writeMicroseconds(cruise);
    }
   
 }
  

 
delay(15);
     
}

ISR(TIMER1_COMPA_VECT)
{
    float to = millis();
    float dt = 0.0;
    while(dt<=200)
    {
      esc.writeMicroseconds(1350);
      dt = millis()-to;
    }
    
    pwm = map(LeftStick, 255, 0, a, b); 
    steering.writeMicroseconds(pwm);
}
void setup_ADC()
{
  // select current ADC channel A0
  ADMUX = 0; // A0 is default

  // set ADC reference (max input voltage) to 5V (micrcontroller Vcc) 
  // note: the default (0) takes the reference from the Aref pin
  ADMUX |= BIT(REFS0);

  // set ADC control and status register A
  ADCSRA = 0;

  ADCSRA |= BIT(ADEN); // ADC enable


  


  
  ADCSRA |= BIT(ADPS1) | BIT(ADPS2); // 64 prescaler
  // this gives a conversion time of 60 microseconds for one channel

//  ADCSRA |= BIT(ADPS0) | BIT(ADPS2); // 32 prescaler  
  
//  ADCSRA |= BIT(ADPS2); 
  // 16 prescaler (smaller than this are very inaccurate)
  
  // note: smaller prescaler values will result in faster conversion times.
  // however, at some point accuracy might be lost -- check a given 
  // prescaler for known input voltages to verify accuracy
}

void traction(){
  int i,n;
  float input_A0,voltage_A0, input_A5, voltage_A5;
  unsigned long int t,t1,dt,k,sum;
  float y,r,u,kp,e,ed,u_max,u_min;
  int u_ESC;
  const float ADC_to_V = 1.0/1023.0*5;
  static float ei = 0.0, ep = 0.0;
  // use small tp value so initial T is large (-> initial ed is zero)
  static float tp = -1.0e6; 
  float ki,kd,T,rd;
  
  // battery voltage (V) -- assume constant for now 
  const float V_tacho = 5.0;
  const float V_tacho_inv = 1/V_tacho;

  int flag = 1;


  
  n = 100;
  sum = 0;
  for(i=0;i<n;i++) {

    ADCSRA |= BIT(ADSC); // ADC start conversion
    // BIT(ADSC) will read as 1 when a conversion is in process
    // and turn to 0 when complete  
    
    // block / wait until ADC conversion is complete
    k = 0;
    while( ADCSRA & BIT(ADSC) ) k++; 
  
    // read the ADC (10-bits) // 0 - 1023
    sum += ADC;
    
//    sum += analogRead(A0);
  }
  
  input_A0 = (float)sum / n; // average analog input


  ADMUX |= BIT(MUX0) | BIT(MUX2); // select channel A5
  sum = 0;
  for(i=0;i<n;i++) {

    ADCSRA |= BIT(ADSC); // ADC start conversion
    // BIT(ADSC) will read as 1 when a conversion is in process
    // and turn to 0 when complete  
    
    // block / wait until ADC conversion is complete
    k = 0;
    while( ADCSRA & BIT(ADSC) ) k++; 
  
    // read the ADC (10-bits) // 0 - 1023
    sum += ADC;
    
//    sum += analogRead(A5);
  }
  input_A5 = (float)sum / n; // average analog input

  
  // note that the simple division of float below takes around 40 
  // us compared to around 4 us for equivalent multiplication
  // which adds a significant time to ADC conversion (112 us vs 152 us).
  // -> avoid using float division if possible
  // -> use multiplication instead of division when possible
  // -> avoid putting float division in loops
//  voltage_A0 = input_A0 / 1023.0 * 5;
  voltage_A0 = input_A0 * ADC_to_V; // this is much faster than / above
  voltage_A5 = input_A5 * ADC_to_V; // this is much faster than / above

  //reference slip r = 0.6
  r = 0.6;

  //sensor value y
  y = (voltage_A0-voltage_A5)/voltage_A0;

  //error e
  e = r - y;


  if(e>=0){
    flag = 0;
  }

  if(e<0){

    kp = 20.0;

    u = kp*e;

    
    u_max = 4;  // u_max <= V_bat
    u_min = -4; // u_min >= -V_bat

    // software saturation of inputs
    if( u > u_max ) u = u_max;
    if( u < u_min ) u = u_min;

    u_ESC = 1500+u*V_tacho_inv*100;
    esc.writeMicroseconds(u_ESC);
  }
}
