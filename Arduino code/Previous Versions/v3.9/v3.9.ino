//see changelog
//NOTE: NEEDS TESTING 

#include <TFMini.h>
#include <SoftwareSerial.h>
#include <PS2X_lib.h>  
#include <Servo.h>

#include <Arduino.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#define BIT(a) (1 << (a))

PS2X ps2x; // create PS2 Controller Class
Servo servo1;
Servo esc;

SoftwareSerial mySerial(4,5);      // Uno RX (TFMINI TX), Uno TX (TFMINI RX)
TFMini tfmini;

int LeftStick = 128;
int RightStick = 128;
int PadLeft;
int PadRight;
int angle  ;                  //sets the steering servo to mid point i.e. straight ahead
int throttle = 1500;          // assuming that 1500 microseconds is 0 speed
int calib = 1500;             //used for calibrations of the steering, initialized at 1500 i.e. straight forawrd
int cruise = 1500; 
int a, b, x, n = 5, i = 0;
int pin_lidar = 2;

//void setup_ADC();
//void traction();


void setup()
{
 Serial.begin(115200);

 cli();  // disable interrupts

 ps2x.config_gamepad(12,10,9,11, false, false);      //GamePad(clock, command, attention, data, Pressures?, Rumble?)

 servo1.attach(6);

 esc.attach(3);

//    setup_ADC();

//Lidar Setup:
//------------
  mySerial.begin(TFMINI_BAUDRATE);// Step 2: Initialize the data rate for the SoftwareSerial port
  tfmini.begin(&mySerial);  // Step 3: Initialize the TF Mini sensor
  pinMode(pin_lidar,OUTPUT);
  EICRA |= BIT(ISC00) | BIT(ISC01);// interrupt on a rising edge of INT0
  EIMSK |= BIT(INT0); // external interrupt for INT0 (pin 2 for UNO -- see attach ref)
  TCNT0 = 0;// initialize timer0 (ie micros())
 
  sei(); // enable interrupts
  delay(1000);


while (1)
{

//Reads Input from Controller
   ps2x.read_gamepad();                 //reads input from ps2 controller
   LeftStick = ps2x.Analog(PSS_LX);     //left stick horizontal controls steering
   RightStick = ps2x.Analog(PSS_RY);    //right stick vertical controls speed
   PadLeft =  ps2x.Button(PSB_PAD_LEFT); // left d-pad button to 
   PadRight = ps2x.Button(PSB_PAD_RIGHT);

//Steering Calibration:
  if (ps2x.Button(PSB_PAD_LEFT)) //decrements of 10 microseconds to adjust the steering
   calib -= 10;

  if (ps2x.Button(PSB_PAD_RIGHT)) //increments of 10 microseconds to adjust the steering
   calib += 10;

   a = calib - 300;               //a = newly calibrated "straight" position - ~35 deg to the left
   b = calib + 300;               //b = newly calibrated "straight" position + ~35 deg to the right

//Steering Control:
  if(LeftStick <=145 && LeftStick >=111)    //this serves both to create a small deadzone in the steering as well as constant straight-steering control, needs testing         
      servo1.writeMicroseconds(calib);                     //standard servos use 2000 as max and 1000 as min, so 1500 is the middle
   else {
     angle = map(LeftStick, 0, 255, a, b); //this allows for approx. 35 deg of turnning on either side, further testing needed
     servo1.writeMicroseconds(angle);
   }
   
 //if (ps2x.ButtonPressed(PSB_RED))
  //{
  //traction();
  // }

//Lidar
  digitalWrite(pin_lidar, LOW);

 float d1 = tfmini.getDistance();
/* 
 if (d1< 50.0) {
  digitalWrite(pin_lidar, HIGH) ;
  Serial.println("too furious");    
 }
*/

//Acceleration-Throttle
if (ps2x.Button(PSB_R1)) {            //counts as launch control? 
   if (d1< 50.0) {
        digitalWrite(pin_lidar, HIGH) ;
        Serial.println("too furious");    
    } else 
            if (throttle <= 1695){          // Driving Forward
                n++;
                x = n^2;
                throttle = 1580 + x;
                esc.writeMicroseconds(throttle);
            
            }else{
                esc.writeMicroseconds(1700);
            } 
}else{                          
         throttle = 1500;
         n = 5; 
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
/*
//Cruise Control:
if (cruise <= 1600 && cruise >=1400)
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
*/
delay(15);     
}
} // end setup

void loop(){}

//Lidar External Interrupt Service Routine
ISR(INT0_vect) //tfmini interrupt
{
delay (1000);
/*
  Serial.println("Started INT0 interrupt");  
  cli();
  Serial.println("too fast");
   float dt=0.0;
   static float t0= millis();
   float t1 = millis();
   
  Serial.print("t0= ");
  Serial.println(t0);
  Serial.print("t1= ");
  Serial.println(t1);
  
    Serial.println("Started INT0 interrupt");     
    while(dt<=1000){
      Serial.print("Started INT0 While loop    ");
      Serial.print("dt= ");
      Serial.println(dt);      
      esc.writeMicroseconds(1350);
  Serial.print("t1= ");
  Serial.println(t1);
      t1=  millis();
      dt++;
    }
*/
    Serial.println("Resetting pin2=LOW");
    digitalWrite(pin_lidar, LOW);
    sei();
    Serial.println("Exiting INT0 interrupt");    

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
  const float V_bat = 11.5;
  const float V_bat_inv = 1/V_bat;

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

  if(voltage_A0<voltage_A5){
    flag = 0;
  }

  if(voltage_A0>voltage_A5){
    //reduce input to ESC
  }
}
