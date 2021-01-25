//see changelog
//NOTE: V1_PROFI_CARRERA_1/18_SCALE

#include <TFMini.h>
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

//SoftwareSerial mySerial(4,5);      // Uno RX (TFMINI TX), Uno TX (TFMINI RX)
//TFMini tfmini;

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

int lidar_flag = 0;

int front_light_flag = 0;

int cruz_flag = 0;
//const int ledPin =  13; // for lidar
const int ledPin =  2; // for front light
const int led2Pin =  7; // for cruise control
const int led3Pin =  8; // for traction


void setup_ADC();
void traction();

  
void setup(){
           ps2x.config_gamepad(12,10,9,11, false, false);      //GamePad(clock, command, attention, data, Pressures?, Rumble?)
           servo1.attach(6);
           esc.attach(3);
           pinMode(ledPin, OUTPUT);
           pinMode(led2Pin, OUTPUT);
           pinMode(led3Pin, OUTPUT);//reserved for traction
           
            Serial.begin(115200);
            
            setup_ADC();
           
          //Lidar Setup:
          //------------
//            mySerial.begin(TFMINI_BAUDRATE);// Step 2: Initialize the data rate for the SoftwareSerial port
//            tfmini.begin(&mySerial);  // Step 3: Initialize the TF Mini sensor
            
            delay(1000);
          
          while (1){
            
              //Read Input from Controller
              //--------------------------
                 ps2x.read_gamepad();                 //reads input from ps2 controller
                 LeftStick = ps2x.Analog(PSS_LX);     //left stick horizontal controls steering
                 RightStick = ps2x.Analog(PSS_RY);    //right stick vertical controls speed
                 PadLeft =  ps2x.Button(PSB_PAD_LEFT); // left d-pad button to 
                 PadRight = ps2x.Button(PSB_PAD_RIGHT);
              
              //Steering Calibration:
              //---------------------
                if (ps2x.Button(PSB_PAD_LEFT)) //decrements of 10 microseconds to adjust the steering
                 calib -= 10;
              
                if (ps2x.Button(PSB_PAD_RIGHT)) //increments of 10 microseconds to adjust the steering
                 calib += 10;
              
                 a = calib - 300;               //a = newly calibrated "straight" position - ~35 deg to the left
                 b = calib + 300;               //b = newly calibrated "straight" position + ~35 deg to the right
              
              //Steering Control:
              //-----------------
                if(LeftStick <=145 && LeftStick >=111){    //this serves both to create a small deadzone in the steering as well as constant straight-steering control, needs testing         
                    servo1.writeMicroseconds(calib);   }                  //standard servos use 2000 as max and 1000 as min, so 1500 is the middle
                 else {
                   angle = map(LeftStick, 0, 255, a, b); //this allows for approx. 35 deg of turnning on either side, further testing needed
                   servo1.writeMicroseconds(angle);
                 }
          
              //************************************************************

              //************************************************************
                          //Front_Light
                          //-----
                            //digitalWrite(pin_front_light, LOW);
                           if (ps2x.ButtonPressed(PSB_L3)) // makes light go on
                            {
                            if (front_light_flag == 1) {
                                digitalWrite(ledPin, LOW);
                                front_light_flag = 0;
                                Serial.println("Turning off front Light, front_lihgt_flag= 0");
                            }else {
                                digitalWrite(ledPin, HIGH);
                                front_light_flag = 1;
                                Serial.println("Turning ON front light, front_lihgt_flag= 1");
                            }
                          }

                //*********************************************************************
                //*********************************************************************
                    //Acceleration-Throttle
                    //---------------------
                    if (ps2x.Button(PSB_R1)) {            //counts as launch control? 
                          Serial.println("Step 1");
//                              float d1 = tfmini.getDistance();
                              Serial.print("d1= ");
//                              Serial.println(d1);
                              //delay(50);
                           if ( lidar_flag == 1){     
                    //           esc.writeMicroseconds(1350);
                    //           delay(800);     
                              Serial.println("Step 2");
                               esc.writeMicroseconds(1500);
                               //delay(1000);
                              }
                     else if (throttle <= 1645){
                            Serial.println("Step 3");
                            n++;
                            x = n^2;
                            throttle = 1580 + x;
                            esc.writeMicroseconds(throttle);
                          }else{
                            Serial.println("Step 4");
                            esc.writeMicroseconds(1650);
                          } 
                        }  
                    else
                       {
                       Serial.println("Step 5");
                       throttle = 1500;
                       n = 5; 
                       esc.writeMicroseconds(throttle); 
                       }
                    
                    if (ps2x.Button(PSB_R2)) //break button
                       {
                    /*     float to = millis();
                         float dt = 0.0;
                        while(dt<=200)
                    */    
                          Serial.println("Step 6");
                          esc.writeMicroseconds(1350);
                    /*      dt = millis()-to;
                        }
                       }
                    */
                    }
                    
                    //Cruise Control:
                    //---------------
                      if (ps2x.Button(PSB_PAD_UP)) 
                       cruise += 20;
                    
                      if (ps2x.Button(PSB_PAD_DOWN)) 
                       cruise -= 20;
                    
  //                    if (ps2x.ButtonPressed(PSB_GREEN)) 
   //                         {     
     //                        if(cruz_flag==1) cruz_flag=0;
       //                      else cruz_flag=1;
         //                   }
                            
                      if (cruz_flag == 1) digitalWrite(led2Pin, HIGH);
                      else digitalWrite(led2Pin, LOW);
                             
                        while (cruz_flag==1) 
                          { 
                             if (ps2x.ButtonPressed(PSB_PINK))
                              {
                                if(cruz_flag == 1) cruz_flag=0;
                                else cruz_flag=1;
                                }
                      
                             angle = map(LeftStick, 0, 255, a, b); //this allows for approx. 35 deg of turning on either side
                             servo1.writeMicroseconds(angle);
                             
                             esc.writeMicroseconds(cruise);
                            
                          } 
                      
                        if (ps2x.ButtonPressed(PSB_BLUE)){
                          
                          digitalWrite(led3Pin, HIGH);
                          traction();
                          delay(2000);
                          digitalWrite(led3Pin, LOW);
                        }
                      delay(15);     
                      }
                    }

void setup_ADC(){
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



//**************************************************************************************

void traction()
                      {
                        int i,n;
                        float input_A0,voltage_A0, input_A3, voltage_A3;
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
                        const float gear_ratio = 16.0/40.0;
                      
                        int flag = 1;
                        while(flag ==1)
                        {
                        n = 100;
                        sum = 0;
                        for(i=0;i<n;i++)//average of one hundred readings for front left wheel 
                        {
                      
                          ADCSRA |= BIT(ADSC); // ADC start conversion
                          // BIT(ADSC) will read as 1 when a conversion is in process
                          // and turn to 0 when complete  
                          
                          // block / wait until ADC conversion is complete
                          k = 0;
                          while( ADCSRA & BIT(ADSC) ) k++; 
                        
                          // read the ADC (10-bits) // 0 - 1023
                          sum += ADC;
                          
                        }
                        
                        input_A0 = (float)sum / n; // average analog input
                      
                      
                        ADMUX |= BIT(MUX0) | BIT(MUX1); // select channel A3
                        sum = 0;
                        for(i=0;i<n;i++)//average of one hundred readings for main motor
                        {
                      
                          ADCSRA |= BIT(ADSC); // ADC start conversion
                          // BIT(ADSC) will read as 1 when a conversion is in process
                          // and turn to 0 when complete  
                          
                          // block / wait until ADC conversion is complete
                          k = 0;
                          while( ADCSRA & BIT(ADSC) ) k++; 
                        
                       
                          sum += ADC;
                         
                        }
                        input_A3 = (float)sum / n; // average analog input
                      
                        voltage_A0 = input_A0 * ADC_to_V; // this is much faster than / above
                        voltage_A3 = input_A3 * ADC_to_V *gear_ratio; // this is much faster than / above
                      
                        //reference slip r = 0.6
                        r = 0.6;
                      
                        //sensor value y
                        if(voltage_A3 >0)
                        {
                        y = (voltage_A3-voltage_A0)/voltage_A3;
                      
                      
                        //error e
                        e = r - y;
                        }
                        else{
                          e =0.1;//end loop
                        }
                      
                        //if slip is below reference value of 0.6, e will be positive. Assumption that slip below 0.6 is acceptable.
                        //flag of zero will allow exit from traction loop
                        if(e>=0)
                        {
                          flag = 0;
                        }
                        
                         //if slip is above reference value of 0.6, e will be negative.Input to esc will be reduced based on Kp. Program will loop to
                         //measure slip again and this loop will continue until slip falls below 0.6
                        if(e<0)
                        {
                      
                          kp = 10.0;
                      
                          u = kp*e;
                      
                          
                          u_max = 4;  // u_max <= V_bat
                          u_min = -4; // u_min >= -V_bat
                      
                          // software saturation of inputs
                          if( u > u_max ) u = u_max;
                          if( u < u_min ) u = u_min;
                      
                          throttle = throttle+u*V_tacho_inv*100;
                          if (throttle > 1680) 
                          {
                            throttle =1500;
                            flag = 0;
                          }
                          
                          if (throttle < 1500) 
                          {
                            throttle =1500;
                            flag = 0;
                          }
                          
                          esc.writeMicroseconds(throttle);
                        }
                        }
                      }
                      
void loop(){}
