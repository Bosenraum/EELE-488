/*
  EELE 488 Engineering Design Montana State University 
  Garret Hilton, Meshal Albiaz, Austin Rosenbaum
  Taos TSL1401 image sensor chip 2010-07-24
  datasheet: http://www.ams.com/eng/content/download/250163/975677/file/TSL1401CL.pdf
  A Reference for the camera read process:
    - http://forums.parallax.com/showthread.php/125594-TSL1401-and-Arduino
 
    
  
*/
#include <Servo.h>                           // Include servo library
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
//#include "threshold.h"
#include <AFMotor.h>

// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
//Threshold t = Threshold(70, lcd);
AF_DCMotor motorR(3);
AF_DCMotor motorL(2);
// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

Servo servoSteer;                             // Declare left servo
int middle = 63;                             // Set the default value of the line to 63.
int threshold = 70;
uint8_t constantSpeed = 160;
boolean StopMotor = false;
float kp = 0.5;
float kd = 1;
int error;
int lastError = 0;
int constError = 10;
int lastMiddle = 63;
int Middle = 63;
int stepDown = 0;
float elapsedTotalTime = 0;
float startTotalTime =0;
float a1 = 0;
float b1 = 0;
float c1 = 0;
float d1 = 0;
float a2 = 0;
float b2 = 0;
int testy = 2;
int center;
int left;
int right;
                     // Sensor interface: 
#define AOpin  0     // Analog output - yellow
#define SIpin  8     // Start Integration - orange
#define CLKpin 9     // Clock - red
                     // Vcc - brown
                     // GND - black
 
#define NPIXELS 128  // No. of pixels in array
 
byte Pixel[NPIXELS]; // Field for measured values <0-255>
 
 
#define FASTADC 1   
 // defines for setting and clearing register bits
 #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
 #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
 
//int setThreshold(void){};
 
void setup(void)
{

   #if FASTADC
    // set prescale to 16
    sbi(ADCSRA,ADPS2);
    cbi(ADCSRA,ADPS1);
    cbi(ADCSRA,ADPS0);
   #endif
   pinMode(SIpin, OUTPUT);
   pinMode(CLKpin, OUTPUT);
   //pinMode (AOpin, INPUT);
 
   digitalWrite(SIpin, LOW);   // IDLE state
   digitalWrite(CLKpin, LOW);  // IDLE state
   pinMode(10, OUTPUT);        //Servo pin
   lcd.begin(16,2);
   servoSteer.attach(10);
   char line[127]; 
   boolean done = false;
   byte tempValue;
   setThreshold(threshold);
   calibrateServoMenu();
   setSPEED();
   c1 = left;
   d1 = right;
   a1 = c1 / (63.5*63.5) - center/(63.5*63.5);
   b1 = (-a1)*63.5 - (c1 / 63.5) + center/(63.5);
   a2 = d1 / (63.5*63.5) - center/(63.5*63.5);
   b2 = -(a2)*63.5 - (d1 / 63.5) + center/(63.5);
   Serial.begin (115200);
   motorL.setSpeed(constantSpeed);
   motorR.setSpeed(constantSpeed);
   motorR.run(RELEASE);
   motorL.run(RELEASE);
   motorL.run(BACKWARD);
   motorR.run(FORWARD);
   motorL.setSpeed(constantSpeed);
   motorR.setSpeed(constantSpeed);
   startTotalTime = millis();
}
 
 
 
void loop (void)
{
   unsigned long StartTime = millis();
   int i;
   int expTime;
   digitalWrite (CLKpin, LOW);
   digitalWrite (SIpin, HIGH);
   digitalWrite (CLKpin, HIGH);
   digitalWrite (SIpin, LOW);
 
   delayMicroseconds (1);            
 
/* and now read the real image */
   int count = 0;
   int total = 0;
   float servoMilliseconds = 1500;
   int servoPWMtime = 1500;
   byte tempValue = (byte)(127);
   for (i = 0; i < NPIXELS; i++) {
       delayMicroseconds(1);
       if(i < 25 || i > 101){
          tempValue = analogRead (AOpin)/4 ; // 8-bit is enough
        }else{
          tempValue = analogRead (AOpin)/4 ; // 8-bit is enough
        }
       if((int)tempValue < threshold){
             Pixel[i] = 177; // print line
             count++;
             total = total + i;
        }else{
             Pixel[i] = 95;  // white space
          }
       //Pixel[i] = analogRead (AOpin)/4 ; // 8-bit is enough
       digitalWrite (CLKpin, LOW);
       delayMicroseconds (1);
       digitalWrite (CLKpin, HIGH);
   }
   if (count > 0){
       middle = total/count;
    }else{
       middle = middle;
      }
   Serial.write("\n");
   Serial.write(count);
   Serial.write("\n");
   if (count > 50){
	   StopMotor = true;
     lcd.setBacklight(RED);
     elapsedTotalTime =   (millis() - startTotalTime)/1000;
     motorL.run(FORWARD);
     motorR.run(BACKWARD);
     motorL.setSpeed(150);
     motorR.setSpeed(150);
     	   
   }
   if (StopMotor == true){
     if(stepDown < 150){
         motorL.setSpeed(150 + stepDown);
         motorR.setSpeed(150 + stepDown);
         stepDown += 2; 
      }else{
         motorL.setSpeed(0);
         motorR.setSpeed(0);
         lcd.setCursor(0,0);
         lcd.print("Time = ");
         lcd.print(elapsedTotalTime);
        }

     
    }
	   
   Pixel[middle] = 178;              //set the middle line char
   //////////////////////////////PD Compensator/////////////////////////////
//   error = middle - constError;
//   Middle = kp*error + kd*(error - lastError);
//   lastError = error;
//   servoMilliseconds = Middle*4.273+1300;
   /////////////////////////////////////////////////////////////////////////

   //////////////////////////////////Two input average//////////////////////
//   Middle = 0.5*middle + 0.5*lastMiddle;
//   servoMilliseconds = Middle*4.273+1300;
//   lastMiddle = middle;
   /////////////////////////////////////////////////////////////////////////

   //////////////////////////////Original Code//////////////////////////////
//   servoMilliseconds = middle*4.273+1280;  //works tested 2/18/18 200 top speed time = 15sec  center = 1550
   /////////////////////////////////////////////////////////////////////////
   /////////////////////////////Piecewise //////////////////////////////////
//   if (middle < 22){                          //works speed tested 2/18/18 top speed time 13.7 sec speed = 200;
//      servoMilliseconds = middle*5+1235;
//   }
//   if ((21 < middle) && (middle < 43) ){
//      servoMilliseconds = middle*4.5+1267;
//   }
//   if ((42 < middle) && (middle < 86) ){
//      servoMilliseconds = middle*3.0+1361;   
//    }
//   if ((85 < middle) && (middle < 105) ){
//      servoMilliseconds = middle*4.5+1267;
//   }
//   if (middle > 104){
//      servoMilliseconds = middle*5+1235;
//    }
   ////////////////////////////////////////////////////////////////////////////
   //                         2nd order Equation 
   ////////////////////////////////////////////////////////////////////////////
   
   if (middle <= 63){                          //works speed tested 2/18/18 top speed time 13.7 sec speed = 200;
      servoMilliseconds =a1*middle*middle + b1*middle + c1;
   }
   if ((63 < middle)){
      servoMilliseconds =a2*middle*middle + b2*middle + d1;
   }
  /////////////////////////////////////////////////////////////////////////////
    
   servoPWMtime = servoMilliseconds; 
   servoSteer.writeMicroseconds(servoPWMtime);

   delayMicroseconds(500); 
   Serial.write ((byte)0);            // sync byte = 0
//   delayMicroseconds(7000);
   for (i = 0; i < NPIXELS-0; i++) {  
         Serial.write((byte)(Pixel[i]));
   }
   unsigned long CurrentTime = millis();
   unsigned long ElapsedTime = CurrentTime - StartTime;
   Serial.write("         ");
   Serial.print(ElapsedTime);
   Serial.write("         ");
   Serial.print(middle);
   Serial.write("\n");

  uint8_t buttons = lcd.readButtons();

  if (buttons) {
    if (buttons & BUTTON_UP) {
      lcd.setBacklight(RED);
    }
    if (buttons & BUTTON_DOWN) {
      lcd.setBacklight(YELLOW);
    }
    if (buttons & BUTTON_LEFT) {
      lcd.setBacklight(GREEN);
    }
    if (buttons & BUTTON_RIGHT) {
      lcd.setBacklight(TEAL);
    }
    if (buttons & BUTTON_SELECT) {
      lcd.setBacklight(VIOLET);
    }
  }
   
}
/////////////////////////////////////////////////////////////////////////////////
void calibrateServo(){
    boolean done = false;
    int servoPWMtime = 1500;
    while(done == false){ //loop to get center
      lcd.setCursor(0,0);
      lcd.print("Set Center Point");
      lcd.setCursor(0,1);
      lcd.print("PWM = ");
      lcd.print(servoPWMtime);
      uint8_t buttons = lcd.readButtons();
      if (buttons) {
        if (buttons & BUTTON_UP) {
          servoPWMtime +=5;
        }
        if (buttons & BUTTON_DOWN) {
          servoPWMtime -=5;
        }
        if (buttons & BUTTON_SELECT) {
          done = true;
          center = servoPWMtime;
          lcd.setBacklight(GREEN);
          delay(500);
          lcd.setBacklight(BLUE);
        }
      }
      servoSteer.writeMicroseconds(servoPWMtime);
    }
    servoPWMtime = 1350;
    done = false;
    while(done == false){ //loop to get left Point
      lcd.setCursor(0,0);
      lcd.print("Set Left Point");
      lcd.setCursor(0,1);
      lcd.print("PWM = ");
      lcd.print(servoPWMtime);
      uint8_t buttons = lcd.readButtons();
      if (buttons) {
        if (buttons & BUTTON_UP) {
          servoPWMtime +=5;
        }
        if (buttons & BUTTON_DOWN) {
          servoPWMtime -=5;
        }
        if (buttons & BUTTON_SELECT) {
          done = true;
          left = servoPWMtime;
          lcd.setBacklight(GREEN);
          delay(500);
          lcd.setBacklight(BLUE);
        }
      }
      servoSteer.writeMicroseconds(servoPWMtime);
    }
    servoPWMtime = 1650;
    done = false;
    while(done == false){ //loop to get right Point
      lcd.setCursor(0,0);
      lcd.print("Set Right Point");
      lcd.setCursor(0,1);
      lcd.print("PWM = ");
      lcd.print(servoPWMtime);
      uint8_t buttons = lcd.readButtons();
      if (buttons) {
        if (buttons & BUTTON_UP) {
          servoPWMtime +=5;
        }
        if (buttons & BUTTON_DOWN) {
          servoPWMtime -=5;
        }
        if (buttons & BUTTON_SELECT) {
          done = true;
          right = servoPWMtime;
          lcd.setBacklight(GREEN);
          delay(500);
          lcd.setBacklight(BLUE);
        }
      }
      servoSteer.writeMicroseconds(servoPWMtime);
    }  
  }
/////////////////////////////////////////////////////////////////////////////////
void calibrateServoMenu(){
    lcd.setBacklight(GREEN);
    boolean done = false;
    char string[]= "           ";
    while(done == false){
      lcd.setCursor(0,0);
      lcd.print("Recalibrate?");
      lcd.setCursor(0,1);
      lcd.print("Yes");
      lcd.print(string);
      lcd.print("No");
      uint8_t buttons = lcd.readButtons();
      if (buttons) {
        if (buttons & BUTTON_LEFT) {
          string[0]  = '<';
          string[1]  = '-';
          string[2]  = '-';
          string[8]  = ' ';
          string[9]  = ' ';
          string[10] = ' ';
        }
        if (buttons & BUTTON_RIGHT) {
          string[0]  = ' ';
          string[1]  = ' ';
          string[2]  = ' ';
          string[8]  = '-';
          string[9]  = '-';
          string[10] = '>';
        }
        if (buttons & BUTTON_SELECT) {
          done = true;
          lcd.setBacklight(GREEN);
          delay(500);
          lcd.setBacklight(BLUE);
          if (string[0]=='<'){
            calibrateServo();
          }
          
        }
      }
    }
  }
/////////////////////////////////////////////////////////////////////////////////
void setThreshold(int threshold){
     boolean done = false;
     byte tempValue;
     char line[127]; 
     testy = 1;
     while(done == false){
     lcd.setCursor(0,0);
     lcd.print("Set Thold ");
     lcd.print(threshold);
     digitalWrite (CLKpin, LOW);
     digitalWrite (SIpin, HIGH);
     digitalWrite (CLKpin, HIGH);
     digitalWrite (SIpin, LOW);
     for (int i = 0; i < NPIXELS; i++) {
       delayMicroseconds(1);
       tempValue = analogRead (AOpin)/4 ; // 8-bit is enough
       if((int)tempValue < threshold){
             line[i] = '#'; // print line
       }else{
             line[i] = '_';  // white space
       }
       
       digitalWrite (CLKpin, LOW);
       delayMicroseconds (1);
       digitalWrite (CLKpin, HIGH);
     }
     Serial.write("         ");
     delay(10);
     lcd.setCursor(0,1);
     lcd.print(line[0]);
     lcd.print(line[1]);
     lcd.print(line[2]);
     lcd.print(line[3]);
     lcd.print(line[4]);
     lcd.print(line[5]);
     lcd.print(line[6]);
     lcd.print(line[7]);
     lcd.print(line[120]);
     lcd.print(line[121]);
     lcd.print(line[122]);
     lcd.print(line[123]);
     lcd.print(line[124]);
     lcd.print(line[125]);
     lcd.print(line[126]);
     lcd.print(line[127]);
     uint8_t buttons = lcd.readButtons();

     if (buttons) {
        if (buttons & BUTTON_UP) {
          threshold +=1;
        }
        if (buttons & BUTTON_DOWN) {
          threshold -=1;
        }
        if (buttons & BUTTON_SELECT) {
          done = true;
          lcd.setBacklight(GREEN);
          delay(500);
          lcd.setBacklight(BLUE);
        }
     }
     
    }
  }
/////////////////////////////////////////////////////////////////////////////////
void setSPEED(){
  lcd.clear();
  boolean done = false;
  while( done == false){
    lcd.setCursor(0,0);
    lcd.print("Set Speed ");
    lcd.print(constantSpeed);
    lcd.print("  ");

    uint8_t buttons = lcd.readButtons();

    if (buttons) {
      if (buttons & BUTTON_UP) {
        constantSpeed +=5;
      }
      if (buttons & BUTTON_DOWN) {
        constantSpeed -=5;
      }
      if (buttons & BUTTON_LEFT) {
        constantSpeed = 0;
      }
      if (buttons & BUTTON_RIGHT) {
        constantSpeed = 200;
      }
      if (buttons & BUTTON_SELECT) {
        done = true;
        lcd.setBacklight(GREEN);
        lcd.setBacklight(WHITE);
      } 
    }
   }
  }
