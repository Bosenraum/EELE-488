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
#include "threshold.h"
#include <AFMotor.h>

// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
Threshold t = Threshold(70, lcd);
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

Servo servoLeft;                             // Declare left servo
int middle = 63;                             // Set the default value of the line to 63.
int threshold = 70;
uint8_t constantSpeed = 120;
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
 
 
 
void setup(void)
{
   
   pinMode(SIpin, OUTPUT);
   pinMode(CLKpin, OUTPUT);
   //pinMode (AOpin, INPUT);
 
   digitalWrite(SIpin, LOW);   // IDLE state
   digitalWrite(CLKpin, LOW);  // IDLE state
   pinMode(10, OUTPUT);        //Servo pin
   lcd.begin(16,2);
   //threshold = t.setThreshold();
   
   char line[127]; 
   boolean done = false;
   byte tempValue;
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
          delay(1000);
          lcd.setBacklight(TEAL);
        }
     }
     
    }
   lcd.clear();
   done = false;
   while( done == false){
     lcd.setCursor(0,0);
     lcd.print("Set Speed ");
     lcd.print(constantSpeed);
     lcd.print("  ");

     uint8_t buttons = lcd.readButtons();

     if (buttons) {
        if (buttons & BUTTON_UP) {
          constantSpeed +=1;
        }
        if (buttons & BUTTON_DOWN) {
          constantSpeed -=1;
        }
        if (buttons & BUTTON_SELECT) {
          done = true;
          lcd.setBacklight(GREEN);
          delay(1000);
          lcd.setBacklight(WHITE);
          } 
      }
   }
   servoLeft.attach(10); 
 
#if FASTADC
  // set prescale to 16
  sbi(ADCSRA,ADPS2);
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);
#endif
 
   Serial.begin (115200);
   motorL.setSpeed(constantSpeed);
   motorR.setSpeed(constantSpeed);
   motorR.run(RELEASE);
   motorL.run(RELEASE);
   motorL.run(BACKWARD);
   motorR.run(FORWARD);
   motorL.setSpeed(constantSpeed);
   motorR.setSpeed(constantSpeed);
  
}
 
 
 
void loop (void)
{
   unsigned long StartTime = millis();
   int i;
   int expTime;
   
   
   //motor.run(RELEASE);
 
   //delayMicroseconds (1);  /* Integration time in microseconds */
   //delay();              /* Integration time in miliseconds  */
 
 
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
   Pixel[middle] = 178;
   servoMilliseconds = middle*4.273+1300;
   servoPWMtime = servoMilliseconds; 
   servoLeft.writeMicroseconds(servoPWMtime);

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
