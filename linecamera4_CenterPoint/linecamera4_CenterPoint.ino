/*
  TSL1401test --- Taos TSL1401 image sensor chip 2010-07-24
  datasheet: http://www.ams.com/eng/content/download/250163/975677/file/TSL1401CL.pdf
  
  trace: http://ap.urpi.fei.stuba.sk/sensorwiki/index.php/TSL1401_Line_Sensor
  other inos:
    - http://forums.parallax.com/showthread.php/125594-TSL1401-and-Arduino
    - https://github.com/ap-tech/Getting-started-with-the-TSL1401CL-linescan-camera-with-arduino-and-processing.-/blob/master/TSL1401CL%20linescan%20camera%20code./Linescane_camera_code/Linescane_camera_code.ino
    
  
*/
#include <Servo.h>                           // Include servo library
 
Servo servoLeft;                             // Declare left servo
int middle = 63;                             // Set the default value of the line to 63.
 
                     // Sensor interface: 
#define AOpin  0     // Analog output - yellow
#define SIpin  3     // Start Integration - orange
#define CLKpin 2     // Clock - red
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
  
   servoLeft.attach(10); 
 
#if FASTADC
  // set prescale to 16
  sbi(ADCSRA,ADPS2);
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);
#endif
 
   Serial.begin (115200);
}
 
 
 
void loop (void)
{
   unsigned long StartTime = millis();
   int i;
   int expTime;
 
 
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
       if((int)tempValue < 70){
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
   //Serial.write("         ");
   //Serial.print(ElapsedTime);
   Serial.write("         ");
   Serial.print(middle);
   Serial.write("\n");
   
}
