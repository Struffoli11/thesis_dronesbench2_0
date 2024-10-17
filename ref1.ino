
//#include "GY_85.h"
#include "HX711.h"
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include <Arduino.h>
#include "Wire.h"
#include <I2Cdev.h>
#include "L3G4200D.h"
#include "ADXL345.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "Adafruit_BMP085_U.h"
//Adafruit_BMP085 bmp;
L3G4200D gyro;
ADXL345 accel;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);//MMC5883MA MMC5883(Wire);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

//GY_85 GY85;     //create the object
#define IR1 A0
#define IR2 A1
#define IR3 A2
#define key A3

#define SDA A4
#define CLK A5
#define Vin A6
#define Vin1 A7

#define CLK1 13

#define DOUT1 11
#define DOUT2 10
#define DOUT3 9

#define HX711 1
#define GY801 1
#define BlueTooth 0
const String seriale="00000003";
const String versione="4.0";
const String data="16/12/2016";

const int LedCentro = 2;  // led 
const int LedPwr = 3;  // led 
const int LedSud = 4;  // led 
const int rele=  5;  // Uscita rele
const int LedOvest = 6;  // led 
const int LedEst = 7;  // led 
const int LedNord = 8;  // led 
const int PwrOn = 12;  // accensione della scheda 



//HX711 scale(DOUT1,CLK1);


char inputString[10];
byte rxcont=0;
String s;         // a string to hold incoming data
boolean statopwm, pressable,stringComplete,tasto,oldtasto = false;  // whether the string is complete
int ax = 0, ay = 0, az = 0, cx = 0, cy = 0, cz = 0, i, xxg, yyg,zzg, AIR1,AIR2,AIR3;
float gx = 0, gy = 0, gz = 0, gt = 0, tensione=0, corrente=0, tensionedrone,pressione;
word counter=0, calibration_factor, ncurrentoffset=506,spegnimento;
    
int Mstrain1,Mstrain2,Mstrain3,LData1_16,LData2_16,LData3_16,LData5_16;    
byte stato,Leds,channel,oldchannel;
signed long LData1,LData2,LData3,LData4,LData5,LData6;
signed long OMstrain1,OMstrain2,OMstrain3;
int TensioneDrone,tmp,LData4_16;
int CorrenteDrone,LData6_16;
unsigned long compassX,compassY,compassZ,gyroX,gyroY,gyroZ,airtemp,tensionebanco,tensionebanco1,correntebanco;
boolean sendall=HIGH;

boolean HX711Ready(void){
  int i=1000;
  boolean ok=LOW;
  while ( (ok==LOW) && (i>0) )
    {
      ok=((digitalRead(DOUT1)==LOW)&&(digitalRead(DOUT2)==LOW)&&(digitalRead(DOUT3)==LOW));
      i--;
      delay(5);
      }
  
  return ok; 
}


void HX711Read(void){
  // channel = 1 A gain= 128    3 A gain= 64  2 B gain=32
  int i; 
  
  LData1=0;
  LData2=0;
  LData3=0;
  LData4=0;
  LData5=0;
  LData6=0;
  digitalWrite(CLK1,LOW);
  delay(100);  
  while ( HX711Ready()== LOW);
   
  for (i=0;i<24;i++)
    {
      digitalWrite(CLK1,HIGH); //pulse the clock 24 times to read the data 
      LData1=LData1*2;
      LData2=LData2*2;
      LData3=LData3*2;
      
      digitalWrite(CLK1,LOW);
      digitalWrite(CLK1,LOW);
  
      if (digitalRead(DOUT1)==HIGH) LData1++;
      if (digitalRead(DOUT2)==HIGH) LData2++;
      if (digitalRead(DOUT3)==HIGH) LData3++;
    
    }
  digitalWrite(CLK1,LOW);
  channel=2;
    
  for (i=0;i<channel;i++)
    {
      digitalWrite(CLK1,HIGH);
      digitalWrite(CLK1,HIGH); //why two times?

      digitalWrite(CLK1,LOW);
      digitalWrite(CLK1,LOW);
    }
  
  while ( HX711Ready()== LOW);
 
  for (i=0;i<24;i++)
    {
      digitalWrite(CLK1,HIGH);
      LData4=LData4*2;
      LData5=LData5*2;
      LData6=LData6*2;
      
      digitalWrite(CLK1,LOW);
  
      if (digitalRead(DOUT1)==HIGH) LData4++;
      if (digitalRead(DOUT2)==HIGH) LData5++;
      if (digitalRead(DOUT3)==HIGH) LData6++;
    
    }
  channel=1;
  digitalWrite(CLK1,HIGH);
      
  
  
  
  
  // LData1= (LData1^0x800000) & 0x00FFFFFF;
  // LData2= (LData2^0x800000) & 0x00FFFFFF;
  // LData3= (LData3^0x800000) & 0x00FFFFFF;
   
  LData1_16=(LData1 >>8)&0xFFFF;
  LData2_16=(LData2 >>8)&0xFFFF;
  LData3_16=(LData3 >>8)&0xFFFF;
   
  LData4_16=(LData4 >>8)&0xFFFF;
  LData5_16=(LData5 >>8)&0xFFFF;
  LData6_16=(LData6 >>8)&0xFFFF;
   
  LData1= (LData1 <<8) & 0xFFFFFF00;
  LData2= (LData2 <<8) & 0xFFFFFF00;
  LData3= (LData3 <<8) & 0xFFFFFF00;
   
}

void ShowLed(byte b){

  if ((b & 1)>0) digitalWrite(LedCentro,HIGH); else digitalWrite(LedCentro,LOW); 
  if ((b & 2)>0) digitalWrite(LedNord,HIGH); else digitalWrite(LedNord,LOW); 
  if ((b & 4)>0) digitalWrite(LedEst,HIGH); else digitalWrite(LedEst,LOW); 
  if ((b & 8)>0) digitalWrite(LedSud,HIGH); else digitalWrite(LedSud,LOW); 
  if ((b & 16)>0) digitalWrite(LedOvest,HIGH); else digitalWrite(LedOvest,LOW); 

}


//........................................................ SETUP ......................................................
void setup() {
  //...................................................
  // initialize serial port per Blue Tooth and cable:
  if(BlueTooth>0) {
    Serial.begin(9600);   // default value for blue tooth module
    Serial.println("AT");     delay(2000);
    Serial.println("AT+NAMEDronesBench");   delay(2000);  // imposta nome DronesBench al modulo bluetooth
    Serial.println("AT+BAUD8"); delay(2000); // imposta baudrate a 115200
 }
    Serial.begin(115200);        // imposta a 115200 anche il baud rate di arduino

//analogReference(INTERNAL);
    analogReference(EXTERNAL); //the voltage applied to the AREF pin is used as reference

    pressable=0;  
    if (GY801 >0) { //what is the meaning of this section of code?
      Wire.begin();
      TWBR=12;
      delay(10);  
      GY85.init();    delay(10);
      pressable=bmp.begin();
    }

 pinMode(DOUT1, INPUT);
 pinMode(DOUT2, INPUT);
 pinMode(DOUT3, INPUT);
 pinMode(CLK1, OUTPUT);

    pinMode(CLK1, OUTPUT);
    digitalWrite(CLK1,LOW);
 
    pinMode(LedCentro, OUTPUT);    pinMode(LedNord, OUTPUT);    pinMode(LedSud, OUTPUT);    pinMode(LedEst, OUTPUT);    pinMode(LedOvest, OUTPUT);  
    pinMode(LedPwr, OUTPUT);
    pinMode(rele, OUTPUT);
  
    pinMode(PwrOn, OUTPUT);
    digitalWrite(PwrOn,HIGH);
   
  counter=0;   
    for (i=0;i<10;i++){ //??
      ShowLed(1);  delay(100);
      ShowLed(30); delay(100);
    }
    ShowLed(0);
    channel=3;
     
   digitalWrite(CLK1,LOW);
   if(HX711>0){
     HX711Ready();
     HX711Read();
    } else{
        LData1=1;
        LData2=1;
        LData3=1;
       }
    OMstrain1=LData1;
    OMstrain2=LData2;
    OMstrain3=LData3;
//Serial.println(OMstrain1);
//Serial.println(OMstrain2);
//Serial.println(OMstrain3);
 channel=2;
  

}

//........................................................ LOOP ......................................................
void loop() {
  byte a,sensore,k;
  unsigned long duration1,duration2,durationoffset;
  const long rshunt=4.7;  // ohm
 //.................  lettura accellerometro ........................
    if (GY801>0) {
       xxg = GY85.accelerometer_x( GY85.readFromAccelerometer() );
       yyg = GY85.accelerometer_y( GY85.readFromAccelerometer() );
       zzg = GY85.accelerometer_z( GY85.readFromAccelerometer() );

//... bolla elettronica  
       Leds=0;
       if (abs(xxg) > 1) {
            if (xxg < 0) Leds=Leds | 4; else Leds=Leds | 16;
            }
        if (abs(yyg) > 1) {
            if (yyg < 0) Leds=Leds | 2; else Leds=Leds | 8;
        }
        if (Leds==0) Leds=1;
        ShowLed(Leds);  
    
        compassX = GY85.compass_x( GY85.readFromCompass() );
        compassY = GY85.compass_y( GY85.readFromCompass() );
        compassZ = GY85.compass_z( GY85.readFromCompass() );
      
        gyroX = GY85.gyro_x( GY85.readGyro() );
        gyroY = GY85.gyro_y( GY85.readGyro() );
        gyroZ = GY85.gyro_z( GY85.readGyro() );
          
      
        airtemp = GY85.temp  ( GY85.readGyro() );
       if(pressable) {
         airtemp = bmp.readTemperature();
         pressione=bmp.readPressure();
        } else {
         airtemp =0;
         pressione=0;
         }
        } else ShowLed(31);      

        tensionebanco=analogRead(Vin)*15.625;
        
        tensionebanco1=analogRead(Vin1)*15.625;
        if(tensionebanco>6000) 
        correntebanco=(tensionebanco-tensionebanco1)/rshunt;
        else correntebanco=0;
        //correntebanco=tensionebanco1;
  
        
       for (k = 0; k<8;k++) {
         AIR1=(AIR1*7+analogRead(IR1))/8;
         AIR2=(AIR2*7+analogRead(IR2))/8;
         AIR3=(AIR3*7+analogRead(IR3))/8;
         }
  
          if(HX711>0) HX711Read();
            Mstrain1=LData1_16;
            Mstrain2=LData2_16;
            Mstrain3=LData3_16;
         
 // ............. lettura corrente drone da sensore di hall .................................................
            tmp=(int)((long)(LData6_16*0.123+2337)*1.42);
                   
           CorrenteDrone=(CorrenteDrone+tmp)/2;
  //         CorrenteDrone=LData6_16;
 // ............. lettura tensione drone .................................................
           //tmp=LData4_16 *0.3173;
           tmp=LData4_16 *0.3008879;
           TensioneDrone=(TensioneDrone+tmp)/2;
                
       // TensioneDrone=LData6_16; //debug
          for(k=0;k<30;k++){
            tasto= (analogRead(key)>100);
            if (tasto==oldtasto) break;
            delay(1);
          }
          if (tasto == HIGH) spegnimento++; else spegnimento=0;
          if ((tasto != oldtasto) &&(tasto == HIGH)) {
            digitalWrite(rele,!digitalRead(rele)); 
          sendall=HIGH;
          
          }
          oldtasto=tasto;
          if (spegnimento > 10)  {
            digitalWrite(PwrOn,LOW);
            ShowLed(31); 
            delay(100);
            if(HX711>0) while(HIGH);
           }
           if ((stato&4)>0){
             AIR1=400;
             AIR2=400;
             AIR3=400;
           Mstrain1=2000;
           Mstrain2=2000;
           Mstrain3=2000;
           xxg=0;
           yyg=0;
           zzg=0;
           compassX=0;
           compassY=0;
           compassZ=0;
           gyroX=0;
           gyroY=0;
           gyroZ=0;
           airtemp=21;
           pressione=103000;
           tensionebanco=12000;
           correntebanco=120;
           TensioneDrone=1110;
           CorrenteDrone=3000;
           
           
           }
              if (sendall){
                      Serial.print('[');
                       Serial.print('M');
                       Serial.print(AIR1);  Serial.print(char(9));
                       Serial.print(AIR2);  Serial.print(char(9));
                       Serial.print(AIR3);  Serial.print(char(9));
                       Serial.print(Mstrain1);  Serial.print(char(9));
                       Serial.print(Mstrain2);  Serial.print(char(9));
                       Serial.print(Mstrain3);  Serial.print(char(9));
                       Serial.print(xxg);  Serial.print(char(9));
                       Serial.print(yyg);  Serial.print(char(9));
                       Serial.print(zzg);  Serial.print(char(9));
                       Serial.print(compassX);  Serial.print(char(9));
                       Serial.print(compassY);  Serial.print(char(9));
                       Serial.print(compassZ);  Serial.print(char(9));
                       Serial.print(gyroX);  Serial.print(char(9));
                       Serial.print(gyroY);  Serial.print(char(9));
                       Serial.print(gyroZ);  Serial.print(char(9));
                       Serial.print(airtemp);  Serial.print(char(9));
                       Serial.print(pressione);  Serial.print(char(9));
                       Serial.print(tensionebanco);  Serial.print(char(9));
                       Serial.print(correntebanco);  Serial.print(char(9));
                       Serial.print(TensioneDrone);  Serial.print(char(9));
                       Serial.print(CorrenteDrone);  Serial.print(char(9));
                       Serial.print(stato);                    Serial.print(char(9));
                 
                       Serial.print(']');
                       Serial.print(char(13));
                       Serial.print(char(10));
   }
   digitalWrite(LedPwr,HIGH);
   
              if (sendall) delay(20); else delay(200);                   
   digitalWrite(LedPwr,LOW);
   //delay(100);                   
   if (digitalRead (rele)) stato= stato | 1; else stato=stato &254;
 
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString[rxcont++] = inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    ShowLed(31); 
    if (inputString[0]!='I') rxcont=0;
              
    if (inChar == 'E') {
      
      if (inputString[1]=='R') {
         if (inputString[2]=='A') digitalWrite(rele,HIGH); else digitalWrite(rele,LOW); 
      
      }
      if (inputString[1]=='V') {
          Serial.print("[V Seriale: ");
          Serial.print(seriale);
          Serial.print("# Versione: ");
          Serial.print(versione);
          Serial.print("# data: ");
          Serial.print(data);
          
          
          Serial.println("#]");
      
      }
      if (inputString[1]=='S') {
         if (inputString[2]=='A') sendall=HIGH; else sendall=LOW; 
      
      }
      if (inputString[1]=='D') {
         if (inputString[2]=='A') stato=stato|4; else stato=stato &(255-4);; 
      
      }
         
      inputString[0]=0;
      inputString[1]=0;
      inputString[2]=0;
      inputString[3]=0;
      
      rxcont=0;
      stringComplete = true;
      
    }
  }
}



