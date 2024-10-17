#include <AM2302-Sensor.h>
#include <Wire.h>
#include <Arduino.h>
#include <I2Cdev.h>
#include "L3G4200D.h"
#include "ADXL345.h"
#include <Adafruit_HMC5883_U.h>
#include "Adafruit_BMP085_U.h"

//Adafruit_BMP085 bmp;
L3G4200D gyro;
ADXL345 accel;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);//MMC5883MA MMC5883(Wire);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);


#define IR1 A0
#define IR2 A1
#define IR3 A2
#define key A3

#define SDA A4
#define CLK A5
#define Vin A6
#define Vin1 A7

#define CLK1 13

/* questi sono i pin di uscita 
   di ogni ADC Hx711 usato per le quattro celle di carico
*/
#define DOUT1 9
#define DOUT2 11
#define DOUT3 10
// giugno 2024 : aggiunta la quarta cella di carico
#define DOUT4 8

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


boolean pressable= false;
int16_t ax, ay, az;
int16_t avx, avy, avz;
float gx = 0, gy = 0, gz = 0, gt = 0;
signed long  Mstrain1, Mstrain2, Mstrain3, Mstrain4; //valori letti da celle di carico
signed long LData1,LData2,LData3,LData4; //celle di carico
signed long LData5,LData6; //tensione e corrente drone
signed long TensioneDrone;
signed long CorrenteDrone;

// per controllo dell'IMU
sensors_event_t event;
AM2302::AM2302_Sensor am2302{5}; //Temperature & Humidity Sensor

void clk(){
  digitalWrite(CLK1, HIGH);
  digitalWrite(CLK1, LOW);
}


void imu_setup(){

  Wire.begin();                       

  Serial.println("Initializing I2C devices...");
  gyro.initialize();
  accel.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(gyro.testConnection() ? "L3G4200D connection successful" : "L3G4200D connection failed");
  Serial.println("Testing device connections...");
  Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");

  // data seems to be best when full scale is 2000
  gyro.setFullScale(2000);
  if (!bmp.begin()) Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
  
  if(!mag.begin())
    {
      /* There was a problem detecting the HMC5883 ... check your connections */
      Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
      while(1);
  }

  am2302.begin(); //temperature & humidity sensor

}

void HX711Read(void){

  //set Hx711 in Normal Mode
  digitalWrite(CLK1,LOW);

  // waits for each load cell to be ready for data retrieval
  while (digitalRead(DOUT1) != LOW && (digitalRead(DOUT2) != LOW && digitalRead(DOUT3) != LOW && (digitalRead(DOUT4) != LOW ) )) //wait until Data Line goes LOW
      ;
   
  // read a measurement from the four load cells, channel A, gain 128, +-20mV range.
  for (int i=0;i<24;i++)
    {
      // each pulse shifts out one bit, starting at the MSB, until all 24 bits are shifted out
      clk();
      LData1=LData1 << 1; // shift
      LData2=LData2 << 1 ; // shift
      LData3=LData3 << 1; // shift
      LData4=LData4 << 1; // shift 
      
      if (digitalRead(DOUT1)==HIGH) LData1++; //c1
      if (digitalRead(DOUT2)==HIGH) LData2++; //c2
      if (digitalRead(DOUT3)==HIGH) LData3++; //c3
      if (digitalRead(DOUT4)==HIGH) LData4++; //c4
    }
  clk();
  clk(); //26th pulse to change to channel B to take the Drone's current & voltage measurement

  delay(30); //wait for the channel to change properly

  digitalWrite(CLK1,LOW);

  while (digitalRead(DOUT1) != LOW && digitalRead(DOUT4) != LOW) //wait until Data Line goes LOW
    ;
   
  // read a measurement from the four load cells, channel A, gain 128, +-20mV range.
  for (int i=0;i<24;i++)
    {
      // each pulse shifts out one bit, starting at the MSB, until all 24 bits are shifted out
      clk();
      LData5=LData5 << 1; // shift
      LData6=LData6 << 1; // shift
      
      if (digitalRead(DOUT1)==HIGH) LData5++; //corrente
      if (digitalRead(DOUT4)==HIGH) LData6++; //tensione
    }
    clk(); //25th pulse to go back to channel A
  
    LData1 = LData1 << 8;
    LData2 = LData2 << 8;
    LData3 = LData3 << 8;
    LData4 = LData4 << 8;
    LData5 = LData5 << 8;
    LData6 = LData6 << 8;

    digitalWrite(CLK1,LOW);
   
}




void setup() {

  
  Serial.begin(115200);        
  analogReference(EXTERNAL);
  pinMode(DOUT1, INPUT);
  pinMode(DOUT2, INPUT);
  pinMode(DOUT3, INPUT);
  pinMode(DOUT4, INPUT);

  pinMode(LedPwr, OUTPUT);

  pinMode(CLK1, OUTPUT);

  digitalWrite(PwrOn,HIGH);
  digitalWrite(CLK1,LOW); //set Hx711 in normal mode

  // setup accelerometer, gyroscope, magnetometer & barometric sensor
  imu_setup();

  //read to set channel A and discard the raw value
  HX711Read();
}

//........................................................ LOOP ......................................................
void loop() {

  const long rshunt=4.7;  // ohm


    //...........LETTURA MAGNETOMETRO....................


        /* OLD_FIRMWARE
        // compassX = GY85.compass_x( GY85.readFromCompass() );
        // compassY = GY85.compass_y( GY85.readFromCompass() );
        // compassZ = GY85.compass_z( GY85.readFromCompass() );
        */

        mag.getEvent(&event);
        /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
        Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
        Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
        Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");


    //.................LETTURA GIROSCOPIO....................  
        
        
        /* OLD_FIRMWARE
        // gyroX = GY85.gyro_x( GY85.readGyro() );
        // gyroY = GY85.gyro_y( GY85.readGyro() );
        // gyroZ = GY85.gyro_z( GY85.readGyro() );
        */
        Serial.print("     ");
        gyro.getAngularVelocity(&avx, &avy, &avz);

        Serial.print("gyro X:");
        Serial.print(avx);
        Serial.print("\tY:");
        Serial.print(avy);
        Serial.print("\tZ:");
        Serial.print(avz);

     //..........LETTURA ACCELEROMETRO........................

      /* old_firmware
      // xxg = GY85.accelerometer_x( GY85.readFromAccelerometer() );
      // yyg = GY85.accelerometer_y( GY85.readFromAccelerometer() );
      // zzg = GY85.accelerometer_z( GY85.readFromAccelerometer() );
      */

      accel.getAcceleration(&ax, &ay, &az);

      // display tab-separated accel x/y/z values
      Serial.print("\taccel X:");
      Serial.print(ax);
      Serial.print("\tY:");
      Serial.print(ay);
      Serial.print("\tZ:");
      Serial.println(az);
    //...................LETTURA SENSORE TEMPERATURA E PRESSIONE.............
      auto status = am2302.read();
      Serial.print("\tTemperature: ");
    Serial.print(am2302.get_Temperature());

   Serial.print("\tHumidity:    ");
   Serial.print(am2302.get_Humidity());

      //......LETTURA CELLE DI CARICO (4) ..................

      HX711Read();
      Mstrain1 = (signed long) LData1;
      Mstrain2 = (signed long) LData2;
      Mstrain3 = (signed long) LData3;
      Mstrain4 = (signed long) LData4;
      Serial.print("\tLOAD_CELLS C1:");
      Serial.print(Mstrain1);
      Serial.print("\t\tC2:");
      Serial.print(Mstrain2);
      Serial.print("\t\tC3:");
      Serial.print(Mstrain3);
      Serial.print("\t\tC4:");
      Serial.print(Mstrain4);

      CorrenteDrone = (signed long) LData5;
      TensioneDrone = (signed long) LData6;

      Serial.print("\tCorrente Drone:     ");
      Serial.print(CorrenteDrone);
      Serial.print("\tTensione Drone:     ");
      Serial.println(TensioneDrone);
      delay(100);    
      LData1 = 0;  //c1
      LData2 = 0;  //c2
      LData3 = 0;  //c3
      LData4 = 0;  //c4
      LData5 = 0;  // corrente drone
      LData6 = 0;  // tensione drone              
}





