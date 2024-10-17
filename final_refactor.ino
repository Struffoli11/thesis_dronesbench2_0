#include <L3G.h>

#include <ADXL345_WE.h>

/*
  As of June 19th 2024, this is the firmware installed on the Arduino Nano that
  controls the data acquisition and transfer from the monitoring board of the DronesBench 2.0 project. 
  This firmware is nothing more than a refactorized version of the firmware used previously on DronesBench 1.0.
  Indeed, what has been done can be summarized into five tasks:
  1. rearranging code that was previously chaotic and/or erroneous
  2. removing parts of the old firmware that are now not used anymore
  3. modularizing the code so that new functions and modules can be added more quickly
  4. adding the necessary functions that handle the communication between the various modules installed on the monitoring board
  5. extending the number of load cells measurements to four.

  For further development, it is strongly suggested to better define the architectural style of the firmware itself
  in order to achive greater flexibility and manageability, as we tried to do as stated at point (4). 
*/


#include <AM2302-Sensor.h>
#include <Wire.h>
#include <Arduino.h>
#include <I2Cdev.h>
#include <Adafruit_HMC5883_U.h>
#include "Adafruit_BMP085_U.h"


//Adafruit_BMP085 bmp;
L3G gyro;
ADXL345_WE adxl;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);  //MMC5883MA MMC5883(Wire);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

#define SDA A4
#define CLK A5
#define Vin A6
#define Vin1 A7

#define CLK1 13

/* 
  a list of the pins used
  to read data that the HX711
  converts from each of the four Load Cells
*/
#define DOUT1 9   //C1
#define DOUT2 11  //C2
#define DOUT3 10  //C3
// june 2024 : the fourth load cell is added into the firmware
#define DOUT4 8  //C4

#define BlueTooth 0
const String seriale = "00000003";
const String versione = "4.0";
const String data = "16/12/2016";

const int LedCentro = 2;  // led
const int LedPwr = 3;     // led
const int LedSud = 4;     // led
const int rele = 5;       // Uscita rele
const int LedOvest = 6;   // led
const int LedEst = 7;     // led
const int LedNord = 8;    // led
const int PwrOn = 12;     // accensione della scheda



//HX711 scale(DOUT1,CLK1);


int16_t ax, ay, az;
int16_t avx, avy, avz;
float gx = 0, gy = 0, gz = 0, gt = 0;
long LData1 = 0, LData2 = 0, LData3 = 0, LData4 = 0;
long LData5 = 0, LData6 = 0;  //voltage and drone's current read from one of the Hx711 ADC converters

//sign bits0

long mock1, mock2;
long mock3, mock4, mock5, mock6;

sensors_event_t event;              // imu event data structure
AM2302::AM2302_Sensor am2302{ 5 };  //Temperature & Humidity Sensor

/*
  This function is used to generate a pulse on the Clock Line of the HX711.
*/
void clk() {
  digitalWrite(CLK1, HIGH);
  digitalWrite(CLK1, LOW);
}

/*
  This function sets up the IMU module used on the monitoring board.
  The IMU is composed of a three-axis gyroscope, a three-axis accelerometer,
  a three-axis magnetometer (ignored) and a temperature and pressure sensor.
  To setup each sensor, library functions were used to hide low-level programming of the module itself.
  Please refer to each of the sensors libraries to get more informations. 
*/
void imu_setup() {
  gyro.init();
  gyro.enableDefault(); //full scale = +/- 245 dps (degrees per second)
  adxl.init();
  adxl.setDataRate(ADXL345_DATA_RATE_12_5);
  adxl.setRange(ADXL345_RANGE_2G);

  // verify connection
  // data seems to be best when full scale is 2000
  //gyro.setFullScale(2000);
  bmp.begin();

  mag.begin();

  am2302.begin();  //temperature & humidity sensor
}

/*
  This function reads data from channel A of each HX711, and sets channel B for the next data transfer.
  Initially, the clock line is set to low level to put the HX711 in normal mode.
  Then, a series of pulses on the clock line (equal for each of the four HX711)
  comands to transfer the data.
  According to the number of pulses sent on the clock line, we select the gain and the channel used
  for the next data transmittion.
  Please refer to the HX711 datasheet for more information. 
*/
void HX711ReadA(void) {
  mock3 = 0;
  mock4 = 0;
  mock5 = 0;
  mock6 = 0;
  LData1 = 0;
  LData2 = 0;
  LData3 = 0;
  LData4 = 0;
  //set Hx711 in Normal Mode
  digitalWrite(CLK1, LOW);

  // waits for each load cell to be ready for data retrieval
  while (digitalRead(DOUT1) != LOW && (digitalRead(DOUT2) != LOW && digitalRead(DOUT3) != LOW && (digitalRead(DOUT4) != LOW)))  //wait until Data Line goes LOW
    ;

  // read a measurement from the four load cells, channel A, gain 128, +-20mV range.
  for (int i = 0; i < 24; i++) {
    // each pulse shifts out one bit, starting at the MSB, until all 24 bits are shifted out
    clk();
    LData1 = LData1 << 1;  // shift
    LData2 = LData2 << 1;  // shift
    LData3 = LData3 << 1;  // shift
    LData4 = LData4 << 1;  // shift

    if (digitalRead(DOUT1) == HIGH) LData1++;  //c1
    if (digitalRead(DOUT2) == HIGH) LData2++;  //c2
    if (digitalRead(DOUT3) == HIGH) LData3++;  //c3
    if (digitalRead(DOUT4) == HIGH) LData4++;  //c4
  }
  clk();
  clk();  //26th pulse to change to channel B to take the Drone's current & voltage measurement soon after

  digitalWrite(CLK1, LOW);

  //-------------------------------------------------------------------------------------------
  //VERY IMPORTANT : note that changing gain/channel takes up to 400 ms (page 3 of HX711 datasheet)
  delay(100);
  //-------------------------------------------------------------------------------------------

  mock3 = LData1;
  mock4 = LData2;
  mock5 = LData3;
  mock6 = LData4;

  mock3 = mock3 >> 23;  // extract sign bit
  mock4 = mock4 >> 23;
  mock5 = mock5 >> 23;
  mock6 = mock6 >> 23;




  if (mock3 == 1) {
    LData1 |= 0xFF000000;
  } else LData1 |= 0x00000000;

  if (mock4 == 1) {
    LData2 |= 0xFF000000;
  } else LData2 |= 0x00000000;

  if (mock5 == 1) {
    LData3 |= 0xFF000000;
  } else LData3 |= 0x00000000;

  if (mock6 == 1) {
    LData4 |= 0xFF000000;
  } else LData4 |= 0x00000000;


  // //TESTING



  // Serial.println();
  // Serial.println(mock3, BIN);
  // Serial.println(LData1, BIN);
  // Serial.println(LData1);
  // Serial.println(Mstrain1, BIN);

  // Serial.println(mock4);
  // Serial.println(LData2, BIN);
  // Serial.println(LData2);
  // Serial.println(Mstrain2, BIN);

  // Serial.println(mock5);
  // Serial.println(LData3, BIN);
  // Serial.println(LData3);
  // Serial.println(Mstrain3, BIN);

  // Serial.println(mock6);
  // Serial.println(LData4, BIN);
  // Serial.println(LData4);
  // Serial.println(Mstrain4, BIN);
  // Serial.println();
}

/*
  This function reads data from channel B of each HX711, and sets channel A for the next data transfer.
  Initially, the clock line is set to low level to put the HX711 in normal mode.
  Then, a series of pulses on the clock line (equal for each of the four HX711)
  comands to transfer the data.
  According to the number of pulses sent on the clock line, we select the gain and the channel used
  for the next data transmittion.
  Please refer to the HX711 datasheet for more information.
  */
void HX711ReadB() {
  mock1 = 0;
  mock2 = 0;  //sign bit
  LData5 = 0;
  LData6 = 0;
  digitalWrite(CLK1, LOW);  //SCK is made LL

  while (digitalRead(DOUT1) != LOW && digitalRead(DOUT4) != LOW)  //wait until Data Line goes LOW
    ;

  //x=0;
  for (int i = 0; i < 24; i++)  //read 24-bit data from HX711
  {
    clk();  //generate CLK pulse to get MSB-it at A1-pin
    //bitWrite(x, 0, digitalRead(DOUT1));
    //x = x << 1;
    LData5 = LData5 << 1;
    LData6 = LData6 << 1;
    if (digitalRead(DOUT1) == HIGH) LData5++;
    if (digitalRead(DOUT4) == HIGH) LData6++;
  }
  clk();  //25th pulse


  digitalWrite(CLK1, LOW);

  //-------------------------------------------------------------------------------------------
  //VERY IMPORTANT : note that changing gain/channel takes up to 400 ms (page 3 of HX711 datasheet)
  delay(100);
  //-------------------------------------------------------------------------------------------
  mock1 = LData5;
  mock2 = LData6;

  mock1 = mock1 >> 23;  // extract sign bit
  mock2 = mock2 >> 23;

  if (mock1 == 1) {
    LData5 |= 0xFF000000;
  } else LData5 |= 0x00000000;

  if (mock2 == 1) {
    LData6 |= 0xFF000000;
  } else LData6 |= 0x00000000;
}


void setup() {

  Wire.begin();
  Serial.begin(115200);
  analogReference(EXTERNAL);
  pinMode(DOUT1, INPUT);
  pinMode(DOUT2, INPUT);
  pinMode(DOUT3, INPUT);
  pinMode(DOUT4, INPUT);

  pinMode(LedPwr, OUTPUT);

  pinMode(CLK1, OUTPUT);

  digitalWrite(PwrOn, HIGH);
  digitalWrite(CLK1, LOW);  //set Hx711 in normal mode

  // setup accelerometer, gyroscope, magnetometer & barometric sensor
  imu_setup();
  //accelSetup();
  //read to set channel A and discard the raw values
  HX711ReadA();
  HX711ReadB();
}

void loop() {

  const long rshunt = 4.7;  // ohm
  mag.getEvent(&event);
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  //Serial.print("X: ");
  Serial.print(event.magnetic.x);
  Serial.print("  ");
  //Serial.print("Y: ");
  Serial.print(event.magnetic.y);
  Serial.print("  ");
  //Serial.print("Z: ");
  Serial.print(event.magnetic.z);
  Serial.print("  ");
  //Serial.println("uT");
  //microTesla

  //Serial.print("     ");
  gyro.read();

  //Serial.print("gyro X:");
  Serial.print(gyro.g.x);
  Serial.print("  ");
  //Serial.print("\tY:");
  Serial.print(gyro.g.y);
  Serial.print("  ");
  //Serial.print("\tZ:");
  Serial.print(gyro.g.z);
  Serial.print("  ");

  xyzFloat raw = adxl.getRawValues();
  xyzFloat g = adxl.getGValues();

  //Serial.print("Raw-x = ");
  //Serial.print(raw.x);
  //Serial.print("  |  Raw-y = ");
  //Serial.print(raw.y);
  //Serial.print("  |  Raw-z = ");
  //Serial.print(raw.z);

  //Serial.print(" g-x   = ");
  Serial.print(g.x);
  Serial.print("  ");
  Serial.print(g.y);
  Serial.print("  ");
  Serial.print(g.z);
  Serial.print("  ");


  auto status = am2302.read();
  //Serial.print("\tTemperature: ");
  Serial.print(am2302.get_Temperature());
  Serial.print("  ");
  //Serial.print("\tHumidity:    ");
  Serial.print(am2302.get_Humidity());
  Serial.print("  ");


  HX711ReadA();
  //10 Settembre 2024
  //Mstrain1 = (signed long) LData1;
  //Mstrain2 = (signed long) LData2;
  //Mstrain3 = (signed long) LData3;
  //Mstrain4 = (signed long) LData4;
  //Serial.print("\tLOAD_CELLS C1:");
  Serial.print(LData1);
  Serial.print("  ");
  //Serial.print("\t\tC2:");
  Serial.print(LData2);
  Serial.print("  ");
  //Serial.print("\t\tC3:");
  Serial.print(LData3);
  Serial.print("  ");
  //Serial.print("\t\tC4:");
  Serial.print(LData4);
  Serial.print("  ");

  //delay(500);//note that changing gain/channel takes up to 400 ms (page 3 of HX711 datasheet)

  HX711ReadB();
  //Serial.print("\tCorrente Drone:     ");
  Serial.print(LData5);
  Serial.print("  ");
  //Serial.print("\tTensione Drone:     ");
  Serial.println(LData6);

  //delay(300);
}
