//  THIS CODE IS UPDATED TO NOV 10,
//  Code is being transfered from Arduino Due to Teensy
#include  <ODriveArduino.h>
#include  "fsm.h"
#include  "hlc.h"
#include  "llc.h"
#include  "arduino.h"
#include <Wire.h>
#include <SPI.h>
#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 myIMU;
#define radtodeg (180.0f / PI)
//#include <Filters.h>

template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

const int Enc1 = 10;
const int Enc2 = 37;

String piSend;
String piRecv;
String comma = ", ";

// calibration bool
int fl_Spring = 0;
int fl_Shoe = 0;
int fl_Sensor = 0;

// high level control
float reference = 0.0;

float hipAngle = 0.0;
float hipLast = 0.0;
float hipAngle_Vel = 0.0;
float kneeAngle = 0.0;
float kneeLast = 0.0;
float kneeAngle_Vel = 0.0;
float kneeAngleCount = 0.0;

float hipCal = 0.0;
float hipVCal = 0.0;
float kneeCal = 0.0;
float kneeVCal = 0.0;

float qI = 0.0;
float qJ = 0.0;
float qK = 0.0;
float qR = 0.0;

// low level control
float rawAngleCount = 0.0;
float rawAngle = 0.0;
float springCal = 0.0;

// shoe stuff
float k = 0.0;
float angle = 0.0;
float spoint = 0.0;

int ain0 = 0;
int ain1 = 0;
int ain2 = 0;
int ain3 = 0;

float rm45 = 0.0;
float rtoe = 0.0;
float rheel = 0.0;
float rm12 = 0.0;
float rm45Cal = 0.0;
float rtoeCal = 0.0;
float rheelCal = 0.0;
float rm12Cal = 0.0;

// odrive  ***** uncomment for odrive communication
HardwareSerial& odrive_serial = Serial2;
ODriveArduino odrive( odrive_serial );

// frequency test
float hlc_freq = 0.0;
float fsm_freq = 0.0;
float current_time = 0.0;
float timeDiff = 0.0;
float timeLast = 0.0;

hlc hlc;
fsm fsm;
llc llc;

void setup() {
  
  Serial.begin(115200);
  Serial.setTimeout(2);
  odrive_serial.begin(115200); //***** uncomment for odrive communication
  odrive_serial.setTimeout(2);
  //Serial2.begin(115200);
  
  Wire.begin();

  if (myIMU.begin() == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }
  
  Wire.setClock(400000); //Increase I2C data rate to 400kHz
  myIMU.enableRotationVector(3); //Send data update every 1ms

  pinMode(Enc1, OUTPUT);
  pinMode(Enc2, OUTPUT);

  digitalWrite(Enc1,HIGH);
  digitalWrite(Enc2,HIGH);

  SPI.begin();

  digitalWrite(Enc1,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(Enc1,HIGH);       // Terminate SPI conversation 

  digitalWrite(Enc2,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(Enc2,HIGH);       // Terminate SPI conversation 
}

void loop() {
  // put your main code here, to run repeatedly:
  if (myIMU.dataAvailable() == true)
  {
  //  delay(20);
    qI = myIMU.getQuatI();
    qJ = myIMU.getQuatJ();
    qK = myIMU.getQuatK();
    qR = myIMU.getQuatReal();
    //float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

    hipLast = hipAngle - hipCal;
    euler(qR, qI, qJ, qK);
    hipAngle_Vel = (hipLast - hipAngle_Vel);
    
    kneeLast = kneeAngle;
    kneeAngleCount = (int)readEncoder(1);
    //kneeFilter.input(kneeAngleCount);
    //kneeAngleCount = kneeFilter.output();
    kneeAngle = kneeAngleCount*360/40000 - kneeCal;
    kneeAngle_Vel = (kneeLast - kneeAngle) - kneeVCal;
   
    rawAngleCount = (int)readEncoder(2);
    rawAngle = rawAngleCount - springCal;
    
    if (Serial.available() > 0){
      piRecv = Serial.readStringUntil('\n');
      getPiData(piRecv);
    }
    //Serial.println(rawAngle);
    //ain1 = analogRead(A0);
    //ain2 = analogRead(A1);
    //ain3 = analogRead(A2);
    //ain0 = analogRead(A3);
  
    //rm45 = (ain0 - fl_First)*2100.6/1000;
    //rtoe = (ain1 - fl_Second)*2100.6/1000;
    //rheel = (ain2 - fl_Third)*2100.6/1000;
    //rm12 = (ain3 - fl_Fourth)*2100.6/1000;
  
    if (fl_Spring == 1) {
      calibrateSpring(fl_Spring);
      fl_Spring = 0;
    }
    if (fl_Shoe == 1) {
      calibrateShoe(fl_Shoe);
      fl_Shoe = 0;
    }
    if (fl_Sensor == 1) {
      calibrateSensor(fl_Sensor);
      fl_Sensor = 0;
    }
  
    fsm.readShoe(rm45, rtoe, rheel, rm12);
    reference = hlc.controller(kneeAngle, kneeAngle_Vel, hipAngle, hipAngle_Vel, fsm.k, fsm.angle, fsm.spoint);
    
    current_time = millis();
    timeDiff = current_time - timeLast;
    timeLast = current_time;
    llc.controller(reference, rawAngle);
    piSend = (String)kneeAngle + comma + (String)hipAngle + comma + (String)llc.signalx[0]+ comma + (String)llc.measurement + comma + (String)timeDiff + comma + (String)reference + comma+(String)llc.motorDirection;
    Serial.println(piSend);
    odrive.SetCurrent(0,llc.current);
  }
  
}

void calibrateSpring (int calibrate) {
  if (calibrate == 1){
    springCal = springCal + rawAngle;
  }
}

void calibrateShoe (int calibrate) {
  if (calibrate == 1){
    rm45Cal = rm45Cal + rm45;
    rtoeCal = rtoeCal + rtoe;
    rheelCal = rheelCal + rheel;
    rm12Cal = rm12Cal + rm12;
  }
}

void calibrateSensor (int calibrate) {
  if (calibrate == 1){
    hipCal = hipCal + hipAngle;
    hipVCal = hipVCal + hipAngle_Vel;
    kneeCal = kneeCal + kneeAngle;
    kneeVCal = kneeVCal + kneeAngle_Vel;
  }
}

void clearEncoderCount() {
  // Set encoder1's data register to 0
  digitalWrite(Enc1,LOW);      // Begin SPI conversation  
  
  // Write to DTR
  SPI.transfer(0x98);    
  
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(Enc1,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(Enc1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(Enc1,HIGH);     // Terminate SPI conversation
  // Set encoder2's data register to 0
  digitalWrite(Enc2,LOW);      // Begin SPI conversation  
  
  // Write to DTR
  SPI.transfer(0x98);    
  
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(Enc2,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(Enc2,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(Enc2,HIGH);     // Terminate SPI conversation
}

long readEncoder(int encoder) {
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;  
  
  // Read encoder 1
  if (encoder == 1) {
    digitalWrite(Enc1,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                     // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(Enc1,HIGH);     // Terminate SPI conversation 
  }

  // Read encoder 2

  else if (encoder == 2){
   digitalWrite(Enc2,LOW);      // Begin SPI conversation
   SPI.transfer(0x60);                     // Request count
   count_1 = SPI.transfer(0x00);           // Read highest order byte
   count_2 = SPI.transfer(0x00);           
   count_3 = SPI.transfer(0x00);           
   count_4 = SPI.transfer(0x00);           // Read lowest order byte
   digitalWrite(Enc2,HIGH);     // Terminate SPI conversation
  }
  
  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;
  
  return count_value;
}

void euler(float q0, float q1, float q2, float q3){ 
  hipAngle = -asin(2.0f * (q1 * q3 - q0 * q2));
  hipAngle *= radtodeg;
}

void getPiData(String message){
  int commaIndex = message.indexOf(',');
  int secondCommaIndex = message.indexOf(',', commaIndex+1);
  int thirdCommaIndex = message.indexOf(',', secondCommaIndex+1);
  int fourthCommaIndex = message.indexOf(',', thirdCommaIndex+1);
  int fifthCommaIndex = message.indexOf(',', fourthCommaIndex+1);
  int sixthCommaIndex = message.indexOf(',', fifthCommaIndex+1);
  int seventhCommaIndex = message.indexOf(',', sixthCommaIndex+1);
  int eighthCommaIndex = message.indexOf(',', seventhCommaIndex+1);
  int ninthCommaIndex = message.indexOf(',', eighthCommaIndex+1);
  int tenthCommaIndex = message.indexOf(',', ninthCommaIndex+1);
  int eleventhCommaIndex = message.indexOf(',', tenthCommaIndex+1);
  int twelthCommaIndex = message.indexOf(',',eleventhCommaIndex+1);
  int thirteenthCommaIndex = message.indexOf(',', twelthCommaIndex+1);
  int fourteenthCommaIndex = message.indexOf(',',thirteenthCommaIndex+1);

  String firstValue = message.substring(0, commaIndex);
  String secondValue = message.substring(commaIndex+1, secondCommaIndex);
  String thirdValue = message.substring(secondCommaIndex+1, thirdCommaIndex);
  String fourthValue = message.substring(thirdCommaIndex+1, fourthCommaIndex);
  String fifthValue = message.substring(fourthCommaIndex+1, fifthCommaIndex);
  String sixthValue = message.substring(fifthCommaIndex+1, sixthCommaIndex);
  String seventhValue = message.substring(sixthCommaIndex+1, seventhCommaIndex);
  String eighthValue = message.substring(seventhCommaIndex+1, eighthCommaIndex);
  String ninthValue = message.substring(eighthCommaIndex+1, ninthCommaIndex);
  String tenthValue = message.substring(ninthCommaIndex+1, tenthCommaIndex);
  String eleventhValue = message.substring(tenthCommaIndex+1, eleventhCommaIndex);
  String twelthValue = message.substring(eleventhCommaIndex+1, twelthCommaIndex);
  String thirteenthValue = message.substring(twelthCommaIndex+1, thirteenthCommaIndex);
  String fourteenthValue = message.substring(thirteenthCommaIndex+1, fourteenthCommaIndex);
  String fifthteenthValue = message.substring(fourteenthCommaIndex+1);

  fl_Shoe = firstValue.toInt();
  fl_Sensor = secondValue.toInt();
  fl_Spring = thirdValue.toInt();
  hlc.gravity = fourthValue.toInt(); 
  hlc.impedance = fifthValue.toInt();
  llc.Kp = sixthValue.toFloat();
  llc.Ki = seventhValue.toFloat();
  llc.Kd = eighthValue.toFloat();
  llc.motorDirection = ninthValue.toFloat();
  llc.paused = tenthValue.toInt();
  hlc.sinusoidal = eleventhValue.toInt();
  hlc.manual = twelthValue.toInt();
  hlc.manRef = thirteenthValue.toFloat();
  hlc.freq = fourteenthValue.toFloat();
  hlc.amp = fifthteenthValue.toFloat();
}  
