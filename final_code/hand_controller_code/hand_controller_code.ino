#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> 
#endif
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h> 
//9 DOF Sensor
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK 10
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI 11
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}
    int SpecialMessage = 50;
    float cgx = 0.0;
    float cgy = 0.0;
    float cgz =0.0;
    float cax = 0.0;
    float cay =0.0;
    float caz = 0.0;
    uint16_t cgxi = 0;
    uint16_t  cgyi = 0;
    uint16_t  cgzi = 0;
    uint16_t  caxi = 0;
    uint16_t  cayi = 0;
    uint16_t  cazi = 0;

//Transmitter
RH_ASK driver(2000, 8, 6, 0); //Recieve, Transmit, Push to talk
uint8_t msg[10] = {SpecialMessage,caxi,cayi,cazi,(cgxi>>8),cgxi,(cgyi>>8),cgyi,(cgzi>>8),cgzi};

//FlexSensor
const int FLEX_PIN = A9; // Pin connected to voltage divider output
const float VCC = 3.25; // Measured voltage of Ardunio 3.3V line
const float R_DIV = 35700.0; // Measured resistance of 36k resistor
const float STRAIGHT_RESISTANCE = 23683.90; // resistance when straight
const float BEND_RESISTANCE = 46185.88; // resistance at 90 deg

//LED
int RGB_red = 12;
int RGB_green = 13;
int RGB_blue = 14;
int Stop = 7;
int Hover = 8;
int enable = 0;

int HoverState =0;
int StopState =0;

void setup() {

  //Transmitter
  driver.init();
  
  Serial.begin(115200);
  //9 DOF Sensor
  lsm.begin();
  setupSensor();

  //Buttons
  pinMode(Stop, INPUT_PULLUP);
  pinMode(Hover, INPUT_PULLUP);
  //FlexSensor
  pinMode(FLEX_PIN, INPUT);
  //LED
  pinMode(RGB_red, OUTPUT);
  pinMode(RGB_green, OUTPUT);
  pinMode(RGB_blue, OUTPUT);
}

  void hoverInterrupt(){
  if(HoverState == 0 && StopState ==0&&enable ==1){
     msg[0] = 85;
     digitalWrite(RGB_green, LOW);  
     enable = 0;
     HoverState = 1;
     for(int i=0; i<3; i++){
      driver.send(msg, 10);
      driver.waitPacketSent();
      delay(200);
     }
  }
  else if(HoverState == 1 && StopState ==0){
    digitalWrite(RGB_green, HIGH);  
    msg[0] = 0;
    enable = 1;
    HoverState = 0;
  }

}

void stopInterrupt(){

     msg[0] = 170;
     digitalWrite(RGB_green, LOW);  
     enable = 0;
     StopState = 1;
     HoverState = 0;
     
      for(int i=0; i<3; i++){
      driver.send(msg, 10);
      driver.waitPacketSent();
      delay(200);
     }
     msg[0] = 50;
 
}

void loop() {
  if (digitalRead(Stop)== LOW){
      stopInterrupt();
    }
    if (digitalRead(Hover)== LOW){
      hoverInterrupt();
    }
 
  if(enable == 0 && HoverState ==0){
    int flexADC = analogRead(FLEX_PIN);
    float flexV = flexADC * VCC / 1023.0;
    float flexR = R_DIV * (VCC / flexV - 1.0);
     if(flexR> BEND_RESISTANCE){ 
        msg[0] = 255;
        enable = 1; 
        StopState = 0;
        digitalWrite(RGB_green, HIGH);   
     }
  }
  if(enable ==1){
    digitalWrite(RGB_green, HIGH);   
  }
  driver.send(msg, 10);
  driver.waitPacketSent();
  delay(200);
  
  while(enable ==1){
    if (digitalRead(Stop)== LOW){
      stopInterrupt();
      break;
    }
    if (digitalRead(Hover)== LOW){
      hoverInterrupt();
      break;
    }
    lsm.read(); 

    sensors_event_t a, m, g, temp;
  
    lsm.getEvent(&a, &m, &g, &temp); 
  
    int SpecialMessage = 0;
    float cgx = g.gyro.x*(32768/286.8) + 32768;
    float cgy = g.gyro.y*(32768/286.8) + 32768;
    float cgz = g.gyro.z*(32768/286.8) + 32768;
     if (digitalRead(Stop)== LOW){
      stopInterrupt();
      break;
    }
  
    float cax = a.acceleration.x*(128/19.6) + 128;
    float cay = a.acceleration.y*(128/19.6) + 128;
    float caz = a.acceleration.z*(128/19.6) + 128;
     if (digitalRead(Stop)== LOW){
      stopInterrupt();
      break;
    }
    if (digitalRead(Hover)== LOW){
      hoverInterrupt();
      break;
    }
  
    uint16_t cgxi = (uint16_t ) cgx;
    uint16_t  cgyi = (uint16_t ) cgy;
    uint16_t  cgzi = (uint16_t ) cgz;

     if (digitalRead(Stop)== LOW){
      stopInterrupt();
      break;
    }
    if (digitalRead(Hover)== LOW){
      hoverInterrupt();
      break;
    }
    uint16_t  caxi = (uint16_t ) cax;
    uint16_t  cayi = (uint16_t ) cay;
    uint16_t  cazi = (uint16_t ) caz;
    if (digitalRead(Stop)== LOW){
      stopInterrupt();
      break;
      
    }
    if (digitalRead(Hover)== LOW){
      hoverInterrupt();
      break;
    }
    msg[0] = 0;
    msg[1] = caxi;
    msg[2] = cayi;
    msg[3] = cazi;
    msg[4] = (cgxi>>8);
    msg[5] = cgxi;
    msg[6] = (cgyi>>8);
    msg[7] = cgyi;
    msg[8] = (cgzi>>8);
    msg[9] = cgzi; 
    int flexADC = analogRead(FLEX_PIN);
    float flexV = flexADC * VCC / 1023.0;
    float flexR = R_DIV * (VCC / flexV - 1.0);
     if(flexR> BEND_RESISTANCE){ 
        msg[0] = 1;
     }
         if (digitalRead(Stop)== LOW){
      stopInterrupt();
      break;
    }
    if (digitalRead(Hover)== LOW){
      hoverInterrupt();
      break;
    }
    driver.send(msg, 10);
    driver.waitPacketSent();
    if (digitalRead(Stop)== LOW){
      stopInterrupt();
      break;
    }
    if (digitalRead(Hover)== LOW){
      hoverInterrupt();
      break;
    }
    
  }

    
}

