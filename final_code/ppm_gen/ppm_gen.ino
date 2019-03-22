// PPM Code from https://code.google.com/archive/p/generate-ppm-signal/

//433MHz radio model
#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif

// PPM Config
#define chanel_number 8  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 20000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 13  //set PPM signal output pin on the arduino

//Need double check with Vincent
#define throttle 0
#define yaw 1
#define roll 2
#define pitch 3
#define arm 4


// Minimums
int motorMin = 1000;

//gesture constant
float xAngle = 0.0;
float yAngle = 0.0;
float zAngle = 0.0;
unsigned long oldTime;
unsigned long newTime;
int counter = 0;

boolean isThrottle = false;
double velocity = 0;
//---------------

//radio RH_ASK driver
RH_ASK driver(2000, 8, 6, 0); // ESP8266 or ESP32: do not use pin 11 or 2

//gesture controller info convertion 
float accToFloat(uint8_t accRead){
  float acc= 19.6/128*(accRead-128);
  return acc*(-1.0);
}

float gyroToFloat(uint8_t upper, uint8_t lower){
  uint16_t gyroRead = upper;
  gyroRead = gyroRead << 8;
  gyroRead += lower;
  float gyro = 286.8/32768.0*(gyroRead-32768.0);
  return gyro*(-1.0);
}
//-------------------

/*this array holds the servo values for the ppm signal
 change these values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number];

void setup(){  
  Serial.begin(9600);

  #ifdef RH_HAVE_SERIAL
    Serial.begin(9600);    // Debugging only
  #endif
    if (!driver.init())
  #ifdef RH_HAVE_SERIAL
    Serial.println("init failed");
  #else
    ;
  #endif
  
  //initialize servo middle pwm values
  for(int i=0; i<chanel_number-2; i++){
  ppm[i]= default_servo_value;
  }
  ppm[6] = motorMin;
  ppm[7] = motorMin;

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

void loop(){
  // Serial receive
  //recvWithStartEndMarkers();
  //
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);

  //Serial.println(driver.recv(buf, &buflen));

  //Iterate every time the 433MHz receiver receive a new set of data
  if (driver.recv(buf, &buflen)) // Non-blocking
  {
    float tDifference = (newTime-oldTime)*0.001;


    uint8_t specCom = buf[0];
    // hover: 85. stop: 170, takeOff 255, throttle: 1
    float ax = accToFloat(buf[1]);
    float ay = accToFloat(buf[2]);
    float az = accToFloat(buf[3]);
    float am = sqrt(ax*ax+ay*ay+az*az);
    float xPotion = ax/am;
    float yPotion = ay/am;
    float zPotion = az/am;


    float gx = gyroToFloat(buf[4], buf[5])*3.14/180.0;//+0.2;
    float gy = gyroToFloat(buf[6], buf[7])*3.14/180.0;//-0.51;
    float gz = gyroToFloat(buf[8], buf[9])*3.14/180.0;//-2.25;


      
    xAngle += gx*0.14;
    yAngle += gy*0.14;
    zAngle += gz*0.14;

    if(specCom == 1){
      if(az > 0){
        Serial.println("down");
        ppm[throttle] = 1000;
      }
      else{
        Serial.println("up");
        ppm[throttle] = 2000;
      }
    }
    else if(specCom == 255){
      Serial.println("Take off");
    }
    else if (specCom == 85){
      Serial.println("Hover");
    }
    else if (specCom == 170){
      Serial.println("Stop");
    }
    else if (specCom == 50){
      Serial.println("Disabled");
    }
    else{
      int pitchVal = xPotion * 500 + 1500;
      int rollVal = yPotion * 500 + 1500;
      Serial.print("pitch:");
      Serial.print(pitchVal);
      Serial.print(" ");
      Serial.print("roll:");
      Serial.println(rollVal);
      ppm[pitch] = pitchVal;
      ppm[roll] = rollVal;
    }
    Serial.println(specCom);
  }
  delay(10);
  
}

//Magical interruption function that generate ppm with timer1
ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}

