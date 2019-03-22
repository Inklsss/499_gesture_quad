// PPM Code from https://code.google.com/archive/p/generate-ppm-signal/
// PPM Config
#define chanel_number 8  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 20000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 13  //set PPM signal output pin on the arduino
// Serial Config
const byte numChars = 41;
char receivedChars[numChars];
char tempChars[numChars];
int receivedData[chanel_number];
const int buttonPin = 2;     // the number of the pushbutton pin
boolean newData = false;
boolean setUp = true;
boolean ifStop = false;
boolean EmergencyStop = false;
// Minimums
int motorMin = 1000;
/*this array holds the servo values for the ppm signal
 change these values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number];
void setup(){  
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);// inputpin for emergency stop
  attachInterrupt(0, pin_ISR, CHANGE);  
  Serial.begin(9600);
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
  Serial.print("Setup Value: ");
  Serial.println(setUp);
  Serial.println("ifStop Value: ");
  Serial.println(ifStop);
  if (setUp){
      ppm[0] = 1478;// analogToRange(A0, 0, 2000);
      ppm[1] = 1478;//analogToRange(A1, 0, 2000);
      ppm[2] = 900;//analogToRange(A2, 0, 2000);throttle value
      ppm[3] = 1478;//analogToRange(A3, 0, 2000);
      ppm[4] = 1000;//analogToRange(A4, 0, 2000);arm mode
      ppm[5] = 1478;//analogToRange(A5, 0, 2000);
      ppm[6] = 1478;//analogToRange(A6, 0, 2000);
      ppm[7] = 1478; //analogToRange(A7, 0, 2000);
      setUp = false;
      delay(3000);
      //ppm[4] = 1500;//analogToRange(A4, 0, 2000);arm mode
  }
  else{
    if (ifStop){
      ppm[4] = 1000;
      while(1){   
      }
    }
    else{
     ppm[4] = 1900;
     Serial.println(ppm[4]);
      delay(500);
      for(int i=900;i<1100;i+=10){
        ppm[2] = i;
        delay(1000);
      }
      delay(15000);
      for(int i=1100;i>900;i-=10){
        ppm[2] = i;
        delay(1000);
      }
      ppm[4] = 1000;
      ifStop = true;
    }
  }     
}

void pin_ISR() {
  
   EmergencyStop = true;
 
}

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
      if(EmergencyStop){ 
        if(cur_chan_numb == 4){
          OCR1A = (1000 - PPM_PulseLen) * 2;
          calc_rest = calc_rest + 1000;
          cur_chan_numb++;
        }
        else{
         OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
        calc_rest = calc_rest + ppm[cur_chan_numb];
        cur_chan_numb++;
        }
      }
      else{
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
      }
    }     
  }
}
// Serial functions below from http://forum.arduino.cc/index.php?topic=396450
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============


void parseData() {      // split the data into its parts
    char * strtokIndx; // this is used by strtok() as an index
    strtokIndx = strtok(receivedChars,",");//tempChars,",");
    Serial.println(strtokIndx);
    receivedData[0] = atoi(strtokIndx);
    for (int i=1;i<chanel_number;i++) {
      strtokIndx = strtok(NULL, ",");
      receivedData[i] = atoi(strtokIndx);
    }
}
//============
void showParsedData() {
  for (int i=0;i<chanel_number;i++) {
    Serial.print(receivedData[i]);
    Serial.println();
  }
}

int analogToRange(int pinNum, int minInt, int maxInt){
  int val = analogRead(pinNum);
  double difference = double(maxInt - minInt);
  int result = int(difference*val/1024);
  return result;
}

