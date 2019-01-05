#include <PulsePosition.h>
#include <FlexCAN.h>

//CAN
static CAN_message_t msg0;
elapsedMillis RXtimer;
elapsedMillis LEDtimer;
elapsedMillis PPMtimer;
uint32_t RXCount = 0;
boolean displayCAN = true;
boolean maxPPM = true;
float uOut = 1000;
boolean grow = true;
boolean growT = true;
float aOut = 1500;
float eOut = 1500;
float tOut = 1000;
float rOut = 1500;
float bumpsize = 1;

elapsedMillis timeSinceLastSend;


#define SWITCH_POS_1 1850 //ATTI
#define SWITCH_POS_2 1550 //FS
#define SWITCH_POS_3 1200 //FS
#define SWITCH_POS_4 1175 //Manual


//PWM Signal
//#define DEADBAND 10 //Controller tolerance
//#define CONTROL_RATE 800 //control rate of timer in Hz (this is 2X receiver output! NYYYYYQUIST!)
#define INPUT_RATE 400 //PWM input rate
#define M1PIN_IN 24
#define M2PIN_IN 25
#define M3PIN_IN 26
#define M4PIN_IN 27
#define M1_FLAG 1
#define M2_FLAG 2
#define M3_FLAG 4
#define M4_FLAG 8


int16_t PWM1;
int16_t PWM2;
int16_t PWM3;
int16_t PWM4;
volatile uint8_t bUpdateFlagsShared;
uint16_t startM1 = 0;
uint16_t startM2 = 0;
uint16_t startM3 = 0;
uint16_t startM4 = 0;
volatile uint16_t outM1 = 0;
volatile uint16_t outM2 = 0;
volatile uint16_t outM3 = 0;
volatile uint16_t outM4 = 0;

//PPM
PulsePositionOutput ppmOut;
  
const uint8_t redLEDpin = 13;
boolean redLEDstate;

void INT_M1() {
  //if high, start timer.
  bUpdateFlagsShared |= M1_FLAG;
  if (digitalRead(M1PIN_IN) == HIGH) {
    startM1 = micros();
  } else {
    outM1 = (uint16_t) (micros() - startM1);
  }
}

void INT_M2() {
  //if high, start timer.
  bUpdateFlagsShared |= M2_FLAG;
  if (digitalRead(M2PIN_IN) == HIGH) {
    startM2 = micros();
  } else {
    outM2 = (uint16_t) (micros() - startM2);
  }
}

void INT_M3() {
  //if high, start timer.
  bUpdateFlagsShared |= M3_FLAG;
  if (digitalRead(M3PIN_IN) == HIGH) {
    startM3 = micros();
  } else {
    outM3 = (uint16_t) (micros() - startM3);
  }
}

void INT_M4() {
  //if high, start timer.
  bUpdateFlagsShared |= M4_FLAG;
  if (digitalRead(M4PIN_IN) == HIGH) {
    startM4 = micros();
  } else {
    outM4 = (uint16_t) (micros() - startM4);
  }
}

class CANClass : public CANListener 
{
  public:
     void printFrame(CAN_message_t &frame, int mailbox);
     void frameToCommand(CAN_message_t &frame, int mailbox);
     void gotFrame(CAN_message_t &frame, int mailbox); //overrides the parent version so we can actually do something
  
  private:
    //int 
};

volatile double requestedX = 0, requestedY = 0, requestedZ = 0, requestedYaw = 0;
void CANClass::frameToCommand(CAN_message_t &frame, int mailbox)
{
  if (frame.id != 0x1E040170) {
    return;
  }
  uint8_t a = frame.buf[0];
  frame.buf[0] = frame.buf[1];
  frame.buf[1] = a;
  a = frame.buf[2];
  frame.buf[2] = frame.buf[3];
  frame.buf[3] = a;
  a = frame.buf[4];
  frame.buf[4] = frame.buf[5];
  frame.buf[5] = a;
  a = frame.buf[6];
  frame.buf[6] = frame.buf[7];
  frame.buf[7] = a;
  
  int16_t intZ = ((int16_t*)frame.buf)[0];
  int16_t intX = ((int16_t*)frame.buf)[1];
  int16_t intY = ((int16_t*)frame.buf)[2];
  int16_t intYaw = ((int16_t*)frame.buf)[3];
  requestedX = intX / double(1000);
  requestedY = intY / double(1000);
  requestedZ = intZ / double(1000);
  requestedYaw = intYaw / double(1000);
}

void CANClass::printFrame(CAN_message_t &frame, int mailbox)
{
  if(displayCAN){
   Serial.print(mailbox);
   Serial.print(" ID: ");
   Serial.print(frame.id, HEX);
   Serial.print(" Data: ");
   for (int c = 0; c < frame.len; c++) 
   {
      Serial.print(frame.buf[c], HEX);
      Serial.write(' ');
   }
   Serial.println();
  }
   RXCount++;
}

void CANClass::gotFrame(CAN_message_t &frame, int mailbox)
{
    printFrame(frame, mailbox);
    frameToCommand(frame, mailbox);
}

CANClass CANClass0;

// -------------------------------------------------------------
//PWM Reading
void setup(void)
{
  delay(1000);
  Serial.println(F("Hello Neo."));
  pinMode(redLEDpin,OUTPUT);
  //PWM setup
  pinMode(M1PIN_IN, INPUT);
  pinMode(M2PIN_IN, INPUT);
  pinMode(M3PIN_IN, INPUT);
  pinMode(M4PIN_IN, INPUT);
  attachInterrupt(M1PIN_IN, INT_M1, CHANGE);
  attachInterrupt(M2PIN_IN, INT_M2, CHANGE);
  attachInterrupt(M3PIN_IN, INT_M3, CHANGE);
  attachInterrupt(M4PIN_IN, INT_M4, CHANGE);

  //SBUS Setup
  uOut = SWITCH_POS_1;
  ppmOut.begin(5);
  ppmOut.write(1, aOut);
  ppmOut.write(2, eOut);
  ppmOut.write(3, tOut);
  ppmOut.write(4, rOut);
  ppmOut.write(7, uOut);
  
  while (!Serial.available());
  Serial.read();
  ppmOut.write(1, 1000);
  ppmOut.write(2, 1000);
  ppmOut.write(3, 1000);
  ppmOut.write(4, 2000);
  delay(5000);
  
  
  //CAN Setup
  Can0.begin(1000000);  
  Can0.attachObj(&CANClass0);
  
  
  CAN_filter_t allPassFilter;
  allPassFilter.id=0;
  allPassFilter.ext=1;
  allPassFilter.rtr=0;

  //leave the first 4 mailboxes to use the default filter. Just change the higher ones
  for (uint8_t filterNum = 4; filterNum < 16;filterNum++){
    Can0.setFilter(allPassFilter,filterNum);
  }
  for (uint8_t filterNum = 0; filterNum < 16;filterNum++){
     CANClass0.attachMBHandler(filterNum);
  }
  //CANClass0.attachGeneralHandler();
  
}

// -------------------------------------------------------------
void loop(void)
{
  static uint16_t pulseM1;
  static uint16_t pulseM2;
  static uint16_t pulseM3;
  static uint16_t pulseM4;
  static uint8_t bUpdateFlags;
  //Service PWM input interrupts
  if (bUpdateFlagsShared)
  {
    byte sregRestore = SREG;
    cli();
    bUpdateFlags = bUpdateFlagsShared;

    if(bUpdateFlags & M1_FLAG)
    {
      pulseM1 = outM1;
    }
    if(bUpdateFlags & M2_FLAG)
    {
      pulseM2 = outM2;
    }
    if(bUpdateFlags & M3_FLAG)
    {
      pulseM3 = outM3;
    }
    if(bUpdateFlags & M4_FLAG)
    {
      pulseM4 = outM4;
    }

    bUpdateFlagsShared = 0;

    SREG = sregRestore;
  }
  //Push the pulses to CAN
  /*if (bUpdateFlags & M1_FLAG && pulseM1 != 0)
  {
    Serial.println("M1 Output: " + pulseM1);
    pulseM1 = 0;
  }
  if (bUpdateFlags & M2_FLAG && pulseM2 != 0)
  {
    Serial.println("M2 Output: " + pulseM2);
    pulseM2 = 0;
  }
  if (bUpdateFlags & M3_FLAG && pulseM3 != 0)
  {
    Serial.println("M3 Output: " + pulseM3);
    pulseM3 = 0;
  }
  if (bUpdateFlags & M4_FLAG && pulseM4 != 0)
  {
    Serial.println("M4 Output: " + pulseM4);
    pulseM4 = 0;
  }*/

  /*if (timeSinceLastSend > 10) {
    CAN_message_t message;
    message.id = 0x1E040271;
    message.ext = 1;
    //message.flags.remote = 0;
    //message.flags.overrun = 0;
    message.len = 8;
    message.buf[0] = ((uint8_t*)(&((int16_t)pulseM1)))[1];
    message.buf[1] = ((uint8_t*)(*(int16_t)pulseM1))[0];
    message.buf[2] = ((uint8_t*)(*(int16_t)pulseM2))[1];
    message.buf[3] = ((uint8_t*)(*(int16_t)pulseM2))[0];
    message.buf[4] = ((uint8_t*)(*(int16_t)pulseM3))[1];
    message.buf[5] = ((uint8_t*)(*(int16_t)pulseM3))[0];
    message.buf[6] = ((uint8_t*)(*(int16_t)pulseM4))[1];
    message.buf[7] = ((uint8_t*)(*(int16_t)pulseM4))[0];
    Can0.write(message);
    timeSinceLastSend = 0;
  }*/
  
  //TODO: Get some data from the NAZA in.
  //TODO: Do some CAN processing.
  uOut = SWITCH_POS_1;
  //Serial.println(requestedX);
  int a = (int)map(requestedX, -1, 1, 1000, 2000);
  int e = (int)map(requestedY, -1, 1, 1000, 2000);
  int t = (int)map(requestedZ, -1, 1, 1000, 2000);
  int r = (int)map(requestedYaw, -1, 1, 1000, 2000);
  Serial.println(a);
  ppmOut.write(1, a);
  ppmOut.write(2, e);
  ppmOut.write(3, t);
  ppmOut.write(4, r);
  ppmOut.write(7, uOut);
/*
  if (PPMtimer > 10) {
    //maxPPM = !maxPPM;
    if (grow) {
      aOut += bumpsize;
      eOut += bumpsize;
      rOut += bumpsize;
    } else {
      aOut -= bumpsize;
      eOut -= bumpsize;
      rOut -= bumpsize;
    }
    if (growT) {
      tOut += bumpsize;
    } else {
      tOut -= bumpsize;
    }
     PPMtimer = 0;
    uOut += 1;
    if (uOut > 2000) {
      uOut = 1000;
    }
    if (aOut == 2000 || aOut == 1000) {
      grow = !grow;
    }
    if (tOut == 2000 || tOut == 1000) {
      growT = !growT;
    }
  }
  */
  
  
  if (RXtimer > 10000){
    Serial.println("Total Received Messages in 10 Sec:");
    Serial.println(RXCount);
    RXtimer = 0;
    RXCount=0;
    displayCAN = !displayCAN;
  }
  if (LEDtimer >2500){
    LEDtimer = 0;
    redLEDstate = !redLEDstate;
    digitalWrite(redLEDpin, redLEDstate);
  }
}
