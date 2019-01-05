#include <Servo.h>
#include <string.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN 9

Servo motor;

int val;
bool startMotor;
String incomingByte;

String startMotorString;

void setup() {
   
  Serial.begin(9600);
  Serial.println("Program begin...");
  Serial.println("This program will calibrate the ESC.");

  Serial.println("waiting for sending maximum output");
  while (!Serial.available());
  Serial.read();

  motor.attach(MOTOR_PIN);

  Serial.println("Now writing maximum output.");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  motor.writeMicroseconds(MAX_SIGNAL);

  // Wait for input
  delay(1000);

  // Send min output
  Serial.println("Sending minimum output");
  motor.writeMicroseconds(MIN_SIGNAL);
  startMotor = true;
  Serial.println("Press any key to start the motor.");
  while (!Serial.available());
  Serial.read();
}

const char * const boolToString(bool b)
{
  return b ? "true" : "false";
}

void loop() {
  if (Serial.available() > 0) {
    incomingByte = Serial.readString();
    Serial.print("Received: ");
    Serial.println(incomingByte);
    startMotor = !startMotor;
    if(startMotor){
      Serial.println("Start Motor = True");
    }else{
      Serial.println("Start Motor = False");
    }
  }
  
  if(startMotor){
    val = 1050;
  }else{
    val = 0;
  }
  
  motor.writeMicroseconds(val);
  delay(500);
}
