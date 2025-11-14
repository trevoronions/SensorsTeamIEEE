#include <Arduino.h>
#include <Servo.h>

//Testing to see if this updates on git 
/*
This sketch uses the PING ))) example format provided by Arduino, with some modifications, to 
use an HC-SR04 sensor to detect distance and control the wheel motors based on the response. 
Draft 4 uses an H-bridge MOSFET (L298N) that can reverse polarity to reverse the direction of the motors.
If the cart/chassis does not detect an object in front of it, it will go forward. If it detects an object within a certain range,
it will go backwards. 
New/replaced variables include in1, in2, and enA for motor 1. in1 and in2 are used for motor direction logic,
enA handles the PWM for motor speed. 
This version successfully deals with one motor. The second motor will be implemented later.
This sketch can be modified to deal with tank turning, if code for the second motor and other sensors is implemented.
*/
//Alterations done by Maximo Ruiz and Trevor Onions (sensors team)

//DC Motor Pin locations 
  // const byte in1 = 13; //in1 pin on H-Bridge is connected to pin 13. in1 is one part of the logic for motor direction.
  // const byte in2 = 12; //in2 pin on H-Bridge is connected to pin 12. in2 is the second part of motor direction logic. 
  // const byte enA = 11; //enA pin on H-Bridge is connected to pin 11. Handles the PWM for the motor.

// 
const byte trigSensor1 = 7; //trigger pin is pin 7.
const byte echoSensor1 = 8; // echo pin is pin 8
const byte trigSensor2 = 12;
const byte echoSensor2 = 13;

const byte redLedPin = 6; //red LED pin is connected to pin 6.
const byte servoDir = 3; //PWM on MG90S servo is connected to pin 3. Handles the direction of the servo motor.
int x = 45;
int y = 45; 
bool isForward = true;
Servo myServo;
int pos = 0;
//NOTE: The motor power will be supplied externally using an H-Bridge transistor for pin logic. The Arduino board I/O pins can only output 40mA of current.
//The H-Bridge will handle power to the motors. This code/sketch handles the direction and PWM logic. 

void setup() {
  Serial.begin(9600); //default Baud rate of 9600
  
  myServo.attach(3);
  pinMode(trigSensor1, OUTPUT); //trigger signal to send to sensor.
  pinMode(echoSensor1, INPUT); //signal received from sensor.
  pinMode(redLedPin, OUTPUT); //power sent to LED.
  
  pinMode(servoDir, OUTPUT); //servoDir is output.
}

void servoSweep() {
  if (pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  else if (pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}

void loop() {
  delay(30);
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:
  long duration, inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigSensor1, LOW); //trigSensor1 set to LOW.
  delayMicroseconds(2); 
  digitalWrite(trigSensor1, HIGH); //trigSensor1 sends signal for 10 microseconds.
  delayMicroseconds(10); //10 microseconds.
  digitalWrite(trigSensor1, LOW); //trigSensor1 turns off.

  // Ultrasonic sensor sends signal back after receiving trigger input.
  // duration is time taken by the ultrasonic burst to leave and return to the sensor.
  duration = pulseIn(echoSensor1, HIGH);

  // convert the duration into a distance

  cm = microsecondsToCentimeters(duration); //value2

  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  delay(50); //NOTE: Not sure what this delay here is for, it came with the example template.
  //If 100ms delay serves no purpose, reduce the delay time to make robot/motors respond faster.
  //*****END OF ULTRASONIC SENSOR CODE.*****

 
  
  analogWrite(servoDir, x); // Sends the x's value to the servo to change its position
  //delay(10);
  Serial.println(x); //prints the value of x to show where the servos located
  // analogWrite(servoDir, 50); //To test what values the servos like to take.

  //NOTE: servo works properly, but seems to swing back to one position randomly at first.
  

  //*****END OF SERVO MOTOR POSITION CODE*****

  //*****BEGINNING OF LED AND MOTOR OUTPUT CODE.*****

  int target_distance_cm = 10; //maximum distance from sensor needed to turn on LED and turn off motor.

  if (cm < target_distance_cm) //If the distance detected by sensor is less than target_distance centimeters from an object.
  {
    digitalWrite(redLedPin, HIGH); //Red LED turns on.
      // digitalWrite(in1, LOW); //sets in1 low to move motor1 backwards.
    //servoRotate(45, 255, 7);
    servoSweep();
    // digitalW/sets PWM of motor1 signal to 255 (100%) for full speed.
    delay(5); //without the rite(in2, HIGH); //sets in2 high to keep motor1 backwards.
      //analogWrite(enA, 255); //delay, the motor gets stuck switching between both directions very quickly. Likely due to the ultrasonic sensor not sending data in time.
  }
  else //If distance detected is greater than target_distance centimeters...
  {
    digitalWrite(redLedPin, LOW); //red LED stays off.  
    delay(5); //without the delay, the motor gets stuck switching between both directions very quickly. Likely due to the ultrasonic sensor not sending data in time.

    //NOTE: If the code works as intended, the cart/chassis will eventually reach a wall, go backwards, then get stuck between forwards and backwards.
    //I'll need to figure out how to make the cart avoid the object entirely so it doesn't get stuck logically.
  }
  //*****END OF LED AND MOTOR OUTPUT CODE*****

}


