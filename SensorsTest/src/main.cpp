#include <Arduino.h>
#include <Servo.h>

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

extern long microsecondsToCentimeters(long microseconds); //setting microsecondsToCentimeters as an external function to work in ultraSon function.
//NOTE TO SELF: setting a function as "extern" in C/C++ is the same as setting a function as "public" in Java.

const byte trigSensor1 = 7; //trigger pin is pin 7.
const byte echoSensor1 = 8; //echo pin is pin 8
const byte trigSensor2 = 12; //second sensor's trigger pin is pin 12. (not implemented yet)
const byte echoSensor2 = 13; //second sensor's echo pin is pin 13. (not implemented yet)
Servo servo1; //First servo (main servo for testing)
bool isForward = true; //Used for servo logic in servoSweep function. When true, the servo's angle will increase. When false, the servo's angle will decrease.
int pos = 0; //Angle of the servo.

const int targetDistancecm = 15;
//NOTE: The motor power will be supplied externally using an H-Bridge transistor for pin logic. The Arduino board I/O pins can only output 40mA of current.
//The H-Bridge will handle power to the motors. This code/sketch handles the direction and PWM logic. 

void setup() {
  Serial.begin(9600); //default Baud rate of 9600
  
  servo1.attach(3); //servo1 is on pin 3.
  pinMode(trigSensor1, OUTPUT); //trigger signal to send to sensor.
  pinMode(echoSensor1, INPUT); //signal received from sensor.
}

//function servoSweep sweeps the servo from 0 to 180 degrees and back. Parameter "servoNum" is the specific servo that will be moved.
void servoSweep(Servo servoNum) { 
  if (pos <= 180 && isForward){ //If servo position is <= 180 degrees and isForward is true...
    pos += 5; //Increase angle by 5 degrees every cycle/iteration.
    servoNum.write(pos); //Servo moves to position/angle.
    if (pos == 180) { //If position is 180.
      isForward = false; //Make isForward false.
    }
    delay(10); //10 ms delay to let servo move before next position input.
  }
  else{ //If pos is 180 and/or isForward is false...
    pos -= 5; //decrease angle by 5 degrees every cycle/iteration.
    servoNum.write(pos); //Servo moves to position/angle.
    if (pos == 0) { //If the position/angle of servo is 0...
      isForward = true; //Make isForward true.
    }
    delay(10); //10 ms delay to let servo move before next position input.
  }
  //NOTE TO SELF: check if 10 ms delay is necessary here.
}

/**ultraSon handles an ultrasonic sensor.
 * @param sensorNum The sensor chosen.
 * @param trigSensor The trigger pin for the sensor.
 * @param echoSensor The echo pin for the sensor.
*/
int ultraSon(/*int sensorNum, */int trigSensor, int echoSensor) //TEMPORARILY REMOVED sensorNum
{
  //NOTE TO SELF: in the const ints above, put the trigSensor and echoSensor values for each sensor into an array. 
  //Index the array here based on sensorNum later. That way, only sensorNum will be needed.

  int distance; //"distance" is the distance calculated by the sensor in cm.
  int durationFunc; //"durationFunc" is the same as "duration", but in this function.

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigSensor, LOW); //trigSensor1 set to LOW.
  delayMicroseconds(2); //2 microseconds.
  digitalWrite(trigSensor, HIGH); //trigSensor1 sends signal for 10 microseconds.
  delayMicroseconds(10); //10 microseconds.
  digitalWrite(trigSensor, LOW); //trigSensor1 turns off.
  // Ultrasonic sensor sends signal back after receiving trigger input.

  durationFunc = pulseIn(echoSensor, HIGH); // duration is time taken by the ultrasonic burst to leave and return to the sensor.
  //duration2 = pulseIn(echoSensor2, HIGH);
  // convert the duration into a distance

  distance = microsecondsToCentimeters(durationFunc); //convert duration of signal to distance in cm.

  return distance; //return "distance"
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
  long duration1, duration2, cm1, cm2;

  // // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  // digitalWrite(trigSensor1, LOW); //trigSensor1 set to LOW.
  // delayMicroseconds(2); 
  // digitalWrite(trigSensor1, HIGH); //trigSensor1 sends signal for 10 microseconds.
  // delayMicroseconds(10); //10 microseconds.
  // digitalWrite(trigSensor1, LOW); //trigSensor1 turns off.

  // // Ultrasonic sensor sends signal back after receiving trigger input.
  // // duration is time taken by the ultrasonic burst to leave and return to the sensor.
  // duration1 = pulseIn(echoSensor1, HIGH);
  // //duration2 = pulseIn(echoSensor2, HIGH);
  // // convert the duration into a distance

  // cm1 = microsecondsToCentimeters(duration1); //value2
  //cm2 = microsecondsToCentimeters(duration2);

  cm1 = ultraSon(trigSensor1, echoSensor1);
  
  Serial.println(cm1);
  //Serial.println(cm2 + "cm2");
  Serial.println();

  if (cm1 < targetDistancecm) //If the distance detected by sensor is less than target_distance centimeters from an object.
  {
    Serial.println("Go");
    servoSweep(servo1);
    delay(5); 

  }
  else
  {
    Serial.println("Stop");
    delay(5); //without the delay, the motor gets stuck switching between both directions very quickly. Likely due to the ultrasonic sensor not sending data in time.

    //NOTE: If the code works as intended, the cart/chassis will eventually reach a wall, go backwards, then get stuck between forwards and backwards.
    //I'll need to figure out how to make the cart avoid the object entirely so it doesn't get stuck logically.
  }
  //*****END OF LED AND MOTOR OUTPUT CODE*****
  delay(50);
}


