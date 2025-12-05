#include <Arduino.h>
#include <Servo.h>

/*
  This sketch uses the PING ))) example format provided by Arduino, with some modifications, to 
  use multiple HC-SR04 sensors to detect distance and control multiple servo motors based on the response. 
  */
//Alterations done by Maximo Ruiz and Trevor Onions (sensors team)

extern long microsecondsToCentimeters(long microseconds); //setting microsecondsToCentimeters as an external function to work in ultraSon function.
//NOTE TO SELF: setting a function as "extern" in C/C++ allows the function to be used within other functions.

const byte trigSensor1 = 7; //trigger pin is pin 7.
const byte echoSensor1 = 8; //echo pin is pin 8
const byte trigSensor2 = 12; //second sensor's trigger pin is pin 12. (not implemented yet)
const byte echoSensor2 = 13; //second sensor's echo pin is pin 13. (not implemented yet)
Servo servo1; //First servo (main servo for testing)
Servo servo2; //Second servo
bool isForward1 = true; //Used for servo logic in servoSweep function. When true, the servo's angle will increase. When false, the servo's angle will decrease.
bool isForward2 = true; //Same idea as isForward1.
int pos1 = 0; //Angle of servo1.
int pos2 = 0; //Angle of servo2.

const int targetDistanceCm1 = 15; //minimum distance for servo1 to move.
const int targetDistanceCm2 = 15; //miniumum distance for servo2 to move.

void setup() {
  Serial.begin(9600); //default Baud rate of 9600
  
  servo1.attach(3); //servo1 is on pin 3.
  servo2.attach(5); //servo2 is on pin 5.
  pinMode(trigSensor1, OUTPUT); //trigger signal to send to sensor1.
  pinMode(echoSensor1, INPUT); //signal received from sensor1.
  pinMode(trigSensor2, OUTPUT); //trigger signal to send to sensor2.
  pinMode(echoSensor2, INPUT); //signal received from sensor2.
}

/**servoSweep sweeps the servo from 0 to 180 degrees and back. 
 * @param servoNum is the number corresponding to the specific servo that will be moved.
*/
void servoSweep(int servoNum) { 
  switch(servoNum) //switch statement reads value of servoNum parameter.
  {
    case(1): //if servoNum is 1.
      if (pos1 <= 180 && isForward1){ //If servo position is <= 180 degrees and isForward is true...
        pos1 += 5; //Increase angle by 5 degrees every cycle/iteration.
        servo1.write(pos1); //Servo moves to position/angle.
        if (pos1 == 180) { //If position is 180.
          isForward1 = false; //Make isForward false.
        }
        delay(10); //10 ms delay to let servo move before next position input.
      }
      else{ //If pos is 180 and/or isForward is false...
        pos1 -= 5; //decrease angle by 5 degrees every cycle/iteration.
        servo1.write(pos1); //Servo moves to position/angle.
        if (pos1 == 0) { //If the position/angle of servo is 0...
          isForward1 = true; //Make isForward true.
        }
        delay(10); //10 ms delay to let servo move before next position input.
      }
    break;
    case(2): //if servoNum is 2.
      if (pos2 <= 180 && isForward2){ //If servo position is <= 180 degrees and isForward is true...
        pos2 += 5; //Increase angle by 5 degrees every cycle/iteration.
        servo2.write(pos2); //Servo moves to position/angle.
        if (pos2 == 180) { //If position is 180.
          isForward2 = false; //Make isForward false.
        }
        delay(10); //10 ms delay to let servo move before next position input.
      }
      else{ //If pos is 180 and/or isForward is false...
        pos2 -= 5; //decrease angle by 5 degrees every cycle/iteration.
        servo2.write(pos2); //Servo moves to position/angle.
        if (pos2 == 0) { //If the position/angle of servo is 0...
          isForward2 = true; //Make isForward true.
        }
        delay(10); //10 ms delay to let servo move before next position input.
      }
    break;
    default: Serial.println("servoSweep function must have value of 1 or 2.");
    break;
  }

  //NOTE TO SELF: check if 10 ms delay is necessary at the end.
}

/**ultraSon handles an ultrasonic sensor.
 * @param trigSensor The trigger pin for the sensor.
 * @param echoSensor The echo pin for the sensor.
*/
int ultraSon(int trigSensor, int echoSensor)
{
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
  long cm1, cm2; //Distance detected by sensor1 and sensor2, respectively.

  cm1 = ultraSon(trigSensor1, echoSensor1); //Distance detected by sensor 1 is put into cm1.
  cm2 = ultraSon(trigSensor2, echoSensor2); //Distance detected by sensor 2 is put into cm2.
  
  //For printing the cm values:
  //Serial.println(cm1); 
  //Serial.println(cm2);

  if (cm1 < targetDistanceCm1) //If the distance detected by sensor1 is less than targetDistanceCm1 centimeters from an object.
  {
    Serial.println("One Go");
    servoSweep(1); //choose servo1
    delay(5); 
  }
  else
  {
    Serial.println("One Stop");
    delay(5); //without the delay, the motor gets stuck switching between both directions very quickly. Likely due to the ultrasonic sensor not sending data in time.
  }

  if (cm2 < targetDistanceCm2) //If distance detected by sensor2 is less than targetDistanceCm2 cm from object...
  {
    Serial.println("Two Go");
    servoSweep(2); //choose servo2
    delay(5); 
  }
  else
  {
    Serial.println("Two Stop");
    delay(5);
  }

  //*****END OF LED AND MOTOR OUTPUT CODE*****
  delay(50); //NOTE TO SELF: Is this big of a delay necessary?
}


