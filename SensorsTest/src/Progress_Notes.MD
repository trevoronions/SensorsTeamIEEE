
## 11/20/25 (by Max)
-Added comments to clarify function of code.
-Big servo works perfectly fine with current code.
-Small servo only works when power has a perfect connection. Big servo works fine with a loose connection, but small servo won't work at all unless the power wires are reconnected to the breadboard.
--Possible power problem?
-Rafael mentioned that the components (servos and sensors) should have external power.
--Can be done with a 9V battery connected to positive and negative terminals of breadboard. Just have to find the right wires/connection to connect the battery to the breadboard.
-Started writing code for the sensor function.

Overall the code works just fine for one servo and one sensor. Next task would be to implement the sensor code as a function so we can move on to using multiple sensors and multiple servos (if that's still a goal). We still have to determine what each sensor will be meant for (front, sides, etc), but right now we shouldn't worry about it yet.

## 11/21/25 (by Max)
-Turned code that dealt with sensor into a function.
-Added second sensor and second servo.
-Modified servo code to influece one or both servos. They no longer share the same position variable that made them go at double speed when both sensors detect something.
-Successfully compiled code that deals with two sensors and two servos.
-Still a rough draft. The code still needs to give the sensor and servo functions more accessibility to sensor distance and servo speed.

Overall we can now use two sensors and two servos, with each servo corresponding to a sensor. Just needs some cleaning up and accessibility to sensor distance and servo speed needs to be added.