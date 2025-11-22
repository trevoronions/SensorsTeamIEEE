
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
-Successfully compiled code that deals with two sensors and two servos. Not perfect, but it works.
--For some reason, both servos go at double speed when both sensors detect an object. If only one sensor detects an object, its corresponding servo will respond normally.
--Still a rough draft, so the actual code needs some refining to make adding move sensors and servos less tedious, and fix the problem mentioned above.
-UPDATE: found out I never added a second variable for the second servo position. The servos went at double speed because their functions used the same position variable.

Overall we can now use two sensors and two servos, with each servo corresponding to a sensor. Just need to figure out why the servos go at double speed when both sensors detect something.