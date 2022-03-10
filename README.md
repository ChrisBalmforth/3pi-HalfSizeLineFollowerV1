3pi+ Line Follower V1 - constant motor speed

Based on the Pololu 3pi+ example LineFollowerPID.

Works with full size or half size robots by adjusting PD values and motor speeds. Both 
versions use 3 centre sensors for line sensing, half size version uses 2 outer sensors for 
marker detection and full size version uses external digital sensors connected in their place.

Reads all 5 sensors at once then uses 3 middle sensors to calculate position on line and 2 
outer sensors to detect start/finish and curve markers.

Uses a slow mapping lap to count side markers including crossings then fast laps don't need to 
look for crossings. Uses the 3 centre sensors to follow the line, and the 2 outer sensors to 
detect side markers. Uses the code from the library function readLineWhite() to calculate the 
position on the line.

First press left button to calibrate sensors. Robot rotates back and forth over the white line.
Then press centre button to perform runs. First lap is the slow mapping lap to count the number
of side markers and line crossings. Subsequent laps are speed runs, with the speed increasing 
each lap. The robot stops and pauses each lap after crossing the finish line. The run can be 
repeated after the last lap by pressing the right button.

Number of laps to run and the motor speed on each lap can be set by adjusting the variables at 
the start of the program, and the PD constants can be tuned there too.