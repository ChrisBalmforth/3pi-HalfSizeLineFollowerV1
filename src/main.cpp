/* 3Pi+ LineFollowerHalfSize v1 - constant speed

Reads all 5 sensors at once then uses 3 middle sensors to 
calculate position on line and 2 outer sensors for CD and SF

Uses a slow mapping lap to count side markers including crossings
then fast laps don't need to look for crossings

Based on 3PiPlus LineFollowerFullSize v3.1
Speeds and PD values adjusted for 30:1 geared motor

*/

#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>

using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Motors motors;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;
LCD lcd;
LineSensors totalsensor;
Encoders encoders;

////////////////////////
//                    //
//     PD values      //
//                    //
////////////////////////
const float KP = 0.45;	// timeout = 2000
const float KD	= 4.0;  // timeout = 2000

////////////////////////
//                    //
//  Global constants  //
//  Change here       //
//                    //
////////////////////////
const uint16_t lowBattery = 4400;     // Lowest allowed battery voltage
const uint8_t linesensorCount = 3;    // Number of line sensors
const uint8_t sidesensorCount = 2;    // Number of side sensors
const uint8_t totalsensorCount = 5;   // Total number of sensors
const uint16_t centre = 1000;         // line sensor position when centred
const uint16_t calibrationSpeed = 80; // Speed during calibration
const uint8_t lapTotal = 4;           // Total number of laps to run before stopping
const uint16_t timeOut = 2000;        // Timeout for line sensors
const float encCal = 3.55;            // Number of encoder clicks per mm travelled
const float accelRate = 0.3;          // Rate of acceleration in motor speed units per ms

////////////////////////
//                    //
//  Global variables  //
//                    //
////////////////////////
uint16_t totalsensorValues[totalsensorCount]; // Array for all 5 sensor readings
uint16_t linesensorValues[linesensorCount];   // Array for 3 line sensor readings
int16_t position = centre;    // initial position is centered on the white line
int16_t error = 0;            // initial proportional error = 0
int16_t lastError = 0;        // initial value of last proportional error = 0
int16_t derivError = 0;       // initial derivative error = 0
int16_t MAX[4] = {180, 200, 220, 240};	// Max speed for each lap
int16_t maxSpeed = MAX[0];    // Top speed is 400
bool stop = false;            // Stops run
int8_t lapCount = 0;          // Current number of laps completed
int8_t sfCount = 0;           // Curent number of start/finish markers passed
int32_t encCount = 0;         // Lap length as measured by right encoder
bool sfFlag = false;          // Start/finish marker detected
bool cdFlag = false;          // Curve marker detected
uint16_t markerCount = 0;     // Current number of side markers counted
uint16_t markerTotal = 0;     // Total number of side markers (inc. cross)


// Check battery voltage, stop and warn if low
void checkBattery()
{
  if (readBatteryMillivolts() < lowBattery)
  {
    // Print message on LCD screen
    lcd.clear();
    lcd.gotoXY(0,0);
    lcd.print("BATT LOW");
    lcd.gotoXY(0,1);
    lcd.print(readBatteryMillivolts());
    lcd.gotoXY(4,1);
    lcd.print("mV");

    // Halt, sound siren and flash red and green leds
    motors.setSpeeds(0,0);
    while (1)
    {
      ledRed(1);
      ledGreen(0);
      buzzer.play("v10a");
      delay(500);
      ledRed(0);
      ledGreen(1);
      buzzer.play("v10c");
      delay(500);
    }
  }
}

void calibrateSensors()
{
  lcd.clear();
  lcd.gotoXY(0,0);
  lcd.print("Calibr");
  lcd.gotoXY(0,1);
  lcd.print(" ating");

  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(1000);
  for (uint16_t i = 0; i < 80; i++)
  {
    if (i > 20 && i <= 60)
    {
      motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
      delay(20);
    }
    else
    {
      motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
      delay(20);
    }
    totalsensor.calibrate();
  }
  motors.setSpeeds(0, 0);
  lcd.clear();
}

// Calculate position on line from array of line sensor values
int16_t calcPosition(uint16_t sensorValues[linesensorCount])
{
	bool on_line = false;           // true if mouse on line
  uint8_t i = 0;                  // loop counter
	uint32_t avg = 0;               // this is long before division
	uint16_t sum = 0;               // this is for the denominator which is <= 64000
	static int16_t last_value = 0;  // assume initially that the line is left.

	for (i = 0; i < linesensorCount; i++) 
  {
		uint16_t value = 1000 - sensorValues[i];

		// keep track of whether we see the line at all
		if (value > 200) {on_line = 1;}

		// only average in values that are above a noise threshold
		if (value > 50) 
    {
			avg += (long)(value) * (i * 1000);
			sum += value;
		}
	}

	if(!on_line)
	{
		// If it last read to the left of center, return 0.
		if(last_value < (linesensorCount-1)*1000/2) {return 0;}
		
		// If it last read to the right of center, return the max.
		else {return (linesensorCount-1)*1000;}
	}
  // Calculate position and return it
	last_value = avg/sum;
	return last_value;
} 

// Procedure to read sensors and set position, SF and CD flags
void readSensors()
{
  // read all calibrated sensor values
  totalsensor.readCalibrated(totalsensorValues);

  // put linesensor values into linesensorValues array
  for (uint8_t i = 0; i < linesensorCount; i++)
  {
    linesensorValues[i] = totalsensorValues[i+1];
  }
  // Set SF and CD flags
  if (totalsensorValues[0] < 500) {cdFlag = true;} else {cdFlag = false;}
  if (totalsensorValues[4] < 500) {sfFlag = true;} else {sfFlag = false;}

  // Calculate position on line
  position = calcPosition(linesensorValues);
}

// Procedure to display sensor readings on LCD
void displaySensors()
{
  // Print line position
  lcd.clear();
  lcd.gotoXY(2,0);
  lcd.print(position);

  // Print the side sensor status
  if (cdFlag)
  {
    lcd.gotoXY(0,1);
    lcd.print("CD");
  }
  if (sfFlag)
  {
    lcd.gotoXY(4,1);
    lcd.print("SF");
  }
}

// Procedure to set motors from current line position
void setMotors()
{
  // Calculate new motor settings
  // The "proportional" term should be 0 when we are on the line		
  error = centre - position;
  // right of centre is +ve error, left is -ve error

  // Compute the derivErrorative (change) of the position
  derivError = error - lastError;

  // Remember the last error value
  lastError = error;
  
  // Compute the difference between the two motor power settings m1 and m2
  // If this is a positive number the robot will turn to the left.
  // If it is a negative number, the robot will turn to the right,
  // and the magnitude of the number determines the sharpness of the turn.
  long powerDifference = KP*error + KD*derivError;

  // Compute new motor settings: neither motor is ever set to a negative value.
  if(powerDifference > maxSpeed) {powerDifference = maxSpeed;}
  if(powerDifference < -maxSpeed) {powerDifference = -maxSpeed;}
  if(powerDifference > 0) {motors.setSpeeds(maxSpeed-powerDifference, maxSpeed);}
  else {motors.setSpeeds(maxSpeed, maxSpeed+powerDifference);}
}

// Procedure to thoroughly check for side markers
void markerFullCheck()
{
	// Return if both side sensors see black
	if ((sfFlag == false) and (cdFlag == false)) {return;}
	
  // Initialise marker detect flags
  bool sfDetect = false;
  bool cdDetect = false;
	
  // Keep running to end of side marker
	while ((sfFlag == true) or (cdFlag == true))
	{
    readSensors();
    setMotors();    
    if (sfFlag == true) {sfDetect = true;}
    if (cdFlag == true) {cdDetect = true;}
	}
  // If crossing increment markerCount and return
  if ((sfDetect == true) and (cdDetect == true))
  {markerCount++; return;}

  // If CD marker sound buzzer and return
  if (cdDetect == true) {buzzer.play("!L16 c"); return;}

  // If SF marker increment markerCount and sfCount
  if (sfDetect == true)
  {
    markerCount++; sfCount++; buzzer.play("!L16 a");

    // if end of lap raise stop flag
    if (sfCount > 1) {sfCount = 0; stop = true;}

    return;
  }
}

// Procedure to quickly check for start/finish marker or crossing
void markerQuickcheck()
{
	// Return if start/finish sensor sees black
  if (sfFlag == false) {return; }
  
  // Carry on steering if sf marker or crossing detected
  while (sfFlag)
  {
    readSensors();
    setMotors();    
  }
  // Increment marker count at end of sf marker and return
  markerCount++;
}

// Procedure to drive forwards a specified number of mm
// following the line at the current speed
void drive_mm(int16_t distance)
{
  int16_t startReading = encoders.getCountsRight();
  while (encoders.getCountsRight() < (startReading + int(distance*encCal)))
  {
    // Read line sensors and set motors to follow line
    readSensors();
    setMotors();
  }
}

// Procedure at end of each lap
void endLap()
{
  // Update lap count and display
  lapCount++;
  lcd.clear();
  lcd.gotoXY(0,0);
  lcd.print("Lap ");
  lcd.print(lapCount);

  // Reset stop flag
  stop = false;

  // Run further 50mm then stop
  drive_mm(50);
  motors.setSpeeds(0,0);
}

void setup()
{
  // Set up line sensor
  totalsensor.setTimeout(timeOut);

  // Play a little welcome song
  buzzer.play(">g32>>c32");

  // Check battery level, stop if low
  checkBattery();

  //Clear display and show battery voltage on top row
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print(' ');
  lcd.print(readBatteryMillivolts());
  lcd.print(F(" mV"));
  lcd.gotoXY(0, 1);
  lcd.print("CAL RUN");

  // Button A calibrates sensors
  // Button B starts run
  while (!buttonB.isPressed())
  {
    if (buttonA.isPressed())
    {
      // Beep and calibrate
      buzzer.play("!L16 a");
      calibrateSensors();
      while (!buttonB.isPressed()) 
      {
        readSensors();
        displaySensors(); 
        delay(50);
      }
    }
  }
}

// Main loop runs forever
void loop() 
{
  // Initialise run variables
  lapCount = 0;
  sfCount = 0;
  stop = false;

  // Display lap number
  lcd.clear();
  lcd.gotoXY(0,0);
  lcd.print("Lap ");
  lcd.print(lapCount);
  
  // Play music and wait for it to finish before we start driving.
  buzzer.play("L16 cdegreg4");
  while (buzzer.isPlaying());

  // Run mapping lap
  markerCount = 0;
  maxSpeed = MAX[lapCount];
  while (!stop)
  {
    // Read line sensors and set motors
    readSensors();
    setMotors();

    // Full test for side markers
    markerFullCheck();
  }
  // End of lap
  endLap();

  // Update markerTotal
  markerTotal = markerCount;

  // Display markerTotal
  lcd.clear();
  lcd.gotoXY(0,0);
  lcd.print(markerTotal);

  // Pause before restart
  delay(2500);

  // Run remaining laps
  while (lapCount < lapTotal)
  {
    // Reset marker counter
    markerCount = 0;
    
    // Set current maxSpeed
    maxSpeed = MAX[lapCount];

    // Run lap until end of markers
    while (markerCount < markerTotal) 
    {
      // Read line sensors and set motors
      readSensors();
      setMotors();

      // Quick check for side markers
      markerQuickcheck();
    }

    // End of lap
    endLap();

    // Pause before restart
    delay(2500);
  }

  // Halt until button C is pressed
  lcd.clear();
  lcd.gotoXY(0,0);
  lcd.print("Rept run");
  lcd.gotoXY(0,1);
  lcd.print("       v");
  while (!buttonC.isPressed()){delay(50);}
}