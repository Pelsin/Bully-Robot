#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>

ZumoMotors motors;
ZumoReflectanceSensorArray reflectanceSensors;
 
#define LED 13
 
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  900
  
// these might need to be tuned for different motor types
#define REVERSE_SPEED     400 
#define TURN_SPEED        400
#define FORWARD_SPEED     400
#define REVERSE_DURATION  40 
#define TURN_DURATION     150

Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12

#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];

boolean armed = false;

ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

void waitForButtonAndCountDown()
{
  button.waitForButton();
  delay(1000);
}

void constuct(){
  // Reset weapons here
  armed = false;
  
  if (button.isPressed())
  {
    
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    
    button.waitForRelease();
    waitForButtonAndCountDown();
  }
}

void arm_weapons(){
  if(!armed){
    // Weapons
    Serial.print("Arming weapons");
    armed = true;
  }
 
}

void setup()
{
  Serial.begin(9600);
  Serial.print("Setup");
   waitForButtonAndCountDown();
}

void loop()
{
  // Will continue after button press
  constuct();
  
  // Arm weapons
  arm_weapons();
  
  // Move
  onwards();
}

void onwards(){
  sensors.read(sensor_values);
  
  if (sensor_values[0] < QTR_THRESHOLD)
  {
    // if leftmost sensor detects line, reverse and turn to the right
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else if (sensor_values[5] < QTR_THRESHOLD)
  {
    // if rightmost sensor detects line, reverse and turn to the left
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else
  {
    // otherwise, go straight
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
}
