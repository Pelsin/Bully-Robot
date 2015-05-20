#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>

ZumoMotors motors;
ZumoReflectanceSensorArray reflectanceSensors;

#define LED 13

// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  850

// these might need to be tuned for different motor types
#define REVERSE_SPEED     400
#define TURN_SPEED        400
#define FORWARD_SPEED     400
#define REVERSE_DURATION  150
//#define TURN_DURATION     150

// Servo settings
#define SERVO_PIN 11
#define UP_POSITION 1000
#define DOWN_POSITION 2000

// This is the time since the last rising edge in units of 0.5us.
uint16_t volatile servoTime = 0;
// This is the pulse width we want in units of 0.5us.
uint16_t volatile servoHighTime = 3000;
// This is true if the servo pin is currently high.
boolean volatile servoHigh = false;

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

void constuct() {
  
  // Reset weapons here
  armed = false;
  servoSetPosition(UP_POSITION); // Send pulses.

  if (button.isPressed())
  {
    
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);

    button.waitForRelease();
    waitForButtonAndCountDown();
  }
  
}

void arm_weapons() {
  if (!armed) {
    // Weapons
    Serial.print("Arming weapons");
    servoSetPosition(DOWN_POSITION); // Send pulses.
    armed = true;
  }

}

void setup()
{
  Serial.begin(9600);
  Serial.print("Setup");
  servoInit();
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

void onwards() {
  sensors.read(sensor_values);

  if (sensor_values[0] < QTR_THRESHOLD)
  {
    // if leftmost sensor detects line, reverse and turn to the right
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(random(50,400));
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else if (sensor_values[5] < QTR_THRESHOLD)
  {
    // if rightmost sensor detects line, reverse and turn to the left
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(random(50,400));
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else
  {
    // otherwise, go straight
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
}

ISR(TIMER2_COMPA_vect)
{
  // The time that passed since the last interrupt is OCR2A + 1
  // because the timer value will equal OCR2A before going to 0.
  servoTime += OCR2A + 1;
  static uint16_t highTimeCopy = 3000;
  static uint8_t interruptCount = 0;
  if (servoHigh)
  {
    if (++interruptCount == 2)
    {
      OCR2A = 255;
    }
    // The servo pin is currently high.
    // Check to see if is time for a falling edge.
    // Note: We could == instead of >=.
    if (servoTime >= highTimeCopy)
    {
      // The pin has been high enough, so do a falling edge.
      digitalWrite(SERVO_PIN, LOW);
      servoHigh = false;
      interruptCount = 0;
    }
  }
  else
  {
    // The servo pin is currently low.
    if (servoTime >= 40000)
    {
      // We've hit the end of the period (20 ms),
      // so do a rising edge.
      highTimeCopy = servoHighTime;
      digitalWrite(SERVO_PIN, HIGH);
      servoHigh = true;
      servoTime = 0;
      interruptCount = 0;
      OCR2A = ((highTimeCopy % 256) + 256) / 2 - 1;
    }
  }
}

void servoInit()
{
  digitalWrite(SERVO_PIN, LOW);
  pinMode(SERVO_PIN, OUTPUT);
  // Turn on CTC mode. Timer 2 will count up to OCR2A, then
  // reset to 0 and cause an interrupt.
  TCCR2A = (1 << WGM21);
  // Set a 1:8 prescaler. This gives us 0.5us resolution.
  TCCR2B = (1 << CS21);
  // Put the timer in a good default state.
  TCNT2 = 0;
  OCR2A = 255;
  TIMSK2 |= (1 << OCIE2A); // Enable timer compare interrupt.
  sei(); // Enable interrupts.
}
void servoSetPosition(uint16_t highTimeMicroseconds)
{
  TIMSK2 &= ~(1 << OCIE2A); // disable timer compare interrupt
  servoHighTime = highTimeMicroseconds * 2;
  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt
}
