#include <Arduino.h>
#include "BasicStepperDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step

// 28BYJ-48 motor runs in full step mode, each step corresponds to a rotation of 11.25°.
// That means there are 32 steps per revolution (360°/11.25° = 32). What this means is that
// there are actually 32*63.68395 steps per revolution = 2037.8864 ~ 2038 steps!


// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 32
// Target RPM for cruise speed
#define RPM 120*2
// Acceleration and deceleration values are always in FULL steps / s^2
#define MOTOR_ACCEL 500
#define MOTOR_DECEL 250

// Microstepping mode. If you hardwired it to save pins, set to the same value here.
#define MICROSTEPS 1

#define DIR 4
#define STEP 5
#define SLEEP 8 // optional (just delete SLEEP from everywhere if not used)

/*
 * Choose one of the sections below that match your board
 */

// #include "DRV8834.h"
// #define M0 6
// #define M1 7
// DRV8834 stepper(MOTOR_STEPS, DIR, STEP, SLEEP, M0, M1);


#include <AccelStepper.h>

// Define a stepper and the pins it will use
// AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
int motorDirPin = 4; //digital pin 2     < ===THIS IS A DIRECTION PIN
int motorStepPin = 5; //digital pin 3

//set up the accelStepper intance
//the "1" tells it we are using a driver
AccelStepper stepper1(1, motorStepPin, motorDirPin);
AccelStepper stepper2(AccelStepper::FULL4WIRE, 17,19,18,20);

void setup()
{
  // Change these to suit your stepper if you want
  stepper1.setMaxSpeed(400);
  stepper1.setAcceleration(50);
  stepper1.moveTo(2038);

  stepper2.setMaxSpeed(400);
  stepper2.setAcceleration(50);
  stepper2.moveTo(2038);
}

void loop()
{
    // If at the end of travel go to the other end
    if (stepper1.distanceToGo() == 0)
      stepper1.moveTo(-stepper1.currentPosition());
    if (stepper2.distanceToGo() == 0)
      stepper2.moveTo(-stepper2.currentPosition());
      
    stepper1.run();
    stepper2.run();
}
