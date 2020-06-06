#include <Arduino.h>
#include <AccelStepper.h>

// 28BYJ-48 motor runs in full step mode, each step corresponds to a rotation of 11.25°.
// That means there are 32 steps per revolution (360°/11.25° = 32). What this means is that
// there are actually 32*63.68395 steps per revolution = 2037.8864 ~ 2038 steps!

// Define a stepper and the pins it will use
// AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5


// Motor one bipolar, converted 28BYJ-48 with DRV8834 driver
int motor1DirPin = 4;
int motor1StepPin = 5;

// Motor two unipolar ULN2003
int motor2pin1 = 17;
int motor2pin2 = 18;
int motor2pin3 = 19;
int motor2pin4 = 20;

// Motor three, DC pot with 6612 DRIVER
int motor3pin1 = 15;
int motor3pin2 = 16;

//set up the accelStepper intance
//the "1" tells it we are using a driver
AccelStepper stepper1(AccelStepper::DRIVER, motor1StepPin, motor1DirPin);
AccelStepper stepper2(AccelStepper::FULL4WIRE, motor2pin1,motor2pin3,motor2pin2,motor2pin4);
AccelStepper stepper3(AccelStepper::DRIVER, motor3pin1,motor3pin2);

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
