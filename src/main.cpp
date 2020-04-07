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

#include "DRV8834.h"
#define M0 6
#define M1 7
DRV8834 stepper(MOTOR_STEPS, DIR, STEP, SLEEP, M0, M1);

void setup() {
    /*
     * Set target motor RPM.
     */
    stepper.begin(RPM);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    // stepper.setEnableActiveState(LOW);
    stepper.enable();
    stepper.setSpeedProfile(stepper.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
    // set current level (for DRV8880 only).
    // Valid percent values are 25, 50, 75 or 100.
    // stepper.setCurrent(100);
}

void loop() {
    delay(1000);

    /*
     * Moving motor in full step mode is simple:
     */
    stepper.setMicrostep(1);  // Set microstep mode to 1:1

    // One complete revolution is 360°
    stepper.rotate(360*64/8);     // forward revolution
    stepper.rotate(-360*64/8);    // reverse revolution

    // One complete revolution is also MOTOR_STEPS steps in full step mode
    stepper.move(MOTOR_STEPS*64/8);    // forward revolution
    stepper.move(-MOTOR_STEPS*64/8);   // reverse revolution

    /*
     * Microstepping mode: 1, 2, 4, 8, 16 or 32 (where supported by driver)
     * Mode 1 is full speed.
     * Mode 32 is 32 microsteps per step.
     * The motor should rotate just as fast (at the set RPM),
     * but movement precision is increased, which may become visually apparent at lower RPMs.
     */
    stepper.setMicrostep(8);   // Set microstep mode to 1:8

    // In 1:8 microstepping mode, one revolution takes 8 times as many microsteps
    stepper.move(8 * MOTOR_STEPS*64/8);    // forward revolution
    stepper.move(-8 * MOTOR_STEPS*64/8);   // reverse revolution

    // One complete revolution is still 360° regardless of microstepping mode
    // rotate() is easier to use than move() when no need to land on precise microstep position
    stepper.rotate(360*64/8);
    stepper.rotate(-360*64/8);

    delay(5000);
}
