/*
 * Project DogDoor
 * Description:
 * Author: Copyright (C) 2019 Martin Rowan
 * Date:
 */

SYSTEM_THREAD(ENABLED);  // Have Particle processing in a separate thread - https://docs.particle.io/reference/device-os/firmware/photon/#system-thread
// Include the AccelStepper library:
#include <AccelStepper.h>

// Used pins
// Stepper
#define dirPin D2
#define stepPin D3
#define enPin D1
#define motorInterfaceType 1
// Limit switches
#define home_switch D4
#define closed_switch D5
#define obstruct_switch D6
// Presence Sensors
// TODO: Test and add

// Particle Cloud variables
int home_switch_status = 0;
int closed_switch_status = 0;
int obstruct_switch_status = 0;

// Global Vars
int stepper_home = 0;
int stepperSpeed = 15000;
int stepperAccel = 7500;
long closedPosistion = 0;
long initial_closedPosistion = 46000;
int moveFurtherIncrement = 20000;

// Create AccelStepper
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
// Create timer to send current variable values to cloud for switch state
Timer timer(1000, publishVariables); // Every second

void setup() {
    // Set up Particle cloud variables
    Particle.variable("home_switch", home_switch_status);
    Particle.variable("closed_switch", closed_switch_status);
    Particle.variable("obstruct_switch", obstruct_switch_status);

    // Start serial connection
    Serial.begin(9600);
    // timer
    timer.start();

    // Set up the limit switches:
    pinMode(home_switch, INPUT_PULLUP);
    pinMode(closed_switch, INPUT_PULLUP);
    pinMode(obstruct_switch, INPUT_PULLUP);
    // Set up the obstruction switch as interupt
    attachInterrupt(obstruct_switch, obstructionISR, FALLING);

    // Set up stepper motor and initialise with ensuring door is closed
    stepper.setPinsInverted (/*direction*/ true, /*step*/ false, /*enable*/ true);
    stepper.setEnablePin(enPin);
    stepper.setMaxSpeed(stepperSpeed);
    stepper.setAcceleration(stepperAccel);
    closeDoor();

}

void publishVariables()
{
    // Update Particle Cloud variables
    home_switch_status = digitalRead(home_switch);
    closed_switch_status = digitalRead(closed_switch);
    obstruct_switch_status = digitalRead(obstruct_switch);

    // For Debugging outside of main loops
    // static int count = 0;
    // Serial.print("[");
    // Serial.print(count++);
    // Serial.print("] - Pos: ");
    // Serial.print(stepper.currentPosition());
    // Serial.print(" - ToGo: ");
    // Serial.print(stepper.distanceToGo());
    // Serial.print(" - closedPosistion: ");
    // Serial.print(closedPosistion);
    // Serial.print(" - home_switch_status: ");
    // Serial.print(home_switch_status);
    // Serial.print(" - closed_switch_status: ");
    // Serial.print(closed_switch_status);
    // Serial.println();
}

void obstructionISR() {
    Serial.println("Obstruction! Opening");
    Particle.publish("obstruction-detected", PRIVATE);
    openDoor();
    // TODO: Reverse the direction - Aim for getting to zero
}

void openDoor() {
    Serial.println("Opening door!");
    Particle.publish("Opening", PRIVATE);
    stepper.enableOutputs();
    stepper.moveTo(stepper_home);
    long moveFurther = 0;
    long pos;
    while (digitalRead(home_switch)) {
        stepper.run();
        pos = stepper.currentPosition();
        if (pos <= stepper_home && pos <= moveFurther ) {
            Serial.println("openDoor - Homing!");
            moveFurther = pos - moveFurtherIncrement;
            stepper.moveTo(moveFurther);
        }
    }
    stepper_home = 0;
    stepper.setCurrentPosition(stepper_home); // Ensure stepper knows it where it should be
    Serial.println("Opened!");
    Particle.publish("Opened", PRIVATE);
}

void closeDoor() {
    Serial.println("Closing door!");
    Particle.publish("Closing", PRIVATE);
    stepper.enableOutputs();

    stepper.moveTo(closedPosistion);
    long moveFurther = 0;
    long pos = stepper.currentPosition();
    while (digitalRead(closed_switch)) {
        stepper.run();
        pos = stepper.currentPosition();
        if ( pos == 0 || (pos >= closedPosistion && pos >= moveFurther )) {
            Serial.println("closeDoor - Homing!");
            moveFurther = pos + moveFurtherIncrement;
            stepper.moveTo(moveFurther);
        }
    }
    // TODO: Fix if starting from closed
    closedPosistion = pos; // The current pos is what we know we need to do next time
    if (closedPosistion < 100) {  // We must be closed already, so at full range
        closedPosistion = initial_closedPosistion;
    }
    stepper.setCurrentPosition(closedPosistion); // Ensure stepper knows it where it should be
    Serial.println("Closed!");
    Particle.publish("Closed", PRIVATE);
    stepper.disableOutputs();
}

void loop() {
    //
    // Do homing in setup?
    // Wait for pet sensor one or two to be triggered trigger
    // Open door
    // timer reset on each detection from either sesnsor
    // Close door on timer expiry
    if (digitalRead(closed_switch)) {
        closeDoor();
        delay(5000);
    }
    if (digitalRead(home_switch)) {
        openDoor();
        delay(5000);
    }
}
