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

// Global Vars
int stepper_home = 0;
int stepperSpeed = 15000;
int stepperAccel = 7500;
long maxSteps = 46000;
int extraStepIncement = 20000;

// Create AccelStepper
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
    //start serial connection
    Serial.begin(9600);
    // Set up the limit switches:
    pinMode(home_switch, INPUT_PULLUP);
    pinMode(closed_switch, INPUT_PULLUP);
    pinMode(obstruct_switch, INPUT_PULLUP);
    // Set up the obstruction switch as interupt:
    attachInterrupt(obstruct_switch, obstruction, FALLING);
    // Stepper motor initialisation/homing
    stepper.setPinsInverted (/*direction*/ false, /*step*/ false, /*enable*/ true);
    stepper.setEnablePin(enPin);
    stepper.setMaxSpeed(stepperSpeed);
    stepper.setAcceleration(stepperAccel);
    // home_stepper();
    closeDoor();

}

void obstruction() {
    Serial.println("Obstruction! Opening");
    openDoor();
    // TODO: Reverse the direction - Aim for getting to zero
}

void openDoor() {
    Serial.println("Opening door!");
    stepper.enableOutputs();
    // stepper.moveTo(-maxSteps); // This is wrong as doesn't decel.
    stepper.moveTo(stepper_home);
    long extraSteps = 0;
    long pos = stepper.currentPosition();
    while (digitalRead(home_switch)) {
        stepper.run();
        pos = stepper.currentPosition();
        if (pos < stepper_home && pos <= extraSteps ) {
            Serial.println("openDoor - Homing!");
            extraSteps = pos - extraStepIncement;
            stepper.moveTo(extraSteps);
        }
    }
    stepper_home = 0;
    stepper.setCurrentPosition(stepper_home);
    Serial.println("Opened!");
    stepper.disableOutputs();
}

void closeDoor() {
    Serial.println("Closing door!");
    stepper.enableOutputs();
    stepper.moveTo(maxSteps);
    long extraSteps = 0;
    long pos = stepper.currentPosition();
    while (digitalRead(closed_switch)) {
        stepper.run();
        pos = stepper.currentPosition();
        if (pos > maxSteps && pos >= extraSteps ) {
            Serial.println("closeDoor - Homing!");
            extraSteps = pos + extraStepIncement;
            stepper.moveTo(extraSteps);
        }
    }
    maxSteps = stepper.currentPosition(); // The current pos is what we know we need to do next time. At least at to start with.
    stepper.setCurrentPosition(maxSteps); // Update if we need to go further
    Serial.println("Closed!");
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
    // // Set the target position:
    // // stepper.moveTo(45000);
    // Serial.println("Move to Max");
    // stepper.moveTo(maxSteps);
    // // Run to target position with set speed and acceleration/deceleration:
    // stepper.enableOutputs();
    // stepper.runToPosition();

    // stepper.enableOutputs();
    // delay(1000);

    // // Move back to zero:


    // delay(1000);
}
