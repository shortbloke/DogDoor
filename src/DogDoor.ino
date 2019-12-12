/*
 * Project DogDoor
 * Description: Stepper motor controlled Dog Door.
 * Author: Copyright (C) 2019 Martin Rowan
 * Date: 12 December 2019
 */

SYSTEM_THREAD(ENABLED);  // Have Particle processing in a separate thread - https://docs.particle.io/reference/device-os/firmware/photon/#system-thread
// Include the AccelStepper library:
#include <AccelStepper.h>

// Stepper
#define enPin D0
#define dirPin D1
#define stepPin D2
#define motorInterfaceType 1
// Limit switches
#define homeSwitch D3
#define closedSwitch D4
#define obstructSwitch D5
// Presence Sensors
#define indoorSensor D6
#define outdoorSensor D7

// Particle Cloud variables
int homeSwitchStatus = 0;
int closedSwitchStatus = 0;
int obstructSwitchStatus = 0;
int desiredDoorStateStatus = 0;
int currentDoorStateStatus = 0;

// Global Const
const bool debug = true;
// Stepper (MicroStepping 1/32)
const int stepperSpeed = 10000;
const int stepperAccel = 5000;
const long initialClosedPosition = 46000;
const int moveFurtherIncrement = 5000;
// Timers
const int keepOpenTime = 10000;  // 10 seconds
const int cloudPublishInterval = 30000;  // 30 seconds
const int serialLogInterval = 1000;  // 1 second

// Global Vars
int openPosition = 0;
long closedPosition = initialClosedPosition;

// Global enum for states
enum doorStates {
    DISABLED = 0,
    OPEN = 1,
    MOVING = 2,
    CLOSED = 3,
    OBSTRUCTED = 4,
    LOCKED = 5,
};
enum doorStates desiredDoorState;
enum doorStates currentDoorState;

// Create AccelStepper
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
// Serial logging timer
Timer serialTimer(serialLogInterval, serialLog);
// Create timer to send current variable values to cloud for switch state
Timer particleVarPublishTimer(cloudPublishInterval, publishVariables);
// Timer for how long the door is kept open after no presense detected.
Timer keepOpenTimer(keepOpenTime, timerCallback, true);

void setup() {
    // Set up Particle cloud variables
    Particle.variable("homeSwitch", homeSwitchStatus);
    Particle.variable("closedSwitch", closedSwitchStatus);
    Particle.variable("obstructSwitch", obstructSwitchStatus);
    Particle.variable("desiredDoorState", desiredDoorStateStatus);
    Particle.variable("currentDoorState", currentDoorStateStatus);

    if (debug) {
        // Start Serial Debugging
        Serial.begin(9600);
        serialTimer.start();
    }
    // Start background timers
    particleVarPublishTimer.start();

    // Set up the limit switches
    pinMode(homeSwitch, INPUT_PULLUP);
    pinMode(closedSwitch, INPUT_PULLUP);
    pinMode(obstructSwitch, INPUT_PULLUP);
    // Set up interupts to handle switch changes
    attachInterrupt(homeSwitch, switchISR, CHANGE);
    attachInterrupt(closedSwitch, switchISR, CHANGE);
    attachInterrupt(obstructSwitch, switchISR, FALLING);

    // Set up the sensors
    pinMode(indoorSensor, INPUT);
    attachInterrupt(indoorSensor, presenseSensorISR, RISING);
    pinMode(outdoorSensor, INPUT);
    attachInterrupt(outdoorSensor, presenseSensorISR, RISING);

    // Set up stepper motor and initialise with ensuring door is closed
    stepper.setPinsInverted (/*direction*/ true, /*step*/ false, /*enable*/ true);
    stepper.setEnablePin(enPin);
    stepper.setMaxSpeed(stepperSpeed);
    stepper.setAcceleration(stepperAccel);
    stepper.disableOutputs();

    switchISR();  // Get Initial currentDoorState by calling the switchISR
    desiredDoorState = CLOSED;  // Set initial State

    // Set stepper likely closed position
    // closedPosition = initialClosedPosition;
    if (currentDoorState == CLOSED){
        // We're already closed at satrtup. Set open to be the inverse of the normal closed pos.
        openPosition = -initialClosedPosition;
    }
}

boolean switchDepressed(int inputSwitch) {
    // Switch depressed sets pin low. digitalRead returns true when high and false when low
    // To make understanding of swich easier, this function inverts the state.
    boolean state = digitalRead(inputSwitch);
    return !state;
}

boolean presenseDetected() {
    // True if either sensor is active (high)
    if ( (digitalRead(indoorSensor)) or (digitalRead(outdoorSensor)) ) {
        return true;
    }
    return false;
}

void serialLog() {
    Serial.print("Current: ");
    Serial.print(currentDoorState);
    Serial.print(" - Desired: ");
    Serial.print(desiredDoorState);
    Serial.print(" - Current Pos: ");
    Serial.print(stepper.currentPosition());
    Serial.print(" - openPosition: ");
    Serial.print(openPosition);
    Serial.print(" - closedPosition: ");
    Serial.print(closedPosition);
    Serial.print(" - Target Pos: ");
    Serial.print(stepper.targetPosition());
    Serial.print(" - Current speed: ");
    Serial.print(stepper.speed());
    Serial.println();
}

void publishVariables() {
    // Update Particle Cloud variables
    homeSwitchStatus = switchDepressed(homeSwitch);
    closedSwitchStatus = switchDepressed(closedSwitch);
    obstructSwitchStatus = switchDepressed(obstructSwitch);

    desiredDoorStateStatus = desiredDoorState;
    currentDoorStateStatus = currentDoorState;
}

void switchISR() {
    if (switchDepressed(homeSwitch)) {
        currentDoorState = OPEN;
    } else if (switchDepressed(closedSwitch)) {
        currentDoorState = CLOSED;
    } else if (switchDepressed(obstructSwitch)) {
        currentDoorState = OBSTRUCTED;
    } else {
        currentDoorState = MOVING;
    }
}

void presenseSensorISR() {
    desiredDoorState = OPEN;
    keepOpenTimer.changePeriodFromISR(keepOpenTime);  // Set or reset the timer
}

void timerCallback() {
    if (presenseDetected()) {
        // Timer expired, but reset as presense detected
        keepOpenTimer.changePeriod(keepOpenTime);  //Set or reset the timer
    } else {
        // Timer expied and presence not detected.
        desiredDoorState = CLOSED;
    } 
}

void openDoor() {
    if (currentDoorState != OPEN) {
        if (stepper.distanceToGo() == 0) {
            // We expect to be open, but we're not. Move further.
            openPosition = openPosition - moveFurtherIncrement;
        }
        stepper.enableOutputs();
        stepper.moveTo(openPosition);
        stepper.run();
    }
}

void closeDoor() {
    if (closedPosition < 100) {  // Improbable value, reset closedPosition
                                 // For example if we start close to the closedSwitch
        closedPosition = initialClosedPosition;
    }
    if (currentDoorState != CLOSED) {
        if (stepper.distanceToGo() == 0) {
            // We expect to be closed but we're not. Move further
            closedPosition = closedPosition + moveFurtherIncrement;
        }
        stepper.enableOutputs();
        stepper.moveTo(closedPosition);
        stepper.run();
    }
}

void loop() {
    if ( desiredDoorState == OPEN) {
        if (switchDepressed(homeSwitch)) {
            openPosition = 0;
            stepper.setCurrentPosition(openPosition);  // Ensure stepper knows it where it should be
            stepper.disableOutputs();  // Disable Stepper to save power
        } else {
            keepOpenTimer.changePeriod(keepOpenTime); // Set or reset the timer
            openDoor();
        }
    } else if (desiredDoorState == CLOSED ) { 
        // This ensures that if obstructed is detected we stop immediately, not decelerating
        // This approach does set closedPosition to the point the obstuction was detected
        // will re-home on next correct closing 
        if ( (switchDepressed(closedSwitch)) or (currentDoorState == OBSTRUCTED) ) {
            if (currentDoorState == OBSTRUCTED) {
                desiredDoorState = OPEN;
                currentDoorState = MOVING;
            }
            closedPosition = stepper.currentPosition();
            stepper.setCurrentPosition(closedPosition);
            stepper.disableOutputs();  // Disable Stepper to save power
        } else {
            closeDoor();
        }
    } else if (desiredDoorState == DISABLED) {
        // Do Something else?
        stepper.disableOutputs();
    } else {
        // Current desiredDoorState not yet handled, default to closed
        desiredDoorState = CLOSED;
    }

}
