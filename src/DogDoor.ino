/*
 * Project DogDoor
 * Description: Stepper motor controlled Dog Door.
 * Author: Copyright (C) 2019-2020 Martin Rowan
 * Date: <TBD>
 */

/* 
 * TODO: Review logging: https://docs.particle.io/reference/device-os/firmware/photon/#logging
 * TODO: Review use of onboard RGB LED: https://docs.particle.io/reference/device-os/firmware/photon/#rgb
 * TODO: Review if PRODUCT_ID and PRODUCT_VERSION need to be defined: https://docs.particle.io/reference/device-os/firmware/photon/#macros
 * TODO: Review what information makes sense to publish to Cloud: https://docs.particle.io/reference/device-os/firmware/photon/#cloud-functions
 * TODO: Measure presenece sensor outputs to ensure high voltage is not > 3v3
 * TODO: Status LEDs connected to Analog Outputs?
 * TODO: Setup manual toggle switch
 * TODO: Setup manual trigger switch, or use button for system.reset or RST pin
 * TODO: Re-cable motors with sheilded wire
 * TODO: Review disabling OTA updates whilst motor is active: https://docs.particle.io/reference/device-os/firmware/photon/#ota-updates
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
// Presence Sensors
#define indoorSensor D5
#define outdoorSensor D6
// Manual push button switch
#define manualSwitch D7
// Keep Open and keep closed toggle switches
#define keepOpenSwitch A0
#define keepClosedSwitch A1

// Latching Toggle Switch
# define keepOpen A0
# define keepClosed A1

// Particle Cloud variables
int homeSwitchStatus = 0;
int closedSwitchStatus = 0;
int desiredDoorStateStatus = 0;
int currentDoorStateStatus = 0;

// Global Const
const bool debug = true;
const bool publish = true;

// Stepper (MicroStepping 1/32)
const int stepperSpeed = 25000;
const int stepperAccel = 15000;
const long initialClosedPosition = 72000;
const int moveFurtherIncrement = 10000;

// Timers
const int keepOpenTime = 10000;  // 10 seconds
const int cloudPublishInterval = 5000;  // 5 seconds
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
    if (publish) {
        // Set up Particle cloud variables
        Particle.variable("homeSwitch", homeSwitchStatus);
        Particle.variable("closedSwitch", closedSwitchStatus);
        Particle.variable("desiredDoorState", desiredDoorStateStatus);
        Particle.variable("currentDoorState", currentDoorStateStatus);
        // Start background timers
        particleVarPublishTimer.start();
    }

    if (debug) {
        // Start Serial Debugging
        Serial.begin(9600);
        serialTimer.start();
    }

    // Set up the limit switches
    pinMode(homeSwitch, INPUT_PULLUP);
    pinMode(closedSwitch, INPUT_PULLUP);
    // Set up interupts to handle switch changes
    attachInterrupt(homeSwitch, limitSwitchISR, CHANGE);
    attachInterrupt(closedSwitch, limitSwitchISR, CHANGE);

    // Set up the trigger sensors
    pinMode(indoorSensor, INPUT_PULLDOWN);  // PULLDOWN - High Signal must not exceed 3v3
    pinMode(outdoorSensor, INPUT_PULLDOWN); // PULLDOWN - High Signal must not exceed 3v3
    pinMode(manualSwitch, INPUT_PULLDOWN); // PULLDOWN - High Signal must not exceed 3v3
    attachInterrupt(indoorSensor, presenseSensorISR, RISING);
    attachInterrupt(outdoorSensor, presenseSensorISR, RISING);
    attachInterrupt(manualSwitch, presenseSensorISR, RISING);

    // Setup the toggle switch
    pinMode(keepOpenSwitch, INPUT_PULLDOWN);
    pinMode(keepClosedSwitch, INPUT_PULLDOWN);

    // Set up stepper motor and initialise with ensuring door is closed
    stepper.setPinsInverted (/*direction*/ true, /*step*/ false, /*enable*/ true);
    stepper.setEnablePin(enPin);
    stepper.setMaxSpeed(stepperSpeed);
    stepper.setAcceleration(stepperAccel);
    stepper.disableOutputs();

    limitSwitchISR();  // Get Initial currentDoorState
    desiredDoorState = CLOSED;  // Set initial State

    if (currentDoorState == CLOSED){
        // We're already closed at satrtup. Set open to be the inverse of the normal closed pos.
        openPosition = -initialClosedPosition;
    }
}

boolean presenseDetected() {
    // True if either sensor is active (high)
    if ( (digitalRead(indoorSensor)) or (digitalRead(outdoorSensor)) or (digitalRead(manualSwitch)) ) {
        if (debug) {
            Serial.print("Presence Detected - Indoor: ");
            Serial.print(digitalRead(indoorSensor));
            Serial.print(" - Outdoor: ");
            Serial.print(digitalRead(outdoorSensor));
            Serial.print(" - Manual Button: ");
            Serial.println(digitalRead(manualSwitch));
        }
        return true;
    }
    return false;
}

void serialLog() {
    Serial.println();
    Serial.println("-----------------------------------------------------------------------------------------------------");
    // Serial.print("C.State: ");
    // Serial.print(currentDoorState);
    // Serial.print(" - D.State: ");
    // Serial.print(desiredDoorState);
    Serial.print(" - C.Pos: ");
    Serial.print(stepper.currentPosition());
    Serial.print(" - Open Pos: ");
    Serial.print(openPosition);
    Serial.print(" - Closed Pos: ");
    Serial.print(closedPosition);
    Serial.print(" - Target Pos: ");
    Serial.print(stepper.targetPosition());
    Serial.print(" - Speed: ");
    Serial.print(stepper.speed());
    Serial.print(" - keepOpenTimer: ");
    Serial.print(keepOpenTimer.isActive());
    Serial.println();

    Serial.print("Home: ");
    Serial.print(digitalRead(homeSwitch));
    Serial.print(" Closed: ");
    Serial.print(digitalRead(closedSwitch));
    // Serial.print(" Indoor: ");
    // Serial.print(digitalRead(indoorSensor));
    // Serial.print(" Outdoor: ");
    // Serial.print(digitalRead(outdoorSensor));
    // Serial.print(" Push Button: ");
    // Serial.print(digitalRead(manualSwitch));
    // Serial.print(" KeepOpen: ");
    // Serial.print(digitalRead(keepOpenSwitch));
    // Serial.print(" KeepClosed: ");
    // Serial.print(digitalRead(keepClosedSwitch));
    // Serial.println();
}

void publishVariables() {
    // Update Particle Cloud variables
    homeSwitchStatus = digitalRead(homeSwitch);
    closedSwitchStatus = digitalRead(closedSwitch);

    desiredDoorStateStatus = desiredDoorState;
    currentDoorStateStatus = currentDoorState;
}

void limitSwitchISR() {
    if (digitalRead(homeSwitch)) {
        currentDoorState = OPEN;
    } else if (digitalRead(closedSwitch)) {
        currentDoorState = CLOSED;
    } else {
        currentDoorState = MOVING;
    }
}

void presenseSensorISR() {
    if (!digitalRead(keepClosedSwitch)){
        desiredDoorState = OPEN;
        keepOpenTimer.changePeriodFromISR(keepOpenTime);  // Set or reset the timer
    }
}

void timerCallback() {
    if (presenseDetected()) {
        // Timer expired, but reset as presense detected
        if (debug) {
            Serial.println("timerCallback - Extending timer");
        }
        keepOpenTimer.changePeriod(keepOpenTime);  //Set or reset the timer
    } else {
        // Timer expied and presence not detected.
        if (debug) {
            Serial.println("timerCallback - Timer expired");
        }
        desiredDoorState = CLOSED;
    } 
}

void openDoor() {
    if (currentDoorState != OPEN) {
        if ((openPosition - stepper.currentPosition()) == 0) {
            // We expect to be open, but we're not. Move further.
            if (debug) {
                Serial.println("Opening - Homing");
            }
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
        if ((closedPosition - stepper.currentPosition()) == 0) {
            // We expect to be closed but we're not. Move further
            if (debug) {
                Serial.println("Closing - Homing");
            }
            closedPosition = closedPosition + moveFurtherIncrement;
        }
        stepper.enableOutputs();
        stepper.moveTo(closedPosition);
        stepper.run();
    }
}

void loop() {
    if ( ((desiredDoorState == OPEN) or (digitalRead(keepOpenSwitch))) and (!digitalRead(keepClosedSwitch)) ) {
        if (digitalRead(homeSwitch)) {
            openPosition = 0;
            stepper.setCurrentPosition(openPosition);  // Ensure stepper knows it where it should be
        } else {
            openDoor();
        }
        if (currentDoorState != OPEN) {
            keepOpenTimer.changePeriod(keepOpenTime); // Set or reset the timer
        }
    } else if ( ((desiredDoorState == CLOSED) or (digitalRead(keepClosedSwitch))) and (!digitalRead(keepOpenSwitch)) ) { 
        // This ensures that if obstructed is detected we stop immediately, not decelerating
        // This approach sets closedPosition to the point the obstuction was detected
        // will re-home on next correct closing
        if ( (digitalRead(closedSwitch)) or (currentDoorState == OBSTRUCTED) ) {
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
        stepper.disableOutputs();
    }
}
