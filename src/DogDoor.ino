/*
 * Project DogDoor
 * Description: Stepper motor controlled Dog Door.
 * Author: Copyright (C) 2019-2020 Martin Rowan
 * Date: <TBD>
 */

/* 
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

/**********************************************************************************************************************
 * Define GPIO Pins
 *********************************************************************************************************************/
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

/**********************************************************************************************************************
 * Globals
 *********************************************************************************************************************/
// Particle Cloud
int homeSwitchStatus = 0;
int closedSwitchStatus = 0;
int desiredDoorStateStatus = 0;
int currentDoorStateStatus = 0;
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

// Limit end stop positions
int openPosition = 0;
long closedPosition = initialClosedPosition;

// Enum for states
enum doorStates {
    DISABLED = 0,
    OPEN = 1,
    MOVING = 2,
    CLOSED = 3,
    OBSTRUCTED = 4,
};
enum doorStates desiredDoorState;
enum doorStates currentDoorState;

/**********************************************************************************************************************
 * Global Objects
 *********************************************************************************************************************/
// Create serial Logging handler
SerialLogHandler logHandler(LOG_LEVEL_INFO);
// Create AccelStepper
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
// Serial logging timer
Timer serialLogTimer(serialLogInterval, serialLog);
// Create timer to send current variable values to cloud for switch state
Timer particleVarPublishTimer(cloudPublishInterval, publishVariables);
// Timer for how long the door is kept open after no presense detected.
Timer keepOpenTimer(keepOpenTime, timerCallback, true);

/**********************************************************************************************************************
 * Setup
 *********************************************************************************************************************/
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
    Log.info("System version: %s", (const char*)System.version());
    serialLogTimer.start();

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

/**********************************************************************************************************************
 * Interrupt Service Routines
 *********************************************************************************************************************/
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

/**********************************************************************************************************************
 * Functions
 *********************************************************************************************************************/
boolean presenseDetected() {
    // True if either sensor is active (high)
    if ( (digitalRead(indoorSensor)) or (digitalRead(outdoorSensor)) or (digitalRead(manualSwitch)) ) {
        Log.trace("presenseDetected: Indoor: %d - Outdoor: %d - Manual: %d",
                   digitalRead(indoorSensor), digitalRead(outdoorSensor), digitalRead(manualSwitch));
        return true;
    }
    return false;
}

void serialLog() {
    Log.trace("State: Current: %d - Desired: %d",
               currentDoorState, desiredDoorState);
    Log.trace("Stepper: Pos: %d - Open: %d - Closed %d - Target: %d - Speed: %d",
               stepper.currentPosition(), openPosition, closedPosition, stepper.targetPosition(),
               stepper.speed());
    Log.trace("KeepOpenTimer: %d", keepOpenTimer.isActive());
    Log.trace("Limit Switch: Home: %d - Closed %d",
               digitalRead(homeSwitch),
               digitalRead(closedSwitch));
    Log.trace("Sensors: Indoor: %d - OutDoor: %d - Push Button: %d - Keep Open: %d - Keep Closed: %d",
                digitalRead(indoorSensor), digitalRead(outdoorSensor), digitalRead(manualSwitch),
                digitalRead(keepOpenSwitch), digitalRead(keepClosedSwitch));
}

void publishVariables() {
    // Update Particle Cloud variables
    homeSwitchStatus = digitalRead(homeSwitch);
    closedSwitchStatus = digitalRead(closedSwitch);

    desiredDoorStateStatus = desiredDoorState;
    currentDoorStateStatus = currentDoorState;
}

void timerCallback() {
    if (presenseDetected()) {
        // Timer expired, but reset as presense detected
        Log.trace("timerCallback - Extending timer");
        keepOpenTimer.changePeriod(keepOpenTime);  //Set or reset the timer
    } else {
        // Timer expied and presence not detected.
        Log.trace("timerCallback - Timer expired");
        desiredDoorState = CLOSED;
    } 
}

void openDoor() {
    if (currentDoorState != OPEN) {
        if ((openPosition - stepper.currentPosition()) == 0) {
            // We expect to be open, but we're not. Move further.
            Log.warn("Opening - Homing");
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
            Log.warn("Closing - Homing");
            closedPosition = closedPosition + moveFurtherIncrement;
        }
        stepper.enableOutputs();
        stepper.moveTo(closedPosition);
        stepper.run();
    }
}


/**********************************************************************************************************************
 * Main Loop
 *********************************************************************************************************************/
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
