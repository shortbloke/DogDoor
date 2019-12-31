/*
 * Project DogDoor
 * Description: Stepper motor controlled Dog Door.
 * Author: Copyright (C) 2019-2020 Martin Rowan
 * Date: <TBD>
 */

/*
 * TODO: Reduced number of duplicated calls, store values in variables. e.g. pin states, mapped to a more understandable format?
 */

SYSTEM_THREAD(ENABLED);  // Have Particle processing in a separate thread - https://docs.particle.io/reference/device-os/firmware/photon/#system-thread
SYSTEM_MODE(SEMI_AUTOMATIC);

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
#define homeSwitchPin D3
#define closedSwitchPin D4
// Keep Open and keep closed toggle switches
#define keepClosedSwitchPin D5
#define keepOpenSwitchPin D6
// Manual push button switch
#define manualSwitchPin D7

// Presence Sensors
#define indoorIRSensorPin A0
#define outdoorIRSensorPin A1
// LEDs
#define biLedKeepClosed A2
#define biLedKeepOpen A3
#define biLedIndoor A4
#define biLedOutdoor A5

/**********************************************************************************************************************
 * Global
 *********************************************************************************************************************/
// Build
const char version[] = "1.0.0";
const char buildDate[] = __DATE__ " " __TIME__;

// Device Info
String deviceID = "";
String sysVer = "";
String buildString = "";
int resetReason = 0;
int resetReasonData = 0;

// Particle Cloud
int topLimitSwitchStatus = 0;
int bottomLimitSwitchStatus = 0;
int keepClosedSwitchStatus = 0;
int keepOpenSwitchStatus = 0;
int manualButtonSwitchStatus = 0;
int desiredDoorStateStatus = 0;
int currentDoorStateStatus = 0;
int indoorSensorTriggeredState = 0;
int outdoorSensorTriggeredState = 0;
int stepperEnableStatus = 0;

bool initialVarPublishComplete = false;
const bool publish = true;

// Stepper (MicroStepping 1/32)
const int stepperSpeed = 25000;
const int stepperAccel = 15000;
const long initialClosedPosition = 72000;
const int moveFurtherIncrement = 50000;

// IR Sensors
long indoorSensorValue = 0;
long outdoorSensorValue = 0;
const long indoorIRSensorTriggerThreshold = 700;
const long outdoorIRSensorTriggerThreshold = 1000;
const int irSensorPollInterval = 250;  // 1/4 of a second

// Timers
const int keepOpenTime = 10000;  // 10 seconds
const int cloudPublishInterval = 10000;  // 10 seconds
const int traceLogInterval = 5000;  // 5 second
const int periodicLogInterval = 60000;  //  1 minute

// Limit end stop positions
long openPosition = 0;
long closedPosition = initialClosedPosition;

// Enum for states
enum doorStates {
    STATE_OPEN = 0,
    STATE_MOVING = 1,
    STATE_CLOSED = 2,
    STATE_KEEPOPEN = 3,
    STATE_KEEPCLOSED = 4,
};
enum doorStates desiredDoorState;
enum doorStates currentDoorState;
enum doorStates lastDesiredDoorState;
enum doorStates lastCurrentDoorState;

// Enum for LED status
enum ledStatus {
    LED_OFF = 0,
    LED_KEEPCLOSED = 1,
    LED_KEEPOPEN = 2,
    LED_INDOOR = 3,
    LED_OUTDOOR = 4,
};

enum inputs {
    INPUT_KEEPCLOSEDSWITCH = 0,
    INPUT_KEEPOPENSWITCH = 1,
    INPUT_MANUALPUSHSWITCH = 2,
    INPUT_TOPLIMITSWITCH = 3,
    INPUT_BOTTOMLIMITSWITCH = 4,
    INPUT_INDOORSENSOR = 5,
    INPUT_OUTDOORSENSOR = 6,
    INPUT_STEPPERENABLE = 7,
};

/**********************************************************************************************************************
 * Global Objects
 *********************************************************************************************************************/
// Create serial Logging handler, set log level
SerialLogHandler logHandler(LOG_LEVEL_TRACE);
// SerialLogHandler logHandler(LOG_LEVEL_INFO);

// Create AccelStepper
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
// Poll IR sensors
Timer pollIRSensorsTimer(irSensorPollInterval, pollIRSensorISR);
// Trace logging timer
Timer traceLogTimer(traceLogInterval, traceLog);
// Periodic Log message timer
Timer periodicLogTimer(periodicLogInterval, periodicLog);
// Timer for how long the door is kept open after no presence detected.
Timer keepOpenTimer(keepOpenTime, timerCallback, true);

/**********************************************************************************************************************
 * Setup
 *********************************************************************************************************************/
void setup() {
    // Setup Device information, doesn't change post boot
    deviceID = System.deviceID();
    sysVer = System.version();
    buildString = String::format("Version: %s - Build: %s", version, buildDate);
    resetReason = (int) System.resetReason();
    resetReasonData = (int) System.resetReasonData();

    // Setup the LEDs
    pinMode(biLedKeepClosed, OUTPUT);
    pinMode(biLedKeepOpen, OUTPUT);
    pinMode(biLedIndoor, OUTPUT);
    pinMode(biLedOutdoor, OUTPUT);

    // Setup the limit switches
    pinMode(homeSwitchPin, INPUT_PULLUP);
    pinMode(closedSwitchPin, INPUT_PULLUP);
    // Setup interrupts to handle switch changes
    // attachInterrupt(homeSwitchPin, limitSwitchISR, CHANGE);
    // attachInterrupt(closedSwitchPin, limitSwitchISR, CHANGE);

    // Setup the control switches
    pinMode(manualSwitchPin, INPUT_PULLUP);
    pinMode(keepOpenSwitchPin, INPUT_PULLUP);
    pinMode(keepClosedSwitchPin, INPUT_PULLUP);
    // Setup interrupts to handle switch changes
    // attachInterrupt(manualSwitchPin, switchISR, CHANGE);
    // attachInterrupt(keepOpenSwitchPin, switchISR, CHANGE);
    // attachInterrupt(keepClosedSwitchPin, switchISR, CHANGE);

    // Setup stepper motor and initialise with ensuring door is closed
    stepper.setPinsInverted (/*direction*/ true, /*step*/ false, /*enable*/ true);
    stepper.setEnablePin(enPin);
    stepper.setMaxSpeed(stepperSpeed);
    stepper.setAcceleration(stepperAccel);
    stepper.disableOutputs();

    // Start background timers
    // traceLogTimer.start();
    periodicLogTimer.start();
    pollIRSensorsTimer.start();

    // Initial states
    // limitSwitchISR();
    desiredDoorState = STATE_CLOSED;
    // if (currentDoorState == STATE_CLOSED){
    //     // We're already closed at startup. Set open to be the inverse of the normal closed pos.
    openPosition = -initialClosedPosition;
    // }
    
    // First time call to periodicLog
    periodicLog();
}

/**********************************************************************************************************************
 * Status LED Controls
 *********************************************************************************************************************/
void setStatusLed1(ledStatus status) {
    switch(status) {
        case LED_OFF:
            digitalWrite(biLedKeepClosed, LOW);
            digitalWrite(biLedKeepOpen, LOW);
            break;
        case LED_KEEPCLOSED:
            digitalWrite(biLedKeepClosed, HIGH);
            digitalWrite(biLedKeepOpen, LOW);
            break;
        case LED_KEEPOPEN:
            digitalWrite(biLedKeepClosed, LOW);
            digitalWrite(biLedKeepOpen, HIGH);
            break;
    }
}

void setStatusLed2(ledStatus status) {
    switch(status) {
        case LED_OFF:
            digitalWrite(biLedIndoor, LOW);
            digitalWrite(biLedOutdoor, LOW);
            break;
        case LED_INDOOR:
            digitalWrite(biLedIndoor, HIGH);
            digitalWrite(biLedOutdoor, LOW);
            break;
        case LED_OUTDOOR:
            digitalWrite(biLedIndoor, LOW);
            digitalWrite(biLedOutdoor, HIGH);
            break;
    }
}

/**********************************************************************************************************************
 * Sensor state mapping function
 *********************************************************************************************************************/
bool readSensorState(inputs input) {
    // Helper function to return true for any specified sensor
    // Provides more consistent mapping, especially for PULLUP inputs.
    // Reads values into variables used for logging and publishing to cloud, without needing to read I/O again
    switch(input) {
        case INPUT_KEEPCLOSEDSWITCH:
            keepClosedSwitchStatus = !digitalRead(keepClosedSwitchPin);
            return keepClosedSwitchStatus;
        case INPUT_KEEPOPENSWITCH:
            keepOpenSwitchStatus = !digitalRead(keepOpenSwitchPin);
            return keepOpenSwitchStatus;
        case INPUT_MANUALPUSHSWITCH:
            manualButtonSwitchStatus = !digitalRead(manualSwitchPin);
            return manualButtonSwitchStatus;
        case INPUT_TOPLIMITSWITCH:
            topLimitSwitchStatus = digitalRead(homeSwitchPin);  // Normally closed switch
            return topLimitSwitchStatus;
        case INPUT_BOTTOMLIMITSWITCH:
            bottomLimitSwitchStatus = digitalRead(closedSwitchPin);  // Normally closed switch
            return bottomLimitSwitchStatus;
        case INPUT_INDOORSENSOR:
            if (indoorSensorValue >= indoorIRSensorTriggerThreshold) {
                indoorSensorTriggeredState = 1;
                return true;
            } else {
                indoorSensorTriggeredState = 0;
                return false;
            }
        case INPUT_OUTDOORSENSOR:
            if (outdoorSensorValue >= outdoorIRSensorTriggerThreshold) {
                outdoorSensorTriggeredState = 1;
                return true;
            } else {
                outdoorSensorTriggeredState = 0;
                return false;
            }
        case INPUT_STEPPERENABLE:
            stepperEnableStatus = (int) digitalRead(enPin);
            return stepperEnableStatus;
    }
}

/**********************************************************************************************************************
 * Calculate Current and Desired States
 *********************************************************************************************************************/
doorStates getCurrentDoorState() {
    // Check the limit switches
    if (readSensorState(INPUT_TOPLIMITSWITCH)) {
        return STATE_OPEN;
    } else if (readSensorState(INPUT_BOTTOMLIMITSWITCH)) {
        return STATE_CLOSED;
    // } else if (!readSensorState(INPUT_STEPPERENABLE)) {
    //     return STATE_CLOSED;  // Stepper disabled, door may drop below limit switch to end stop
    } else {
        return STATE_MOVING;
    }
}

doorStates getDesiredDoorState() {
    // Check state of switches
    if (readSensorState(INPUT_KEEPCLOSEDSWITCH)) {
        setStatusLed1(LED_KEEPCLOSED);
        return STATE_KEEPCLOSED;
    } else if (readSensorState(INPUT_KEEPOPENSWITCH)) {
        setStatusLed1(LED_KEEPOPEN);
        return STATE_KEEPOPEN;
    } else if (readSensorState(INPUT_MANUALPUSHSWITCH)) {
        setStatusLed1(LED_OFF);
        return STATE_OPEN;
    } else {
        setStatusLed1(LED_OFF);
        // Check state of sensors
        if (readSensorState(INPUT_INDOORSENSOR)) {
            keepOpenTimer.changePeriod(keepOpenTime);  // Reset the timer to prevent closing
            setStatusLed2(LED_INDOOR);
            return STATE_OPEN;
        } else if (readSensorState(INPUT_OUTDOORSENSOR)) {
            keepOpenTimer.changePeriod(keepOpenTime);  // Reset the timer to prevent closing
            setStatusLed2(LED_OUTDOOR);
            return STATE_OPEN;
        } else {
            setStatusLed2(LED_OFF);
            return desiredDoorState;  // Return the existing value
        }
    }
}

/**********************************************************************************************************************
 * Interrupt Service Routines
 *********************************************************************************************************************/
void timerCallback() {
    // Timer expired and presence not detected.
    Log.trace("timerCallback - Timer expired");
    desiredDoorState = STATE_CLOSED;
    setStatusLed2(LED_OFF);
}

// void limitSwitchISR() {
//     getCurrentDoorState();
// }

// void switchISR() {
//     getDesiredDoorState();
// }

void pollIRSensorISR() {
    indoorSensorValue = analogRead(indoorIRSensorPin);
    delay(5);
    outdoorSensorValue = analogRead(outdoorIRSensorPin);
}

/**********************************************************************************************************************
 * Stepper Actions
 *********************************************************************************************************************/
void openDoor() {
    if (currentDoorState != STATE_OPEN) {
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
    if (currentDoorState != STATE_CLOSED) {
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
 * Log Functions
 *********************************************************************************************************************/
void traceLog() {
    Log.trace("State: Current: %d - Desired: %d",
               currentDoorStateStatus, desiredDoorStateStatus);
    Log.trace("Stepper: Pos: %d - Open: %d - Closed %d - Target: %d - Speed: %d",
               stepper.currentPosition(), openPosition, closedPosition, stepper.targetPosition(), stepper.speed());
    Log.trace("KeepOpenTimer: %d", keepOpenTimer.isActive());
    Log.trace("Limit Switch: Home: %d - Closed %d", topLimitSwitchStatus, bottomLimitSwitchStatus);
    Log.trace("Sensors: Indoor: %d - OutDoor: %d - Push Button: %d - Keep Open: %d - Keep Closed: %d",
               indoorSensorValue, outdoorSensorValue, manualButtonSwitchStatus,
               keepOpenSwitchStatus, keepClosedSwitchStatus);
}

void periodicLog() {
    Log.info("Device ID: %s", (const char*)deviceID);
    Log.info("OS version: %s", (const char*)sysVer);
    Log.info("Free Memory: %d", (int) System.freeMemory());
    Log.info("Uptime (secs): %d", (int) System.uptime());
    Log.info("System restart reason: %d - Data: %d", resetReason, resetReasonData);
    Log.info("App Version: %s", (const char*)buildString);
    Log.info("Current State: %d - Desired State: %d", currentDoorStateStatus, desiredDoorStateStatus);
    Log.info("Sensors: Indoor: %d - OutDoor: %d - Push Button: %d - Keep Open: %d - Keep Closed: %d - Top Limit: %d - Bottom Limit: %d",
              indoorSensorValue, outdoorSensorValue, manualButtonSwitchStatus,
              keepOpenSwitchStatus, keepClosedSwitchStatus, topLimitSwitchStatus, bottomLimitSwitchStatus);
}

/**********************************************************************************************************************
 * Particle Cloud Setup and Publishing for data
 *********************************************************************************************************************/
void setupParticleCloud() {
    if (!Particle.connected()) {
        Particle.connect();
    } else if (!initialVarPublishComplete) {
        Log.info("Connected publishing variables");
        // Set up Particle cloud variables
        Particle.variable("Top Limit Switch", topLimitSwitchStatus);
        Particle.variable("Bottom Limit Switch ", bottomLimitSwitchStatus);
        Particle.variable("Keep Closed Switch", keepClosedSwitchStatus);
        Particle.variable("Keep Open Switch", keepOpenSwitchStatus);
        Particle.variable("Manual Switch", manualButtonSwitchStatus);
        Particle.variable("Indoor Sensor Triggered", indoorSensorTriggeredState);
        Particle.variable("Outdoor Sensor Triggered", outdoorSensorTriggeredState);
        Particle.variable("Desired State", desiredDoorStateStatus);
        Particle.variable("Current State", currentDoorStateStatus);
        // Particle.variable("Stepper Enabled", stepperEnableStatus);
        initialVarPublishComplete = true;
    }
}

/**********************************************************************************************************************
 * Main Loop
 *********************************************************************************************************************/
void loop() {
    getCurrentDoorState();
    getDesiredDoorState();

    if (desiredDoorState != lastDesiredDoorState) {
        Log.trace("DESIRED STATE CHANGED FROM: %d to %d", lastDesiredDoorState, desiredDoorState);
        lastDesiredDoorState = desiredDoorState;
    }
    if (currentDoorState != lastCurrentDoorState) {
        Log.trace("CURRENT STATE CHANGED FROM: %d to %d", lastCurrentDoorState, currentDoorState);
        lastCurrentDoorState = currentDoorState;
    }

    if (publish) {
        setupParticleCloud();
        desiredDoorStateStatus = (int) desiredDoorState;
        currentDoorStateStatus = (int) currentDoorState;
    }

    if ( (desiredDoorState == STATE_OPEN) or (desiredDoorState == STATE_KEEPOPEN) ) {
        if (currentDoorState != STATE_OPEN) {
            // Opening
            keepOpenTimer.changePeriod(keepOpenTime);  // Set or reset the timer whilst moving
            openDoor();
        } else {
            // Door Open
            openPosition = 0;
            stepper.setCurrentPosition(openPosition); // Set or reset stepper home position
        }
    } else if ( (desiredDoorState == STATE_CLOSED) or (desiredDoorState == STATE_KEEPCLOSED) ) {
        if (currentDoorState != STATE_CLOSED) {
            closeDoor();
        } else {
            closedPosition = stepper.currentPosition();
            stepper.setCurrentPosition(closedPosition);
            stepper.disableOutputs();  // Disable Stepper to save power
        }
    }
}
