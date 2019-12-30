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
int homeSwitchStatus = 0;
int closedSwitchStatus = 0;
int keepClosedSwitchStatus = 0;
int keepOpenSwitchStatus = 0;
int desiredDoorStateStatus = 0;
int lastDesiredDoorStatus = 0;
int currentDoorStateStatus = 0;
int indoorDistanceStatus = 0;
int outdoorDistanceStatus = 0;
bool initialVarPublishComplete = false;
const bool publish = true;

// Stepper (MicroStepping 1/32)
const int stepperSpeed = 25000;
const int stepperAccel = 15000;
const long initialClosedPosition = 72000;
const int moveFurtherIncrement = 10000;

// IR Sensors
long indoorIRSensor = 0;
long outdoorIRSensor = 0;
const long indoorIRSensorTriggerThreshold = 700;
const long outdoorIRSensorTriggerThreshold = 1000;

// Timers
const int keepOpenTime = 10000;  // 10 seconds
const int cloudPublishInterval = 10000;  // 10 seconds
const int traceLogInterval = 5000;  // 5 second
const int periodicLogInterval = 60000;  //  1 minute
const int irSensorPollInterval = 250;  // 1/4 of a second

// Limit end stop positions
long openPosition = 0;
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

// Enum for LED status
enum ledStatus {
    OFF = 0,
    KEEPCLOSED = 1,
    KEEPOPEN = 2,
    INDOOR = 3,
    OUTDOOR = 4,
};

/**********************************************************************************************************************
 * Global Objects
 *********************************************************************************************************************/
// Create serial Logging handler, set log level
// SerialLogHandler logHandler(LOG_LEVEL_TRACE);
SerialLogHandler logHandler(LOG_LEVEL_INFO);
// Create AccelStepper
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
// Poll IR sensors
Timer pollIRSensorsTimer(irSensorPollInterval, pollIRSensorISR);
// Trace logging timer
Timer traceLogTimer(traceLogInterval, traceLog);
// Periodic Log message timer
Timer periodicLogTimer(periodicLogInterval, periodicLog);
// Create timer to send current variable values to cloud for switch state
Timer particleVarPublishTimer(cloudPublishInterval, publishVariables);
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
    resetReason = System.resetReason();
    resetReasonData = System.resetReasonData();

    // Setup the LEDs
    pinMode(biLedKeepClosed, OUTPUT);
    pinMode(biLedKeepOpen, OUTPUT);
    pinMode(biLedIndoor, OUTPUT);
    pinMode(biLedOutdoor, OUTPUT);

    // Setup the limit switches
    pinMode(homeSwitchPin, INPUT_PULLUP);
    pinMode(closedSwitchPin, INPUT_PULLUP);
    // Setup interrupts to handle switch changes
    attachInterrupt(homeSwitchPin, limitSwitchISR, CHANGE);
    attachInterrupt(closedSwitchPin, limitSwitchISR, CHANGE);

    // Setup the control switches
    pinMode(manualSwitchPin, INPUT_PULLUP);
    attachInterrupt(manualSwitchPin, switchISR, FALLING);

    pinMode(keepOpenSwitchPin, INPUT_PULLUP);
    pinMode(keepClosedSwitchPin, INPUT_PULLUP);
    attachInterrupt(keepOpenSwitchPin, switchISR, CHANGE);
    attachInterrupt(keepClosedSwitchPin, switchISR, CHANGE);

    // Setup stepper motor and initialise with ensuring door is closed
    stepper.setPinsInverted (/*direction*/ true, /*step*/ false, /*enable*/ true);
    stepper.setEnablePin(enPin);
    stepper.setMaxSpeed(stepperSpeed);
    stepper.setAcceleration(stepperAccel);
    stepper.disableOutputs();

    // Start background timers
    traceLogTimer.start();
    periodicLogTimer.start();
    pollIRSensorsTimer.start();

    // Initial states
    limitSwitchISR();
    desiredDoorState = CLOSED;
    if (currentDoorState == CLOSED){
        // We're already closed at startup. Set open to be the inverse of the normal closed pos.
        openPosition = -initialClosedPosition;
    }
}

void statusLed1(ledStatus status) {
    switch(status) {
        case OFF:
            digitalWrite(biLedKeepClosed, LOW);
            digitalWrite(biLedKeepOpen, LOW);
            break;
        case KEEPCLOSED:
            digitalWrite(biLedKeepClosed, LOW);
            digitalWrite(biLedKeepOpen, LOW);
            break;
        case KEEPOPEN:
            digitalWrite(biLedKeepClosed, LOW);
            digitalWrite(biLedKeepOpen, HIGH);
            break;

    }
}

void statusLed2(ledStatus status) {
    switch(status) {
        case OFF:
            digitalWrite(biLedIndoor, LOW);
            digitalWrite(biLedOutdoor, LOW);
            break;
        case INDOOR:
            digitalWrite(biLedIndoor, HIGH);
            digitalWrite(biLedOutdoor, LOW);
            break;
        case OUTDOOR:
            digitalWrite(biLedIndoor, LOW);
            digitalWrite(biLedOutdoor, HIGH);
            break;

    }
}

/**********************************************************************************************************************
 * Interrupt Service Routines
 *********************************************************************************************************************/
void limitSwitchISR() {
    if (digitalRead(homeSwitchPin)) {
        currentDoorState = OPEN;
    } else if (digitalRead(closedSwitchPin)) {
        currentDoorState = CLOSED;
    } else {
        currentDoorState = MOVING;
    }
}

void switchISR() {
    if (!digitalRead(keepOpenSwitchPin) ) {
        if (desiredDoorState == CLOSED) {
            currentDoorState = OBSTRUCTED;  // Force a quicker change of direction for opening
        } else {
            desiredDoorState = OPEN;
        }
        statusLed1(KEEPOPEN);
    } else if (!digitalRead(keepClosedSwitchPin)) {
        desiredDoorState = CLOSED;
        statusLed1(KEEPCLOSED);
    } else if (!digitalRead(manualSwitchPin)) {
        if (desiredDoorState == CLOSED) {
            currentDoorState = OBSTRUCTED;  // Force a quicker change of direction for opening
        } else {
            desiredDoorState = OPEN;
        }
        statusLed1(OFF);
    } else {
        statusLed1(OFF);
    }
}

void pollIRSensorISR() {
    indoorIRSensor = analogRead(indoorIRSensorPin);
    outdoorIRSensor = analogRead(outdoorIRSensorPin);
}

/**********************************************************************************************************************
 * Functions
 *********************************************************************************************************************/
boolean presenceDetected() {
    if ( (indoorIRSensor >= indoorIRSensorTriggerThreshold) or (outdoorIRSensor >= outdoorIRSensorTriggerThreshold) ) {
        keepOpenTimer.changePeriod(keepOpenTime);  // Set or reset the timer
        if (desiredDoorState == CLOSED) {
            currentDoorState = OBSTRUCTED;  // Force a quicker change of direction for opening
        } else if (currentDoorState != OBSTRUCTED) {
            desiredDoorState = OPEN;
        }
        if (indoorIRSensor >= indoorIRSensorTriggerThreshold) {
            statusLed2(INDOOR);
        } else {
            statusLed2(OUTDOOR);
        }
        return true;
    }
    return false;
}



void traceLog() {
    Log.trace("State: Current: %d - Desired: %d",
               currentDoorState, desiredDoorState);
    Log.trace("Stepper: Pos: %d - Open: %d - Closed %d - Target: %d - Speed: %d",
               stepper.currentPosition(), openPosition, closedPosition, stepper.targetPosition(),
               stepper.speed());
    Log.trace("KeepOpenTimer: %d", keepOpenTimer.isActive());
    Log.trace("Limit Switch: Home: %d - Closed %d",
               digitalRead(homeSwitchPin),
               digitalRead(closedSwitchPin));
    Log.trace("Sensors: Indoor: %d - OutDoor: %d - Push Button: %d - Keep Open: %d - Keep Closed: %d",
                indoorIRSensor, outdoorIRSensor, digitalRead(manualSwitchPin),
                digitalRead(keepOpenSwitchPin), digitalRead(keepClosedSwitchPin));
}

void periodicLog() {
    Log.info("Device ID: %s", (const char*)deviceID);
    Log.info("OS version: %s", (const char*)sysVer);
    Log.info("Free Memory: %d", System.freeMemory());
    Log.info("Uptime (secs): %d", System.uptime());
    Log.info("System restart reason: %d - Data: %d", resetReason, resetReasonData);
    Log.info("App Version: %s", (const char*)buildString);
    Log.info("Desired State: %d - Current State: %d", desiredDoorState, currentDoorState);
    Log.info("Sensors: Indoor: %d - OutDoor: %d - Push Button: %d - Keep Open: %d - Keep Closed: %d",
             indoorIRSensor, outdoorIRSensor, digitalRead(manualSwitchPin),
             digitalRead(keepOpenSwitchPin), digitalRead(keepClosedSwitchPin));
}

void timerCallback() {
    // Timer expired and presence not detected.
    Log.trace("timerCallback - Timer expired");
    desiredDoorState = CLOSED;
    statusLed2(OFF);
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
 * Particle Cloud Setup and Publishing for data
 *********************************************************************************************************************/
void setupParticleCloud() {
    if (!Particle.connected()) {
        Particle.connect();
    } else if (!initialVarPublishComplete) {
        Log.info("Connected publishing variables");
        // Set up Particle cloud variables
        Particle.variable("HomeSwitch", homeSwitchStatus);
        Particle.variable("ClosedSwitch", closedSwitchStatus);
        Particle.variable("KeepClosedSwitch", keepOpenSwitchStatus);
        Particle.variable("KeepOpenSwitch", keepClosedSwitchStatus);
        Particle.variable("IndoorDistance", indoorDistanceStatus);
        Particle.variable("OutdoorDistance", outdoorDistanceStatus);
        Particle.variable("DesiredDoorState", desiredDoorStateStatus);
        Particle.variable("CurrentDoorState", currentDoorStateStatus);
        // Start background timers
        particleVarPublishTimer.start(); 
        // First time call to periodicLog
        periodicLog();
        initialVarPublishComplete = true;
    }
}

void publishVariables() {
    // Update Particle Cloud variables - Called by timer
    homeSwitchStatus = digitalRead(homeSwitchPin);
    closedSwitchStatus = digitalRead(closedSwitchPin);
    keepClosedSwitchStatus = digitalRead(keepClosedSwitchPin);
    keepOpenSwitchStatus = digitalRead(keepOpenSwitchPin);
    indoorDistanceStatus = indoorIRSensor;
    outdoorDistanceStatus = outdoorIRSensor;
    desiredDoorStateStatus = desiredDoorState;
    currentDoorStateStatus = currentDoorState;
}

/**********************************************************************************************************************
 * Main Loop
 *********************************************************************************************************************/
void loop() {
    if (publish) {
        setupParticleCloud();
    }

    if ( ((desiredDoorState == OPEN) or (!digitalRead(keepOpenSwitchPin)) or presenceDetected()) and (digitalRead(keepClosedSwitchPin)) ) {
        if (digitalRead(homeSwitchPin)) {
            openPosition = 0;
            stepper.setCurrentPosition(openPosition);  // Ensure stepper knows it where it should be
        } else {
            openDoor();
        }
        if (currentDoorState != OPEN) {
            keepOpenTimer.changePeriod(keepOpenTime); // Set or reset the timer
        }
    } else if ( ((desiredDoorState == CLOSED) or (!digitalRead(keepClosedSwitchPin))) and (digitalRead(keepOpenSwitchPin)) ) { 
        // This ensures that if obstructed is detected we stop immediately, not decelerating
        // This approach sets closedPosition to the point the obstruction was detected
        // will re-home on next correct closing
        if ( (digitalRead(closedSwitchPin)) or (currentDoorState == OBSTRUCTED) ) {
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
