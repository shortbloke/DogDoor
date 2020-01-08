/*
 * Project DogDoor
 * Description: Stepper motor controlled Pet/Dog door.
 *              Uses:
 *                - Particle Photon microcontroller
 *                - Sharp GP2Y0A21 infrared proximity/distance sensors for presence detection.
 *                - Home and End limit microswitches
 *                - 2 x NEMA17 Stepper motors via TB6600 Stepper controller, set to 1/32 micro-stepping
 *                - AccelStepper library provides acceleration function to stepper controller
 *                - State information accessible via Particle.io cloud
 *                - Override control and system commands via Particle.io cloud
 *
 *              In order for stepper to run as quickly and smoothly as possible, time outside the loop needs to be
 *              minimised. This is achieved using interrupts to trigger the reading of values and storing the result
 *              in variables which are significantly quicker to access than reading the I/O:
 *               - Timer based polling infrared sensors
 *               - Limit switches read on state change
 *               - Manual control switches read on state change
 * 
 * Author: Copyright (C) 2019-2020 Martin Rowan
 * Date: 1st January 2020
 */

SYSTEM_THREAD(ENABLED);  // Have Particle processing in a separate thread - https://docs.particle.io/reference/device-os/firmware/photon/#system-thread
SYSTEM_MODE(SEMI_AUTOMATIC);

// Include the AccelStepper library (AccelStepperSpark):
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
const char version[] = "1.1.0";
const char buildDate[] = __DATE__ " " __TIME__;

// Performance profiling
bool performanceProfiling = false;
// For Main Loop
uint32_t duration = 0;
// For Stepper motor time to open door
uint32_t openDuration = 0;
uint32_t openStart = 0;
uint32_t openEnd = 0;

// Device Info
String deviceString = "";
String buildString = "";
int resetReason = 0;
int resetReasonData = 0;
const int wdTimeout = 30000;  // 30 seconds watchdog timeout

// Particle Cloud
volatile int topLimitSwitchStatus = 0;
volatile int bottomLimitSwitchStatus = 0;
volatile int keepClosedSwitchStatus = 0;
volatile int keepOpenSwitchStatus = 0;
volatile int manualButtonSwitchStatus = 0;
String desiredDoorStateStatus = "";
String currentDoorStateStatus = "";
String overriddenDesiredDoorStateStatus = "";
bool initialPublishComplete = false;
const bool publish = true;
const int vitalsPublishInterval = 300;  // 5 minutes
const int checkConnectedInterval = 600000; // 10 minutes
bool systemResetRequested = false;

// Periodic Serial Logging
const int periodicLogInterval = 60000;  //  1 minute
volatile bool periodicLogNow = false;

// Stepper (MicroStepping 1/32)
const float stepperSpeed = 25000;
const float stepperAccel = 25000;
const long initialClosedPosition = 72000;
bool stepperPowerSave = false;

// IR Sensors
volatile long indoorSensorValue = 0;
volatile long outdoorSensorValue = 0;
const long indoorIRSensorTriggerThreshold = 600;
const long outdoorIRSensorTriggerThreshold = 1000;
const int irSensorPollInterval = 100;
volatile bool indoorDetected = false;
volatile bool outdoorDetected = false;
bool lastIndoorDetected = false;
bool lastOutdoorDetected = false;

// Limit end stop positions
long openPosition = 0;
long closedPosition = initialClosedPosition;

// Keep the door open for a period.
const int keepOpenTime = 10000;  // 10 seconds 

// Keep in sync in ENUM for ease of logging
const char* doorStatesCString[] = {"OPEN", "MOVING", "CLOSED", "KEEPOPEN", "KEEPCLOSED"};
// Enum for states
enum doorStates {
    STATE_OPEN = 0,
    STATE_MOVING = 1,
    STATE_CLOSED = 2,
    STATE_KEEPOPEN = 3,
    STATE_KEEPCLOSED = 4,
};
volatile enum doorStates desiredDoorState = STATE_CLOSED;
volatile enum doorStates currentDoorState;
enum doorStates lastDesiredDoorState;
enum doorStates lastCurrentDoorState;

// Cloud Function Variables
bool overrideDoorState = false;
enum doorStates overriddenDesiredDoorState;

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
SerialLogHandler logHandler(LOG_LEVEL_INFO);
// Application Watchdog
ApplicationWatchdog wd(wdTimeout, System.reset);
// Particle Publish connection watchdog
Timer particleCloudConnectedTimer(checkConnectedInterval, checkParticleCloudConnection);
// Create AccelStepper
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
// Poll IR sensors
Timer pollIRSensorsTimer(irSensorPollInterval, pollIRSensorISR);
// Periodic Log message timer
Timer periodicLogTimer(periodicLogInterval, setPeriodicLogNow);
// Timer for how long the door is kept open after no presence detected.
Timer keepOpenTimer(keepOpenTime, timerCallback, true);

/**********************************************************************************************************************
 * Setup
 *********************************************************************************************************************/
void setup() {
    // Setup Device information, doesn't change post boot
    deviceString = String::format("ID: %s - OS: %s",
                                   System.deviceID().c_str(),
                                   System.version().c_str());
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
    attachInterrupt(homeSwitchPin, limitSwitchISR, CHANGE);
    attachInterrupt(closedSwitchPin, limitSwitchISR, CHANGE);

    // Setup the control switches
    pinMode(manualSwitchPin, INPUT_PULLUP);
    pinMode(keepOpenSwitchPin, INPUT_PULLUP);
    pinMode(keepClosedSwitchPin, INPUT_PULLUP);
    // Setup interrupts to handle switch changes
    attachInterrupt(manualSwitchPin, switchISR, CHANGE);
    attachInterrupt(keepOpenSwitchPin, switchISR, CHANGE);
    attachInterrupt(keepClosedSwitchPin, switchISR, CHANGE);

    // Setup stepper motor and initialise with ensuring door is closed
    stepper.setPinsInverted (/*direction*/ true, /*step*/ false, /*enable*/ true);
    stepper.setEnablePin(enPin);
    stepper.setMaxSpeed(stepperSpeed);
    stepper.setAcceleration(stepperAccel);
    stepper.disableOutputs();

    // Start background timers
    periodicLogTimer.start();
    pollIRSensorsTimer.start();

    // Initial states
    switchISR();
    limitSwitchISR();
    
    if (currentDoorState == STATE_CLOSED){
        // We're already closed at startup. Set open to be the inverse of the normal closed pos.
        openPosition = -initialClosedPosition;
    }
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
 * Read switches and sensors.
 * Stores values to reduce number of calls to read inputs (slow)
 *********************************************************************************************************************/

void readSwitchValues(){
    keepClosedSwitchStatus = !digitalRead(keepClosedSwitchPin);
    keepOpenSwitchStatus = !digitalRead(keepOpenSwitchPin);
    manualButtonSwitchStatus = !digitalRead(manualSwitchPin);
}

void readLimitSwitchValues() {
    topLimitSwitchStatus = digitalRead(homeSwitchPin);  // Normally closed switch
    bottomLimitSwitchStatus = digitalRead(closedSwitchPin);  // Normally closed switch
}

bool indoorPresenceDetected() {
    indoorSensorValue = analogRead(indoorIRSensorPin);
    if (indoorSensorValue >= indoorIRSensorTriggerThreshold) {
        // Read again to try and avoid false trigger
        indoorSensorValue = analogRead(indoorIRSensorPin);
        if (indoorSensorValue >= indoorIRSensorTriggerThreshold) {
            keepOpenTimer.changePeriodFromISR(keepOpenTime);  // Reset the timer to prevent closing
        return true;
        }
    }
    return false;
}

bool outdoorPresenceDetected() {
    outdoorSensorValue = analogRead(outdoorIRSensorPin);
    if (outdoorSensorValue >= outdoorIRSensorTriggerThreshold) {
        outdoorSensorValue = analogRead(outdoorIRSensorPin);
        if (outdoorSensorValue >= outdoorIRSensorTriggerThreshold) {
            keepOpenTimer.changePeriodFromISR(keepOpenTime);  // Reset the timer to prevent closing
        return true;
        }
    }
    return false;
}


/**********************************************************************************************************************
 * Calculate Current and Desired States
 *********************************************************************************************************************/
doorStates getCurrentDoorState() {
    if (topLimitSwitchStatus) {
        return STATE_OPEN;
    } else if (bottomLimitSwitchStatus) {
        return STATE_CLOSED;
    } else {
        return STATE_MOVING;
    }
}

doorStates getDesiredDoorState() {
    if (overrideDoorState){
        return overriddenDesiredDoorState;
    } else if (keepClosedSwitchStatus) {
        setStatusLed1(LED_KEEPCLOSED);
        return STATE_KEEPCLOSED;
    } else if (keepOpenSwitchStatus) {
        setStatusLed1(LED_KEEPOPEN);
        return STATE_KEEPOPEN;
    } else if (manualButtonSwitchStatus) {
        setStatusLed1(LED_OFF);
        return STATE_OPEN;
    } else {
        setStatusLed1(LED_OFF);
        if (indoorDetected) {
            setStatusLed2(LED_INDOOR);
            return STATE_OPEN;
        } else if (outdoorDetected) {
            setStatusLed2(LED_OUTDOOR);
            return STATE_OPEN;
        } else if (keepOpenTimer.isActive()) {
            setStatusLed1(LED_KEEPOPEN);
            return STATE_OPEN;
        } else {
            setStatusLed2(LED_OFF);
            return STATE_CLOSED;
        }
    }
}


/**********************************************************************************************************************
 * Interrupt Service Routines
 *********************************************************************************************************************/
void timerCallback() {
    Log.trace("timerCallback - Timer expired");
    if (overriddenDesiredDoorState == STATE_OPEN) {  // Clear the override on timeout.
        overrideDoorState = false;
    }
    desiredDoorState = getDesiredDoorState();
}

void limitSwitchISR() {
    readLimitSwitchValues();
    currentDoorState = getCurrentDoorState();
}

void switchISR() {
    readSwitchValues();
    desiredDoorState = getDesiredDoorState();

}

void pollIRSensorISR() {
    indoorDetected = indoorPresenceDetected();
    outdoorDetected = outdoorPresenceDetected();
    desiredDoorState = getDesiredDoorState();
}

void setPeriodicLogNow() {
    periodicLogNow = true;
}

void checkParticleCloudConnection() {
    if (!Particle.connected) {
        System.reset();
    }
}

/**********************************************************************************************************************
 * Stepper Actions
 *********************************************************************************************************************/
void driveStepper(long targetPosition) {
    // TODO: Check performance. Can we loop more quickly if we don't enable and repeat setting moveto?
    if (stepperPowerSave) {
        stepper.enableOutputs();
        stepperPowerSave = false;
    }
    stepper.moveTo(targetPosition);
    stepper.run();
}

void stopStepper() {
    // Set the stepper to think it's already reached the closed position.
    closedPosition = stepper.currentPosition();
    stepper.setCurrentPosition(closedPosition);
}

void openDoor() {
    if (currentDoorState != STATE_OPEN) {
        if ( (performanceProfiling) and (openStart == 0) ) {
            openStart = System.ticks();
        }
        // Opening
        keepOpenTimer.changePeriod(keepOpenTime);  // Set or reset the timer whilst moving
        
        if (!stepper.isRunning()) {
            // Move completed, but we're not there yet. Move more.
            Log.warn("Opening - Homing");
            openPosition = openPosition - initialClosedPosition;
        }
        driveStepper(openPosition);
    } else {
        if ( (performanceProfiling) and (openStart > 0) ) {
            openEnd = System.ticks();
            openDuration = (openEnd-openStart)/System.ticksPerMicrosecond();
            openStart = 0;
        }
        // Door Open
        openPosition = 0; // Set Zero position
        stepper.setCurrentPosition(openPosition); // Set or reset stepper home position
    }
}

void closeDoor() {
    if (currentDoorState != STATE_CLOSED) {
        // Closing
        if (closedPosition < 100) {  // Improbable value (i.e. powered on closed)
            closedPosition = initialClosedPosition;
        }
        if (!stepper.isRunning()) {  // TODO: check this doesn't trigger on 1st time through
            // Move completed, but we're not there yet. Move more.
            Log.warn("Closing - Homing");
            closedPosition = closedPosition + initialClosedPosition;
        }
        driveStepper(closedPosition);
    } else if (!stepperPowerSave) {
        // Door Closed
        stepper.disableOutputs();  // Disable Stepper to save power
        closedPosition = stepper.currentPosition();
        stepper.setCurrentPosition(closedPosition);
        stepperPowerSave = true;
    }
}

/**********************************************************************************************************************
 * Check for state changes
 *********************************************************************************************************************/
void checkStateChange() {
    // Check for desiredDoorState changes - Log once
    if (desiredDoorState != lastDesiredDoorState) {
        Log.info("DESIRED STATE CHANGED FROM: %s [%d] to %s [%d]",
                doorStatesCString[lastDesiredDoorState], lastDesiredDoorState,
                doorStatesCString[desiredDoorState], desiredDoorState);
        desiredDoorStateStatus = String::format("%s", doorStatesCString[desiredDoorState]);
        if ( (lastDesiredDoorState == STATE_CLOSED) and (desiredDoorState != STATE_KEEPCLOSED) ) {
            // We were closing, but now need to open. Ideally as quickly as possible.
            stopStepper();
        }
        lastDesiredDoorState = desiredDoorState;
    }

    // check for currentDoorState changes - Log once
    if (currentDoorState != lastCurrentDoorState) {
        Log.info("CURRENT STATE CHANGED FROM: %s [%d] to %s [%d]",
                    doorStatesCString[lastCurrentDoorState], lastCurrentDoorState,
                    doorStatesCString[currentDoorState], currentDoorState);
        currentDoorStateStatus = String::format("%s", doorStatesCString[currentDoorState]);
        lastCurrentDoorState = currentDoorState;
    }
}

/**********************************************************************************************************************
 * Log Functions
 *********************************************************************************************************************/
void periodicLog() {
    Log.info("Device: %s", deviceString.c_str());
    Log.info("App Version: %s", buildString.c_str());
    Log.info("Free Memory (B): %d", System.freeMemory());
    Log.info("Uptime (mins): %d", (int) (System.uptime() / 60));
    Log.info("System restart reason: %d - %d", resetReason, resetReasonData);
    Log.info("Current State: %s [%d] - Desired State: %s [%d]",
              doorStatesCString[currentDoorState], currentDoorState,
              doorStatesCString[desiredDoorState], desiredDoorState);
    Log.info("Indoor: %d - Outdoor: %d - Button: %d",
              indoorSensorValue, outdoorSensorValue, manualButtonSwitchStatus);
    Log.info("KeepOpen: %d - KeepClosed: %d - Top: %d - Bottom: %d",
              keepOpenSwitchStatus, keepClosedSwitchStatus, topLimitSwitchStatus, bottomLimitSwitchStatus);
    periodicLogNow = false;  // Reset, will be enabled by Timer ISR
}

void checkSensorChange() {
    // Check indoorDetected change - Log once 
    if (indoorDetected != lastIndoorDetected) {
        if (indoorDetected) {
            Log.info("MOTION DETECTED INDOOR: %d", indoorSensorValue);
            if (publish) {
                Particle.publish("INDOOR-MOTION-DETECTED",
                                String::format("%d", indoorSensorValue),
                                PRIVATE);
            }
        }
        lastIndoorDetected = indoorDetected;
    }
    // Check outdoorDetected change - Log once
    if (outdoorDetected != lastOutdoorDetected) {
        if (outdoorDetected) {
            Log.info("MOTION DETECTED OUTDOOR: %d", outdoorSensorValue);
            if (publish) {
                Particle.publish("OUTDOOR-MOTION-DETECTED",
                                String::format("%d", outdoorSensorValue),
                                PRIVATE);
            }
        }
        lastOutdoorDetected = outdoorDetected;
    }
}

void nonCriticalTasks() {
    if (publish) {
        setupParticleCloud();  // Connect to Particle Cloud and publish variables and functions
    }
    if (periodicLogNow) {
        periodicLog();  // Log to serial port
    }
}

/**********************************************************************************************************************
 * Particle Cloud Setup and Publishing for data
 *********************************************************************************************************************/
int setDesiredState(String requestedState) {
    Log.info("setDesiredState: %s", requestedState.c_str());
    if (strcasecmp(requestedState.c_str(), "open")==0) {
        overriddenDesiredDoorState = STATE_OPEN;
        overrideDoorState = true;
    } else if (strcasecmp(requestedState.c_str(), "keepopen")==0) {
        overriddenDesiredDoorState = STATE_KEEPOPEN;
        overrideDoorState = true;
    } else if (strcasecmp(requestedState.c_str(), "keepclosed")==0) {
        overriddenDesiredDoorState = STATE_KEEPCLOSED;
        overrideDoorState = true;
    } else if (strcasecmp(requestedState.c_str(), "reset")==0) {
        overrideDoorState = false;
    } else {
        Log.warn("setDesiredState received invalid requestedState value: %s", requestedState.c_str());
        Particle.publish("setDesiredState-error", requestedState, PRIVATE);
        overriddenDesiredDoorStateStatus = String::format("%s", doorStatesCString[overrideDoorState]);
        return 0;
    }
    Particle.publish("setDesiredState-success", requestedState, PRIVATE); 
    return 1;
}

int remoteCommand(String command) {
    Log.info("remoteCommand: %s", command.c_str());
    if (strcasecmp(command.c_str(), "reset")==0) {
        Particle.publish("remoteCommand", "reset", PRIVATE);
        systemResetRequested = true;
        return 1;
    }
    return 0;
}

int profile(String command) {
    if (strcasecmp(command.c_str(), "true")==0) {
        performanceProfiling = true;
        return 1;
    } else if (strcasecmp(command.c_str(), "false")==0) {
        performanceProfiling = false;
        return 1;
    }
    return 0;
}

void setupParticleCloud() {
    if (!Particle.connected()) {
        Particle.connect();
        initialPublishComplete = false;
    } else if (!initialPublishComplete) {
        Log.info("Connected publishing variables");
        particleCloudConnectedTimer.start();  // Connection watchdog.
        // Setup Particle cloud variables
        Particle.variable("TopLimitSwitch", (int*)&topLimitSwitchStatus, INT);
        Particle.variable("BottomLimitSwitch", (int*)&bottomLimitSwitchStatus, INT);
        Particle.variable("KeepClosedSwitch", (int*)&keepClosedSwitchStatus, INT);
        Particle.variable("KeepOpenSwitch", (int*)&keepOpenSwitchStatus, INT);
        Particle.variable("ManualSwitch", (int*)&manualButtonSwitchStatus, INT);
        Particle.variable("DesiredState", desiredDoorStateStatus);
        Particle.variable("CurrentState", currentDoorStateStatus);
        Particle.variable("IndoorSensor", (int*)&indoorSensorValue, INT);
        Particle.variable("OutDoorSensor", (int*)&outdoorSensorValue, INT);
        Particle.variable("overrideDoorState", overrideDoorState);
        Particle.variable("overriddenDesiredState", overriddenDesiredDoorStateStatus);
        Particle.variable("PerfProfiling", performanceProfiling);
        Particle.variable("LoopDuration", duration);
        Particle.variable("OpenDuration", openDuration);

        // Setup Particle cloud functions
        Particle.function("setDesiredState", setDesiredState);
        Particle.function("remoteCommand", remoteCommand);
        Particle.function("PerfProfiling", profile);
        // publish vitals on a timed interval
        Particle.publishVitals(vitalsPublishInterval);
        initialPublishComplete = true;
    }
}


/**********************************************************************************************************************
 * Main Loop
 *********************************************************************************************************************/
void loop() {
    uint32_t start;
    if (performanceProfiling) {
        start = System.ticks();
    }

    if (currentDoorState == desiredDoorState) {
        // Ok we're not needing to move the stepper so can do some other things in the loop. 
        // Otherwise we want to keep any many cycles free for running the stepper
        nonCriticalTasks();
        if (systemResetRequested) {
            delay(2000);  // Wait a couple of seconds to allow outstanding messages to be sent.
            System.reset();
        }
    }

    checkStateChange();  // Check if curent or desired states have changed
    checkSensorChange();  // Check if sensors have detected something, if so log once.

    if ( (desiredDoorState == STATE_OPEN) or (desiredDoorState == STATE_KEEPOPEN) ) {
        openDoor();
    } else if ( (desiredDoorState == STATE_CLOSED) or (desiredDoorState == STATE_KEEPCLOSED) ) {
        closeDoor();
    } else {
        Log.warn("Unexpected State - Desired: %d, Current: %d", desiredDoorState, currentDoorState);
    }
    if (performanceProfiling) {
        uint32_t end = System.ticks();
        duration = (end-start)/System.ticksPerMicrosecond();
    }
}