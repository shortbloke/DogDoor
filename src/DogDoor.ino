/*
 * Project DogDoor
 * Description: Stepper motor controlled Pet/Dog door.
 *              Uses:
 *                - Particle Photon microcontroller
 *                - Sharp GP2Y0A21 infrared proximity/distance sensors for presence detection.
 *                - Home and End limit microswitches
 *                - 2 x NEMA17 Stepper motors via TB6600 Stepper controller, set to 1/32 micro-stepping
 *                - AccelStepper library provides acceleration function to stepper controller
 *                - MQTT library to provide control and state information reporting to HomeAssistant.io
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
 */

SYSTEM_THREAD(ENABLED);  // Have Particle processing in a separate thread - https://docs.particle.io/reference/device-os/firmware/photon/#system-thread
SYSTEM_MODE(SEMI_AUTOMATIC);

#include <AccelStepper.h>  // Include the AccelStepper library (AccelStepperSpark):
#include <MQTT.h>  // Include MQTT Library for integration with HomeAssistant.io

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

// TODO: Clean up ordering and grouping of vars. It's a bit of a mess.
// TODO: Review var naming.
// TODO: Review function names

// Build
const char version[] = "1.4.1";
const char buildDate[] = __DATE__ " " __TIME__;

// MQTT
byte mqttServer[] = {192,168,200,45};
const char *mqttUser = "username";
const char *mqttPassword = "password";
const bool enableMqtt = true;
bool mqttConnected = false;

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
const int checkConnectedInterval = 600000;  // 10 minutes
bool systemResetRequested = false;

// Periodic Serial Logging
const int periodicLogInterval = 60000;  //  1 minute

// Flags
volatile bool periodicLogNow = false;
volatile bool checkStateNow = false;
volatile bool checkSensorNow = false;
volatile bool runNonCriticalTasksNow = true;  // Run soon after startup
const int runNonCriticalTasksMaxInterval = 300000;  // 5 minutes

// Stepper (MicroStepping 1/32)
const float stepperSpeed = 25000;
const float stepperAccel = 25000;
const long initialClosedPosition = 72000;
bool stepperPowerSave = false;
bool openDoorInitialLoop = true;
bool closeDoorInitialLoop = true;

// IR Sensors
volatile long indoorSensorValue = 0;
volatile long outdoorSensorValue = 0;
const long indoorIRSensorTriggerThreshold = 800;
const long outdoorIRSensorTriggerThreshold = 1000;
const int irSensorPollInterval = 250;  // 1/4 second
volatile bool indoorDetected = false;
volatile bool outdoorDetected = false;
bool lastIndoorDetected = false;
bool lastOutdoorDetected = false;

// Limit end stop positions
long openPosition = 0;
long closedPosition = initialClosedPosition;

// Keep the door open for a period.
const int keepOpenTime = 10000;  // 10 seconds
const int keepOpenTimerMaxResetsWaitingToOpen = 6;
int keepOpenTimerResetCountWaitingToOpen = 0;

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
// Ensure non-critical tasks run periodically
Timer nonCriticalTaskTimer(runNonCriticalTasksMaxInterval, setRunNonCriticalTasksNow);
// Create AccelStepper
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
// Poll IR sensors
Timer pollIRSensorsTimer(irSensorPollInterval, pollIRSensorISR);
// Periodic Log message timer
Timer periodicLogTimer(periodicLogInterval, setPeriodicLogNow);
// Timer for how long the door is kept open after no presence detected.
Timer keepOpenTimer(keepOpenTime, timerCallback, true);
// MQTT Client
MQTT mqttClient(mqttServer, 1883, mqttCallback);

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
    if (currentDoorState != STATE_OPEN) {
        keepOpenTimer.changePeriodFromISR(keepOpenTime);
        keepOpenTimerResetCountWaitingToOpen++;
        if (keepOpenTimerResetCountWaitingToOpen > keepOpenTimerMaxResetsWaitingToOpen) {
            runNonCriticalTasksNow = true;
            systemResetRequested = true;
        }
    }
}

void limitSwitchISR() {
    readLimitSwitchValues();
    currentDoorState = getCurrentDoorState();
    checkStateNow = true;  // Since ISR called, something has changed
}

void switchISR() {
    readSwitchValues();
    desiredDoorState = getDesiredDoorState();
    checkStateNow = true;  // Since ISR called, something has changed
}

void pollIRSensorISR() {
    bool initialIndoor = indoorDetected;
    bool initialOutdoor = outdoorDetected; 
    indoorDetected = indoorPresenceDetected();
    outdoorDetected = outdoorPresenceDetected();
    if ( (initialIndoor != indoorDetected) or (initialOutdoor != outdoorDetected) ){
        checkSensorNow = true;  // Trigger only on change
    }
    doorStates initialState = desiredDoorState;
    desiredDoorState = getDesiredDoorState();
    if (initialState != desiredDoorState) {
        checkStateNow = true;  // Trigger only on change
    }
}

void setPeriodicLogNow() {
    periodicLogNow = true;
}

void checkParticleCloudConnection() {
    if (!Particle.connected) {
        System.reset();
    }
}

void setRunNonCriticalTasksNow() {
    runNonCriticalTasksNow = true;
}

/**********************************************************************************************************************
 * Stepper Actions
 *********************************************************************************************************************/
void stopStepper() {
    // Set the stepper to think it's already reached the closed position.
    closedPosition = stepper.currentPosition();
    stepper.setCurrentPosition(closedPosition);
}

void openDoor() {
    if (currentDoorState != STATE_OPEN) {
        if (openDoorInitialLoop) {
            if (performanceProfiling) {
                openStart = System.ticks();
            }
            keepOpenTimer.start();
            stepper.enableOutputs();
            stepper.moveTo(openPosition);
            openDoorInitialLoop = false;
        } else if (stepper.distanceToGo() == 0) {
            // Move completed, but we're not there yet. Move more.
            Log.warn("Opening - Homing");
            openPosition = openPosition - initialClosedPosition;
            stepper.moveTo(openPosition);
        }
        stepper.run();
    } else {
        if ( (performanceProfiling) and (openStart > 0) ) {
            openEnd = System.ticks();
            openDuration = (openEnd-openStart)/System.ticksPerMicrosecond();
            openStart = 0;
        }
        // Door Open
        // openPosition = 0;  // Set Zero position
        openPosition = -10000;  // Remove decleration on open.
        stepper.setCurrentPosition(0);  // Set or reset stepper home position
        openDoorInitialLoop = true;  // reset flag
    }
}

void closeDoor() {
    if (currentDoorState != STATE_CLOSED) {
        if (closeDoorInitialLoop) {
            if (closedPosition < 100) {  // Improbable value (i.e. powered on closed)
                closedPosition = initialClosedPosition;
            }
            stepper.enableOutputs();
            stepper.moveTo(closedPosition);
            stepperPowerSave = false;
            closeDoorInitialLoop = false;
        } else if (stepper.distanceToGo() == 0) {
            // Move completed, but we're not there yet. Move more.
            Log.warn("Closing - Homing");
            closedPosition = closedPosition + initialClosedPosition;
            stepper.moveTo(closedPosition);
            stepperPowerSave = false;
        }
        stepper.run();
    } else if (!stepperPowerSave) {
        // Door Closed
        stepper.disableOutputs();  // Disable Stepper to save power
        closedPosition = stepper.currentPosition();
        stepper.setCurrentPosition(closedPosition);
        stepperPowerSave = true;
        closeDoorInitialLoop = true;
    }
}

/**********************************************************************************************************************
 * Check for state changes
 *********************************************************************************************************************/
void checkStateChange() {
    checkStateNow = false;
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
        if ( (enableMqtt) and (mqttClient.isConnected()) ) {
            mqttClient.publish("homeassistant/cover/petdoor/state", doorStatesCString[currentDoorState]);
            Log.info("MQTT Publish currentDoorState change");
            if (publish) {
                Particle.publish("mqtt publish . petdoor/state", String(doorStatesCString[currentDoorState]), PRIVATE);
            }
        }
        currentDoorStateStatus = String::format("%s", doorStatesCString[currentDoorState]);
        lastCurrentDoorState = currentDoorState;
    }
}

/**********************************************************************************************************************
 * Log Functions
 *********************************************************************************************************************/
void periodicLog() {
    periodicLogNow = false;  // Reset, will be enabled by Timer ISR
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
}

void checkSensorChange() {
    checkSensorNow = false;
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
    runNonCriticalTasksNow = false;
    nonCriticalTaskTimer.changePeriod(runNonCriticalTasksMaxInterval); 
    if (publish) {
        setupParticleCloud();  // Connect to Particle Cloud and publish variables and functions
    }
    if (enableMqtt) {
        if (!mqttConnected) {
            setupMqtt();
        } else {
            mqttConnected = mqttClient.loop();
        }
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
    }
    if (!initialPublishComplete) {
        Log.info("Connected publishing variables");
        particleCloudConnectedTimer.start();  // Connection watchdog.
        // Setup Particle cloud variables
        bool pubv1 = Particle.variable("TopLimitSwitch", (int*)&topLimitSwitchStatus, INT);
        bool pubv2 = Particle.variable("BottomLimitSwitch", (int*)&bottomLimitSwitchStatus, INT);
        bool pubv3 = Particle.variable("KeepClosedSwitch", (int*)&keepClosedSwitchStatus, INT);
        bool pubv4 = Particle.variable("KeepOpenSwitch", (int*)&keepOpenSwitchStatus, INT);
        bool pubv5 = Particle.variable("ManualSwitch", (int*)&manualButtonSwitchStatus, INT);
        bool pubv6 = Particle.variable("DesiredState", desiredDoorStateStatus);
        bool pubv7 = Particle.variable("CurrentState", currentDoorStateStatus);
        bool pubv8 = Particle.variable("IndoorSensor", (int*)&indoorSensorValue, INT);
        bool pubv9 = Particle.variable("OutDoorSensor", (int*)&outdoorSensorValue, INT);
        bool pubv10 = Particle.variable("overrideDoorState", overrideDoorState);
        bool pubv11 = Particle.variable("overriddenDesiredState", overriddenDesiredDoorStateStatus);
        bool pubv12 = Particle.variable("PerfProfiling", performanceProfiling);
        bool pubv13 = Particle.variable("LoopDuration", duration);
        bool pubv14 = Particle.variable("OpenDuration", openDuration);
        bool pubv15 = Particle.variable("MQTTConnected", mqttConnected);
        bool publishVariablesSuccess = (pubv1 and pubv2 and pubv3 and pubv4 and pubv5 and pubv6 and pubv7 and pubv8
                                        and pubv9 and pubv10 and pubv11 and pubv12 and pubv13 and pubv14 and pubv15);

        // Setup Particle cloud functions
        bool pubf1 = Particle.function("setDesiredState", setDesiredState);
        bool pubf2 = Particle.function("remoteCommand", remoteCommand);
        bool pubf3 = Particle.function("PerfProfiling", profile);
        bool publishFunctionsSuccess = (pubf1 and pubf2 and pubf3);

        // publish vitals on a timed interval
        Particle.publishVitals(vitalsPublishInterval);

        initialPublishComplete = (publishVariablesSuccess and publishVariablesSuccess);
        Particle.publish("initialPublishComplete", String(initialPublishComplete), PRIVATE);
    }
}


/**********************************************************************************************************************
 * MQTT
 *********************************************************************************************************************/
// recieve message
void setupMqtt() {
    if (!mqttClient.isConnected()) {
        mqttConnected = false;
        mqttClient.connect("particle", mqttUser, mqttPassword);
    } 
    if (!mqttConnected) {
        Log.info("MQTT Connected!");
        bool pub1 = mqttClient.publish("homeassistant/cover/petdoor/availability", "online", true);
        bool pub2 = mqttClient.publish("homeassistant/cover/petdoor/state", doorStatesCString[currentDoorState], true);
        bool sub1 = mqttClient.subscribe("homeassistant/cover/petdoor/set");
        mqttConnected = (pub1 and pub2 and sub1);  // All returned sucess
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;
    String message(p);

    if (strcasecmp(topic, "homeassistant/cover/petdoor/set")==0) {
        Log.info("MQTT - Command Received: homeassistant/cover/petdoor/set", false);
        if (publish) {
            Particle.publish("mqtt command received. petdoor/set", message, PRIVATE);
        }
        setDesiredState(message);  // Use existing Particle Cloud Function to process command
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
    if ( (currentDoorState == desiredDoorState) or (runNonCriticalTasksNow) ) {
        // Ok we're not needing to move the stepper so can do some other things in the loop. 
        // Otherwise we want to keep any many cycles free for running the stepper
        nonCriticalTasks();
        if (systemResetRequested) {
            if (mqttConnected) {
                mqttClient.publish("homeassistant/cover/petdoor/availability", "offline", true);
            }
            delay(2000);  // Wait a couple of seconds to allow outstanding messages to be sent.
            System.reset();
        }
    }

    if ( (currentDoorState != STATE_MOVING) and (checkSensorNow) ){  // Only check if we're not already opening
        checkSensorChange();  // Check if sensors have detected something, if so log once.
    }

    if (checkStateNow) {
        checkStateChange();  // Check if curent or desired states have changed
    }

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