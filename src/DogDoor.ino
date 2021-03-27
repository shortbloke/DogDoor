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
 *                - Events published via MQTT for HomeAssistant Integration
 *                - Override control and system commands via Particle.io cloud
 *
 *              In order for stepper to run as quickly and smoothly as possible, time outside the loop needs to be
 *              minimised. This is achieved using interrupts to trigger the reading of values and storing the result
 *              in variables which are significantly quicker to access than reading the I/O:
 *               - Timer based polling infrared sensors
 *               - Limit switches read on state change
 *               - Manual control switches read on state change
 * 
 * Author: Copyright (C) 2019-2021 Martin Rowan
 */

SYSTEM_THREAD(ENABLED); // Have Particle processing in a separate thread - https://docs.particle.io/reference/device-os/firmware/photon/#system-thread
SYSTEM_MODE(SEMI_AUTOMATIC);

#include <AccelStepper.h>
#include <MQTT.h>

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

// Build
const char version[] = "1.7.0";
const char buildDate[] = __DATE__ " " __TIME__;

// MQTT
byte mqttServer[] = {192, 168, 200, 45};
const char *mqttUser = "username";
const char *mqttPassword = "password";
const char *mqttClientName = "particle";
const char *mqttAvailabilityTopic = "homeassistant/cover/petdoor/availability";
const char *mqttStateTopic = "homeassistant/cover/petdoor/state";
const char *mqttSetTopic = "homeassistant/cover/petdoor/set";
const bool enableMqtt = true;
long mqttLastReconnectAttempt = 0;
int mqttReconnectIntveral = 10000; // 10 Seconds

// Performance profiling
bool performanceProfiling = false;
uint32_t duration = 0; // For Main Loop
// For Stepper motor time to open door
uint32_t openDuration = 0;
uint32_t openStart = 0;
uint32_t openEnd = 0;

// Device Info
String deviceString = "";
String buildString = "";
int resetReasonCode = 0;
int resetReasonData = 0;
String reason = "";
const int wdTimeout = 30000; // 30 seconds watchdog timeout

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
const int vitalsPublishInterval = 300; // 5 minutes
bool systemResetRequested = false;

// Periodic Serial Logging
const int periodicLogInterval = 60000; //  1 minute

// Flags
volatile bool periodicLogNow = false;
volatile bool stateChanged = false;
volatile bool sensorsChanged = false;
volatile bool runNonCriticalTasksNow = true;       // Run soon after startup
const int runNonCriticalTasksMaxInterval = 300000; // 5 minutes

// Stepper (MicroStepping 1/32)
const float stepperSpeed = 50000;
const float stepperAccel = 50000;
const long initialClosedPosition = 72600;
long openPosition = 0;
long closedPosition = initialClosedPosition;
bool stepperPowerSave = false;
bool openDoorInitialLoop = true;
bool closeDoorInitialLoop = true;

// IR Sensors
volatile long indoorSensorValue = 0;
volatile long outdoorSensorValue = 0;
const long indoorIRSensorTriggerThreshold = 1200;
const long outdoorIRSensorTriggerThreshold = 1300;
const int irSensorPollInterval = 250; // 1/4 second
volatile bool indoorDetected = false;
volatile bool outdoorDetected = false;
bool lastIndoorDetected = false;
bool lastOutdoorDetected = false;

// Keep the door open for a period.
const int keepOpenTime = 2000; // 2 seconds
const int keepOpenTimerMaxResetsWaitingToOpen = 60;
int keepOpenTimerResetCountWaitingToOpen = 0;

// Keep in sync in ENUM for ease of logging
const char *doorStatesCString[] = {"OPEN", "MOVING", "CLOSED", "KEEPOPEN", "KEEPCLOSED"};
// Enum for states
enum doorStates
{
    STATE_OPEN = 0,
    STATE_MOVING = 1,
    STATE_CLOSED = 2,
    STATE_KEEPOPEN = 3,
    STATE_KEEPCLOSED = 4,
};

// Keep in sync in ENUM for ease of mqtt publishing
const char *mqttStatesCString[] = {"open", "opening", "closed", "closing"};
// Enum for states
enum mqttStates
{
    MQTT_OPEN = 0,
    MQTT_OPENING = 1,
    MQTT_CLOSED = 2,
    MQTT_CLOSING = 3
};

volatile enum doorStates desiredDoorState = STATE_CLOSED;
volatile enum doorStates currentDoorState;
enum doorStates lastDesiredDoorState;
enum doorStates lastCurrentDoorState;

// Cloud Function Variables
bool overrideDoorState = false;
enum doorStates overriddenDesiredDoorState;

enum mqttStates mqttState;

// Enum for LED status
enum ledStatus
{
    LED_OFF = 0,
    LED_KEEPCLOSED = 1,
    LED_KEEPOPEN = 2,
    LED_INDOOR = 3,
    LED_OUTDOOR = 4,
};

enum inputs
{
    INPUT_KEEPCLOSEDSWITCH = 0,
    INPUT_KEEPOPENSWITCH = 1,
    INPUT_MANUALPUSHSWITCH = 2,
    INPUT_TOPLIMITSWITCH = 3,
    INPUT_BOTTOMLIMITSWITCH = 4,
    INPUT_INDOORSENSOR = 5,
    INPUT_OUTDOORSENSOR = 6,
    INPUT_STEPPERENABLE = 7,
};

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
SerialLogHandler logHandler(LOG_LEVEL_INFO);
MQTT mqttClient(mqttServer, 1883, mqttCallback);

// ApplicationWatchdog wd(wdTimeout, System.reset);
Timer periodicLogTimer(periodicLogInterval, periodicLogTimerCallBack);
Timer nonCriticalTaskTimer(runNonCriticalTasksMaxInterval, nonCriticalTaskTimerCallback);
Timer pollIRSensorsTimer(irSensorPollInterval, pollIRSensorsTimerCallback);
Timer keepOpenTimer(keepOpenTime, keepOpenTimerCallback, true);

/**********************************************************************************************************************
 * Setup
 *********************************************************************************************************************/
void setup()
{
    // Setup Device information, doesn't change post boot
    deviceString = String::format("ID: %s - OS: %s",
                                  System.deviceID().c_str(),
                                  System.version().c_str());
    buildString = String::format("Version: %s - Build: %s", version, buildDate);
    resetReasonCode = (int)System.resetReason();
    resetReasonData = (int)System.resetReasonData();
    resetReason();

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
    stepper.setPinsInverted(/*direction*/ true, /*step*/ false, /*enable*/ true);
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
    if (currentDoorState == STATE_CLOSED)
    {
        // We're already closed at startup. Set open to be the inverse of the normal closed pos.
        openPosition = -initialClosedPosition;
    }

    if (publish)
    {
        setupParticleCloud();
    }
}

/**********************************************************************************************************************
 * MQTT
 *********************************************************************************************************************/
void setupParticleCloud()
{
    Log.info("Creating Particle variables");
    // Setup Particle cloud variables
    bool pubv1 = Particle.variable("TopLimitSwitch", (int *)&topLimitSwitchStatus, INT);
    bool pubv2 = Particle.variable("BottomLimitSwitch", (int *)&bottomLimitSwitchStatus, INT);
    bool pubv3 = Particle.variable("KeepClosedSwitch", (int *)&keepClosedSwitchStatus, INT);
    bool pubv4 = Particle.variable("KeepOpenSwitch", (int *)&keepOpenSwitchStatus, INT);
    bool pubv5 = Particle.variable("ManualSwitch", (int *)&manualButtonSwitchStatus, INT);
    bool pubv6 = Particle.variable("DesiredState", desiredDoorStateStatus);
    bool pubv7 = Particle.variable("CurrentState", currentDoorStateStatus);
    bool pubv8 = Particle.variable("IndoorSensor", (int *)&indoorSensorValue, INT);
    bool pubv9 = Particle.variable("OutDoorSensor", (int *)&outdoorSensorValue, INT);
    bool pubv10 = Particle.variable("overrideDoorState", overrideDoorState);
    bool pubv11 = Particle.variable("overriddenDesiredState", overriddenDesiredDoorStateStatus);
    bool pubv12 = Particle.variable("PerfProfiling", performanceProfiling);
    bool pubv13 = Particle.variable("LoopDuration", duration);
    bool pubv14 = Particle.variable("OpenDuration", openDuration);
    bool pubv15 = Particle.variable("ResetReason", reason);
    bool pubv16 = Particle.variable("openPosition", (int *)&openPosition, INT);
    bool pubv17 = Particle.variable("closedPosition", (int *)&closedPosition, INT);
    bool publishVariablesSuccess = (pubv1 and pubv2 and pubv3 and pubv4 and pubv5 and pubv6 and pubv7 and pubv8 and pubv9 and pubv10 and pubv11 and pubv12 and pubv13 and pubv14 and pubv15 and pubv16 and pubv17);

    // Setup Particle cloud functions
    bool pubf1 = Particle.function("setDesiredState", setDesiredState);
    bool pubf2 = Particle.function("remoteCommand", remoteCommand);
    bool pubf3 = Particle.function("PerfProfiling", profile);
    bool publishFunctionsSuccess = (pubf1 and pubf2 and pubf3);
    Log.info("Publishing Particle variables and functions: %s", ((publishVariablesSuccess and publishFunctionsSuccess) ? "TRUE" : "FALSE"));

    Log.info("Connecting to Particle Cloud");
    Particle.connect();
}

/**********************************************************************************************************************
 * Main Loop
 *********************************************************************************************************************/
void loop()
{
    uint32_t startTimeTicks = 0;
    if (performanceProfiling)
    {
        startTimeTicks = System.ticks();
    }
    if ((currentDoorState == desiredDoorState) or (runNonCriticalTasksNow))
    {
        // Ok we're not needing to move the stepper so can do some other things in the loop.
        // Otherwise we want to keep any many cycles free for running the stepper
        nonCriticalTasks(millis());
        if (systemResetRequested)
        {
            if (mqttClient.isConnected())
            {
                mqttClient.publish(mqttAvailabilityTopic, "offline", false);
            }
            Log.warn("System Reset Requested");
            Particle.publish("SystemResetRequested");

            delay(2000); // Wait a couple of seconds to allow outstanding messages to be sent.
            System.reset();
        }
    }

    if ((currentDoorState != STATE_MOVING) and (sensorsChanged))
    {                           // Only check if we're not already opening
        checkForSensorChange(); // Check if sensors have detected something, if so log once.
    }

    if (stateChanged)
    {
        checkForStateChange(); // Check if curent or desired states have changed
    }

    if ((desiredDoorState == STATE_OPEN) or (desiredDoorState == STATE_KEEPOPEN))
    {
        openDoor();
    }
    else if ((desiredDoorState == STATE_CLOSED) or (desiredDoorState == STATE_KEEPCLOSED))
    {
        closeDoor();
    }
    else
    {
        Log.warn("Unexpected State - Desired: %d, Current: %d", desiredDoorState, currentDoorState);
    }
    if (performanceProfiling)
    {
        uint32_t endTimeTicks = System.ticks();
        duration = (endTimeTicks - startTimeTicks) / System.ticksPerMicrosecond();
    }
}

/**********************************************************************************************************************
 * Status LED Controls
 *********************************************************************************************************************/
void setStatusLed1(ledStatus status)
{
    switch (status)
    {
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

void setStatusLed2(ledStatus status)
{
    switch (status)
    {
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
 * Read sensors.
 * Stores values to reduce number of calls to read inputs (slow)
 *********************************************************************************************************************/

bool indoorPresenceDetected()
{
    indoorSensorValue = analogRead(indoorIRSensorPin);
    if (indoorSensorValue >= indoorIRSensorTriggerThreshold)
    {
        // Read again to try and avoid false trigger
        indoorSensorValue = analogRead(indoorIRSensorPin);
        if (indoorSensorValue >= indoorIRSensorTriggerThreshold)
        {
            keepOpenTimer.changePeriodFromISR(keepOpenTime); // Reset the timer to prevent closing
            return true;
        }
    }
    return false;
}

bool outdoorPresenceDetected()
{
    outdoorSensorValue = analogRead(outdoorIRSensorPin);
    if (outdoorSensorValue >= outdoorIRSensorTriggerThreshold)
    {
        outdoorSensorValue = analogRead(outdoorIRSensorPin);
        if (outdoorSensorValue >= outdoorIRSensorTriggerThreshold)
        {
            keepOpenTimer.changePeriodFromISR(keepOpenTime); // Reset the timer to prevent closing
            return true;
        }
    }
    return false;
}

/**********************************************************************************************************************
 * Calculate Current and Desired States
 *********************************************************************************************************************/
doorStates getCurrentDoorState()
{
    if (topLimitSwitchStatus)
    {
        return STATE_OPEN;
    }
    else if (bottomLimitSwitchStatus)
    {
        return STATE_CLOSED;
    }
    else
    {
        return STATE_MOVING;
    }
}

doorStates getDesiredDoorState()
{
    if (overrideDoorState)
    {
        // Set both LEDS to red
        setStatusLed1(LED_KEEPCLOSED);
        setStatusLed2(LED_INDOOR);
        return overriddenDesiredDoorState;
    }
    else if (keepClosedSwitchStatus)
    {
        setStatusLed1(LED_KEEPCLOSED);
        return STATE_KEEPCLOSED;
    }
    else if (keepOpenSwitchStatus)
    {
        setStatusLed1(LED_KEEPOPEN);
        return STATE_KEEPOPEN;
    }
    else if (manualButtonSwitchStatus)
    {
        setStatusLed1(LED_OFF);
        return STATE_OPEN;
    }
    else
    {
        setStatusLed1(LED_OFF);
        if (indoorDetected)
        {
            setStatusLed2(LED_INDOOR);
            return STATE_OPEN;
        }
        else if (outdoorDetected)
        {
            setStatusLed2(LED_OUTDOOR);
            return STATE_OPEN;
        }
        else if (keepOpenTimer.isActive())
        {
            setStatusLed1(LED_KEEPOPEN);
            return STATE_OPEN;
        }
        else
        {
            setStatusLed2(LED_OFF);
            return STATE_CLOSED;
        }
    }
}

/**********************************************************************************************************************
 * GPIO - ISR
 *********************************************************************************************************************/
void limitSwitchISR()
{
    topLimitSwitchStatus = digitalRead(homeSwitchPin);      // Normally closed switch
    bottomLimitSwitchStatus = digitalRead(closedSwitchPin); // Normally closed switch
    currentDoorState = getCurrentDoorState();
    stateChanged = true; // Since ISR called, something has changed
}

void switchISR()
{
    keepClosedSwitchStatus = !digitalRead(keepClosedSwitchPin);
    keepOpenSwitchStatus = !digitalRead(keepOpenSwitchPin);
    manualButtonSwitchStatus = !digitalRead(manualSwitchPin);
    desiredDoorState = getDesiredDoorState();
    stateChanged = true; // Since ISR called, something has changed
}

/**********************************************************************************************************************
 * Timer Callback (ISR)
 *********************************************************************************************************************/
void keepOpenTimerCallback()
{
    Log.trace("keepOpenTimerCallback - Timer expired");
    if (overriddenDesiredDoorState == STATE_OPEN)
    { // Clear the override on timeout.
        overrideDoorState = false;
    }
    desiredDoorState = getDesiredDoorState();
    if ((currentDoorState != STATE_OPEN) and (desiredDoorState != STATE_KEEPCLOSED))
    {
        keepOpenTimer.changePeriodFromISR(keepOpenTime);
        keepOpenTimerResetCountWaitingToOpen++;
        if (keepOpenTimerResetCountWaitingToOpen > keepOpenTimerMaxResetsWaitingToOpen)
        {
            runNonCriticalTasksNow = true;
            systemResetRequested = true;
        }
    }
}

void periodicLogTimerCallBack()
{
    periodicLogNow = true;
}

void pollIRSensorsTimerCallback()
{
    bool initialIndoor = indoorDetected;
    bool initialOutdoor = outdoorDetected;
    indoorDetected = indoorPresenceDetected();
    outdoorDetected = outdoorPresenceDetected();
    if ((initialIndoor != indoorDetected) or (initialOutdoor != outdoorDetected))
    {
        sensorsChanged = true; // Trigger only on change
    }
    doorStates initialState = desiredDoorState;
    desiredDoorState = getDesiredDoorState();
    if (initialState != desiredDoorState)
    {
        stateChanged = true; // Trigger only on change
    }
}

void nonCriticalTaskTimerCallback()
{
    runNonCriticalTasksNow = true;
}

/**********************************************************************************************************************
 * Stepper Control
 *********************************************************************************************************************/
void stopStepper()
{
    // Set the stepper to think it's already reached the closed position.
    closedPosition = stepper.currentPosition();
    stepper.setCurrentPosition(closedPosition);
}

void openDoor()
{
    if (currentDoorState != STATE_OPEN)
    {
        if (openDoorInitialLoop)
        {
            if (performanceProfiling)
            {
                openStart = System.ticks();
            }
            keepOpenTimer.start();
            stepper.enableOutputs();
            stepper.moveTo(openPosition);
            openDoorInitialLoop = false;
        }
        else if (stepper.distanceToGo() == 0)
        {
            // Move completed, but we're not there yet. Move more.
            Log.warn("Opening - Homing");
            openPosition -= 50000;
            stepper.moveTo(openPosition);
        }
        stepper.run();
    }
    else
    {
        if ((performanceProfiling) and (openStart > 0))
        {
            openEnd = System.ticks();
            openDuration = (openEnd - openStart) / System.ticksPerMicrosecond();
            openStart = 0;
        }
        // Door Open
        openPosition = 0; // Set Zero position
        // openPosition = -10000;         // Remove decleration on open.
        stepper.setCurrentPosition(0); // Set or reset stepper home position
        openDoorInitialLoop = true;    // reset flag
    }
}

void closeDoor()
{
    if (currentDoorState != STATE_CLOSED)
    {
        if (closeDoorInitialLoop)
        {
            if (closedPosition < 100)
            { // Improbable value (i.e. powered on closed)
                closedPosition = initialClosedPosition;
            }
            stepper.enableOutputs();
            stepper.moveTo(closedPosition);
            stepperPowerSave = false;
            closeDoorInitialLoop = false;
        }
        else if (stepper.distanceToGo() == 0)
        {
            // Move completed, but we're not there yet. Move more.
            Log.warn("Closing - Homing");
            closedPosition = closedPosition + initialClosedPosition;
            stepper.moveTo(closedPosition);
            stepperPowerSave = false;
        }
        stepper.run();
    }
    else if (!stepperPowerSave)
    {
        // Door Closed
        stepper.disableOutputs(); // Disable Stepper to save power
        closedPosition = stepper.currentPosition();
        stepper.setCurrentPosition(closedPosition);
        stepperPowerSave = true;
        closeDoorInitialLoop = true;
    }
}

/**********************************************************************************************************************
 * Check for state changes
 *********************************************************************************************************************/
void checkForStateChange()
{
    stateChanged = false;
    // Check for desiredDoorState changes - Log once
    if (desiredDoorState != lastDesiredDoorState)
    {
        Log.info("DESIRED STATE CHANGED FROM: %s [%d] to %s [%d]",
                 doorStatesCString[lastDesiredDoorState], lastDesiredDoorState,
                 doorStatesCString[desiredDoorState], desiredDoorState);
        Particle.publish("DesiredDoorState", doorStatesCString[desiredDoorState]);
        Particle.publish("LastDesiredDoorState", doorStatesCString[lastDesiredDoorState]);
        desiredDoorStateStatus = String::format("%s", doorStatesCString[desiredDoorState]);
        if ((lastDesiredDoorState == STATE_CLOSED) and (desiredDoorState != STATE_KEEPCLOSED))
        {
            // We were closing, but now need to open. Ideally as quickly as possible.
            stopStepper();
        }
        lastDesiredDoorState = desiredDoorState;
    }

    // check for currentDoorState changes - Log once
    if (currentDoorState != lastCurrentDoorState)
    {
        Log.info("CURRENT STATE CHANGED FROM: %s [%d] to %s [%d]",
                 doorStatesCString[lastCurrentDoorState], lastCurrentDoorState,
                 doorStatesCString[currentDoorState], currentDoorState);
        Particle.publish("currentDoorState", doorStatesCString[currentDoorState]);
        Particle.publish("LastCurrentDoorState", doorStatesCString[lastCurrentDoorState]);
        if ((enableMqtt) and (mqttClient.isConnected()))
        {
            mqttClient.publish(mqttStateTopic, mqttStatesCString[doorStateToMqttState()]);
            Log.info("MQTT Publish currentDoorState change");
            if (publish)
            {
                Particle.publish("mqtt publish . petdoor/state", String(mqttStatesCString[doorStateToMqttState()]), PRIVATE);
            }
        }
        currentDoorStateStatus = String::format("%s", doorStatesCString[currentDoorState]);
        lastCurrentDoorState = currentDoorState;
    }
}

/**********************************************************************************************************************
 * Log Functions
 *********************************************************************************************************************/
void periodicLog()
{
    periodicLogNow = false; // Reset, will be enabled by Timer ISR
    Log.info("Device: %s", deviceString.c_str());
    Log.info("App Version: %s", buildString.c_str());
    Log.info("Free Memory (B): %lu", System.freeMemory());
    Log.info("Uptime (mins): %d", (int)(System.uptime() / 60));
    Log.info("System restart reason: %d - %d - %s", resetReasonCode, resetReasonData, reason.c_str());
    Log.info("Current State: %s [%d] - Desired State: %s [%d]",
             doorStatesCString[currentDoorState], currentDoorState,
             doorStatesCString[desiredDoorState], desiredDoorState);
    Log.info("Indoor: %lu - Outdoor: %lu - Button: %d",
             indoorSensorValue, outdoorSensorValue, manualButtonSwitchStatus);
    Log.info("KeepOpen: %d - KeepClosed: %d - Top: %d - Bottom: %d",
             keepOpenSwitchStatus, keepClosedSwitchStatus, topLimitSwitchStatus, bottomLimitSwitchStatus);
    if (publish)
    {
        Particle.publishVitals(vitalsPublishInterval);
    }
}

void resetReason(void)
{
    switch (resetReasonCode)
    {
    case RESET_REASON_NONE: // 0
        reason = "NONE";
        break;
    case RESET_REASON_UNKNOWN: // 10
        reason = "UNKNOWN";
        break;
    case RESET_REASON_PIN_RESET: // 20
        reason = "PIN_RESET";
        break;
    case RESET_REASON_POWER_MANAGEMENT: // 30
        reason = "POWER_MANAGEMENT";
        break;
    case RESET_REASON_POWER_DOWN: // 40
        reason = "POWER_DOWN";
        break;
    case RESET_REASON_POWER_BROWNOUT: // 50
        reason = "POWER_BROWNOUT";
        break;
    case RESET_REASON_WATCHDOG: // 60
        reason = "WATCHDOG";
        break;
    case RESET_REASON_UPDATE: // 70
        reason = "UPDATE";
        break;
    case RESET_REASON_UPDATE_TIMEOUT: // 90
        reason = "UPDATE_TIMEOUT";
        break;
    case RESET_REASON_FACTORY_RESET: // 100
        reason = "FACTORY_RESET";
        break;
    case RESET_REASON_SAFE_MODE: // 110
        reason = "SAFE_MODE";
        break;
    case RESET_REASON_DFU_MODE: // 120
        reason = "DFU_MODE";
        break;
    case RESET_REASON_PANIC: // 130
        reason = "PANIC";
        break;
    case RESET_REASON_USER: // 140
        reason = "USER";
        break;
    }
}

void checkForSensorChange()
{
    sensorsChanged = false;
    // Check indoorDetected change - Log once
    if (indoorDetected != lastIndoorDetected)
    {
        if (indoorDetected)
        {
            Log.info("MOTION DETECTED INDOOR: %lu", indoorSensorValue);
            if (publish)
            {
                Particle.publish("INDOOR-MOTION-DETECTED",
                                 String::format("%lu", indoorSensorValue),
                                 PRIVATE);
            }
        }
        lastIndoorDetected = indoorDetected;
    }
    // Check outdoorDetected change - Log once
    if (outdoorDetected != lastOutdoorDetected)
    {
        if (outdoorDetected)
        {
            Log.info("MOTION DETECTED OUTDOOR: %lu", outdoorSensorValue);
            if (publish)
            {
                Particle.publish("OUTDOOR-MOTION-DETECTED",
                                 String::format("%lu", outdoorSensorValue),
                                 PRIVATE);
            }
        }
        lastOutdoorDetected = outdoorDetected;
    }
}

void nonCriticalTasks(long now)
{
    runNonCriticalTasksNow = false;
    nonCriticalTaskTimer.changePeriod(runNonCriticalTasksMaxInterval);
    if (enableMqtt)
    {
        if (!mqttClient.isConnected())
        {
            if (now - mqttLastReconnectAttempt > mqttReconnectIntveral)
            {
                mqttLastReconnectAttempt = now;
                if (reconnectMqtt())
                {
                    mqttLastReconnectAttempt = 0;
                }
            }
        }
        else
        {
            mqttClient.loop();
        }
    }
    if (periodicLogNow)
    {
        periodicLog(); // Log to serial port
    }
}

/**********************************************************************************************************************
 * Particle Cloud Setup and Publishing for data
 *********************************************************************************************************************/
int setDesiredState(String requestedState)
{
    Log.info("setDesiredState: %s", requestedState.c_str());
    if (strcasecmp(requestedState.c_str(), "open") == 0)
    {
        overriddenDesiredDoorState = STATE_OPEN;
        overrideDoorState = true;
    }
    else if (strcasecmp(requestedState.c_str(), "keepopen") == 0)
    {
        overriddenDesiredDoorState = STATE_KEEPOPEN;
        overrideDoorState = true;
    }
    else if (strcasecmp(requestedState.c_str(), "keepclosed") == 0)
    {
        overriddenDesiredDoorState = STATE_KEEPCLOSED;
        overrideDoorState = true;
    }
    else if (strcasecmp(requestedState.c_str(), "reset") == 0)
    {
        overrideDoorState = false;
    }
    else
    {
        Log.warn("setDesiredState received invalid requestedState value: %s", requestedState.c_str());
        Particle.publish("setDesiredState-error", requestedState, PRIVATE);
        overriddenDesiredDoorStateStatus = String::format("%s", doorStatesCString[overriddenDesiredDoorState]);
        return 0;
    }
    Particle.publish("setDesiredState-success", requestedState, PRIVATE);
    return 1;
}

int remoteCommand(String command)
{
    Log.info("remoteCommand: %s", command.c_str());
    if (strcasecmp(command.c_str(), "reset") == 0)
    {
        Particle.publish("remoteCommand", "reset", PRIVATE);
        systemResetRequested = true;
        return 1;
    }
    else if (strcasecmp(command.c_str(), "log") == 0)
    {
        periodicLog();
        return 1;
    }
    return 0;
}

int profile(String command)
{
    if (strcasecmp(command.c_str(), "true") == 0)
    {
        performanceProfiling = true;
        return 1;
    }
    else if (strcasecmp(command.c_str(), "false") == 0)
    {
        performanceProfiling = false;
        return 1;
    }
    return 0;
}

/**********************************************************************************************************************
 * MQTT
 *********************************************************************************************************************/
bool reconnectMqtt()
{
    Log.info("Attempting MQTT connection...");
    if (mqttClient.connect(mqttClientName, mqttUser, mqttPassword))
    {
        Log.info("MQTT Connected!");
        Particle.publish("MQTT Connected!");
        mqttClient.publish(mqttAvailabilityTopic, "online", false);
        mqttClient.publish(mqttStateTopic, mqttStatesCString[doorStateToMqttState()], false);
        mqttClient.subscribe(mqttSetTopic);
    }
    else
    {
        Log.error("MQTT connected failed!");
    }
    return mqttClient.isConnected();
}

int doorStateToMqttState()
{
    if (currentDoorState == STATE_MOVING)
    {
        if (desiredDoorState == STATE_OPEN)
        {
            return MQTT_OPENING;
        }
        else
        {
            return MQTT_CLOSING;
        }
    }
    else if ((currentDoorState == STATE_OPEN) or (currentDoorState == STATE_KEEPOPEN))
    {
        return MQTT_OPEN;
    }
    else if ((currentDoorState == STATE_CLOSED) or (currentDoorState == STATE_KEEPCLOSED))
    {
        return MQTT_CLOSED;
    }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;
    String message(p);

    if (strcasecmp(topic, mqttSetTopic) == 0)
    {
        Log.info("MQTT - Command Received: homeassistant/cover/petdoor/set", false);
        if (publish)
        {
            Particle.publish("mqtt command received. petdoor/set", message, PRIVATE);
        }
        // DISABLED, as set command is received on broker restart and initial boot of photon.
        // setDesiredState(message);  // Use existing Particle Cloud Function to process command
    }
}