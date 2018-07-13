/*
* Project Cellular-MMA8452Q - converged software for Low Power and Solar
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Author: Chip McClelland
* Date:10 January 2018
*/

/*  The idea of this release is to unify the code base between Accelerometer sensors
    Both implementations will move over to the finite state machine approach
    Both implementations will observe the park open and closing hours
    I will add two new states: 1) Low Power mode - maintains functionality but conserves battery by
    enabling sleep  2) Low Battery Mode - reduced functionality to preserve battery charge

    The mode will be set and recoded in the CONTROLREGISTER so resets will not change the mode
    Control Register - bits 7-5, 4-Connected Status, 3 - Verbose Mode, 2- Solar Power Mode, 1 - Low Battery Mode, 0 - Low Power Mode
*/

// The SparkFun MMA8452 breakout board defaults to 1, set to 0 if SA0 jumper on the bottom of the board is set
#define SA0 1
#if SA0
#define MMA8452_ADDRESS 0x1D  // SA0 is high, 0x1C if low
#else
#define MMA8452_ADDRESS 0x1C
#endif

// Easy place to change global numbers
//These defines let me change the memory map and configuration without hunting through the whole program
#define VERSIONNUMBER 9             // Increment this number each time the memory map is changed
#define WORDSIZE 8                  // For the Word size the number of bytes in a "word"
#define PAGESIZE 4096               // Memory size in bytes / word size - 256kb FRAM
#define HOURLYOFFSET 24             // First word of hourly counts (remember we start counts at 1)
#define HOURLYCOUNTNUMBER 4064      // used in modulo calculations - sets the # of hours stored - 256k (4096-14-2)
// First Word - 8 bytes for setting global values
#define VERSIONADDR 0x0             // Where we store the memory map version number
#define SENSITIVITYADDR 0x1         // Sensitivity for Accelerometer sensors
#define DEBOUNCEADDR 0x2            // Where we store debounce in cSec or 1/10s of a sec (ie 1.6sec is stored as 16)
#define RESETCOUNT 0x3              // This is where we keep track of how often the Electron was reset
#define TIMEZONE  0x4               // Store the local time zone data                                    // One byte is open here
#define OPENTIMEADDR 0x5            // Hour for opening the park / store / etc - military time (e.g. 6 is 6am)
#define CLOSETIMEADDR 0x6           // Hour for closing of the park / store / etc - military time (e.g 23 is 11pm)
#define CONTROLREGISTER 0x7         // This is the control register for storing the current state
//Second and Third words bytes for storing current counts
#define CURRENTHOURLYCOUNT 0x8      // Current Hourly Count - 16 bits
#define CURRENTDAILYCOUNT 0xC       // Current Daily Count - 16 bits
#define CURRENTCOUNTSTIME 0xE       // Time of last count - 32 bits
#define HOURLYPOINTERADDR 0x11      // Two bytes for hourly pointer
                                    // Four open bytes here which takes us to the third word
//These are the hourly and daily offsets that make up the respective words
#define HOURLYCOUNTOFFSET 4         // Offsets for the values in the hourly words
#define HOURLYBATTOFFSET 6          // Where the hourly battery charge is stored
// Finally, here are the variables I want to change often and pull them all together here
#define SOFTWARERELEASENUMBER "0.59"

// Included Libraries
#include "Adafruit_FRAM_I2C.h"                           // Library for FRAM functions
#include "FRAM-Library-Extensions.h"                     // Extends the FRAM Library
#include "electrondoc.h"                                 // Documents pinout
#include "MMA8452-Functions.h"                           // Adds the accelerometer functions


// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                             // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                                  // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
FuelGauge batteryMonitor;                                // Prototype for the fuel gauge (included in Particle core library)
PMIC power;                                              //Initalize the PMIC class so you can call the Power Management functions below.

// State Maching Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, LOW_BATTERY_STATE, REPORTING_STATE, RESP_WAIT_STATE };
char stateNames[8][14] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Low Battery", "Reporting", "Response Wait" };
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;


// Pin Constants
const int tmp36Pin =      A0;                       // Simple Analog temperature sensor
const int wakeUpPin =     A7;                       // This is the Particle Electron WKP pin
const int tmp36Shutdwn =  B5;                       // Can turn off the TMP-36 to save energy
const int int2Pin =       D2;                       // Accelerometer inerrupt pin
const int hardResetPin =  D4;                       // Power Cycles the Electron and the Carrier Board
const int userSwitch =    D5;                       // User switch with a pull-up resistor
const int donePin =       D6;                       // Pin the Electron uses to "pet" the watchdog
const int blueLED =       D7;                       // This LED is on the Electron itself


// Timing Variables
const int wakeBoundary = 1*3600 + 0*60 + 0;         // 1 hour 0 minutes 0 seconds
const unsigned long stayAwakeLong = 90000;          // In lowPowerMode, how long to stay awake every hour
const unsigned long webhookWait = 45000;            // How long will we wair for a WebHook response
const unsigned long resetWait = 30000;              // How long will we wait in ERROR_STATE until reset
const int publishFrequency = 1000;                  // We can only publish once a second
unsigned long stayAwakeTimeStamp = 0;               // Timestamps for our timing variables..
unsigned long stayAwake;                            // Stores the time we need to wait before napping
unsigned long webhookTimeStamp = 0;                 // Webhooks...
unsigned long resetTimeStamp = 0;                   // Resets - this keeps you from falling into a reset loop
unsigned long publishTimeStamp = 0;                 // Keep track of when we publish a webhook
unsigned long lastPublish = 0;                      // Can only publish 1/sec on avg and 4/sec burst

// Program Variables
int temperatureF;                                   // Global variable so we can monitor via cloud variable
int resetCount;                                     // Counts the number of times the Electron has had a pin reset
volatile bool ledState = LOW;                       // variable used to store the last LED status, to toggle the light
bool awokeFromNap = false;                          // In low power mode, we can't use standard millis to debounce
bool readyForBed = false;                           // Checks to see if steps for sleep have been completed
bool pettingEnabled = true;                         // Let's us pet the hardware watchdog
bool dataInFlight = false;                          // Tracks if we have sent data but not yet cleared it from counts until we get confirmation
const char* releaseNumber = SOFTWARERELEASENUMBER;  // Displays the release on the menu
byte controlRegister;                               // Stores the control register values
bool lowPowerMode;                                  // Flag for Low Power Mode operations
bool connectionMode;                                // Need to store if we are going to connect or not in the register
bool solarPowerMode;                                // Changes the PMIC settings
bool verboseMode;                                   // Enables more active communications for configutation and setup
char SignalString[17];                              // Used to communicate Wireless RSSI and Description
const char* levels[6] = {"Poor", "Low", "Medium", "Good", "Very Good", "Great"};

// Time Related Variables
int openTime;                                       // Park Opening time - (24 hr format) sets waking
int closeTime;                                      // Park Closing time - (24 hr format) sets sleep
byte lastHour = 0;                                  // For recording the startup values
byte lastDate = 0;                                  // These values make sure we record events if time has lapsed
byte currentHourlyPeriod;                           // This is where we will know if the period changed
byte currentDailyPeriod;                            // We will keep daily counts as well as period counts

// Battery monitoring
int stateOfCharge = 0;                              // stores battery charge level value
int lowBattLimit;                                   // Trigger for Low Batt State

// This section is where we will initialize sensor specific variables, libraries and function prototypes
// Accelerometer Variables
const byte accelFullScaleRange = 2;                 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
const byte dataRate = 3;                            // output data rate - 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
byte Sensitivity;                                   // Hex variable for sensitivity - Initialized in Setup (0 - most to 128 - least sensitive)
int debounce;                                       // This is the numerical value of debounce - in millis()
char debounceStr[8] = "NA";                         // String to make debounce more readalbe on the mobile app
int inputSensitivity;                               // Raw sensitivity input - Initialized in Setup (0 least sensitive -9 most sensitive)
volatile bool sensorDetect = false;                 // This is the flag that an interrupt is triggered
volatile time_t currentEvent = 0;                   // Keeps track of the last time there was an event
volatile byte source;                               // Why was the accelerometer interrupt triggered
int hourlyPersonCount = 0;                          // hourly counter
int hourlyPersonCountSent = 0;                      // Person count in flight to Ubidots
int dailyPersonCount = 0;                           // daily counter

void setup()                                        // Note: Disconnected Setup()
{
  Wire.begin();                                     //Create a Wire object

  pinMode(int2Pin,INPUT);                           // accelerometer interrupt pinMode
  pinMode(wakeUpPin,INPUT);                         // This pin is active HIGH
  pinMode(userSwitch,INPUT);                        // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                         // declare the Blue LED Pin as an output
  pinMode(tmp36Shutdwn,OUTPUT);                     // Supports shutting down the TMP-36 to save juice
  digitalWrite(tmp36Shutdwn, HIGH);                 // Turns on the temp sensor
  pinMode(donePin,OUTPUT);                          // Allows us to pet the watchdog
  pinMode(hardResetPin,OUTPUT);                     // For a hard reset active HIGH

  watchdogISR();                                    // Pet the watchdog

  char responseTopic[125];
  String deviceID = System.deviceID();              // Multiple Electrons share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);          // Puts the deviceID into the response topic array
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event

  Particle.variable("HourlyCount", hourlyPersonCount);                // Define my Particle variables
  Particle.variable("DailyCount", dailyPersonCount);                  // Note: Don't have to be connected for any of this!!!
  Particle.variable("Sensitivity", inputSensitivity);
  Particle.variable("Signal", SignalString);
  Particle.variable("ResetCount", resetCount);
  Particle.variable("Temperature",temperatureF);
  Particle.variable("Release",releaseNumber);
  Particle.variable("stateOfChg", stateOfCharge);
  Particle.variable("lowPowerMode",lowPowerMode);
  Particle.variable("OpenTime",openTime);
  Particle.variable("CloseTime",closeTime);
  Particle.variable("Debounce",debounceStr);


  Particle.function("resetFRAM", resetFRAM);                          // These are the functions exposed to the mobile app and console
  Particle.function("resetCounts",resetCounts);
  Particle.function("HardReset",hardResetNow);
  Particle.function("SetSensivty", setSensivty);
  Particle.function("SendNow",sendNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Solar-Mode",setSolarMode);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("Set-Timezone",setTimeZone);
  Particle.function("Set-OpenTime",setOpenTime);
  Particle.function("Set-Close",setCloseTime);
  Particle.function("Set-Debounce",setDebounce);


  if (!fram.begin()) {                                                // You can stick the new i2c addr in here, e.g. begin(0x51);
    resetTimeStamp = millis();                                        // Can't communicate with FRAM - fatal error
    state = ERROR_STATE;
  }
  else if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) {                 // Check to see if the memory map in the sketch matches the data on the chip
    ResetFRAM();                                                      // Reset the FRAM to correct the issue
    if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) {
      resetTimeStamp = millis();
      state = ERROR_STATE;                                            // Resetting did not fix the issue
    }
    else {
      FRAMwrite8(CONTROLREGISTER,0);                                  // Need to reset so not in low power or low battery mode
      FRAMwrite8(OPENTIMEADDR,0);                                     // These set the defaults if the FRAM is erased
      FRAMwrite8(CLOSETIMEADDR,24);                                   // This will ensure the device does not sleep
      FRAMwrite8(DEBOUNCEADDR,5);                                     // Sets a debounce of 500mSec
    }
  }

  resetCount = FRAMread8(RESETCOUNT);                                   // Retrive system recount data from FRAM
  if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER)  // Check to see if we are starting from a pin reset or a reset in the sketch
  {
    resetCount++;
    FRAMwrite8(RESETCOUNT,static_cast<uint8_t>(resetCount));          // If so, store incremented number - watchdog must have done This
  }
  if (resetCount >=6) {                                               // If we get to resetCount 4, we are resetting without entering the main loop
    FRAMwrite8(RESETCOUNT,4);                                         // The hope here is to get to the main loop and report a value of 4 which will indicate this issue is occuring
    fullModemReset();                                                 // This will reset the modem and the device will reboot
  }

  // Check and import values from FRAM
  inputSensitivity = 10-FRAMread8(SENSITIVITYADDR);                   // Load value from FRAM but will reassign if they are not valid
  if (inputSensitivity < 0 || inputSensitivity >=10) inputSensitivity = 8;  // We store the inverse as humans think of 10 as more sensitive than 0
  debounce = 100*FRAMread8(DEBOUNCEADDR);
  if (debounce <= 100 || debounce > 2000) debounce = 500;             // We store debounce in cSec so mult by 100 for millis
  snprintf(debounceStr,sizeof(debounceStr),"%2.1f sec",(debounce/1000.0));
  openTime = FRAMread8(OPENTIMEADDR);
  if (openTime < 0 || openTime > 22) openTime = 0;                    // Open and close in 24hr format
  closeTime = FRAMread8(CLOSETIMEADDR);
  if (closeTime < 1 && closeTime > 23) closeTime = 24;
  int8_t tempFRAMvalue = FRAMread8(TIMEZONE);
  if (tempFRAMvalue <= 12 && tempFRAMvalue >= -12)  Time.zone((float)tempFRAMvalue);  // Load Timezone from FRAM
  else Time.zone(0);                                                  // Default is GMT in case proper value not in FRAM

  controlRegister = FRAMread8(CONTROLREGISTER);                       // Read the Control Register for system modes
  lowPowerMode    = (0b00000001 & controlRegister);                   // Bitwise AND to set the lowPowerMode flag from control Register
  verboseMode     = (0b00001000 & controlRegister);                   // verboseMode
  solarPowerMode  = (0b00000100 & controlRegister);                   // solarPowerMode
  connectionMode  = (0b00010000 & controlRegister);                   // connected mode 1 = connected and 0 = disconnected

  PMICreset();                                                        // Executes commands that set up the PMIC for Solar charging

  if (connectionMode) connectToParticle();                            // If not lowpower or sleeping, we can connect

  takeMeasurements();                                                 // Populates values so you can read them before the hour

  currentHourlyPeriod = Time.hour();                                  // Sets the hour period for when the count starts (see #defines)
  currentDailyPeriod = Time.day();                                    // And the day  (see #defines)
  // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
  time_t unixTime = FRAMread32(CURRENTCOUNTSTIME);                    // Need to reload last recorded event
  lastHour = Time.hour(unixTime);
  lastDate = Time.day(unixTime);
  dailyPersonCount = FRAMread16(CURRENTDAILYCOUNT);                   // Load Daily Count from memory
  hourlyPersonCount = FRAMread16(CURRENTHOURLYCOUNT);                 // Load Hourly Count from memory

  if (!digitalRead(userSwitch)) {                                     // Rescue mode to locally take lowPowerMode so you can connect to device
    lowPowerMode = false;                                             // Press the user switch while resetting the device
    controlRegister = (0b11111110 & controlRegister);                  // Turn off Low power mode
    controlRegister = (0b00010000 | controlRegister);                 // Turn on the connectionMode
    FRAMwrite8(CONTROLREGISTER,controlRegister);                      // Write it to the register
    openTime = 0;                                                     // Device may alos be sleeping due to time or TimeZone setting
    FRAMwrite8(OPENTIMEADDR,0);                                       // Reset open and close time values to ensure device is awake
    closeTime = 24;
    FRAMwrite8(CLOSETIMEADDR,24);
    connectToParticle();                                              // Connects the Electron to Particle so you can control it
    Particle.publish("Startup","Startup rescue - reset time and power");
  }

  byte c = readRegister(MMA8452_ADDRESS,0x0D);                        // Read WHO_AM_I register for accelerometer
  if (c == 0x2A)                                                      // WHO_AM_I should always be 0x2A
  {
    initMMA8452(accelFullScaleRange, dataRate);                       // init the accelerometer if communication is OK
  }
  else
  {
    resetTimeStamp = millis();                                        // Can't communicate with FRAM - fatal error
    state = ERROR_STATE;
  }
  initMMA8452(accelFullScaleRange,dataRate);                          // Initialize the accelerometer using values from FRAM

  attachInterrupt(int2Pin,sensorISR,RISING);                          // Accelerometer interrupt from low to high
  attachInterrupt(wakeUpPin, watchdogISR, RISING);                    // The watchdog timer will signal us and we have to respond

  if (state != ERROR_STATE) state = IDLE_STATE;                       // IDLE unless error from above code

  stayAwake = stayAwakeLong;                                          // Keeps Electron awake after reboot - helps with recovery
}

void loop()
{
  switch(state) {
  case IDLE_STATE:                                                    // Where we spend most time - note, the order of these conditionals is important
    if (connectionMode && verboseMode && state != oldState) publishStateTransition();
    if(hourlyPersonCountSent) {                                       // Cleared here as there could be counts coming in while "in Flight"
      hourlyPersonCount -= hourlyPersonCountSent;                     // Confirmed that count was recevied - clearing
      FRAMwrite16(CURRENTHOURLYCOUNT, static_cast<uint16_t>(hourlyPersonCount));  // Load Hourly Count to memory
      hourlyPersonCountSent = 0;                                      // Zero out the count until next reporting period
    }
    if (sensorDetect) recordCount();                                  // The ISR had raised the sensor flag
    if (lowPowerMode && (millis() > stayAwakeTimeStamp + stayAwake)) state = NAPPING_STATE;  // When in low power mode, we can nap between taps
    if (Time.hour() != currentHourlyPeriod) state = REPORTING_STATE;  // We want to report on the hour but not after bedtime
    if ((Time.hour() >= closeTime || Time.hour() < openTime)) state = SLEEPING_STATE;   // The park is closed - sleep
    if (stateOfCharge <= lowBattLimit) state = LOW_BATTERY_STATE;     // The battery is low - sleep
    break;

  case SLEEPING_STATE: {                                              // This state is triggered once the park closes and runs until it opens
    if (connectionMode && verboseMode && state != oldState) publishStateTransition();
    if (hourlyPersonCount) {                                          // If this number is not zero then we need to send this last count
      state = REPORTING_STATE;
      break;
    }
    if (connectionMode) disconnectFromParticle();                     // If connected, we need to disconned and power down the modem
    detachInterrupt(int2Pin);                                         // Done sensing for the day
    FRAMwrite16(CURRENTDAILYCOUNT, 0);                                // Reset the counts in FRAM as well
    FRAMwrite8(RESETCOUNT,0);
    FRAMwrite16(CURRENTHOURLYCOUNT, 0);
    digitalWrite(blueLED,LOW);                                        // Turn off the LED
    digitalWrite(tmp36Shutdwn, LOW);                                  // Turns off the temp sensor
    watchdogISR();                                                    // Pet the watchdog
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    System.sleep(SLEEP_MODE_DEEP,wakeInSeconds);                      // Very deep sleep till the next hour - then resets
    } break;

  case NAPPING_STATE: {
      if (connectionMode && verboseMode && state != oldState) publishStateTransition();
      stayAwake = debounce;                                           // Ensures that we stay awake long enough to debounce a tap
      if (connectionMode) disconnectFromParticle();                   // If connected, we need to disconned and power down the modem
      watchdogISR();                                                  // Pet the watchdog
      detachInterrupt(int2Pin);                                       // Detach since sleep will monitor the int2Pin
      int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
      System.sleep(int2Pin, RISING, wakeInSeconds);                   // Wake on either int2Pin or the top of the hour
      if (digitalRead(int2Pin)) {                                     // Need to test if Tap or Time woke us up
        awokeFromNap = true;                                          // This flag will allow us to bypass the debounce in the recordCount function
        recordCount();                                                // Count the tap that awoke the device
        stayAwakeTimeStamp = millis();                                // Allows us to ensure we stay awake long enough to debounce
      }
      attachInterrupt(int2Pin,sensorISR,RISING);                      // Reattach Accelerometer interrupt from low to high
      state = IDLE_STATE;                                             // Back to the IDLE_STATE after a nap will come back after the stayAwake time is over
  } break;

  case LOW_BATTERY_STATE: {                                           // Sleep state but leaves the fuel gauge on
    if (connectionMode && verboseMode && state != oldState) publishStateTransition();
    if (connectionMode) disconnectFromParticle();                     // If connected, we need to disconned and power down the modem
    detachInterrupt(int2Pin);                                         // Done sensing for the day
    digitalWrite(tmp36Shutdwn, LOW);                                  // Turns off the temp sensor
    watchdogISR();                                                    // Pet the watchdog
    int secondsToHour = (60*(60 - Time.minute()));                    // Time till the top of the hour
    System.sleep(SLEEP_MODE_DEEP,secondsToHour);                      // Very deep sleep till the next hour - then resets
  } break;

  case REPORTING_STATE:
    watchdogISR();                                                    // Pet the watchdog once an hour
    stayAwake = stayAwakeLong;                                        // Keeps the Electron awake for longer once reporting - helps with updates and system Checks
    pettingEnabled = false;                                           // Going to see the reporting process through before petting again
    if (!connectionMode) connectToParticle();                         // Need to connect in oder to report
    if (verboseMode && state != oldState) publishStateTransition();
    takeMeasurements();                                               // Update Temp, Battery and Signal Strength values
    sendEvent();                                                      // Send data to Ubidots
    state = RESP_WAIT_STATE;                                          // Wait for Response
    break;

  case RESP_WAIT_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
    if (!dataInFlight)                                                // Response received back to IDLE state
    {
      state = IDLE_STATE;
      pettingEnabled = true;                                          // Enable petting before going back into the loop
      stayAwakeTimeStamp = millis();
    }
    else if (millis() > webhookTimeStamp + webhookWait) {             // If it takes too long - will need to reset
      resetTimeStamp = millis();
      state = ERROR_STATE;                                            // Response timed out
    }
    break;

  case ERROR_STATE:                                                   // To be enhanced - where we deal with errors
    if (verboseMode && state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait)
    {
      Particle.publish("State","ERROR_STATE - Resetting");            // Reset time expired - time to go
      delay(2000);
      if (resetCount <= 3)  System.reset();                           // Today, only way out is reset
      else {                                                          // If we have had 3 resets - time to do something more
        FRAMwrite8(RESETCOUNT,0);                                     // Zero the ResetCount
        fullModemReset();                                             // Full Modem reset and reboots
      }
    }
    break;
  }
}

void recordCount() // This is where we check to see if an interrupt is set when not asleep or act on a tap that woke the Arduino
{
  char data[256];                                                     // Store the date in this character array - not global
  pinSetFast(blueLED);                                                // update the LED pin itself
  sensorDetect = false;                                               // Reset the flag
  source = readRegister(MMA8452_ADDRESS,0x0C);                        // Read the interrupt source reg.
  readRegister(MMA8452_ADDRESS,0x22);                                 // Reads the PULSE_SRC register to reset it

  if ((millis() >= currentEvent + debounce) || awokeFromNap) {        // If this event is outside the debounce time, proceed
    currentEvent = millis();
    awokeFromNap = false;                                             // Reset the awoke flag
  }
  else {
    pinResetFast(blueLED);                                            // If it is not, turn off the LED and return
    return;
  }

  if ((source & 0x08)==0x08)                                          // We are only interested in the TAP register and ignore debounced taps
  {
    hourlyPersonCount++;                                              // Increment the PersonCount
    FRAMwrite16(CURRENTHOURLYCOUNT, hourlyPersonCount);               // Load Hourly Count to memory
    dailyPersonCount++;                                               // Increment the PersonCount
    FRAMwrite16(CURRENTDAILYCOUNT, dailyPersonCount);                 // Load Daily Count to memory
    FRAMwrite32(CURRENTCOUNTSTIME, currentEvent);                     // Write to FRAM - this is so we know when the last counts were saved
    snprintf(data, sizeof(data), "Car, hourlry count: %i, daily count: %i",hourlyPersonCount,dailyPersonCount);
    if (verboseMode) Particle.publish("Count",data);                  // Helpful for monitoring and calibration
  }
  pinResetFast(blueLED);                                              // Turn off the LED
  readRegister(MMA8452_ADDRESS,0x22);                                 // Reads the PULSE_SRC register to reset it - just in case

  if (!digitalRead(userSwitch) && lowPowerMode) {                     // A low value means someone is pushing this button - will trigger a send to Ubidots and take out of low power mode
    Particle.publish("Mode","Normal Operations");
    controlRegister = (0b1111110 & controlRegister);                  // Will set the lowPowerMode bit to zero
    FRAMwrite8(CONTROLREGISTER,controlRegister);
    lowPowerMode = false;
    connectToParticle();                                              // Reconnect to Particle for monitoring and management
  }
}


void sendEvent()
{
  char data[256];                                                     // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"hourly\":%i, \"daily\":%i,\"battery\":%i, \"temp\":%i, \"resets\":%i}",hourlyPersonCount, dailyPersonCount, stateOfCharge, temperatureF,resetCount);
  Particle.publish("Ubidots-Hook", data, PRIVATE);
  webhookTimeStamp = millis();
  hourlyPersonCountSent = hourlyPersonCount;                          // This is the number that was sent to Ubidots - will be subtracted once we get confirmation
  currentHourlyPeriod = Time.hour();                                  // Change the time period
  dataInFlight = true;                                                // set the data inflight flag
}

void UbidotsHandler(const char *event, const char *data)              // Looks at the response from Ubidots - Will reset Photon if no successful response
{                                                                     // Response Template: "{{hourly.0.status_code}}" so, I should only get a 3 digit number back

  char dataCopy[strlen(data)+1];                                      // data needs to be copied since Particle.publish() will clear it
  strncpy(dataCopy, data, sizeof(dataCopy));                          // Copy - overflow safe
  if (!strlen(dataCopy)) {                                            // First check to see if there is any data
    Particle.publish("Ubidots Hook", "No Data");
    return;
  }
  int responseCode = atoi(dataCopy);                                  // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    Particle.publish("State","Response Received");
    dataInFlight = false;                                             // Data has been received
  }
  else Particle.publish("Ubidots Hook", dataCopy);                    // Publish the response code
}

// These are the functions that are part of the takeMeasurements call
void takeMeasurements() {
  if (Cellular.ready()) getSignalStrength();                          // Test signal strength if the cellular modem is on and ready
  getTemperature();                                                   // Get Temperature at startup as well
  stateOfCharge = int(batteryMonitor.getSoC());                       // Percentage of full charge
}


void getSignalStrength()
{
    CellularSignal sig = Cellular.RSSI();                             // Prototype for Cellular Signal Montoring
    int rssi = sig.rssi;
    int strength = map(rssi, -131, -51, 0, 5);
    snprintf(SignalString,17, "%s: %d", levels[strength], rssi);
}

int getTemperature()
{
  int reading = analogRead(tmp36Pin);                                 //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;                                      // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                                                  // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));                    //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  temperatureF = int((temperatureC * 9.0 / 5.0) + 32.0);              // now convert to Fahrenheit
  return temperatureF;
}

// Here are the various hardware and timer interrupt service routines
void sensorISR()
{
  sensorDetect = true;                                                // sets the sensor flag for the main loop
}

void watchdogISR()
{
  digitalWrite(donePin, HIGH);                                        // Pet the watchdog
  digitalWrite(donePin, LOW);
}

// These functions control the connection and disconnection from Particle
bool connectToParticle()
{
  Cellular.on();                                                      // Don't need to make it complicated
  Particle.connect();                                                 // Connect to Particle
  controlRegister = FRAMread8(CONTROLREGISTER);
  connectionMode = true;
  controlRegister = (0b00010000 | controlRegister);                   // Turn on connectionMode
  FRAMwrite8(CONTROLREGISTER, controlRegister);
  return true;
}

bool disconnectFromParticle()
{
  Cellular.off();                                                     // Turn off the cellular modem
  controlRegister = FRAMread8(CONTROLREGISTER);
  connectionMode = false;
  controlRegister = (0b11101111 & controlRegister);                   // Turn off connectionMode
  FRAMwrite8(CONTROLREGISTER, controlRegister);
  return true;
}

// Power Management function
void PMICreset() {
  power.begin();                                                      // Settings for Solar powered power management
  power.disableWatchdog();
  if (solarPowerMode) {
    lowBattLimit = 20;                                                // Trigger for Low Batt State
    power.setInputVoltageLimit(4840);                                 // Set the lowest input voltage to 4.84 volts best setting for 6V solar panels
    power.setInputCurrentLimit(900);                                  // default is 900mA
    power.setChargeCurrent(0,0,1,0,0,0);                              // default is 512mA matches my 3W panel
    power.setChargeVoltage(4208);                                     // Allows us to charge cloe to 100% - battery can't go over 45 celcius
  }
  else  {
    lowBattLimit = 30;                                                // Trigger for Low Batt State
    power.setInputVoltageLimit(4208);                                 // This is the default value for the Electron
    power.setInputCurrentLimit(1500);                                 // default is 900mA this let's me charge faster
    power.setChargeCurrent(0,1,1,0,0,0);                              // default is 2048mA (011000) = 512mA+1024mA+512mA)
    power.setChargeVoltage(4112);                                     // default is 4.112V termination voltage
  }
}

 // These are the particle functions that allow you to configure and run the device
 // They are intended to allow for customization and control during installations
 // and to allow for management.
int resetFRAM(String command)                                         // Will reset the local counts
{
  if (command == "1")
  {
    ResetFRAM();
    return 1;
  }
  else return 0;
}

int resetCounts(String command)                                       // Resets the current hourly and daily counts
{
  if (command == "1")
  {
    FRAMwrite16(CURRENTDAILYCOUNT, 0);                                // Reset Daily Count in memory
    FRAMwrite16(CURRENTHOURLYCOUNT, 0);                               // Reset Hourly Count in memory
    FRAMwrite8(RESETCOUNT,0);                                         // If so, store incremented number - watchdog must have done This
    resetCount = 0;
    hourlyPersonCount = 0;                                            // Reset count variables
    dailyPersonCount = 0;
    hourlyPersonCountSent = 0;                                        // In the off-chance there is data in flight
    dataInFlight = false;
    return 1;
  }
  else return 0;
}


int hardResetNow(String command)                                      // Will perform a hard reset on the Electron
{
  if (command == "1")
  {
    digitalWrite(hardResetPin,HIGH);                                  // This will cut all power to the Electron AND the carrir board
    return 1;                                                         // Unfortunately, this will never be sent
  }
  else return 0;
}

int setDebounce(String command)                                       // This is the amount of time in seconds we will wait before starting a new session
{
  char * pEND;
  float inputDebounce = strtof(command,&pEND);                        // Looks for the first integer and interprets it
  if ((inputDebounce < 0.0) | (inputDebounce > 5.0)) return 0;        // Make sure it falls in a valid range or send a "fail" result
  debounce = int(inputDebounce*1000);                                 // debounce is how long we must space events to prevent overcounting
  int debounceFRAM = constrain(int(inputDebounce*10),1,255);          // Store as a byte in FRAM = 1.6 seconds becomes 16 cSec
  FRAMwrite8(DEBOUNCEADDR,static_cast<uint8_t>(debounceFRAM));        // Convert to Int16 and store
  snprintf(debounceStr,sizeof(debounceStr),"%2.1f sec",inputDebounce);
  if (verboseMode) {                                                  // Publish result if feeling verbose
    waitUntil(meterParticlePublish);
    Particle.publish("Debounce",debounceStr);
    lastPublish = millis();
  }
  return 1;                                                           // Returns 1 to let the user know if was reset
}

int setSensivty(String command)  // Will accept a new debounce value in the form "debounce:xxx" where xxx is an integer for delay in mSec
{
  char data[256];
  char *pEND;
  inputSensitivity = strtol(command,&pEND,0);
  if ((inputSensitivity < 0) | (inputSensitivity > 10)) return 0;
  if(verboseMode) Particle.publish("Event","Set sensitivity");
  FRAMwrite8(SENSITIVITYADDR, 10-inputSensitivity);
  initMMA8452(accelFullScaleRange, dataRate);  // init the accelerometer if communication is OK
  snprintf(data, sizeof(data), "Accelerometer online, sensitivity set to %i",inputSensitivity);
  if (verboseMode) Particle.publish("Count",data);
  return 1;
}
int sendNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = REPORTING_STATE;
    return 1;
  }
  else return 0;
}

int setSolarMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    solarPowerMode = true;
    FRAMread8(CONTROLREGISTER);
    controlRegister = (0b00000100 | controlRegister);          // Turn on solarPowerMode
    FRAMwrite8(CONTROLREGISTER,controlRegister);               // Write it to the register
    PMICreset();                                               // Change the power management Settings
    Particle.publish("Mode","Set Solar Powered Mode");
    return 1;
  }
  else if (command == "0")
  {
    solarPowerMode = false;
    FRAMread8(CONTROLREGISTER);
    controlRegister = (0b11111011 & controlRegister);           // Turn off solarPowerMode
    FRAMwrite8(CONTROLREGISTER,controlRegister);                // Write it to the register
    PMICreset();                                                // Change the power management settings
    Particle.publish("Mode","Cleared Solar Powered Mode");
    return 1;
  }
  else return 0;
}

int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    verboseMode = true;
    FRAMread8(CONTROLREGISTER);
    controlRegister = (0b00001000 | controlRegister);                    // Turn on verboseMode
    FRAMwrite8(CONTROLREGISTER,controlRegister);                        // Write it to the register
    Particle.publish("Mode","Set Verbose Mode");
    return 1;
  }
  else if (command == "0")
  {
    verboseMode = false;
    FRAMread8(CONTROLREGISTER);
    controlRegister = (0b11110111 & controlRegister);                    // Turn off verboseMode
    FRAMwrite8(CONTROLREGISTER,controlRegister);                        // Write it to the register
    Particle.publish("Mode","Cleared Verbose Mode");
    return 1;
  }
  else return 0;
}

int setTimeZone(String command)
{
  char * pEND;
  char data[256];
  time_t t = Time.now();
  int8_t tempTimeZoneOffset = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTimeZoneOffset < -12) | (tempTimeZoneOffset > 12)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  Time.zone((float)tempTimeZoneOffset);
  FRAMwrite8(TIMEZONE,tempTimeZoneOffset);                             // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Time zone offset %i",tempTimeZoneOffset);
  Particle.publish("Time",data);
  delay(1000);
  Particle.publish("Time",Time.timeStr(t));
  return 1;
}

int setOpenTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 23)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  openTime = tempTime;
  FRAMwrite8(OPENTIMEADDR,openTime);                             // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Open time set to %i",openTime);
  Particle.publish("Time",data);
  return 1;
}

int setCloseTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 24)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  closeTime = tempTime;
  FRAMwrite8(CLOSETIMEADDR,closeTime);                             // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Closing time set to %i",closeTime);
  Particle.publish("Time",data);
  return 1;
}


int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
  controlRegister = FRAMread8(CONTROLREGISTER);                       // Get the control register (generla approach)
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    Particle.publish("Mode","Low Power");
    controlRegister = (0b00000001 | controlRegister);                  // If so, flip the lowPowerMode bit
    lowPowerMode = true;
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    Particle.publish("Mode","Normal Operations");
    controlRegister = (0b1111110 & controlRegister);                  // If so, flip the lowPowerMode bit
    lowPowerMode = false;
  }
  FRAMwrite8(CONTROLREGISTER,controlRegister);                         // Write to the control register
  return 1;
}

bool meterParticlePublish(void)
{
  if(millis() - lastPublish >= publishFrequency) return 1;
  else return 0;
}

void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  waitUntil(meterParticlePublish);
  Particle.publish("State Transition",stateTransitionString);
  Serial.println(stateTransitionString);
  lastPublish = millis();
}

void fullModemReset() {  // Adapted form Rikkas7's https://github.com/rickkas7/electronsample
	Particle.disconnect(); 	                                         // Disconnect from the cloud
	unsigned long startTime = millis();  	                           // Wait up to 15 seconds to disconnect
	while(Particle.connected() && millis() - startTime < 15000) {
		delay(100);
	}
	// Reset the modem and SIM card
	// 16:MT silent reset (with detach from network and saving of NVM parameters), with reset of the SIM card
	Cellular.command(30000, "AT+CFUN=16\r\n");
	delay(1000);
	// Go into deep sleep for 10 seconds to try to reset everything. This turns off the modem as well.
	System.sleep(SLEEP_MODE_DEEP, 10);
}
