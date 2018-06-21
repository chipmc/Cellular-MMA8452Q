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
    Control Register - bits 7-4, 3 - Verbose Mode, 2- Solar Power Mode, 1 - Low Battery Mode, 0 - Low Power Mode
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
#define DEBOUNCEADDR 0x2A           // Where we store debounce
#define RESETCOUNT 0x3              // This is where we keep track of how often the Electron was reset
#define TIMEZONE  0x4                 // Store the local time zone data                                    // One byte is open here
#define OPENTIMEADDR 0x5              // Hour for opening the park / store / etc - military time (e.g. 6 is 6am)
#define CLOSETIMEADDR 0x6             // Hour for closing of the park / store / etc - military time (e.g 23 is 11pm)
#define CONTROLREGISTER 0x7         // This is the control register for storing the current state
//Second and Third words bytes for storing current counts
#define CURRENTHOURLYCOUNT 0x8  // Current Hourly Count - 16 bits
#define CURRENTDAILYCOUNT 0xC   // Current Daily Count - 16 bits
#define CURRENTCOUNTSTIME 0xE       // Time of last count - 32 bits
#define HOURLYPOINTERADDR 0x11       // Two bytes for hourly pointer
                                    // Four open bytes here which takes us to the third word
//These are the hourly and daily offsets that make up the respective words
#define HOURLYCOUNTOFFSET 4         // Offsets for the values in the hourly words
#define HOURLYBATTOFFSET 6          // Where the hourly battery charge is stored
// Finally, here are the variables I want to change often and pull them all together here
#define SOFTWARERELEASENUMBER "0.53"

// Included Libraries
#include "Adafruit_FRAM_I2C.h"                           // Library for FRAM functions
#include "FRAM-Library-Extensions.h"                     // Extends the FRAM Library
#include "electrondoc.h"                                 // Documents pinout

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);    // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);         // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
FuelGauge batteryMonitor;       // Prototype for the fuel gauge (included in Particle core library)
PMIC power;                      //Initalize the PMIC class so you can call the Power Management functions below.

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
const int intPin =        D3;                       // PIR Sensor Interrupt pin
const int hardResetPin =  D4;                       // Power Cycles the Electron and the Carrier Board
const int userSwitch =    D5;                       // User switch with a pull-up resistor
const int donePin =       D6;                       // Pin the Electron uses to "pet" the watchdog
const int blueLED =       D7;                       // This LED is on the Electron itself


// Timing Variables
const unsigned long stayAwake = 90000;                    // In lowPowerMode, how long to stay awake every hour
const unsigned long webhookWait = 45000;                  // How long will we wair for a WebHook response
const unsigned long resetWait = 30000;                    // How long will we wait in ERROR_STATE until reset
const int publishFrequency = 1000;                // We can only publish once a second
unsigned long stayAwakeTimeStamp = 0;               // Timestamps for our timing variables
unsigned long webhookTimeStamp = 0;
unsigned long resetTimeStamp = 0;
unsigned long publishTimeStamp = 0;                 // Keep track of when we publish a webhook
unsigned long lastPublish = 0;

// Program Variables
int temperatureF;                                   // Global variable so we can monitor via cloud variable
int resetCount;                                     // Counts the number of times the Electron has had a pin reset
volatile bool ledState = LOW;                                // variable used to store the last LED status, to toggle the light
bool readyForBed = false;                           // Checks to see if steps for sleep have been completed
bool pettingEnabled = true;                         // Let's us pet the hardware watchdog
bool dataInFlight = false;                          // Tracks if we have sent data but not yet cleared it from counts until we get confirmation
const char* releaseNumber = SOFTWARERELEASENUMBER;  // Displays the release on the menu
byte controlRegister;                               // Stores the control register values
bool lowPowerMode;                                  // Flag for Low Power Mode operations
bool solarPowerMode;                                // Changes the PMIC settings
bool verboseMode;                                   // Enables more active communications for configutation and setup
char SignalString[17];                           // Used to communicate Wireless RSSI and Description
const char* levels[6] = {"Poor", "Low", "Medium", "Good", "Very Good", "Great"};
char debounceStr[8] = "NA";                     // String to communicate transit time to app


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
int debounce;
int inputSensitivity;                               // Raw sensitivity input - Initialized in Setup (0 least sensitive -9 most sensitive)
volatile bool sensorDetect = false;                 // This is the flag that an interrupt is triggered
volatile time_t currentEvent = 0;               // Keeps track of the last time there was an event
volatile byte source;                               // Why was the accelerometer interrupt triggered
bool sensorLockedUp = false;                        // Flag for automatic reset if reporting 0 during working hours
int hourlyPersonCount = 0;                          // hourly counter
int hourlyPersonCountSent = 0;                      // Person count in flight to Ubidots
int dailyPersonCount = 0;                           // daily counter
uint32_t lastBump = 0;                              // set the time of an event
bool inTest = false;                                // Are we in a test or not

// Sensor specific libraries here
#include "MMA8452-Functions.h"                           // Adds the accelerometer functions

// Sensor specific function prototypes here
// None


void setup()                                                      // Note: Disconnected Setup()
{
  Wire.begin();                                                   //Create a Wire object

  pinMode(int2Pin,INPUT);                                         // accelerometer interrupt pinMode
  pinMode(wakeUpPin,INPUT);                                       // This pin is active HIGH
  pinMode(userSwitch,INPUT);                                      // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                                       // declare the Blue LED Pin as an output
  pinMode(tmp36Shutdwn,OUTPUT);                                   // Supports shutting down the TMP-36 to save juice
  digitalWrite(tmp36Shutdwn, HIGH);                               // Turns on the temp sensor
  pinMode(donePin,OUTPUT);                                        // Allows us to pet the watchdog
  pinMode(hardResetPin,OUTPUT);                                   // For a hard reset active HIGH

  watchdogISR();                                                  // Pet the watchdog

  char responseTopic[125];
  String deviceID = System.deviceID();                                // Multiple Electrons share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);
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
  Particle.variable("Debounce",debounce);


  Particle.function("resetFRAM", resetFRAM);
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


  if (!fram.begin()) {                                                  // You can stick the new i2c addr in here, e.g. begin(0x51);
    resetTimeStamp = millis();                                          // Can't communicate with FRAM - fatal error
    state = ERROR_STATE;
  }
  else if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) {                   // Check to see if the memory map in the sketch matches the data on the chip
    ResetFRAM();                                                        // Reset the FRAM to correct the issue
    if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) {
      resetTimeStamp = millis();
      state = ERROR_STATE;   // Resetting did not fix the issue
    }
    else {
      FRAMwrite8(CONTROLREGISTER,0);                                    // Need to reset so not in low power or low battery mode
      FRAMwrite8(OPENTIMEADDR,0);                                       // These set the defaults if the FRAM is erased
      FRAMwrite8(CLOSETIMEADDR,24);                                     // This will ensure the device does not sleep
    }
  }

  resetCount = FRAMread8(RESETCOUNT);                                   // Retrive system recount data from FRAM
  if (System.resetReason() == RESET_REASON_PIN_RESET)                   // Check to see if we are starting from a pin reset
  {
    resetCount++;
    FRAMwrite8(RESETCOUNT,static_cast<uint8_t>(resetCount));            // If so, store incremented number - watchdog must have done This
  }

  // Import the inputSensitivity and Debounce values from memory
  inputSensitivity = 10-FRAMread8(SENSITIVITYADDR);                     // We store the inverse as humans think of 10 as more sensitive than 0
  openTime = FRAMread8(OPENTIMEADDR);
  closeTime = FRAMread8(CLOSETIMEADDR);

  int8_t tempTimeZoneOffset = FRAMread8(TIMEZONE);                      // Load Time zone data from FRAM
  if (tempTimeZoneOffset <= 12 && tempTimeZoneOffset >= -12)  Time.zone((float)tempTimeZoneOffset);  // Load Timezone from FRAM
  else Time.zone(0);                                                   // Default is GMT in case proper value not in FRAM

  controlRegister = FRAMread8(CONTROLREGISTER);                         // Read the Control Register for system modes
  lowPowerMode = (0b00000001 & controlRegister);                        // Bitwise AND to set the lowPowerMode flag from control Register
  verboseMode = (0b00001000 & controlRegister);                         // verboseMode
  solarPowerMode = (0b00000100 & controlRegister);                      // solarPowerMode

  PMICreset();                                                          // Executes commands that set up the PMIC for Solar charging

  takeMeasurements();

  if (!lowPowerMode && (stateOfCharge >= lowBattLimit) && !(Time.hour() >= closeTime || Time.hour() < openTime)) connectToParticle();  // If not lowpower or sleeping, we can connect

  currentHourlyPeriod = Time.hour();                                    // Sets the hour period for when the count starts (see #defines)
  currentDailyPeriod = Time.day();                                      // And the day  (see #defines)
  // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
  time_t unixTime = FRAMread32(CURRENTCOUNTSTIME);                      // Need to reload program control since reset
  lastHour = Time.hour(unixTime);
  lastDate = Time.day(unixTime);
  dailyPersonCount = FRAMread16(CURRENTDAILYCOUNT);                     // Load Daily Count from memory
  hourlyPersonCount = FRAMread16(CURRENTHOURLYCOUNT);                   // Load Hourly Count from memory

  if (!digitalRead(userSwitch)) {                                       // Rescue mode to locally take lowPowerMode so you can connect to device
    lowPowerMode = false;                                               // Press the user switch while resetting the device
    controlRegister = (0b1111110 & controlRegister);                    // Turn off Low power mode
    FRAMwrite8(CONTROLREGISTER,controlRegister);                        // Write it to the register
    openTime = 0;                                                       // Device may alos be sleeping due to time or TimeZone setting
    FRAMwrite8(OPENTIMEADDR,0);                                         // Reset open and close time values to ensure device is awake
    closeTime = 24;
    FRAMwrite8(CLOSETIMEADDR,24);
    connectToParticle();                                                // Connects the Electron to Particle so you can control it
    Particle.publish("Startup","Startup rescue - reset time and power");
  }

  byte c = readRegister(MMA8452_ADDRESS,0x0D);  // Read WHO_AM_I register for accelerometer
  if (c == 0x2A) // WHO_AM_I should always be 0x2A
  {
    initMMA8452(accelFullScaleRange, dataRate);  // init the accelerometer if communication is OK
  }
  else
  {
    resetTimeStamp = millis();                                          // Can't communicate with FRAM - fatal error
    state = ERROR_STATE;
  }
  initMMA8452(accelFullScaleRange,dataRate);

  attachInterrupt(int2Pin,sensorISR,RISING);                      // Accelerometer interrupt from low to high
  attachInterrupt(wakeUpPin, watchdogISR, RISING);                // The watchdog timer will signal us and we have to respond

  if (state != ERROR_STATE) state = IDLE_STATE;                         // IDLE unless error from above code
}

void loop()
{
  switch(state) {
  case IDLE_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
    if(hourlyPersonCountSent) {                                                   // Cleared here as there could be counts coming in while "in Flight"
      hourlyPersonCount -= hourlyPersonCountSent;                                 // Confirmed that count was recevied - clearing
      FRAMwrite16(CURRENTHOURLYCOUNT, static_cast<uint16_t>(hourlyPersonCount));  // Load Hourly Count to memory
      hourlyPersonCountSent = 0;
    }
    if (sensorDetect) recordCount();                                              // The ISR had raised the sensor flag
    if (lowPowerMode && (millis() > stayAwakeTimeStamp + stayAwake)) state = NAPPING_STATE;
    if (Time.hour() != currentHourlyPeriod) state = REPORTING_STATE;              // We want to report on the hour but not after bedtime
    if ((Time.hour() >= closeTime || Time.hour() < openTime)) state = SLEEPING_STATE;   // The park is closed, time to sleep
    if (stateOfCharge <= lowBattLimit) state = LOW_BATTERY_STATE;                 // The battery is low - sleep
    break;

  case SLEEPING_STATE: {                                        // This state is triggered once the park closes and runs until it opens
    if (verboseMode && state != oldState) publishStateTransition();
    if (!readyForBed)                                           // Only do these things once - at bedtime
    {
      if (hourlyPersonCount) {                                  // If this number is not zero then we need to send this last count
        state = REPORTING_STATE;
        break;
      }
      if (Particle.connected()) {
        disconnectFromParticle();                               // If connected, we need to disconned and power down the modem
      }
      detachInterrupt(int2Pin);                                  // Done sensing for the day
      FRAMwrite16(CURRENTDAILYCOUNT, 0);                    // Reset the counts in FRAM as well
      FRAMwrite8(RESETCOUNT,resetCount);
      FRAMwrite16(CURRENTHOURLYCOUNT, 0);
      dailyPersonCount = resetCount = hourlyPersonCount = 0;
      ledState = false;
      digitalWrite(blueLED,LOW);                                // Turn off the LED
      digitalWrite(tmp36Shutdwn, LOW);                          // Turns off the temp sensor
      watchdogISR();                                                    // Pet the watchdog
      readyForBed = true;                                       // Set the flag for the night
    }
    int secondsToHour = (60*(60 - Time.minute()));              // Time till the top of the hour
    System.sleep(SLEEP_MODE_DEEP,secondsToHour);                // Very deep sleep till the next hour - then resets
    } break;

  case NAPPING_STATE: {
    if (verboseMode && state != oldState) publishStateTransition();
      if (Particle.connected())
        {
          Particle.publish("State","Disconnecting from Particle");
          disconnectFromParticle();                                       // If connected, we need to disconned and power down the modem
        }
        ledState = false;                                                 // Turn out the light
        digitalWrite(blueLED,LOW);                                        // Turn off the LED
        watchdogISR();                                                    // Pet the watchdog
        int secondsToHour = (60*(60 - Time.minute()));                    // Time till the top of the hour
        System.sleep(int2Pin, RISING, secondsToHour);             // Sensor will wake us with an interrupt
        state = IDLE_STATE;                                      // Back to the IDLE_STATE after a nap
    } break;

    case LOW_BATTERY_STATE: {                                             // Sleep state but leaves the fuel gauge on
      if (verboseMode && state != oldState) publishStateTransition();
      if (Particle.connected()) {
        disconnectFromParticle();                                       // If connected, we need to disconned and power down the modem
      }
      detachInterrupt(int2Pin);                                          // Done sensing for the day
      ledState = false;
      digitalWrite(blueLED,LOW);                                        // Turn off the LED
      digitalWrite(tmp36Shutdwn, LOW);                                  // Turns off the temp sensor
      watchdogISR();                                                    // Pet the watchdog
      int secondsToHour = (60*(60 - Time.minute()));                    // Time till the top of the hour
      System.sleep(SLEEP_MODE_DEEP,secondsToHour);                      // Very deep sleep till the next hour - then resets
    } break;

  case REPORTING_STATE: {
    if (verboseMode && state != oldState) publishStateTransition();
      watchdogISR();                                                      // Pet the watchdog once an hour
      pettingEnabled = false;                                             // Going to see the reporting process through before petting again
      if (!Particle.connected()) {
        if (!connectToParticle()) {
          resetTimeStamp = millis();
          state = ERROR_STATE;
          break;
        }
      }
      takeMeasurements();                                                 // Update Temp, Battery and Signal Strength values
      sendEvent();                                                        // Send data to Ubidots
      webhookTimeStamp = millis();
      state = RESP_WAIT_STATE;                                            // Wait for Response
    } break;

  case RESP_WAIT_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
    if (!dataInFlight)                                                  // Response received back to IDLE state
    {
      state = IDLE_STATE;
      pettingEnabled = true;                                            // Enable petting before going back into the loop
      stayAwakeTimeStamp = millis();
    }
    else if (millis() > webhookTimeStamp + webhookWait) {               // If it takes too long - will need to reset
      resetTimeStamp = millis();
      state = ERROR_STATE;                                              // Response timed out
    }
    break;

  case ERROR_STATE:                                          // To be enhanced - where we deal with errors
    if (verboseMode && state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait)
    {
      Particle.publish("State","ERROR_STATE - Resetting");
      delay(2000);
      System.reset();
      /*                                       // This makes sure it goes through before reset
      if (resetCount <= 3)  System.reset();                 // Today, only way out is reset
      else {
        resetCount = 0;
        FRAMwrite8(RESETCOUNT,0);                           // Time for a hard reset
        digitalWrite(hardResetPin,HIGH);                    // Zero the count so only every three
      }
      */
    }
    break;

  }
}

void recordCount() // This is where we check to see if an interrupt is set when not asleep or act on a tap that woke the Arduino
{
  char data[256];                                           // Store the date in this character array - not global
  sensorDetect = false;      // Reset the flag
  if ((source & 0x08)==0x08)  // We are only interested in the TAP register and ignore debounced taps
  {
    hourlyPersonCount++;                    // Increment the PersonCount
    FRAMwrite16(CURRENTHOURLYCOUNT, hourlyPersonCount);  // Load Hourly Count to memory
    dailyPersonCount++;                    // Increment the PersonCount
    FRAMwrite16(CURRENTDAILYCOUNT, dailyPersonCount);   // Load Daily Count to memory
    FRAMwrite32(CURRENTCOUNTSTIME, currentEvent);   // Write to FRAM - this is so we know when the last counts were saved
    snprintf(data, sizeof(data), "Car, hourlry count: %i, daily count: %i",hourlyPersonCount,dailyPersonCount);
    if (verboseMode) Particle.publish("Count",data);
  }
  else if ((source & 0x08) != 0x08) if(verboseMode) Particle.publish("Event","Interrupt not a tap");
  if (!digitalRead(userSwitch)) {                           // A low value means someone is pushing this button - will trigger a send to Ubidots and take out of low power mode
    if (lowPowerMode) {
      Particle.publish("Mode","Normal Operations");
      controlRegister = (0b1111110 & controlRegister);     // Will set the lowPowerMode bit to zero
      FRAMwrite8(CONTROLREGISTER,controlRegister);
      lowPowerMode = false;
      connectToParticle();                                  // Reconnect to Particle for monitoring and management
    }
  }
}


void sendEvent()
{
  char data[256];                                         // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"hourly\":%i, \"daily\":%i,\"battery\":%i, \"temp\":%i, \"resets\":%i}",hourlyPersonCount, dailyPersonCount, stateOfCharge, temperatureF,resetCount);
  Particle.publish("Ubidots-Hook", data, PRIVATE);
  hourlyPersonCountSent = hourlyPersonCount; // This is the number that was sent to Ubidots - will be subtracted once we get confirmation
  currentHourlyPeriod = Time.hour();  // Change the time period
  dataInFlight = true; // set the data inflight flag
}

void UbidotsHandler(const char *event, const char *data)  // Looks at the response from Ubidots - Will reset Photon if no successful response
{
  // Response Template: "{{hourly.0.status_code}}" so, I should only get a 3 digit number back
  char dataCopy[strlen(data)+1];                                    // data needs to be copied since Particle.publish() will clear it
  strncpy(dataCopy, data, sizeof(dataCopy));            // Copy - overflow safe
  if (!strlen(dataCopy)) {                                      // First check to see if there is any data
    Particle.publish("Ubidots Hook", "No Data");
    return;
  }
  int responseCode = atoi(dataCopy);                    // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    Particle.publish("State","Response Received");
    dataInFlight = false;                                 // Data has been received
  }
  else Particle.publish("Ubidots Hook", dataCopy);       // Publish the response code
}

// These are the functions that are part of the takeMeasurements call

void takeMeasurements() {
  if (Cellular.ready()) getSignalStrength();                // Test signal strength if the cellular modem is on and ready
  getTemperature();                                         // Get Temperature at startup as well
  stateOfCharge = int(batteryMonitor.getSoC());             // Percentage of full charge
}


void getSignalStrength()
{
    CellularSignal sig = Cellular.RSSI();  // Prototype for Cellular Signal Montoring
    int rssi = sig.rssi;
    int strength = map(rssi, -131, -51, 0, 5);
    snprintf(SignalString,17, "%s: %d", levels[strength], rssi);
}

int getTemperature()
{
  int reading = analogRead(tmp36Pin);   //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;        // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                    // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));  //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  temperatureF = int((temperatureC * 9.0 / 5.0) + 32.0);  // now convert to Fahrenheit
  return temperatureF;
}

// Here are the various hardware and timer interrupt service routines

void sensorISR()
{
  static int i=0;
  if (millis() >= currentEvent + debounce) {                                 // Can read time in an ISR but it won't increment
    sensorDetect = true;                                  // sets the sensor flag for the main loop
    currentEvent = millis();
    ledState = !ledState;              // toggle the status of the LEDPIN:
    pinSetFast(blueLED);    // update the LED pin itself
  }
  source = readRegister(MMA8452_ADDRESS,0x0C);            // Read the interrupt source reg.
  do {                                                    // All these gymanastics are required to ensure the interrupt is reset
      i++;                                                // Apparently, this is an issue with the MMA8452
      readRegister(MMA8452_ADDRESS,0x22);                 // Reads the PULSE_SRC register to reset it
      if (i>=10)                                          // Not sure if this is requried - take out after a month if never triggered
      {
        initMMA8452(accelFullScaleRange,dataRate);        // Last Resort
        if(digitalRead(int2Pin)) digitalWrite(hardResetPin,HIGH);       // If all else fails, reset the device and the accelerometer
      }
  } while(digitalRead(int2Pin));                          // Won't exit the do loop until the accelerometer's interrupt is reset
  pinResetFast(blueLED);
}


void watchdogISR()
{
  digitalWrite(donePin, HIGH);                              // Pet the watchdog
  digitalWrite(donePin, LOW);
}

// These functions control the connection and disconnection from Particle

bool connectToParticle()
{
  if (!Cellular.ready())
  {
    Cellular.on();                                           // turn on the Modem
    Cellular.connect();                                      // Connect to the cellular network
    if(!waitFor(Cellular.ready,90000)) return false;         // Connect to cellular - give it 90 seconds
  }
  Particle.process();
  Particle.connect();                                      // Connect to Particle
  if(!waitFor(Particle.connected,30000)) return false;     // Connect to Particle - give it 30 seconds
  Particle.process();
  return true;
}

bool disconnectFromParticle()
{
  Particle.disconnect();                                   // Disconnect from Particle in prep for sleep
  waitFor(notConnected,10000);
  Cellular.disconnect();                                   // Disconnect from the cellular network
  delay(3000);
  Cellular.off();                                           // Turn off the cellular modem
  return true;
}

bool notConnected() {
  return !Particle.connected();                             // This is a requirement to use waitFor
}

// Power Management function

void PMICreset() {
  power.begin();                                            // Settings for Solar powered power management
  power.disableWatchdog();
  if (solarPowerMode) {
    lowBattLimit = 20;                                      // Trigger for Low Batt State
    power.setInputVoltageLimit(4840);                       // Set the lowest input voltage to 4.84 volts best setting for 6V solar panels
    power.setInputCurrentLimit(900);                        // default is 900mA
    power.setChargeCurrent(0,0,1,0,0,0);                    // default is 512mA matches my 3W panel
    power.setChargeVoltage(4208);                           // Allows us to charge cloe to 100% - battery can't go over 45 celcius
  }
  else  {
    lowBattLimit = 30;                                      // Trigger for Low Batt State
    power.setInputVoltageLimit(4208);                       // This is the default value for the Electron
    power.setInputCurrentLimit(1500);                       // default is 900mA this let's me charge faster
    power.setChargeCurrent(0,1,1,0,0,0);                    // default is 2048mA (011000) = 512mA+1024mA+512mA)
    power.setChargeVoltage(4112);                           // default is 4.112V termination voltage
  }
}

 // These are the particle functions that allow you to configure and run the device
 // They are intended to allow for customization and control during installations
 // and to allow for management.


int resetFRAM(String command)   // Will reset the local counts
{
  if (command == "1")
  {
    ResetFRAM();
    return 1;
  }
  else return 0;
}

int resetCounts(String command)   // Resets the current hourly and daily counts
{
  if (command == "1")
  {
    FRAMwrite16(CURRENTDAILYCOUNT, 0);   // Reset Daily Count in memory
    FRAMwrite16(CURRENTHOURLYCOUNT, 0);  // Reset Hourly Count in memory
    FRAMwrite8(RESETCOUNT,0);          // If so, store incremented number - watchdog must have done This
    resetCount = 0;
    hourlyPersonCount = 0;                    // Reset count variables
    dailyPersonCount = 0;
    hourlyPersonCountSent = 0;                // In the off-chance there is data in flight
    dataInFlight = false;
    return 1;
  }
  else return 0;
}


int hardResetNow(String command)   // Will perform a hard reset on the Electron
{
  if (command == "1")
  {
    digitalWrite(hardResetPin,HIGH);          // This will cut all power to the Electron AND the carrir board
    return 1;                                 // Unfortunately, this will never be sent
  }
  else return 0;
}

int setDebounce(String command)  // This is the amount of time in seconds we will wait before starting a new session
{
  char * pEND;
  char data[256];
  float inputDebounce = strtof(command,&pEND);                       // Looks for the first integer and interprets it
  if ((inputDebounce < 0.0) | (inputDebounce > 5.0)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  debounce = int(inputDebounce*1000);                   // debounce is how long we must space events to prevent overcounting
  int debounceFRAM = constrain(int(inputDebounce*10),1,255); // Store as a byte in FRAM = 1.6 becomes 16
  FRAMwrite8(DEBOUNCEADDR,static_cast<uint8_t>(debounceFRAM));
  snprintf(debounceStr,sizeof(debounceStr),"%2.1f sec",inputDebounce);
  if (verboseMode) {
    waitUntil(meterParticlePublish);
    snprintf(data, sizeof(data), "Debounce is: %2.1f seconds",debounce);
    Particle.publish("Variables",data);
    lastPublish = millis();
  }
  return 1;
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
  lastPublish = millis();
}
