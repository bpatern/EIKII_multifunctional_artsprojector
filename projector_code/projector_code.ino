

// ------------------------------------------
// SPECTRAL Wandering Sounds + Images Seminar
// ------------------------------------------
// Projector control software for ESP32 micro-controller
// On some 38pin ESP32 boards you need to press GPIO 0 [right] button each time you upload code. If so, solder 10uF cap between EN and GND to enable auto reset.
// Controlling Hobbywing "Quickrun Fusion SE" motor/ESC combo using ESP32 LEDC peripheral (using 16bit timer for extra resolution)
// Note: Hobbywing motor/esc needs to be programmed via "programming card" to disable braking, otherwise it will try to hold position when speed = 0
// (If you hack the ESC wiring according to our docs, you can program it with the ESCprogram() function instead of a programming card)
// "Digital shutter" is accomplished via AS5047 magnetic rotary encoder, setup via SPI registers and monitored via ABI quadrature pulses
// LED is dimmed via CC LED driver board with PWM input, driven by ESP32 LEDC (which causes slight PWM blinking artifacts at low brightness levels)
// Future option exists for current-controlled dimming (set LedDimMode=0): Perhaps a log taper digipot controlled via SPI? Probably can't dim below 20% though.


// TODO Nice but not essential:
// put UI vars in struct, in case we want to receive them from a remote ESP connected via radio
// if ESP32 Arduino core ever gets support for ledc output_invert method, we can simplify the ISR inverted condition

// Include the libraries
// NOTE: In 2023 this code was developed using the ESP32 Arduino core v2.0.9.
// When ESP32 Arduino core v3.x is released there will be breaking changes because LEDC setup will be different!
#include <AS5X47.h>              // https://github.com/adrien-legrand/AS5X47
#include <elapsedMillis.h>       // https://github.com/pfeerick/elapsedMillis
#include <Ramp.h>                // https://github.com/siteswapjuggler/RAMP
#include <SimpleKalmanFilter.h>  // https://github.com/denyssene/SimpleKalmanFilter
#include <Button2.h>             // https://github.com/LennartHennigs/Button2
#include <Adafruit_NeoPixel.h>   // https://github.com/adafruit/Adafruit_NeoPixel

#include <HardwareSerial.h>


// Uncomment ONE of these to load preset configurations from the matching files
#include "spectral_eiki.h"  // "Store projector-specific settings here"
//#include "spectral_p26.h" // "Store projector-specific settings here"



// Encoder
#define EncI 17   // encoder pulse Index (AS5047 sensor)
#define EncA 16   // encoder pulse A (AS5047 sensor)
#define EncB 4    // encoder pulse B (AS5047 sensor)
#define EncCSN 5  // encoder SPI CSN Pin (AS5047 sensor) - Library also reserves GPIO 18, 19, 23 for SPI

// OUTPUT PINS //
#define escPin 22         // PWM output for ESC
#define escProgramPin 21  // serial output to program ESC (needs 10k resistor to 3.3v)
#define ledPin 2          // PWM output for LED (On 38pin HiLetGo ESP32 board, GPIO 2 is the built-in LED)
#define NeoPixelPin 15    // output pin for NeoPixel status LED(s)

// Simple Kalman Filter Library Setup
// see tuning advice here: https://www.mathworks.com/help/fusion/ug/tuning-kalman-filter-to-improve-state-estimation.html
// and this playlist: https://www.youtube.com/watch?v=CaCcOwJPytQ&list=PLX2gX-ftPVXU3oUFNATxGXY90AULiqnWT
float kalmanMEA = 2;    // "measurement noise estimate" (larger = we assmume there is more noise/jitter in input)
float kalmanQ = 0.010;  // "Process Noise" AKA "gain" (smaller = more smoothing but more latency)

SimpleKalmanFilter motPotKalman(kalmanMEA, kalmanMEA, kalmanQ);
SimpleKalmanFilter ledPotKalman(kalmanMEA, kalmanMEA, kalmanQ);

#if (enableSlewPots)
SimpleKalmanFilter motSlewPotKalman(kalmanMEA, kalmanMEA, kalmanQ);
SimpleKalmanFilter ledSlewPotKalman(kalmanMEA, kalmanMEA, kalmanQ);
#endif

#if (enableShutterPots)
SimpleKalmanFilter shutBladesPotKalman(kalmanMEA, kalmanMEA, kalmanQ);
SimpleKalmanFilter shutAnglePotKalman(kalmanMEA, kalmanMEA, kalmanQ);
#endif

// For the FPSreal averaging
const int FPSnumReadings = 3;       // how many FPS readings to average together
float FPSAvgArray[FPSnumReadings];  // the FPS readings array
int FPSAvgReadIndex = 0;            // the index of the current reading
float FPSAvgTotal = 0;              // total of all readings


#if (enableButtons)
// Bounce2 Library setup for buttons
Button2 buttonA;
Button2 buttonB;

bool buttonAstate = 0;
bool buttonBstate = 0;
#endif

int motExtSwitch = 0;
volatile int motModeMidi;

int ledSlewVal = 0;
int ledSlewValOld;
int ledSlewMin = 0;      // the minumum slew value when knob is turned down (msec).
int ledSlewMax = 10000;  // the max slew value when knob is turned up (msec).

int motSlewVal = 0;
int motSlewValOld;
int motSlewMin = 0;      // the minumum slew value when knob is turned down (msec).
int motSlewMax = 10000;  // the max slew value when knob is turned up (msec).

int shutBladesPotVal;
int shutBladesVal;
int shutBladesValOld;
int shutAnglePotVal;
float shutAngleVal;
float shutAngleValOld;





// Ramp library setup
rampInt motAvg;  // ramp object for motor speed slewing
rampInt ledAvg;  // ramp object for LED brightness slewing


#if (enableStatusLed)
  // NeoPixel library setup
#define NUMPIXELS 1  // How many NeoPixels are attached?

Adafruit_NeoPixel pixels(NUMPIXELS, NeoPixelPin, NEO_RGB + NEO_KHZ800);
// last argument should match your LED type. Add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (5mm LEDS, v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
#endif

// UI VARS (could also be updated by remote control in future. Put these in a struct for easier radiolib management?)
int motPotVal = 0;  // current value of Motor pot (not necessarily the current speed since we might be ramping toward this value)
int motPotFPS = 1;  // current requested FPS based on motPotVal and scaling
int motPotFPSOld = 0;
int ledPotVal = 0;  // current value of LED pot (not necessarily the current LED brightness since we might be fading or strobing)
int ledPotValOld;



// LED VARS //
int LedDimMode = 1;               // 0 = current-controlled dimming (NOT YET IMPLEMENTED!), 1 = PWM dimming
int LedInvert = 1;                // set to 1 to invert LED output signal so it's active-low (required by H6cc driver board)
int ledBright = 0;                // current brightness of LED (range depends on Res below. If we're ramping then this will differ from pot value)
float safeSpeedMult = 0.4;        // multiplier to use in "safe mode" to lower lamp brightness at low FPS
const int ledBrightRes = 12;      // bits of resolution for LED dimming
const int ledBrightFreq = 18000;  // PWM frequency (500Hz is published max for H6cc LED driver, 770Hz is closer to shutter segment period, 1000 seems to work best)
const int ledChannel = 0;         // ESP32 LEDC channel number. Pairs share settings (0/1, 2/3, 4/5...) so skip one to insure your settings work!

// MOTOR VARS We are using ESP32 LEDC to drive the RC motor ESC via 1000-2000uS PWM @50Hz (standard servo format)
int motMode = 0;               // motor run mode (-1, 0, 1)
int motModePrev = 0;           // previous motor run mode (-1, 0, 1)
int motSingle = 0;             // indicates that we're traveling in single frame mode
int motSinglePrev = 0;         // were we in single frame mode during the last loop?
int motSingleMinTime = 800;    // the minimum movement time (MS) for each single frame move
volatile int motModeReal = 0;  // Current running direction detected by encoder (-1, 1)
float motSpeedUS = 1500;       // speed of motor (in pulsewidth uS from 1000-2000)
const int motPWMRes = 16;      // bits of resolution for extra control (standard servo lib uses 10bit)

const int motPWMChannel = 2;              // ESP32 LEDC channel number. Pairs share settings (0/1, 2/3, 4/5...) so skip one to insure your settings work!
const int motPWMFreq = 50;                // PWM frequency (50Hz is standard for RC servo / ESC)
int motPWMPeriod = 1000000 / motPWMFreq;  // microseconds per pulse

// prototypes for ISR functions that will be defined in later code
// (prototype must be declared _before_ we attach interrupt because ESP32 requires "IRAM_ATTR" flag which breaks typical Arduino behavior)
void IRAM_ATTR pinChangeISR();
void IRAM_ATTR send_LEDC();
// Start connection to the sensor.
AS5X47 as5047(EncCSN);
// will be used to read data from the magnetic encoder
ReadDataFrame readDataFrame;
volatile bool shutterMap[countsPerFrame];  // array holding values for lamp state at each position of digital shutter
volatile bool shutterStateOld = 0;         // stores the on/off state of the shutter from previous encoder position
static byte abOld;                         // Initialize state
volatile int count;                        // current rotary count
int countOld;                              // old rotary count
volatile int EncIndexCount;                // How many times have the A or B pulses transitioned while index pulse has been high
int as5047MagOK = 0;                       // status of magnet near AS5047 sensor
int as5047MagOK_old = 0;

// Machine Status Variables
volatile long frame = 0;      // current frame number (frame counter)
long frameOld = 0;            // old frame number for encoder
long frameOldsingle = 0;      // old frame number for encoder
volatile float FPScount = 0;  // measured FPS, sourced from encoder counts (100 updates per frame)
volatile float FPSframe = 0;  // measured FPS, sourced from frame counts
float FPSreal = 0;            // non-volatile FPS, sourced from encoder counts at slow speeds and frame counts at high speeds
float FPSrealAvg = 0;         // non-volatile measured FPS after averaging for jitter reduction
float FPStarget = 0;          // requested FPS


//external communication variables and settings//
int sfRequest;
int runRequest;

int frameCountResetRequest;
int opticalPrinter = 0;
int opAlignment = 0;
int capFlag = 0;
int mCopyStatus = 0;
int pullDownPos = 20;
int scanFlag = 0;

//ext midi variables -- may use CC2 for optical printer
int musicMode = 0;
int CC1ProjSpeed;
int CC3ProjBright;
int CC2ProjBlades;
int moveStatus;
int MSFStatus;
int motSingleRun;
int recSpecs;

int receivedRecvdConfirm2;
int receivedRecvdConfirm;





boolean newData = false;
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];

volatile float myFPScount = FPScount;      // copy volatile FPS to nonvolatile variable so it's safe
volatile float myFPSframe = FPSframe;      // copy volatile FPS to nonvolatile variable so it's safe
volatile int myMotModeReal = motModeReal;  // copy volatile FPS to nonvolatile variable so it's safe

// Settings for HobbyWing Quicrun Fusion SE motor
#define ESC_WRITE_BIT_TIME_WIDTH 2500  // uSec length of each pulse ("2500" = 400 baud)
char ESC_settings[] = {
  0,                // RPM Throttle Matching enabled
  2,                // 3S LiPo cells
  0,                // "Low" battery cutoff threshold (Low / Med / High)
  1,                // 105C temp cutoff
  0,                // CCW rotation
  0,                // 6V BEC
  0,                // Drag brake force disabled
  3,                // Drag brake rate 5
  3,                // Max reverse force 100%
  5, 0, 4, 2, 0, 0  // extra bytes sniffed with logic analyzer, don't know what they're for
};

// timers
elapsedMicros framePeriod;    // US since last frame transition (used for FPS calc)
elapsedMicros countPeriod;    // US since last encoder count transition (used for FPS calc)
elapsedMillis timerUI;        // MS since last time we checked/updated the user interface
elapsedMillis serialTimerUI;  //seperate operator for serial comm between aux display, midi, and self
elapsedMillis motSingleRunTimer;
elapsedMillis motModeMidiTimer;
elapsedMillis midiParseTimer;  // MS since last time we checked/updated the user interface
//elapsedMillis timerSingle;    // MS since single frame move began

/////////////////////////
//// ---> SETUP <--- ////
/////////////////////////

void setup() {

  pinMode(ledPin, OUTPUT);  // LEDC setup will take care of this later, but force it now just in case we're using current-controlled dimming
  digitalWrite(ledPin, 1);  // turn off LED during startup to prevent film burns

  Serial.begin(57600);  // Setup Serial Monitor


  // certain received command should force led on, but should maybe check for a physical switch of some sort?

  if (enableMotSwitch) {
    pinMode(motDirFwdSwitch, INPUT_PULLUP);
    pinMode(motDirBckSwitch, INPUT_PULLUP);
  }
#if (enableButtons)
  // Attach buttons (Bounce2 library handles pinmode settings for us)
  buttonA.begin(buttonApin, INPUT_PULLUP, true);
  buttonA.setDebounceTime(20);
  buttonA.setPressedHandler(pressed);
  buttonA.setReleasedHandler(released);

  buttonB.begin(buttonBpin, INPUT_PULLUP, true);
  buttonB.setDebounceTime(20);
  buttonB.setPressedHandler(pressed);
  buttonB.setReleasedHandler(released);
#endif

  if (enableSafeSwitch) {
    pinMode(safeSwitch, INPUT_PULLUP);
  }
  if (enableShutter) {
    pinMode(EncA, INPUT);
    pinMode(EncB, INPUT);
    pinMode(EncI, INPUT);
    attachInterrupt(EncA, pinChangeISR, CHANGE);
    attachInterrupt(EncB, pinChangeISR, CHANGE);
  }

  abOld = count = countOld = 0;

#if (enableStatusLed)
  pixels.begin();                // INITIALIZE NeoPixel object
  updateStatusLED(0, 30, 0, 0);  // start with LED red while booting
#endif

  Serial2.begin(57600, SERIAL_8N1, auxReceiver, auxTransmitter);

  delay(200);
  Serial.println("-----------------------------");
  Serial.println("SPECTRAL Projector Controller");
  Serial.println("-----------------------------");

// Program the ESC settings if user holds down buttonA during startup
#if (enableButtons)
  if (digitalRead(buttonApin) == 0) {
    ESCprogram();
    while (1)
      ;  // don't continue setup since the ESC needs to be rebooted before we can continue
  }
#endif



  // Set rotation direction (see AS5047 datasheet page 17)
  Settings1 settings1;
  settings1.values.dir = encoderDir;
  as5047.writeSettings1(settings1);

  // Set ABI output resolution (see AS5047 datasheet page 19)
  // (pulses per rev: 5 = 50 pulses, 6 = 25 pulses, 7 = 8 pulses)
  Settings2 settings2;
  settings2.values.abires = 6;  // 25 pulses per rev = 100 transitions. This is what we want.
  as5047.writeSettings2(settings2);

  // Disable ABI output when magnet error (low or high) exists (see AS5047 datasheet page 24)
  Zposl zposl;
  Zposm zposm;
  zposl.values.compLerrorEn = 1;
  zposl.values.compHerrorEn = 1;
  as5047.writeZeroPosition(zposm, zposl);

  // This command prints the debug information to the Serial port.
  // All registers of the encoder will be read and printed.
  //as5047.printDebugString();

  // LED PWM setup (BYPASSED because we set up the LEDC on each shutter blade now, and doing it here causes LED to come on during startup)
  // if (LedDimMode) {
  //   ledcSetup(ledChannel, ledBrightFreq, ledBrightRes);   // configure LED PWM function using LEDC channel
  //   ledcAttachPin(ledPin, ledChannel); // attach the LEDC channel to the GPIO to be controlled
  //   ledcWrite(ledChannel, 1); // turn it off
  // }

  // Motor PWM setup
  ledcSetup(motPWMChannel, motPWMFreq, motPWMRes);  // configure motor PWM function using LEDC channel
  ledcAttachPin(escPin, motPWMChannel);             // attach the LEDC channel to the GPIO to be controlled

  //it's important to send the ESC a "0" speed signal (1500us) whenever the motor is stopped. Otherwise the ESC goes into "failsafe" mode thinking that our RC car has lost contact with the TX!
  ledcWrite(motPWMChannel, (1 << motPWMRes) * 1500 / motPWMPeriod);  // duty = # of values at current res * US / pulse period
  Serial.print("Sending neutral signal to ESC...");
  delay(4000);
  Serial.println("Done");

#if (enableStatusLed)
  updateStatusLED(0, 18, 16, 10);  // white LED at idle
#endif

  // initialize all FPS readings to 0:
  for (int thisReading = 0; thisReading < FPSnumReadings; thisReading++) {
    FPSAvgArray[thisReading] = 0;
  }

  fixCount();                                     // at startup we don't know the absolute position via ABI, so ask SPI
  updateShutterMap(shutterBlades, shutterAngle);  //generate initial shutter map ... (1, 0.05 = 1 PPF and narrowest shutter angle)
}

/////////////////////////////////////////////
//// ---> THE LOOP (runs on core 1) <--- ////
/////////////////////////////////////////////

void loop() {

  // Time-critical IO happens in interrupts, but UI and decision-making happens at a slower pace in the main loop

  //update these functions @ 200 Hz
  //  if (timerFPS >= 5) {

  //    timerFPS = 0;
  //  }

  if (musicMode == 1) {
    midiParser();
    midiControlTakeover();
    readUI();
  }

  // update these functions @ 50 Hz
  if (timerUI >= 20) {
    as5047MagCheck();  // check for encoder magnet proximity
    calcFPS();
    if (musicMode == 0) {
      readUI();
    }
    updateMotor();
    updateLed();

    externalcontrol();






    timerUI = 0;
  }






  // These happen once per encoder count (only useful for debugging at slow speeds)
  if (countOld != count) {
    if (debugEncoder) {
      Serial.print("Frame: ");
      Serial.print(frame);
      Serial.print(", Frame Old: ");
      Serial.print(frameOldsingle);
      Serial.print(", Count: ");
      Serial.print(count);
      Serial.print(", Single: ");
      Serial.print(motSingle);
      Serial.print(", Lamp: ");
      Serial.print(shutterMap[count]);
      Serial.print(", Brightness: ");
      Serial.println(ledBright);
    }
    countOld = count;
  }

  // These happen once per frame (only useful for debugging)
  if (frameOld != frame) {
    if (debugFrames) {
      Serial.print("FRAME: ");
      Serial.print(frame);
      Serial.print(", FPSreal:");
      Serial.print(FPSreal);
      Serial.print(", FPS avg:");
      Serial.println(FPSrealAvg);
    }
    frameOld = frame;
  }
}

/////////////////////////////
//// ---> FUNCTIONS <--- ////
/////////////////////////////

// On interrupt, read input pins, compute new state, and adjust count
// This rotary encoder method counts all state changes (so 4x the number of "notches" on encoder) and is reliable with bouncy encoders
// From https://arduino.stackexchange.com/questions/16365/reading-from-a-ky-040-rotary-encoder-with-digispark/16420#16420
void IRAM_ATTR pinChangeISR() {
  enum { upMask = 0x66,
         downMask = 0x99 };
  byte abNew = (digitalRead(EncA) << 1) | digitalRead(EncB);
  byte criterion = abNew ^ abOld;
  // Execute if this is a valid transition on A or B
  if (criterion == 1 || criterion == 2) {
    //FPSreal = 10000.0 / countPeriod;  // update FPS calc based on period between the 100 encoder counts (not signed)
    //countPeriod = 0;

    // There are 3 A or B transitions during each Index pulse. (We count them here, then take action in a child loop.)
    if (digitalRead(EncI)) {
      EncIndexCount++;
    } else {
      EncIndexCount = 0;
    }
    // moving forwards ...
    if (upMask & (1 << (2 * abOld + abNew / 2))) {
      // at index
      if (EncIndexCount == 2) {  // reset counter on 'middle" transition during index condition
        count = 0;
        if (opticalPrinter == 0) {
          frame++;
        } else {
          frame--;
        }

        // frame++;
        FPSframe = 1000000.0 / framePeriod;  // update FPS calc based on period between each frame
        framePeriod = 0;
      } else {
        // normal forwards count
        FPScount = 10000.0 / countPeriod;  // update FPS calc based on period between the 100 encoder counts
        countPeriod = 0;
        motModeReal = 1;  // mark that we're running forwards
      }
      count++;
      // wrap around
      if (count > countsPerFrame - 1) {
        count = 0;
      }
    } else {
      // moving backwards ...
      if (EncIndexCount == 2) {  // reset counter on 'middle" transition during index condition
        // at index
        count = 0;
        if (opticalPrinter == 0) {
          frame--;
        } else {
          frame++;
        }
        FPSframe = 1000000.0 / framePeriod;  // update FPS calc based on period between each frame
        framePeriod = 0;
      } else {
        // normal backwards count
        FPScount = 10000.0 / countPeriod;  // update FPS calc based on period between the 100 encoder counts
        countPeriod = 0;
        motModeReal = -1;  // mark that we're running backwards
      }
      count--;
      // wrap around the circle instead of using negative steps
      if (count < 0) {
        count = countsPerFrame - 1;
      }
    }
  }
  abOld = abNew;  // Save new state

  // Update LED status for this encoder step

  bool shutterState = shutterMap[count];  // copy shutter state to local variable in case it changes during the ISR execution (not possible?)
  if (shutterState != shutterStateOld) {  // only update LED if shutter state changes (not every step)
    send_LEDC();                          // actual update code is abstracted so it can be run in different contexts
  }
  shutterStateOld = shutterState;  // store to global variable for next time
}


// send info to the LEDC peripheral to update LED PWM (abstracted here because it's called from loop or ISR)
void IRAM_ATTR send_LEDC() {
  bool shutterState = shutterMap[count];  // copy shutter state to local variable in case it changes during the ISR execution (not possible?)

  if (LedDimMode) {  // PWM mode
    if (shutterState == 1 || enableShutter == 0) {
      // LED ON for this step of shutter OR shutter is disabled OR single framing, so LED is always on
      ledcSetup(ledChannel, ledBrightFreq, ledBrightRes);  // configure LED PWM function using LEDC channel
      ledcAttachPin(ledPin, ledChannel);                   // attach the LEDC channel to the GPIO to be controlled
      if (LedInvert) {
        ledcWrite(ledChannel, (1 << ledBrightRes) - ledBright);  // set lamp to desired brightness (inverted)
      } else {
        ledcWrite(ledChannel, ledBright);  // set lamp to desired brightness
      }


    } else if (shutterState == 0 && enableShutter == 1) {
      // LED OFF for this segment of shutter

      if (LedInvert) {
        // ledcDetachPin(ledPin);    // detach the LEDC channel to the GPIO to be controlled
        digitalWrite(ledPin, 1);  // send pin high to turn off LED
      } else {
        // digitalWrite(ledPin, 0);  // send pin high to turn off LED
        ledcDetachPin(ledPin);  // detach the LEDC channel to the GPIO to be controlled
      }
      ledcDetachPin(ledPin);
    }

  } else {
    if (LedInvert) {
      if (enableShutter) {
        digitalWrite(ledPin, !(shutterState));  // active low, using shutter
        ledcDetachPin(ledPin);
      } else {
        digitalWrite(ledPin, 0);  // active low, no shutter
      }
    } else {
      if (enableShutter) {
        digitalWrite(ledPin, shutterState);  // active high, using shutter
      } else {
        digitalWrite(ledPin, 1);  // active high, no shutter
      }
    }
  }
}


// check for encoder magnet proximity
void as5047MagCheck() {

  // read magnet AGC data from sensor registers
  readDataFrame = as5047.readRegister(DIAGAGC_REG);
  Diaagc diaagc;
  diaagc.raw = readDataFrame.values.data;
  //Serial.println(diaagc.values.magl);

  // check result for magnet errors and update global var
  if (diaagc.values.magh || diaagc.values.magl) {
    as5047MagOK = 0;
  } else {
    as5047MagOK = 1;
  }

  // take action if global var has changed
  if (as5047MagOK_old != as5047MagOK) {
    if (as5047MagOK) {
      Serial.println("Magnet OK");
      fixCount();  // magnet is back after loss, so fix the count using SPI
    } else {
      Serial.println("Magnet ERROR");
      fixCount();  // magnet is back after loss, so fix the count using SPI
    }
    as5047MagOK_old = as5047MagOK;
  }
}

// Correct the encoder count by asking the sensor for real value over SPI
// This is required during setup since interrupt counts are relative but SPI is absolute
// Also useful if we ever need to recover from a temporary loss of magnet position
void fixCount() {
  count = map(as5047.readAngle(), 0, 360, 0, 100);
  Serial.println("   (Updated count via SPI)");
}

// fill shutterMap array with boolean values to control LED state at each position of shutter rotation
void updateShutterMap(byte shutterBlades, float shutterAngle) {
  //Serial.print("Update ShutterMap");
  // shutterBlades: number of virtual shutter blades (must be > 0)
  // shutterAngle: ratio between on/off for each shutter blade segment (0.5 = 180d)
  if (shutterBlades < 1) shutterBlades = 1;          // it would break if set to 0
  shutterAngle = constrain(shutterAngle, 0.0, 1.0);  // make sure it's 0-1
  for (int myBlade = 0; myBlade < shutterBlades; myBlade++) {
    int countOffset = myBlade * (countsPerFrame / shutterBlades);
    for (int myCount = 0; myCount < countsPerFrame / shutterBlades; myCount++) {
      if (myCount < countsPerFrame / shutterBlades * (1.0 - shutterAngle)) {
        shutterMap[myCount + countOffset] = 0;
      } else {
        shutterMap[myCount + countOffset] = 1;
      }
    }
  }
}

// Read user interface buttons and pots
// NOTE incurs 12ms blocking delay to help calm down crappy ADC between readings.
void readUI() {

  //if (onboardcontrol == 1) {

#if (enableButtons)
  buttonA.loop();  // update button managed by Bounce2 library
  buttonB.loop();  // update button managed by Bounce2 library


#endif

  // update each pot using Kalman filter to reduce horrible terrible awful ADC noise
  if (musicMode == 0) {
    motPotVal = motPotKalman.updateEstimate(analogRead(motPotPin));

    // delay(2);
    //given intervals to control optical printer timing/light. i am a simple man and im happier with less choices here. intervals of 409 are used.
    //this could be done as a formula probably. dunno!
    if (opticalPrinter == 1) {
      ledPotVal = ledPotKalman.updateEstimate(analogRead(ledPotPin));
      if (ledPotVal < 409) {
        ledPotVal = 409;
      } else if (ledPotVal > 409 && ledPotVal < 818) {
        ledPotVal = 818;
      } else if (ledPotVal > 818 && ledPotVal < 1227) {
        ledPotVal = 1227;
      } else if (ledPotVal > 1227 && ledPotVal < 1636) {
        ledPotVal = 1636;
      } else if (ledPotVal > 1636 && ledPotVal < 2045) {
        ledPotVal = 2045;
      } else if (ledPotVal > 2045 && ledPotVal < 2454) {
        ledPotVal = 2454;
      } else if (ledPotVal > 2454 && ledPotVal < 2863) {
        ledPotVal = 2863;
      } else if (ledPotVal > 2863 && ledPotVal < 3272) {
        ledPotVal = 3272;
      } else if (ledPotVal > 3272 && ledPotVal < 3681) {
        ledPotVal = 3681;
      } else if (ledPotVal > 3681 && ledPotVal < 4090) {
        ledPotVal = 4090;
      }
    } else if (opticalPrinter == 0) {
      ledPotVal = ledPotKalman.updateEstimate(analogRead(ledPotPin));
    }
    // delay(2);
  } else if (musicMode == 1) {
    motPotVal = map(CC1ProjSpeed, 0, 100, 0, 4095);
    ledPotVal = map(CC3ProjBright, 0, 100, 0, 4095);

    //inject values during Music Mode
  }

#if (enableSlewPots)
  motSlewVal = motSlewPotKalman.updateEstimate(analogRead(motSlewPotPin));
  // delay(2);
  ledSlewVal = ledSlewPotKalman.updateEstimate(analogRead(ledSlewPotPin));
  // delay(2);
#endif

  // load the hard-coded shutter vals in case the pots are disabled
  shutBladesVal = shutterBlades;
  shutAngleVal = shutterAngle;

#if (enableShutterPots && enableShutter)
  if (musicMode == 0) {
    shutBladesPotVal = shutBladesPotKalman.updateEstimate(analogRead(shutBladesPotPin));
    // delay(2);
  } else if (musicMode == 1) {
    shutBladesPotVal = map(CC2ProjBlades, 0, 100, 0, 4095);
    //inject values during Music Mode
  }
  shutAnglePotVal = shutAnglePotKalman.updateEstimate(analogRead(shutAnglePotPin));
  // delay(2);
  // using custom scaling for shutBladesPotVal to make control feel right
  if (shutBladesPotVal < 800) {
    shutBladesVal = 1;
  } else if (shutBladesPotVal < 2500) {
    shutBladesVal = 2;
  } else {
    shutBladesVal = 3;
  }
  shutAngleVal = mapf(shutAnglePotVal, 0, 4090, 0.1, 1.0);  // map ADC input to range of shutter angle
  shutAngleVal = constrain(shutAngleVal, 0.1, 1.0);         // clip values
#endif

#if (enableSafeSwitch)
  safeMode = digitalRead(safeSwitch);
#endif

  if (debugUI) {
    if (enableMotSwitch) {
      // print selector switch debug info , even though we aren't using it inside this function
      // Motor UI is switch + pot, so use normal pot scaling
      if (!digitalRead(motDirFwdSwitch)) {
        Serial.print("Mot For, ");
      } else if (!digitalRead(motDirBckSwitch)) {
        Serial.print("Mot Back, ");
      } else {
        Serial.print("Mot Stop, ");
      }
    }
    Serial.print("Mot Speed: ");
    Serial.print(motPotVal);
#if (enableSlewPots)
    Serial.print(", Mot Slew: ");
    Serial.print(motSlewVal);
#endif
    Serial.print(", Lamp Bright: ");
    Serial.print(ledPotVal);
#if (enableSlewPots)
    Serial.print(", Lamp Slew: ");
    Serial.print(ledSlewVal);
#endif
#if (enableShutterPots)
    Serial.print(", Shut Blade: ");
    Serial.print(shutBladesPotVal);
    Serial.print(", Shut Angle: ");
    Serial.print(shutAnglePotVal);
#endif
#if (enableSafeSwitch)
    Serial.print(", Safe Mode: ");
    Serial.print(safeMode);
#endif
    Serial.println("");
  }



  //}
  //else{

  //}
}

///////////////////////////////////
//// ---> SERIAL CONTROLS <--- ////
///////////////////////////////////

void externalcontrol() {

  Serial2.print("<");
  Serial2.print(FPSreal);
  Serial2.print(",");
  Serial2.print(shutAngleVal);
  Serial2.print(",");
  Serial2.print(ledPotVal);
  Serial2.print(",");
  Serial2.print(shutBladesPotVal);
  Serial2.print(",");
  Serial2.print(motMode);
  Serial2.print(",");
  Serial2.print(frame);
  Serial2.print(",");
  Serial2.print(motSingle);
  Serial2.print(",");
  Serial2.print(mCopyStatus);
  Serial2.println(">");
  // Serial.println("ok");

  if (musicMode == 0) {
    while (Serial2.available()) {
      char c = Serial2.read();
      if (c == 'g') {
        opticalPrinter = 1;  //i request this every time just to be sure that the proj understands!
        motSingle = -1;
      } else if (c == 'h') {
        opticalPrinter = 1;
        motSingle = 1;
      } else if (c == 'A') {
        capFlag = 1;
        scanFlag = 1;

        Serial2.println("A");  //mcopy confirmation flag)
      } else if (c == 'B') {
        capFlag = 0;
        scanFlag = 1;

        Serial2.println("B");  //mcopy confirmation flag)
      } else if (c == 'x') {
        frame = 0;
      } else if (c == 'u') {
        mCopyStatus = 0;
      } else if (c == 'G') {
        motSingle = -1;
      } else if (c == 'H') {
        motSingle = 1;
        Serial2.println('&');
      } else if (c == 'P') {
        motExtSwitch = 1;
      } else if (c == 'W') {
        motExtSwitch = -1;
      } else if (c == '*') {
        motExtSwitch = 0;
        motSingle = 0;
      } else if (c == '@') {
        musicMode = 1;  //midi
        Serial.println("music mode on");
      }
    }
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') {
      if (opticalPrinter == 0) {
        motSingle = 1;
      } else {
        motSingle = -1;
      }

    } else if (c == '2') {
      if (opticalPrinter == 0) {
        motSingle = -1;
      } else {
        motSingle = 1;
      }

    } else if (c == '-') {
      frame = 0;
      Serial.println("Projector Frame Count Reset!");
    } else if (c == '5') {
      motSingle = 5;
      Serial.println("Shutter OPEN!");
    } else if (c == '4') {
      motSingle = 4;
    } else if (c == 'f') {
      motExtSwitch = 1;
    } else if (c == 'p') {
      motExtSwitch = 0;
    } else if (c == 'r') {
      motExtSwitch = -1;
    } else if (c == '8') {

    } else if (c == '0') {
      Serial.println("Camera frame count reset!");
    } else if (c == '9') {
      Serial.print("Currently, the projector is on frame ");

      Serial.println(frame);
    } else if (c == ';') {
      opticalPrinter = 1;
      Serial.println("optical printer mode enabled! you must manually turn off LED if need be.");
    } else if (c == '`') {
      motSingle = 0;
      Serial.println("Shutter closed.");
    } else if (c == '@') {
      musicMode = 1;  //midi
      Serial.println("music mode on");
    }
  }

  // Serial.print("SPEED ");
  // Serial.println(CC1ProjSpeed);
  // Serial.print("BLADE ");
  // Serial.println(CC2ProjBlades);
  // Serial.print("LED ");
  // Serial.println(CC3ProjBright);
  // Serial.print("sf ");
  // Serial.println(MSFStatus);
  // Serial.print("move ");
  // Serial.println(moveStatus);

  // Serial.print(Serial2.read());   // read it and send it out Serial (USB)
}

//  if (serialtimerUI >= 400) {

void midiControlTakeover() {

  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == 'H') {
      motSingle = 1;
      Serial.println("RECVD");
      Serial2.println('&');
    } else if (c == 'G') {
      motSingle = -1;
      Serial.println("RECVD");
      Serial2.println('&');
    } else if (c == '@') {
      musicMode = 1;  //midi
      Serial.println("music mode on");
    } else if (c == 'L') {
      motExtSwitch = 1;
      motMode = 1;
      //Serial.println("RECVD");
      Serial2.println('$');

    } else if (c == '*') {
      updateShutterMap(0, 0.0);  // force open shutter for single framing

      motExtSwitch = 0;
      motMode = 0;
      receivedRecvdConfirm = 1;
      Serial2.println('$');
      updateShutterMap(shutBladesVal, shutAngleVal);

    } else if (c == 'E') {
      motExtSwitch = -1;
      motMode = -1;
      Serial.println("RECVD");
      Serial2.println('$');


    } else if (c == 'W') {
      receivedRecvdConfirm2 = 1;
    }
  }

  if (receivedRecvdConfirm == 1) {
    if (receivedRecvdConfirm2 == 0) {
      Serial2.println('$');
    } else if (receivedRecvdConfirm2 == 1) {
      receivedRecvdConfirm2 = 0;
      receivedRecvdConfirm = 0;
      // Serial.println("ok");
    }
  }
}





void midiParser() {
  if (midiParseTimer > 50) {
    recvWithStartEndMarkers();
    if (newData == true) {
      strcpy(tempChars, receivedChars);
      sscanf(tempChars, "%d,%d,%d", &CC1ProjSpeed, &CC2ProjBlades, &CC3ProjBright);
      //    showParsedData();
      newData = false;
      midiParseTimer = 0;
    }
  }
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial2.available() > 0 && newData == false) {
    rc = Serial2.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';  // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseData() {


  // char * strtokIndx; // this is used by strtok() as an index
  // char * pointer;

  // strtokIndx = strtok_r(tempChars, ",", &pointer);      // get the first part - the string
  // CC1ProjSpeed = atoi(strtokIndx);     // convert this part to an integer

  // strtokIndx = strtok_r(NULL, ",", &pointer);      // get the first part - the string
  // CC2ProjBlades = atoi(strtokIndx);     // convert this part to an integer

  // strtokIndx = strtok_r(NULL, ",", &pointer);      // get the first part - the string
  // CC3ProjBright = atoi(strtokIndx);     // convert this part to an integer

  // strtokIndx = strtok_r(NULL, ",", &pointer);      // get the first part - the string
  // moveStatus = atoi(strtokIndx);     // convert this part to an integer


  // strtokIndx = strtok_r(NULL, ",", &pointer);      // get the first part - the string
  // MSFStatus = atoi(strtokIndx);     // convert this part to an integer



  // for(int i = 0; i < 25; i++)
  // {
  //   Serial.println(receivedChars[i]);
  // }


  //   char * str;
  // char * p = tempChars; //example "hello,123,3.14"
  // byte counter = 0;
  // // delay(20);
  // while ((str = strtok_r(p, " x, ", &p)) != NULL)  // Don't use \n here it fails
  //   {
  //    switch(counter){
  //     case 0:
  //     CC1ProjSpeed = atoi(str);

  //     break;

  //     case 1:
  //     CC2ProjBlades = atoi(str);

  //     break;

  //     case 2:
  //     CC3ProjBright = atoi(str);

  //     break;

  //     case 3:
  //     moveStatus = atoi(str);

  //     break;


  //     case 4:
  //     MSFStatus = atoi(str);

  //     break;

  //    }

  //    counter++;
  //   }
}



// compute the real FPS, based on encoder rotation, but averaged to reduce error
void calcFPS() {
  noInterrupts();
  myFPScount = FPScount;        // copy volatile FPS to nonvolatile variable so it's safe
  myFPSframe = FPSframe;        // copy volatile FPS to nonvolatile variable so it's safe
  myMotModeReal = motModeReal;  // copy volatile FPS to nonvolatile variable so it's safe
  interrupts();

  if (myFPScount > 5) {
    FPSreal = myFPSframe;
  } else {
    FPSreal = myFPScount;
  }

  FPSreal = FPSreal * FPSmultiplier;  // convert the FPS for the P26

  FPSAvgTotal = FPSAvgTotal - FPSAvgArray[FPSAvgReadIndex];  // subtract the last reading
  FPSAvgArray[FPSAvgReadIndex] = FPSreal;                    // store the new reading
  FPSAvgTotal = FPSAvgTotal + FPSAvgArray[FPSAvgReadIndex];  // add the reading to the total
  FPSAvgReadIndex = FPSAvgReadIndex + 1;                     // advance to the next position in the array
  if (FPSAvgReadIndex >= FPSnumReadings) {                   // if we're at the end of the array...
    FPSAvgReadIndex = 0;                                     // ...wrap around to the beginning:
  }
  FPSrealAvg = FPSAvgTotal / FPSnumReadings;  // calculate the average
  FPSrealAvg = FPSrealAvg * myMotModeReal;    // add sign based on film travel direction

  // If digital shutter is enabled but no encoder movement in last 200 msec, assume that we're stopped.
  if (countPeriod > 200000 && enableShutter) {
    FPSrealAvg = 0;
  }
  float FPSunsigned = abs(FPSrealAvg);
  if (FPSunsigned > 23.5 && FPSunsigned < 24.5) {
    updateStatusLED(0, 0, 30, 0);  // green LED at 24fps
  } else if (FPSunsigned > 17.5 && FPSunsigned < 18.5) {
    updateStatusLED(0, 20, 0, 16);  // purple LED at 18fps
  } else {
    updateStatusLED(0, 18, 16, 10);  // white LED at idle
  }
}

// compute LED brightness (note that the ISR ultimately controls the LED state if enableShutter = 1)
void updateLed() {
  //noInterrupts();

  if (abs(shutBladesVal != shutBladesValOld) || abs(shutAngleVal - shutAngleValOld) >= 0.05) {
    if (enableShutter == 1) {
      //Serial.println("(SHUTTERMAP SHUTTER POTS)");
      updateShutterMap(shutBladesVal, shutAngleVal);
    }
    shutBladesValOld = shutBladesVal;
    shutAngleValOld = shutAngleVal;
  }

#if (enableSlewPots)
  ledSlewVal = map(ledSlewVal, 0, 4095, ledSlewMin, ledSlewMax);  // turn slew val pot into ms ramp time
#endif
  ledAvg.update();  // LED slewing managed by Ramp library
  // if knobs have changed sufficiently, calculate new slewing ramp time
  if (musicMode == 0) {
    if (abs(ledSlewVal - ledSlewValOld) >= 50 || abs(ledPotVal - ledPotValOld) >= 50) {
      //Serial.println("(LED UPDATE SLEW)");
      ledAvg.go(ledPotVal, ledSlewVal);  // set next ramp interpolation in ms
      ledSlewValOld = ledSlewVal;
      ledPotValOld = ledPotVal;
    }

    // set brightness to slewed version of pot value (mapped/clipped because kalman filter sometimes doesn't allow us to reach min/max)
    ledBright = map(ledAvg.getValue(), 40, 4045, ledMin, ledMax);
    ledBright = constrain(ledBright, 0, 4095);
  } else if (musicMode == 1) {

    if (abs(ledSlewVal - ledSlewValOld) >= 50 || abs(ledPotVal - ledPotValOld) >= 50) {
      //Serial.println("(LED UPDATE SLEW)");
      ledAvg.go(ledPotVal, ledSlewVal);  // set next ramp interpolation in ms
      ledSlewValOld = ledSlewVal;
      ledPotValOld = ledPotVal;
    }

    ledBright = map(ledAvg.getValue(), 0, 100, ledMin, ledMax);

    ledBright = constrain(ledBright, 0, 4095);
  }

  // SAFE MODE CHECK
  if (safeMode == 1) {
    if (motSingle != 0) {
      // We're in one of the single modes, so ...
      ledBright = ledBright * safeMin;  // dim lamp to min safe brightness immediately
    } else {
      safeSpeedMult = fscale(4, 24.0, safeMin, 1.0, abs(FPSrealAvg), 0);  // safety multiplier for FPS (with optional nonlinear scaling)
      safeSpeedMult = constrain(safeSpeedMult, safeMin, 1.0);             // clip values
      ledBright = ledBright * safeSpeedMult;                              // decrease brightness to prevent film burns
    }
  }

  if ((opticalPrinter == 1 || scanFlag == 1) && capFlag == 1) {
    ledBright = ledBright;
    updateShutterMap(1, 1.0);  // force open shutter for single framing
    digitalWrite(ledPin, 1);
    send_LEDC();
  } else if ((opticalPrinter == 1 || scanFlag == 1) && capFlag == 0) {
    ledBright = 0;
    send_LEDC();
    //this is for emulating a capper, so that i can work with intervals in Mcopy
  }

  // STOPPED CHECK (Turn off lamp if projector is stopped, except when single buttons are pressed or single is active)
  if (opticalPrinter == 0 && scanFlag == 0) {
    if (FPSrealAvg == 0 && motSingle == 0 && !buttonAstate && !buttonBstate) {
      ledBright = 0;
    }
    //Serial.print("STOP CHECK ");
  }




  //interrupts();

  if (debugLed) {
    Serial.print("LED Slew: ");
    Serial.print(ledSlewVal);
    Serial.print(", LED Pot: ");
    Serial.print(ledPotVal);
    Serial.print(", Spd Mult: ");
    Serial.print(safeSpeedMult);
    Serial.print(", LED Bright: ");
    Serial.print(ledBright);
    Serial.print(", Shutter Blades: ");
    Serial.print(shutBladesVal);
    Serial.print(", Shutter Angle: ");
    Serial.println(shutAngleVal);
  }
  // at slow speeds OR if the shutter is fully open OR shutter disabled, update the LED PWM directly because ISR isn't firing often (or at all)
  if (countPeriod > 50000 || shutAngleVal == 1.0 || !enableShutter || musicMode == 1) {
    send_LEDC();
  }
}

void updateMotor() {

  // mapped/clip motPotVal because kalman filter sometimes doesn't allow us to reach min/max)
  int motPotClipped = map(motPotVal, 40, 4045, 0, 4095);
  motPotClipped = constrain(motPotClipped, 0, 4095);

  // Depending on the motor direction switch, we translate the pot ADC to FPS with optional negative scaling
  // (The Ramp library is currently set up for ints, so our float FPS is multiplied by 100 to make it int 2400)
  // The switch and pot combinations are abstracted into MotMode and motPotFPS variables

  if (enableMotSwitch) {
    if (motSwitchMode == 1) {
      // Motor UI = Eiki: FWD/OFF/REV switch + pot, so use normal slow-24fps pot scaling
      if (!digitalRead(motDirFwdSwitch) || motExtSwitch == 1) {
        // FORWARD
        motPotFPS = mapf(motPotClipped, 0, 4095, 10, 2400);  // convert mot pot value to FPS x 100
        motMode = 1;
      } else if (!digitalRead(motDirBckSwitch) || motExtSwitch == -1) {
        // REVERSE
        motPotFPS = mapf(motPotClipped, 0, 4095, -10, -2400);  // convert mot pot value to FPS x 100
        motMode = -1;
      } else {
        // STOP
        motPotFPS = 0;
        motMode = 0;
        if (opticalPrinter == 0 && motMode == 0) {
          ledBright = 0;
          send_LEDC();
        }
      }
    } else {
      // Motor UI = P26: RUN/STOP switch, FWD/REV switch, and pot, so use normal 0-24fps pot scaling
      // First we test run/stop switch, then motor direction switch
      if (!digitalRead(motDirFwdSwitch)) {
        // RUN
        if (!digitalRead(motDirBckSwitch)) {
          // FORWARD
          motPotFPS = mapf(motPotClipped, 0, 4095, 10, 2400);  // convert mot pot value to FPS x 100
          motMode = 1;

        } else {
          // REVERSE
          motPotFPS = mapf(motPotClipped, 0, 4095, -10, -2400);  // convert mot pot value to FPS x 100
          motMode = -1;
        }
      } else {
        // STOP
        motPotFPS = 0;
        motMode = 0;
      }
    }
  } else {
    // Motor UI is only pot, so use Use FORWARD-STOP-BACK pot scaling with deadband in center
    int bandWidth = 500;  // width of "deadband" in middle of pot range where speed is forced to 0
    int bandMin = 2048 - bandWidth / 2;
    int bandMax = 2048 + bandWidth / 2;

    if (motPotClipped < bandMin) {
      motPotFPS = mapf(motPotClipped, 0, bandMin, -2400, 0);
    } else if (motPotClipped > bandMax) {
      motPotFPS = mapf(motPotClipped, bandMin, 4095, 0, 2400);
    } else {
      motPotFPS = 0;
    }
  }
  if (musicMode == 1) {
    if (motExtSwitch == 0 || motSwitchMode == 0) {
      motPotFPS = 0;
      motMode = 0;
    }
  }


#if (enableSlewPots)
  motSlewVal = map(motSlewVal, 0, 4095, motSlewMin, motSlewMax);  // turn slew val pot into ms ramp time
#endif
  motAvg.update();  // Motor slewing managed by Ramp library
  // if knobs have changed sufficiently, calculate new slewing ramp time
  if (abs(motSlewVal - motSlewValOld) >= 10 || abs(motPotFPS - motPotFPSOld) >= 10) {
    //Serial.println("(MOT RAMP CALC)");
    motAvg.go(motPotFPS, motSlewVal);  // set next ramp interpolation in ms
    motSlewValOld = motSlewVal;
    motPotFPSOld = motPotFPS;
  }

  float FPStemp = motAvg.getValue() / 100.0;  // use slewed value for target FPS (dividing by 100 to get floating point FPS)

  // Add more natural scaling when we translate from pot to actual FPS
  // These values may be negative, but fscale only handles positive values, so...

  if (FPStemp < 0.0) {
    // negative FPS values
    FPStemp = FPStemp * -1.0;                                       // make it positive before fscale function
    FPStarget = (fscale(0.0, 24.0, 0.0, 24.0, FPStemp, 3) * -1.0);  // reverse log scale and turn it negative again
  } else {
    FPStarget = fscale(0.0, 24.0, 0.0, 24.0, FPStemp, 3);  // number is positive so just reverse log scale it
  }

  /////////////////////
  // SINGLE FRAME CODE
  /////////////////////

  // Check if we're in single frame mode and assert control over things

  if (opticalPrinter == 1) {
    pullDownPos = 20;
  } else if (opticalPrinter == 0) {
    pullDownPos = 20;
  }  //in case you want pin registration, though the eiki pin registration seems to be unreliable since the microcontroller does not check for the claw position fast enough believe it or not

  if (motSingle == 1) {
    // EIKI SINGLE FRAME FORWARD
    if (motSingle != motSinglePrev) {
      Serial.println("SINGLE FORWARD MOVE START");
      frameOldsingle = frame;  // log the current frame when we began the single frame move
      motSinglePrev = motSingle;
      updateShutterMap(1, 1.0);  // force open shutter for single framing
    }
    if (frameOldsingle != frame && count > pullDownPos) {  // keep out of pulldown in 0-13 zone, so try to land around 50
      // we're ready to show frame
      Serial.println("SINGLE FORWARD MOVE DONE");
      FPStarget = 0;  // stop
      if (opticalPrinter == 0) {
        motSingle = 0;  // turn off single flag
      } else if (opticalPrinter == 1 && opAlignment == 1) {
        Serial.print("Projector is on frame "),
          Serial.println(frame);

        motSingle = 3;
        opAlignment = 0;
        mCopyStatus = 1;  //mcopy confirmation flag
                          //frame++;
        motSinglePrev = motSingle;
        frameOldsingle = frame;
      } else {
        mCopyStatus = 1;  //mcopy confirmation flag
                          //frame--;
        motSingle = 3;
      }
      Serial.print("Projector is on frame "),
        Serial.println(frame);
      motSinglePrev = motSingle;
      frameOldsingle = frame;
      //updateShutterMap(shutBladesVal, shutAngleVal); // return to normal
    } else {
      // travel to the next frame
      if (musicMode == 0) {
        FPStarget = singleFPS * 1;  // jam in preset speed
      } else if (musicMode == 1) {
        FPStarget = 8;  // jam in preset speed
      }
    }
  } else if (motSingle == -1) {
    // EIKI SINGLE FRAME BACKWARD
    if (motSingle != motSinglePrev) {
      Serial.println("SINGLE BACKWARD MOVE START");
      frameOldsingle = frame;  // log the current frame when we began the single frame move
      motSinglePrev = motSingle;
      updateShutterMap(1, 1.0);  // force open shutter for single framing
    }
    if (frameOldsingle != frame && count < 80) {  // try to land around 50
      // we're ready to show frame
      Serial.println("SINGLE BACKWARD MOVE DONE");
      FPStarget = 0;  // stop
      if (opticalPrinter == 0) {
        motSingle = 0;  // turn off single flag
      } else if (opAlignment == 0) {
        motSingle = -1;
        opAlignment = 1;
        motSinglePrev = motSingle;
        frameOldsingle = frame;
      } else if (opAlignment == 1 && motSingle == -1) {
        motSingle = 1;

        motSinglePrev = motSingle;
        frameOldsingle = frame;
      }
      Serial.print("Projector is on frame "),
        Serial.println(frame);
      // Serial2.println("h"); //mcopy confirmation flag
      motSinglePrev = motSingle;
      frameOldsingle = frame;
      //updateShutterMap(shutBladesVal, shutAngleVal); // return to normal
    } else {
      // travel to the next frame
      if (musicMode == 0) {
        FPStarget = singleFPS * -1;  // jam in preset speed
      } else if (musicMode) {
        FPStarget = -8;  // jam in preset speed
      }
    }
  } else if (motSingle == 2) {
    // Eiki freeze frame (either button pressed while motor was running)
    Serial.println("EIKI FREEZE FRAME");
    FPStarget = 0;
    updateShutterMap(1, 1.0);  // force open shutter for single framing
    // ledBright = ledBright * safeMin;
    safeMode = 1;
    send_LEDC();
  } else if (motSingle == 3) {
    // P26 "BURN" BUTTON special case
    //    Serial.println("P26 BURN MODE (open shutter)");
    updateShutterMap(1, 1.0);  // force open shutter for single framing
    send_LEDC();
  } else if (motSingle == 4) {
    ledcWrite(ledChannel, (ledBright / 2));
  } else if (motSingle == 5) {
    ledBright = 4096;
    ledcWrite(ledChannel, (ledBright));
  }


// Transform FPStarget into motor microseconds using choice of 2 methods
#if motorSpeedMode

  // USE CLOSED LOOP motor control with feedback from encoder (not working well enough to use for Spectral)
  float FPSdiff = abs(FPStarget - FPSrealAvg);
  if (FPStarget > 6) {
    if (FPSdiff < 2) FPSdiff = FPSdiff * 0.2;  // make changes much smaller when close to setpoint, but only at higher speeds so we can get started from stopped condition
  }
  float TESTmotSpeedUS;
  if (FPStarget == 0) {
    TESTmotSpeedUS = 1500;
  } else if (FPStarget < FPSrealAvg) {
    TESTmotSpeedUS = motSpeedUS + FPSdiff;
  } else if (FPStarget > FPSrealAvg) {
    TESTmotSpeedUS = motSpeedUS - FPSdiff;
  }
  motSpeedUS = TESTmotSpeedUS;
  motSpeedUS = constrain(motSpeedUS, 1200, 1800);  // prevent runaway in case of broken belt or other disaster
#else

  // USE HARD-CODED SPEED
  float TESTmotSpeedUS;

  if (FPStarget == 0) {
    TESTmotSpeedUS = 1500;
  } else if (FPStarget > 0) {
    if (opticalPrinter == 0) {
      TESTmotSpeedUS = mapf(FPStarget, 0, 24, 1500 - minUSoffset, motMaxUS);
    } else if (opticalPrinter == 1) {
      TESTmotSpeedUS = mapf(FPStarget, 0, 24, 1500 - minUSoffsetPrinting, motMaxUS);
    }
  } else if (FPStarget < 0) {

    if (opticalPrinter == 0) {
      TESTmotSpeedUS = mapf(FPStarget, 0, -24, 1500 + minUSoffset, motMinUS);
    } else if (opticalPrinter == 1) {
      TESTmotSpeedUS = mapf(FPStarget, 0, -24, 1500 + minUSoffsetPrinting, motMinUS);
    }
  }


  //note that to calculate min speed it is 1500 - measured "Mot uS" -- however, target speed is not spit out correctly!!!
  motSpeedUS = TESTmotSpeedUS;
  //motSpeedUS = mapf(FPStarget, -24.0, 24.0, motMinUS, motMaxUS);  // basic method without enforcing minumum speed
#endif

  int motDuty = (1 << motPWMRes) * motSpeedUS / motPWMPeriod;  // convert pulse width to PWM duty cycle (duty = # of values at current res * US / pulse period)
  ledcWrite(motPWMChannel, motDuty);                           // update motor speed
  if (debugMotor) {
    Serial.print("Mot Mode: ");
    Serial.print(motMode);
    Serial.print(", Single Mode: ");
    Serial.print(motSingle);
    Serial.print(", Mot Slew: ");
    Serial.print(motSlewVal);
    Serial.print(", FPS Target: ");
    Serial.print(FPStarget);
    Serial.print(", FPS Real Avg: ");
    Serial.print(FPSrealAvg);
    Serial.print(", Mot uS: ");
    Serial.print(motSpeedUS);
    Serial.print(", Mot PWM: ");
    Serial.println(motDuty);
  }
  if (debugFPSgraph) {
    Serial.print("-24, ");
    Serial.print(FPSrealAvg);
    Serial.println(", 24");
  }
}

#if (enableButtons)
void pressed(Button2& btn) {
  if (btn == buttonA) {
    if (buttonAstate == 0) {
      if (debugUI) {
        Serial.println("Button A pressed");
      }
      buttonAstate = 1;
      if (motSwitchMode) {
        // Eiki motor switch mode, so we have a button for each single frame direction
        if (motMode == 0) {
          motSingle = 1;  // If stopped when button was pressed, set single frame fwd
        } else {
          motSingle = 2;  // Set freeze frame
        }
      } else {
        // P26 motor switch mode with one button, so we use a specific mode
        motSingle = 3;
      }
    }
  } else if (btn == buttonB) {
    if (buttonBstate == 0) {
      if (debugUI) {
        Serial.println("Button B pressed");
      }
      buttonBstate = 1;
      if (motSwitchMode) {
        // Eiki motor switch mode, so we have a button for each single frame direction
        // P26 doesn't have a buttonB, so need to test further
        if (motMode == 0) {
          motSingle = -1;  // If stopped when button was pressed, set single frame rev
        } else {
          motSingle = 2;  // Set freeze frame
        }
      }
    }
  }
}

void released(Button2& btn) {
  if (btn == buttonA) {
    if (buttonAstate == 1) {

      if (debugUI) {
        Serial.println("Button A released");
      }
      if (motSingle == 2 || motSingle == 3) motSingle = 0;  // turn off single flag if we're leaving Eiki freeze-frame mode or P26 open shutter mode
      motSinglePrev = motSingle;
      buttonAstate = 0;
      updateShutterMap(shutBladesVal, shutAngleVal);  // return shutter map to normal
      Serial.print("Leaving Eiki / P26 FREEZE/BURN mode... motSingle = ");
      Serial.println(motSingle);
      if (motSingle == 2 || motSingle == 3) motSingle = 0;  // turn off single flag if we're leaving Eiki freeze-frame mode or P26 open shutter mode
      motSinglePrev = motSingle;
      buttonAstate = 0;
      updateShutterMap(shutBladesVal, shutAngleVal);  // return shutter map to normal
      Serial.print("Leaving Eiki / P26 FREEZE/BURN mode... motSingle = ");
      Serial.println(motSingle);
    }
  } else if (btn == buttonB) {
    if (buttonBstate == 1) {
      if (debugUI) {
        Serial.println("Button B released");
      }
      if (motSingle == 2 || motSingle == 3) motSingle = 0;  // turn off single flag if we're leaving Eiki freeze-frame mode or P26 open shutter mode
      motSinglePrev = motSingle;
      buttonBstate = 0;
      updateShutterMap(shutBladesVal, shutAngleVal);  // return shutter map to normal
      Serial.print("Leaving Eiki / P26 FREEZE/BURN mode... motSingle = ");
      Serial.println(motSingle);
    }
  }
}
#endif

// ESC programming via Arduino, for HobbyWing Quicrun Fusion SE motor
void ESCprogram() {
  // adapted from these sources:
  // https://www.rcgroups.com/forums/showthread.php?1454178-Reversed-Engineered-a-ESC-Programming-Card
  // https://www.rcgroups.com/forums/showthread.php?1567736-Program-configure-ESC-with-Arduino

  // (ESC settings chart and LED programmer use options 1-9 but the data is 0-8)
  // BEC power turns on 7 ms after power is applied.
  // If program data line is pulled high at boot, ESC goes into program mode ...
  // ESC sends init message 140 ms after power is applied, then waits for ACK from programmer (responds about 1200 ms after init msg starts)
  // if no ACK within 4 sec from init message start, ESC continues to boot (motor winding beeps for LiPo count then periodic single pulses on programmer line)
  // if ACK, then boot pauses indefinitely, waiting for settings from programmer
  // After programmer sends settings, ESC responds with ACK and pauses indefinitely
  // Then ESC power must be toggled to reboot with new settings. (There is no way for programmer to send a "reboot" command)

#if (enableStatusLed)
  updateStatusLED(0, 32, 0, 0);  // red LED while we wait for programming
#endif

  Serial.println("Waiting for HobbyWing ESC ...");

  // wait for beginning of init message from ESC
  pinMode(escProgramPin, OUTPUT);
  digitalWrite(escProgramPin, HIGH);
  pinMode(escProgramPin, INPUT_PULLUP);
  while (digitalRead(escProgramPin) == 0)
    ;
  while (digitalRead(escProgramPin) == 1)
    ;
  while (digitalRead(escProgramPin) == 0)
    ;
  while (digitalRead(escProgramPin) == 1)
    ;
#if (enableStatusLed)
  updateStatusLED(0, 20, 5, 0);  // orange LED
#endif
  Serial.println("ESC booted");
  delay(1300);  // wait for ESC init message to pass before sending ACK (we don't parse it, just waiting for it to pass)

#if (enableStatusLed)
  updateStatusLED(0, 32, 0, 0);  // red LED
#endif
  Serial.println("Sending ACK...");
  ESC_send_ACK();  // tell ESC that we're here
  delay(100);      // wait before sending settings

#if (enableStatusLed)
  updateStatusLED(0, 20, 5, 0);  // orange LED
#endif
  Serial.println("Sending Settings...");
  for (byte i = 0; i < sizeof(ESC_settings); i++) {
    ESC_ser_write(ESC_settings[i]);
  }
  ESC_send_ACK();  // tell ESC that we're done
#if (enableStatusLed)
  updateStatusLED(0, 0, 32, 0);  // green LED
#endif
  Serial.println("DONE");
}

// writes a byte to a psuedo 10-bit UART
void ESC_ser_write(unsigned char x) {
  digitalWrite(escProgramPin, HIGH);  // make sure
  pinMode(escProgramPin, OUTPUT);
  delayMicroseconds(ESC_WRITE_BIT_TIME_WIDTH);

  digitalWrite(escProgramPin, LOW);  // signal start
  delayMicroseconds(ESC_WRITE_BIT_TIME_WIDTH);

  digitalWrite(escProgramPin, HIGH);  // first bit always 1
  delayMicroseconds(ESC_WRITE_BIT_TIME_WIDTH);

  digitalWrite(escProgramPin, LOW);  // 2nd bit always 0
  delayMicroseconds(ESC_WRITE_BIT_TIME_WIDTH);

  // send the byte LSB first
  char i;
  for (i = 0; i < 8; i++) {
    if ((x & (1 << i)) == 0) {
      digitalWrite(escProgramPin, LOW);
    } else {
      digitalWrite(escProgramPin, HIGH);
    }
    delayMicroseconds(ESC_WRITE_BIT_TIME_WIDTH);
  }
  digitalWrite(escProgramPin, HIGH);  // leave as input
  pinMode(escProgramPin, OUTPUT);
}

// must be sent after receiving configuration from ESC upon initialization
void ESC_send_ACK() {
  digitalWrite(escProgramPin, HIGH);  // make sure
  pinMode(escProgramPin, OUTPUT);     // assert pin
  delayMicroseconds(ESC_WRITE_BIT_TIME_WIDTH);
  // send pulses
  char i;
  for (i = 0; i < 6; i++) {
    digitalWrite(escProgramPin, LOW);  // signal start
    delayMicroseconds(ESC_WRITE_BIT_TIME_WIDTH);

    digitalWrite(escProgramPin, HIGH);  // first bit always 1
    delayMicroseconds(ESC_WRITE_BIT_TIME_WIDTH);
  }
  pinMode(escProgramPin, INPUT_PULLUP);  // float pin
}

#if (enableStatusLed)
void updateStatusLED(int x, int R, int G, int B) {
  // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
  pixels.setPixelColor(x, pixels.Color(R, G, B));
  pixels.show();  // Send the updated pixel colors to the hardware.
}
#endif

// map function like Arduino core, but for floats
double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/* fscale https://playground.arduino.cc/Main/Fscale/
 Floating Point Autoscale Function V0.1
 Paul Badger 2007
 Modified from code by Greg Shakar
 - curve param is linear at 0, log(ish) > 0
 
 */
float fscale(float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve) {

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1);   // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve);  // convert linear scale into lograthimic exponent for other pow function

  /*
   Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   Serial.println();
   */

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin) {
    NewRange = newEnd - newBegin;
  } else {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal = zeroRefCurVal / OriginalRange;  // normalize to 0 - 1 float

  /*
  Serial.print(OriginalRange, DEC);  
   Serial.print("   ");  
   Serial.print(NewRange, DEC);  
   Serial.print("   ");  
   Serial.println(zeroRefCurVal, DEC);  
   Serial.println();  
   */

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax) {
    return 0;
  }

  if (invFlag == 0) {
    rangedValue = (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  } else  // invert the ranges
  {
    rangedValue = newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;
}