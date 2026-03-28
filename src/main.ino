//i've moved a lot around. biggest change i had to make was making certain variables global like those at the beginning of CalcFPS. if i didnt make this change i would get crashing if the projector was running for a Long Time. debug stated it was fetching a value before the value was even written. this is probably a result of additional tasks needing to happen during the main loop. splitting operations out to multicore would maybe help, or using the ESP32-s3 with additional SRAM

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


// TODO LIST:
// move "loop" code into freertos tasks
// how do I make the creation of such into a function?
// id assume with FREERTOS that one wouldn't need to put anything into loop. 
// look into libraries which use FREERTOS tasks 

// Include the libraries
// NOTE: In 2023 this code was developed using the ESP32 Arduino core v2.0.9.
// When ESP32 Arduino core v3.x is released there will be breaking changes because LEDC setup will be different!
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>   // https://github.com/adafruit/Adafruit_NeoPixel

#include <AS5X47.h>              // https://github.com/adrien-legrand/AS5X47
#include <elapsedMillis.h>       // https://github.com/pfeerick/elapsedMillis
#include <Ramp.h>                // https://github.com/siteswapjuggler/RAMP
#include <SimpleKalmanFilter.h>  // https://github.com/denyssene/SimpleKalmanFilter
#include <Button2.h>             // https://github.com/LennartHennigs/Button2

#include <FastInterruptEncoder.h>

#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"


// Uncomment ONE of these to load preset configurations from the matching files
#include "spectral_eiki.h"  // "Store projector-specific settings here"
//#include "spectral_p26.h" // "Store projector-specific settings here"
#include "variables.h"

#include "declarations.h"  // "Function declarations for functions defined in later code"

#include "pindef.h"



#include "config_functions.h"

#include "encoder.h"

#include "ui.h"

#include "led.h"






/////////////////////////
//// ---> SETUP <--- ////
/////////////////////////

void setup() {



  lightSetup();

  interfaceConfig();

      uartConfig();

        createTasks();

  if (useAS5047) {
    as5047Config();
  }

  



    mathConfig();


    if (enableShutter != 0) {
      if (enableShutter == 1) {
        digitalShutterConfig();
      } if (enableShutter == 2) {
        mechanicalShutterConfig();
      }
    }

  if (use_HobbywingQuicRun) {
  motorConfig();
  }

  

}



/////////////////////////////////////////////
//// ---> THE LOOP (runs on core 1) <--- ////
/////////////////////////////////////////////

void loop() {






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
      Serial.print(as5047.readAngle());
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


//updated rotary encoder reading to use esp32 hardware sections that look for pulse counts. this way pulses are noticed asynchronously rather than at the time of the interrupt.... the code within the interrupt wasn't really fast enough to both register a change and act on it. now rather than stopping when it notices something it acts more like a bear grabbing a fish from a stream for better or worse!
//if one were to use a PICO instead, use the PIO to achieve the same. 
//stm32 based stuff can use hw counters
void IRAM_ATTR pinChangeISR() {
  int encCountOld = enc.getTicks();

  enc.loop();

  if (digitalRead(EncI)) {
    EncIndexCount++;
  } else {
    EncIndexCount = 0;
  }
  //   //   // moving forwards ...
  if (enc.getTicks() > encCountOld) {
    //   //     // at index
    if (EncIndexCount == 2) {  // reset counter on 'middle" transition during index condition
      // count = 0;
      if (opticalPrinter == 0) {
        frame++;
      } else {
        frame--;
      }

      frame++;
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
    //   //     // moving backwards ...
    if (EncIndexCount == 2) {  // reset counter on 'middle" transition during index condition
      // at index
      // count = 0;
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
void as5047MagCheck(void *pvParameters) { for(;;) {

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
  vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}




// Read user interface buttons and pots
// NOTE incurs 12ms blocking delay to help calm down crappy ADC between readings.


  // update each pot using Kalman filter to reduce horrible terrible awful ADC noise
//   if (musicMode == 0) {

//     motPotVal = motPotKalman.updateEstimate(analogRead(motPotPin));

//     // delay(2);
//     //given intervals to control optical printer timing/light. i am a simple man and im happier with less choices here. intervals of 409 are used.
//     //this could be done as a formula probably. dunno!
//     if (opticalPrinter == 1) {
//       ledPotVal = ledPotKalman.updateEstimate(analogRead(ledPotPin));
//       if (ledPotVal < 409) {
//         ledPotVal = 409;
//       } else if (ledPotVal > 409 && ledPotVal < 818) {
//         ledPotVal = 818;
//       } else if (ledPotVal > 818 && ledPotVal < 1227) {
//         ledPotVal = 1227;
//       } else if (ledPotVal > 1227 && ledPotVal < 1636) {
//         ledPotVal = 1636;
//       } else if (ledPotVal > 1636 && ledPotVal < 2045) {
//         ledPotVal = 2045;
//       } else if (ledPotVal > 2045 && ledPotVal < 2454) {
//         ledPotVal = 2454;
//       } else if (ledPotVal > 2454 && ledPotVal < 2863) {
//         ledPotVal = 2863;
//       } else if (ledPotVal > 2863 && ledPotVal < 3272) {
//         ledPotVal = 3272;
//       } else if (ledPotVal > 3272 && ledPotVal < 3681) {
//         ledPotVal = 3681;
//       } else if (ledPotVal > 3681 && ledPotVal < 4090) {
//         ledPotVal = 4090;
//       }
//     } else if (opticalPrinter == 0) {
//       ledPotVal = ledPotKalman.updateEstimate(analogRead(ledPotPin));
//     }
//     // delay(2);
//   } else if (musicMode == 1) {
//     motPotVal = map(CC1ProjSpeed, 0, 100, 0, 4095);
//     ledPotVal = map(CC3ProjBright, 0, 100, 0, 4095);

//     //inject values during Music Mode
//   }







//   if (debugUI) {
//     if (enableMotSwitch) {
//       // print selector switch debug info , even though we aren't using it inside this function
//       // Motor UI is switch + pot, so use normal pot scaling
//       if (!digitalRead(motDirFwdSwitch)) {
//         Serial.print("Mot For, ");
//       } else if (!digitalRead(motDirBckSwitch)) {
//         Serial.print("Mot Back, ");
//       } else {
//         Serial.print("Mot Stop, ");
//       }
//     }
//     Serial.print("Mot Speed: ");
//     Serial.print(motPotVal);
// #if (enableSlewPots)
//     Serial.print(", Mot Slew: ");
//     Serial.print(motSlewVal);
// #endif
//     Serial.print(", Lamp Bright: ");
//     Serial.print(ledPotVal);
// #if (enableSlewPots)
//     Serial.print(", Lamp Slew: ");
//     Serial.print(ledSlewVal);
// #endif
// #if (enableShutterPots)
//     Serial.print(", Shut Blade: ");
//     Serial.print(shutBladesPotVal);
//     Serial.print(", Shut Angle: ");
//     Serial.print(shutAnglePotVal);
// #endif
// #if (enableSafeSwitch)
//     Serial.print(", Safe Mode: ");
//     Serial.print(safeMode);
// #endif
//     Serial.println("");

//   }



//   //}
//   //else{

//   //}






// compute the real FPS, based on encoder rotation, but averaged to reduce error
void calcFPS(void *pvParameters) { for(;;) { //moveto core 0 with related interrupts
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
  vTaskDelay(50 / portTICK_PERIOD_MS);
}
}

// compute LED brightness (note that the ISR ultimately controls the LED state if enableShutter = 1)


void updateMotor(void *pvParameters) { 
  for(;;) {
      Serial.println("runMotUpdate");


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
      shutterQueue(1, 1.0);  // force open shutter for single framing
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
      shutterQueue(1, 1.0);  // force open shutter for single framing
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
    shutterQueue(1, 1.0);  // force open shutter for single framing
    // ledBright = ledBright * safeMin;
    safeMode = 1;
    send_LEDC();
  } else if (motSingle == 3) {
    // P26 "BURN" BUTTON special case
    //    Serial.println("P26 BURN MODE (open shutter)");
    shutterQueue(1, 1.0);  // force open shutter for single framing
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

     vTaskDelay(20 / portTICK_PERIOD_MS);

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
      shutterQueue(shutBladesVal, shutAngleVal);  // return shutter map to normal
      Serial.print("Leaving Eiki / P26 FREEZE/BURN mode... motSingle = ");
      Serial.println(motSingle);
      if (motSingle == 2 || motSingle == 3) motSingle = 0;  // turn off single flag if we're leaving Eiki freeze-frame mode or P26 open shutter mode
      motSinglePrev = motSingle;
      buttonAstate = 0;
      shutterQueue(shutBladesVal, shutAngleVal);  // return shutter map to normal
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
      shutterQueue(shutBladesVal, shutAngleVal);  // return shutter map to normal
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



#include "commander.h"