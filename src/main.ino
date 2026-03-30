
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

#include "led.h"

#include "config_functions.h"









/////////////////////////
//// ---> SETUP <--- ////
/////////////////////////

void setup() {



  if (useAS5047) {
      xTaskCreatePinnedToCore(
        core0setup,
        "core0setup",
        10000,
        NULL,
        24,
        NULL,
        0
    );
    }

  lightSetup();

  interfaceConfig();

      uartConfig();




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
  vTaskDelay(500 / portTICK_PERIOD_MS); // small delay to ensure everything is set up before we start creating tasks
  shutterMapping = xSemaphoreCreateMutex(); // create the shutter mapping semaphore after the delay to ensure that it is created after the tasks that use it are created, which prevents potential issues with tasks trying to take a semaphore that hasn't been created yet. this is important because the shutter mapping semaphore is used in
          createTasks();


}

#include "encoder.h"

#include "ui.h"


#include "esc.h"

#include "motor_logic.h"

#include "commander.h"

/////////////////////////////////////////////
//// ---> THE LOOP (runs on core 1) <--- ////
/////////////////////////////////////////////

void loop() 
{
}

//loop is no longer used due to FreeRTOS tasks.





  // These happen once per encoder count (only useful for debugging at slow speeds)
  // if (countOld != count) {
  //   if (debugEncoder) {
  //     Serial.print("Frame: ");
  //     Serial.print(frame);
  //     Serial.print(", Frame Old: ");
  //     Serial.print(frameOldsingle);
  //     Serial.print(", Count: ");
  //     Serial.print(count);
  //     Serial.print(", Single: ");
  //     Serial.print(as5047.readAngle());
  //     Serial.print(", Lamp: ");
  //     Serial.print(shutterMap[count]);
  //     Serial.print(", Brightness: ");
  //     Serial.println(ledBright);
  //   }
  //   countOld = count;
  // }

  // // These happen once per frame (only useful for debugging)
  // if (frameOld != frame) {
  //   if (debugFrames) {
  //     Serial.print("FRAME: ");
  //     Serial.print(frame);
  //     Serial.print(", FPSreal:");
  //     Serial.print(FPSreal);
  //     Serial.print(", FPS avg:");
  //     Serial.println(FPSrealAvg);
  //   }
  //   frameOld = frame;
  // }
// }

/////////////////////////////
//// ---> FUNCTIONS <--- ////
/////////////////////////////

// On interrupt, read input pins, compute new state, and adjust count


//updated rotary encoder reading to use esp32 hardware sections that look for pulse counts. this way pulses are noticed asynchronously rather than at the time of the interrupt.... the code within the interrupt wasn't really fast enough to both register a change and act on it. now rather than stopping when it notices something it acts more like a bear grabbing a fish from a stream for better or worse!
//if one were to use a PICO instead, use the PIO to achieve the same. 
//stm32 based stuff can use hw counters





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


// compute LED brightness (note that the ISR ultimately controls the LED state if enableShutter = 1)









