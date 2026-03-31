
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>   // https://github.com/adafruit/Adafruit_NeoPixel

// #include <AS5X47.h>              // https://github.com/adrien-legrand/AS5X47
#include <elapsedMillis.h>       // https://github.com/pfeerick/elapsedMillis
#include <Ramp.h>                // https://github.com/siteswapjuggler/RAMP
#include <SimpleKalmanFilter.h>  // https://github.com/denyssene/SimpleKalmanFilter
#include <Button2.h>             // https://github.com/LennartHennigs/Button2

/* ESP32 FLAGS */
#define CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM 1
/***************/

#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "driver/mcpwm_prelude.h"
#include "driver/GPIO.h"
#include "driver/spi_master.h"
// Uncomment ONE of these to load preset configurations from the matching files
#include "spectral_eiki.h"  // "Store projector-specific settings here"
//#include "spectral_p26.h" // "Store projector-specific settings here"
#include "variables.h"
#include "pindef.h"

// static 
// AS5X47 as5047(EncCSN);
// // will be used to read data from the magnetic encoder
// static 
// ReadDataFrame readDataFrame;
// 
// int as5047MagOK = 0; // status of magnet near AS5047 sensor
// 
// int as5047MagOK_old = 0;
// static int irqCt;

#include "declarations.h"  // "Function declarations for functions defined in later code"
#include "led.h"
#include "config_functions.h"
#include "motor_logic.h"
#include "commander.h"
#include "ui.h"
#include <dma_spiencoder_implementation.h>
#include "encoder.h"
#include "esc.h"



/////////////////////////
//// ---> SETUP <--- ////
/////////////////////////

void setup() {


           xTaskCreatePinnedToCore(
        core1setup,
        "core1setup",
        3000,
        NULL,
        24,
        NULL,
        1
    );
      


  mathConfig();

  // if (use_HobbywingQuicRun) {
  motorConfig();
  // }
  createTasks();


}









/////////////////////////////////////////////
//// ---> THE LOOP (runs on core 1) <--- ////
/////////////////////////////////////////////

void loop() 
{
}

//loop is no longer used due to FreeRTOS tasks.

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









