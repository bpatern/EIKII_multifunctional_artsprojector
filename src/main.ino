
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>   // https://github.com/adafruit/Adafruit_NeoPixel

// #include <AS5X47.h>              // https://github.com/adrien-legrand/AS5X47
#include <elapsedMillis.h>       // https://github.com/pfeerick/elapsedMillis
#include <Ramp.h>                // https://github.com/siteswapjuggler/RAMP
#include <SimpleKalmanFilter.h>  // https://github.com/denyssene/SimpleKalmanFilter
#include <Button2.h>             // https://github.com/LennartHennigs/Button2

/* ESP32 FLAGS */
#define CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM 1
//logic operations on seperate MCU so these flags *should* be safe
#define CONFIG_FREERTOS_IN_IRAM 1
#define CONFIG_SPI_MASTER_IN_IRAM 1
#define CONFIG_ESP32_REV_MIN ECO3 //check your board first!!! this frees up some ram and speeds things up
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
#include "esp_task_wdt.h"
// Uncomment ONE of these to load preset configurations from the matching files
#include "spectral_eiki.h"  // "Store projector-specific settings here"
//#include "spectral_p26.h" // "Store projector-specific settings here"
#include "variables.h"
#include "pindef.h"


#include "declarations.h"  // "Function declarations for functions defined in later code"
#include "led.h"
#include "hobbywing_commander_functions.h"
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
  motorConfig();
  createTasks();

}


//tasks run in the background, check header files to see what does what.








void loop() 
{
          while(1){
              //this blocks the loop from doing things in the background... even if its empty it steals cycles!
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
