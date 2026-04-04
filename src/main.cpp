
// #include <Ramp.h>                // https://github.com/siteswapjuggler/RAMP
// #include <SimpleKalmanFilter.h>  // https://github.com/denyssene/SimpleKalmanFilter
#include <esp_timer.h>

unsigned long micros() {
  return (unsigned long)(esp_timer_get_time());
}
unsigned long millis() {
  return (unsigned long)(esp_timer_get_time() / 1000ULL);
}
#include "elapsedMillis.h"
/* ESP32 FLAGS */
// #define CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM 1
// //logic operations on seperate MCU so these flags *should* be safe
// #define CONFIG_FREERTOS_IN_IRAM 1
// #define CONFIG_SPI_MASTER_IN_IRAM 1
// #define CONFIG_ESP32_REV_MIN ECO3 //check your board first!!! this frees up some ram and speeds things up
/***************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dsp_platform.h"
#include <esp_dsp.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "soc/rtc.h"
#include "ekf.h"
#include "ekf_imu13states.h"

#include <soc/gpio_struct.h>
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

extern "C" void app_main(void) {


        createObj();
        uartConfig();




        
        mathConfig();
        gpioConfig();


        as5047Config();
        // ang = map(readAngle_raw(ANGLE_REG), 0, 16384, 0, 1024);

        lightSetup();
                motorConfig();

        createTasks();

        

        vTaskDelay(30);


        ESP_LOGI(genPurp, "-----------------------------");
        ESP_LOGI(genPurp, "SPECTRAL Projector Controller");
        ESP_LOGI(genPurp, "-----------------------------");

        
        gpio_set_intr_type((gpio_num_t)motDirFwdSwitch, GPIO_INTR_ANYEDGE);
        gpio_set_intr_type((gpio_num_t)motDirBckSwitch, GPIO_INTR_ANYEDGE);
        gpio_set_intr_type((gpio_num_t)safeSwitch, GPIO_INTR_ANYEDGE);
        gpio_set_intr_type((gpio_num_t)buttonApin, GPIO_INTR_ANYEDGE);
        gpio_set_intr_type((gpio_num_t)buttonBpin, GPIO_INTR_ANYEDGE);

        for (int myBlade = 0; myBlade < *shutterBlPtr; myBlade++)
            {
                int countOffset = myBlade * (1024 / *shutterBlPtr);
                for (int myCount = 0; myCount < 1024 / *shutterBlPtr; myCount++)
                {
                    if (myCount < (1024 / *shutterBlPtr * *shAngFptr))
                    {
                        shutterMap[(myCount + countOffset)] = 0;
                    }
                    else
                    {
                        shutterMap[myCount + countOffset] = 1;
                    }
                }
            }

        vTaskResume(readControls);
        vTaskResume(internalSerialRX);
        vTaskResume(internalSerialTX);
        vTaskResume(singleFrame);
        vTaskResume(shutterPotTranslate);
        vTaskResume(readControls);
        vTaskResume(externalControlParse);
        vTaskResume(FPSactor);
        vTaskResume(motorSlewRead);
        vTaskResume(ioTASKHANDLE);

                gptimer_start(ledtick);

         ESP_LOGI(genPurp, "Interrupts Enabled");

        if (debug==1 ) {
    xTaskCreatePinnedToCore(
        debugTask,
        "debugTask",
        5000,
        NULL,
        15,
        &debugPrinter,
        1
    );
    ESP_LOGI(genPurp, "debugging is RUNNING!");
  }

        // while(1)
        // {

        // }

}


//tasks run in the background, check header files to see what does what.










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
