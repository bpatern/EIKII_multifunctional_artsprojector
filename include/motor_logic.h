void updateMotor(void *pvParameters) { 
  static int motPotVal;
  static rampInt motAvg; // ramp object for motor speed slewing

  for(;;) {
    // if (xSemaphoreTake(controlLock, portMAX_DELAY) == pdTRUE) {  // take control lock to read UI values and update shared variables that are used in other tasks and ISRs
      xQueueReceive(motPot, &motPotVal, 10);  // receive motor pot value from UI task via queue


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
        //   send_LEDC();
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

//   if (motSingle == 1) {
//     // EIKI SINGLE FRAME FORWARD
//     if (motSingle != motSinglePrev) {
//       Serial.println("SINGLE FORWARD MOVE START");
//       frameOldsingle = frame;  // log the current frame when we began the single frame move
//       motSinglePrev = motSingle;
//       shutterQueue(1, 1.0);  // force open shutter for single framing
//     }
//     if (frameOldsingle != frame && count > pullDownPos) {  // keep out of pulldown in 0-13 zone, so try to land around 50
//       // we're ready to show frame
//       Serial.println("SINGLE FORWARD MOVE DONE");
//       FPStarget = 0;  // stop
//       if (opticalPrinter == 0) {
//         motSingle = 0;  // turn off single flag
//       } else if (opticalPrinter == 1 && opAlignment == 1) {
//         Serial.print("Projector is on frame "),
//           Serial.println(frame);

//         motSingle = 3;
//         opAlignment = 0;
//         mCopyStatus = 1;  //mcopy confirmation flag
//                           //frame++;
//         motSinglePrev = motSingle;
//         frameOldsingle = frame;
//       } else {
//         mCopyStatus = 1;  //mcopy confirmation flag
//                           //frame--;
//         motSingle = 3;
//       }
//       Serial.print("Projector is on frame "),
//         Serial.println(frame);
//       motSinglePrev = motSingle;
//       frameOldsingle = frame;
//       //updateShutterMap(shutBladesVal, shutAngleVal); // return to normal
//     } else {
//       // travel to the next frame
//       if (musicMode == 0) {
//         FPStarget = singleFPS * 1;  // jam in preset speed
//       } else if (musicMode == 1) {
//         FPStarget = 8;  // jam in preset speed
//       }
//     }
//   } else if (motSingle == -1) {
//     // EIKI SINGLE FRAME BACKWARD
//     if (motSingle != motSinglePrev) {
//       Serial.println("SINGLE BACKWARD MOVE START");
//       frameOldsingle = frame;  // log the current frame when we began the single frame move
//       motSinglePrev = motSingle;
//       shutterQueue(1, 1.0);  // force open shutter for single framing
//     }
//     if (frameOldsingle != frame && count < 80) {  // try to land around 50
//       // we're ready to show frame
//       Serial.println("SINGLE BACKWARD MOVE DONE");
//       FPStarget = 0;  // stop
//       if (opticalPrinter == 0) {
//         motSingle = 0;  // turn off single flag
//       } else if (opAlignment == 0) {
//         motSingle = -1;
//         opAlignment = 1;
//         motSinglePrev = motSingle;
//         frameOldsingle = frame;
//       } else if (opAlignment == 1 && motSingle == -1) {
//         motSingle = 1;

//         motSinglePrev = motSingle;
//         frameOldsingle = frame;
//       }
//       Serial.print("Projector is on frame "),
//         Serial.println(frame);
//       // Serial2.println("h"); //mcopy confirmation flag
//       motSinglePrev = motSingle;
//       frameOldsingle = frame;
//       //updateShutterMap(shutBladesVal, shutAngleVal); // return to normal
//     } else {
//       // travel to the next frame
//       if (musicMode == 0) {
//         FPStarget = singleFPS * -1;  // jam in preset speed
//       } else if (musicMode) {
//         FPStarget = -8;  // jam in preset speed
//       }
//     }
//   } else if (motSingle == 2) {
//     // Eiki freeze frame (either button pressed while motor was running)
//     Serial.println("EIKI FREEZE FRAME");
//     FPStarget = 0;
//     shutterQueue(1, 1.0);  // force open shutter for single framing
//     // ledBright = ledBright * safeMin;
//     safeMode = 1;
//     send_LEDC();
//   } else if (motSingle == 3) {
//     // P26 "BURN" BUTTON special case
//     //    Serial.println("P26 BURN MODE (open shutter)");
//     shutterQueue(1, 1.0);  // force open shutter for single framing
//     send_LEDC();
//   } else if (motSingle == 4) {
//     ledcWrite(ledChannel, (ledBright / 2));
//   } else if (motSingle == 5) {
//     ledBright = 4096;
//     ledcWrite(ledChannel, (ledBright));
//   }


// // Transform FPStarget into motor microseconds using choice of 2 methods
// #if motorSpeedMode

//   // USE CLOSED LOOP motor control with feedback from encoder (not working well enough to use for Spectral)
//   float FPSdiff = abs(FPStarget - FPSrealAvg);
//   if (FPStarget > 6) {
//     if (FPSdiff < 2) FPSdiff = FPSdiff * 0.2;  // make changes much smaller when close to setpoint, but only at higher speeds so we can get started from stopped condition
//   }
//   float TESTmotSpeedUS;
//   if (FPStarget == 0) {
//     TESTmotSpeedUS = 1500;
//   } else if (FPStarget < FPSrealAvg) {
//     TESTmotSpeedUS = motSpeedUS + FPSdiff;
//   } else if (FPStarget > FPSrealAvg) {
//     TESTmotSpeedUS = motSpeedUS - FPSdiff;
//   }
//   motSpeedUS = TESTmotSpeedUS;
//   motSpeedUS = constrain(motSpeedUS, 1200, 1800);  // prevent runaway in case of broken belt or other disaster
// #else

//   // USE HARD-CODED SPEED
//   float TESTmotSpeedUS;

//   if (FPStarget == 0) {
//     TESTmotSpeedUS = 1500;
//   } else if (FPStarget > 0) {
//     if (opticalPrinter == 0) {
//       TESTmotSpeedUS = mapf(FPStarget, 0, 24, 1500 - minUSoffset, motMaxUS);
//     } else if (opticalPrinter == 1) {
//       TESTmotSpeedUS = mapf(FPStarget, 0, 24, 1500 - minUSoffsetPrinting, motMaxUS);
//     }
//   } else if (FPStarget < 0) {

//     if (opticalPrinter == 0) {
//       TESTmotSpeedUS = mapf(FPStarget, 0, -24, 1500 + minUSoffset, motMinUS);
//     } else if (opticalPrinter == 1) {
//       TESTmotSpeedUS = mapf(FPStarget, 0, -24, 1500 + minUSoffsetPrinting, motMinUS);
//     }
//   }


//   //note that to calculate min speed it is 1500 - measured "Mot uS" -- however, target speed is not spit out correctly!!!
//   motSpeedUS = TESTmotSpeedUS;
//   //motSpeedUS = mapf(FPStarget, -24.0, 24.0, motMinUS, motMaxUS);  // basic method without enforcing minumum speed
// #endif

//   int motDuty = (1 << motPWMRes) * motSpeedUS / motPWMPeriod;  // convert pulse width to PWM duty cycle (duty = # of values at current res * US / pulse period)
//   ledcWrite(motPWMChannel, motDuty);                           // update motor speed
//   if (debugMotor) {
//     Serial.print("Mot Mode: ");
//     Serial.print(motMode);
//     Serial.print(", Single Mode: ");
//     Serial.print(motSingle);
//     Serial.print(", Mot Slew: ");
//     Serial.print(motSlewVal);
//     Serial.print(", FPS Target: ");
//     Serial.print(FPStarget);
//     Serial.print(", FPS Real Avg: ");
//     Serial.print(FPSrealAvg);
//     Serial.print(", Mot uS: ");
//     Serial.print(motSpeedUS);
//     Serial.print(", Mot PWM: ");
//     Serial.println(motDuty);
//   }
//   if (debugFPSgraph) {
//     Serial.print("-24, ");
//     Serial.print(FPSrealAvg);
//     Serial.println(", 24");
//   }
      // xSemaphoreGive(controlLock);  // release control lock so other tasks and ISRs can read updated values and run

     vTaskDelay(50 / portTICK_PERIOD_MS);

    }
}
