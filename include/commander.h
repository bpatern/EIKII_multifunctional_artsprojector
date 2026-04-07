 char tab2[256];
   std::string update;





void serialReadTask(void *pvparemeters) {
  for (;;)
  {
  if (musicMode == 1) {
    midiParser();
    midiControlTakeover();
  }
    externalcontrol();
  }
          // unsigned int temp2 = uxTaskGetStackHighWaterMark(nullptr);
    // Serial.print("serial="); Serial.println(temp2);
  vTaskDelay(20 / portTICK_PERIOD_MS);

}




void sendConfirmation(char type) {
    uint8_t confirmationByte;
    xQueueReset(outCommanderQueue); //clean slate for confirmation message to go OUT!
    // uart_send_break(UART_NUM_2); //tell commander to clear data parsing function to prevent momentary weirdness. called here because its easier i think. i want to avoid running this function more than once in an act
    sendingIndividualCommand = 1; //set flag to indicate we're sending an individual command
    if (type == 'A') {
        confirmationByte = 'A';
        xQueueSend(outCommanderQueue, &confirmationByte, portMAX_DELAY); //send confirmation of capture start to aux display
    } if (type == 'B') {
        confirmationByte = 'B';
        xQueueSend(outCommanderQueue, &confirmationByte, portMAX_DELAY); //send confirmation of capture stop to aux display
    } if (type == '&') {
        confirmationByte = '&';
        xQueueSend(outCommanderQueue, &confirmationByte, portMAX_DELAY); //send confirmation of shutter open command to aux display
    } if (type == '$') {
        confirmationByte = '$';
        xQueueSend(outCommanderQueue, &confirmationByte, portMAX_DELAY); //send confirmation of shutter close command to aux display
}
}




///////////////////////////////////
//// ---> SERIAL CONTROLS <--- ////
///////////////////////////////////

void externalcontrol() {

//     uint8_t recvByte;
//     uint8_t peekLED;
static int displayUpdate = 0;
    static int sfStatus = 0;
  static char received[10];
  static bool dataRecvd;


    for(;;) {

 if (intDirection == 'f' || intDirection == 'r')
 {
  displayUpdate = 1;
  if(uxSemaphoreGetCount(singleFraming) == 1)
  {
    sfStatus =0;
  }

 } else if (intDirection == 'p' || intDirection == 'x')
 {
  if(uxSemaphoreGetCount(singleFraming) == 0)
  {
    sfStatus = 1;
    displayUpdate = 1;
  } else 
  {
    sfStatus = 0;
    displayUpdate = 0;
  }
 }
//  char data;
    uint8_t* data = (uint8_t*) malloc(256 + 1);
    // while (1) {
         const int rxBytes = uart_read_bytes(UART_NUM_2, data, 256, 10);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            for(int i = 0; i < rxBytes; i++)
            {
              if (i == 0)
              {
                received[0] = data[i];
              }
            }
            dataRecvd = true;
            
        }

    // }
    free(data);




 update = '<' + std::to_string(Mangle) + ',' + std::to_string(shutAngEstimate) + ',' + std::to_string(ledB) + ',' + std::to_string(shutBladesVal) + ',' + std::to_string(displayUpdate) + ',' + std::to_string(frame) + ',' + std::to_string(sfStatus) + ',' + std::to_string(mCopyStatus) + '>';
        // update = "Hello world";
            const int len = update.length();
                  const char *strn = update.c_str();
        const int txBytes = uart_write_bytes(UART_NUM_2, strn, len);


if (dataRecvd == true)
{
  if(commandLibrary(deviceType, received[0]) == true)
  {
    ESP_LOGI("serial", "message handled successfully");
  } else
  {
    ESP_LOGI("serial", "message handling failure");

  }
  dataRecvd = false;
}

//   received[0] = NULL;
// }  
    // free(data);

//       if (c == 'g') {
//         opticalPrinter = 1;  //i request this every time just to be sure that the proj understands!
//         motSingle = -1;
//       } else if (c == 'h') {
//         opticalPrinter = 1;
//         motSingle = 1;
//       } else if (c == 'A') {
//         capFlag = 1;
//         scanFlag = 1;

//         sendConfirmation('A');

//       } else if (c == 'B') {
//         capFlag = 0;
//         scanFlag = 1;

//         sendConfirmation('B');
//       } else if (c == 'x') {
//         frame = 0;
//       } else if (c == 'u') {
//         mCopyStatus = 0;
//       } else if (c == 'G') {
//         motSingle = -1;
//       } else if (c == 'H') {
//         motSingle = 1;
//         sendConfirmation('&');
//       } else if (c == 'P') {
//         motExtSwitch = 1;
//       } else if (c == 'W') {
//         motExtSwitch = -1;
//       } else if (c == '*') {
//         motExtSwitch = 0;
//         motSingle = 0;
//       } else if (c == '@') {
//         musicMode = 1;  //midi
//         // Serial.println("music mode on");
//       } else if (c == 'z') {

//       }
    
  // }

  // if (Serial.available()) {
  //   char c = Serial.read();
  //   if (c == '1') {
  //     if (opticalPrinter == 0) {
  //       motSingle = 1;
  //     } else {
  //       motSingle = -1;
  //     }

  //   } else if (c == '2') {
  //     if (opticalPrinter == 0) {
  //       motSingle = -1;
  //     } else {
  //       motSingle = 1;
  //     }

  //   } else if (c == '-') {
  //     frame = 0;
  //     Serial.println("Projector Frame Count Reset!");
  //   } else if (c == '5') {
  //     motSingle = 5;
  //     Serial.println("Shutter OPEN!");
  //   } else if (c == '4') {
  //     motSingle = 4;
  //   } else if (c == 'f') {
  //     motExtSwitch = 1;
  //   } else if (c == 'p') {
  //     motExtSwitch = 0;
  //   } else if (c == 'r') {
  //     motExtSwitch = -1;
  //   } else if (c == '8') {

  //   } else if (c == '0') {
  //     Serial.println("Camera frame count reset!");
  //   } else if (c == '9') {
  //     Serial.print("Currently, the projector is on frame ");

  //     Serial.println(frame);
  //   } else if (c == ';') {
  //     opticalPrinter = 1;
  //     Serial.println("optical printer mode enabled! you must manually turn off LED if need be.");
  //   } else if (c == '`') {
  //     motSingle = 0;
  //     Serial.println("Shutter closed.");
  //   } else if (c == '@') {
  //     musicMode = 1;  //midi
  //     Serial.println("music mode on");
  //   }
  // }

vTaskDelay(30/portTICK_PERIOD_MS);}
}

//  if (serialtimerUI >= 400) {

void midiControlTakeover() {

  // while (Serial2.available()) {
  //   char c = Serial2.read();
  //   if (c == 'H') {
  //     motSingle = 1;
  //     Serial.println("RECVD");
  //     sendConfirmation('&');

  //   } else if (c == 'G') {
  //     motSingle = -1;
  //     Serial.println("RECVD");
  //     sendConfirmation('&');
  //   } else if (c == '@') {
  //     musicMode = 1;  //midi
  //     Serial.println("music mode on");
  //   } else if (c == 'L') {
  //     motExtSwitch = 1;
  //     motMode = 1;
  //     //Serial.println("RECVD");
  //     sendConfirmation('$');

  //   } else if (c == '*') {

  //     motExtSwitch = 0;
  //     motMode = 0;
  //     receivedRecvdConfirm = 1;
  //     sendConfirmation('$');

  //   } else if (c == 'E') {
  //     motExtSwitch = -1;
  //     motMode = -1;
  //     Serial.println("RECVD");
  //     sendConfirmation('$');


  //   } else if (c == 'W') {
  //     receivedRecvdConfirm2 = 1;
  //   }
  // }

  // if (receivedRecvdConfirm == 1) {
  //   if (receivedRecvdConfirm2 == 0) {
  //     sendConfirmation('$');
  //   } else if (receivedRecvdConfirm2 == 1) {
  //     receivedRecvdConfirm2 = 0;
  //     receivedRecvdConfirm = 0;
  //     // Serial.println("ok");
  //   }
  // }
}





void midiParser() {
  // if (midiParseTimer > 50) {
  //   recvWithStartEndMarkers();
  //   if (newData == true) {
  //     strcpy(tempChars, receivedChars);
  //     sscanf(tempChars, "%d,%d,%d", &CC1ProjSpeed, &CC2ProjBlades, &CC3ProjBright);
  //     //    showParsedData();
  //     newData = false;
  //     midiParseTimer = 0;
  //   }
  // }
}

void recvWithStartEndMarkers() {
  // static boolean recvInProgress = false;
  // static byte ndx = 0;
  // char startMarker = '<';
  // char endMarker = '>';
  // char rc;

  // while (Serial2.available() > 0 && newData == false) {
  //   rc = Serial2.read();

  //   if (recvInProgress == true) {
  //     if (rc != endMarker) {
  //       receivedChars[ndx] = rc;
  //       ndx++;
  //       if (ndx >= numChars) {
  //         ndx = numChars - 1;
  //       }
  //     } else {
  //       receivedChars[ndx] = '\0';  // terminate the string
  //       recvInProgress = false;
  //       ndx = 0;
  //       newData = true;
  //     }
  //   }

  //   else if (rc == startMarker) {
  //     recvInProgress = true;
  //   }
  // }
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