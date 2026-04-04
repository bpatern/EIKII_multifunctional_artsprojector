

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
  vTaskDelay(10 / portTICK_PERIOD_MS);

}

void serial2RX(void * parameter) {
    char* recv = 0;

  for (;;) {
    uart_read_bytes(UART_NUM_2, (uint8_t*)&recv, 1, portMAX_DELAY);
    xQueueSend(outCommanderQueue, &recv, portMAX_DELAY);
        vTaskDelay(500 / portTICK_PERIOD_MS); // adjust this delay as needed to control the rate of sending data. if we're sending the full data string, we might want to increase this delay to prevent flooding the commander with too many messages.

  }
}

static char send;
void serial2tx(void * parameter) {
  //     // char* sendtest = "<1.35,0.79,4094,1683,0,95,0,0>";
    uint16_t rx_fifo_len, status;
  uint16_t i;
  for (;;) {

    // xQueueReceive(outCommanderQueue, &send, 5);

    // Serial2.println(send);


    vTaskDelay(20 / portTICK_PERIOD_MS); // adjust this delay as needed to control the rate of sending data. if we're sending the full data string, we might want to increase this delay to prevent flooding the commander with too many messages.
  }
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


    for(;;) {


//         if (sendingIndividualCommand == 1) {
//             if (uxQueueMessagesWaiting(outCommanderQueue) == 0) 
//             {
//                 sendingIndividualCommand = 0;
//                 //this makes it so a command is sent with priority, and since the data string is sent constantly this shouldnt be a noticable break.
//             }

//         } 
//         else if (sendingIndividualCommand == 0 || uxQueueMessagesWaiting(outCommanderQueue) == 0) 
//         {
// xQueueReceive(q_peekLED, &peekLED, 5);
//  static String data = "<" + String(FPSreal) + "," + String(shutAngleVal) + "," + String(peekLED) + "," + String(shutBladesPotVal) + "," + String("1") + "," + String(frame) + "," + String(motSingle) + "," + String(mCopyStatus) + ">";
// //  static char* dataChar = data.c_str(); // convert String to const char*
// //  xQueueSend(outCommanderQueue, &data, portMAX_DELAY); // send the data string to the queue for transmission over UART
//  Serial2.println(data);
//         }
//  //ill be thrilled if this works. then i can probably setup a parameter that adjusts speed based on usage or asks for a different string depending on mode.


//   if (musicMode == 0) {
    
//     xQueueReceive(outCommanderQueue, &recvByte, 0); // non-blocking receive from the queue


//       char c = recvByte; // interpret the received byte as a character

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