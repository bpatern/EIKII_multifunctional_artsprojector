static QueueHandle_t outCommanderQueue;
uint8_t sendingIndividualCommand = 0; //flag to indicate whether we're sending an individual command or the full data string. if 1, we send individual command, if 0 we send full data string. this is to prevent flooding the commander with the full data string when we're just trying to send a single command (like a shutter open/close command from the aux display)
static intr_handle_t handle_console;

void IRAM_ATTR uart_intr_handle(void *arg) {


//  uart_clear_intr_status(UART_NUM_2, UART_BUFFER_FULL_ERROR|UART_FIFO_OVF_ERROR);

}

void serial2RX(void * parameter) {
    char* recv = 0;

  for (;;) {
    uart_read_bytes(UART_NUM_2, (uint8_t*)&recv, 1, portMAX_DELAY);
    xQueueSend(outCommanderQueue, &recv, portMAX_DELAY);
        vTaskDelay(500 / portTICK_PERIOD_MS); // adjust this delay as needed to control the rate of sending data. if we're sending the full data string, we might want to increase this delay to prevent flooding the commander with too many messages.

  }
}

void serial2tx(void * parameter) {
      char* send = 0;
  //     // char* sendtest = "<1.35,0.79,4094,1683,0,95,0,0>";
    uint16_t rx_fifo_len, status;
  uint16_t i;
  for (;;) {


  
  
    xQueueReceive(outCommanderQueue, &send, portMAX_DELAY);

    // uart_write_bytes(UART_NUM_2, (const char*)&send, 50);
    // uart_write_bytes_with_break(UART_NUM_2, (const char*)&send, strlen(send), 20); // send break before data to signal start of message. adjust break duration as needed based on testing. 100 ms is a good starting point but might need to be longer or shorter depending on how the commander handles it.

    Serial2.println(send);
    Serial.print(send);
    Serial.println(uxQueueMessagesWaiting(outCommanderQueue)); //debug to see if we're sending too many messages. should be 0 or 1 most of the time, but can be higher if we're sending a lot of data (like the full data string) and the commander is slow to receive it for some reason. if this is consistently high, we might want to adjust the speed at which we send the full data string or implement some sort of flow control.


    vTaskDelay(100 / portTICK_PERIOD_MS); // adjust this delay as needed to control the rate of sending data. if we're sending the full data string, we might want to increase this delay to prevent flooding the commander with too many messages.
  }
}

void uartConfig() {

    //characters stored as integers because they really are integers!!!!!!!!!!!! rah!!!!
    const uart_port_t uart_num = UART_NUM_2;

    uart_config_t commanderlink = {
.baud_rate = 57600,
.data_bits = UART_DATA_8_BITS,
.parity = UART_PARITY_DISABLE,
.stop_bits = UART_STOP_BITS_1,
.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
.source_clk = UART_SCLK_APB,
};

ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 2048, 0, 0, NULL, 0)); // set rx buffer size to 2048 and no tx buffer since we're not using it. also no queue since we'll be using our own FreeRTOS queue
vTaskDelay(100 / portTICK_PERIOD_MS); // small delay to ensure driver is installed before configuring parameters
ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &commanderlink));
ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, auxTransmitter, auxReceiver, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); //last two are for handshake pins. didnt know these exist but might be useful since the run is internal and thus i can do whatever i want
// ESP_ERROR_CHECK(uart_isr_free(UART_NUM_2));
// ESP_ERROR_CHECK(uart_isr_register(UART_NUM_2,uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console));
// ESP_ERROR_CHECK(uart_enable_rx_intr(UART_NUM_2));
Serial2.begin(57600);



outCommanderQueue = xQueueCreate(50, sizeof(char*));   


xTaskCreatePinnedToCore(
    serial2RX,
    "serial2RX",
    1024,
    NULL,
    16, //this should be sort of high. note to self 2 check and adj +stack size with goated freertos debug toolz
    NULL,
    0 
);

xTaskCreatePinnedToCore(
    serial2tx,
    "serial2tx",
    11520,
    NULL,
    20,
    NULL,
    1
);
}

void sendConfirmation(char type) {
    uint8_t confirmationByte;
    xQueueReset(outCommanderQueue); //clean slate for confirmation message to go OUT!
    uart_send_break(UART_NUM_2); //tell commander to clear data parsing function to prevent momentary weirdness. called here because its easier i think. i want to avoid running this function more than once in an act
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

    uint8_t recvByte;


    for(;;) {


        if (sendingIndividualCommand == 1) {
            if (uxQueueMessagesWaiting(outCommanderQueue) == 0) 
            {
                sendingIndividualCommand = 0;
                //this makes it so a command is sent with priority, and since the data string is sent constantly this shouldnt be a noticable break.
            }

        } 
        else if (sendingIndividualCommand == 0 || uxQueueMessagesWaiting(outCommanderQueue) == 0) 
        {
 String data = "<" + String(FPSreal) + "," + String(shutAngleVal) + "," + String(ledPotVal) + "," + String(shutBladesPotVal) + "," + String(motMode) + "," + String(frame) + "," + String(motSingle) + "," + String(mCopyStatus) + ">";
 const char* dataChar = data.c_str(); // convert String to const char*
 xQueueSend(outCommanderQueue, &dataChar, portMAX_DELAY); // send the data string to the queue for transmission over UART
        }
 //ill be thrilled if this works. then i can probably setup a parameter that adjusts speed based on usage or asks for a different string depending on mode.


  if (musicMode == 0) {
    
    xQueueReceive(outCommanderQueue, &recvByte, 0); // non-blocking receive from the queue


      char c = recvByte; // interpret the received byte as a character

      if (c == 'g') {
        opticalPrinter = 1;  //i request this every time just to be sure that the proj understands!
        motSingle = -1;
      } else if (c == 'h') {
        opticalPrinter = 1;
        motSingle = 1;
      } else if (c == 'A') {
        capFlag = 1;
        scanFlag = 1;

        sendConfirmation('A');

      } else if (c == 'B') {
        capFlag = 0;
        scanFlag = 1;

        sendConfirmation('B');
      } else if (c == 'x') {
        frame = 0;
      } else if (c == 'u') {
        mCopyStatus = 0;
      } else if (c == 'G') {
        motSingle = -1;
      } else if (c == 'H') {
        motSingle = 1;
        sendConfirmation('&');
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
      } else if (c == 'z') {
        static String ledValRecvdStr = Serial.readStringUntil('z');
       
        ledBright = ledValRecvdStr.toInt();

        send_LEDC();
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

vTaskDelay(30/portTICK_PERIOD_MS);}
}

//  if (serialtimerUI >= 400) {

void midiControlTakeover() {

  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == 'H') {
      motSingle = 1;
      Serial.println("RECVD");
      sendConfirmation('&');

    } else if (c == 'G') {
      motSingle = -1;
      Serial.println("RECVD");
      sendConfirmation('&');
    } else if (c == '@') {
      musicMode = 1;  //midi
      Serial.println("music mode on");
    } else if (c == 'L') {
      motExtSwitch = 1;
      motMode = 1;
      //Serial.println("RECVD");
      sendConfirmation('$');

    } else if (c == '*') {
      updateShutterMap(0, 0.0);  // force open shutter for single framing

      motExtSwitch = 0;
      motMode = 0;
      receivedRecvdConfirm = 1;
      sendConfirmation('$');
      updateShutterMap(shutBladesVal, shutAngleVal);

    } else if (c == 'E') {
      motExtSwitch = -1;
      motMode = -1;
      Serial.println("RECVD");
      sendConfirmation('$');


    } else if (c == 'W') {
      receivedRecvdConfirm2 = 1;
    }
  }

  if (receivedRecvdConfirm == 1) {
    if (receivedRecvdConfirm2 == 0) {
      sendConfirmation('$');
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