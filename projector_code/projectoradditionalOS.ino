#include <QuickStats.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <SoftwareSerial.h>
#include <elapsedMillis.h>
#include <Control_Surface.h>

#include <AH/Arduino-Wrapper.h>
#include <Adafruit_TinyUSB.h>
// #include <MIDIUSB.h>




#include "graphics.h"


#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define OPTX 12
#define OPRX 13

#define midiRX 16

QuickStats stats;

elapsedMillis fwdProjTimer;
elapsedMillis timerUI;
elapsedMillis animationTimer;

// SerialPIO SoftSerial(OPTX, OPRX, 256);
SerialPIO midiConnection(SerialPIO::NOPIN, midiRX);

HardwareSerialMIDI_Interface midi(midiConnection, 31250);

// USBMIDI_Interface midi_usb;  //

// BidirectionalMIDI_Pipe pipes;



/////////////////////
//MIDI CONTROL CODE//
/////////////////////

int debugEnable = 1;  // for midi debugging! prints info to SMONITOR

#include "midiRef.h"


/////////////////////////


volatile int camFC = 0;

volatile int OPVariables = 4;
int projFWD;
int projREV;
int camFWD;
int camREV;
int shutterStatus;
int fwdFlag;


const byte numChars = 128;
char receivedChars[numChars];
char tempChars[numChars];

char receivedChars1[numChars];
char tempChars1[numChars];

int FRAMERATE = 0;
float SHUTTERANGLE = 0;
float LEDBRIGHT = 0;
int BLADECT = 0;
int machineStatus = 0;
int sfStatus = 0;
int frameCount = 0;
int mCopyStatus = 0;

int angle = 0;
int bladeCount = 0;
int ledPercentage = 0;

int sfUI = 0;
int sfDir;
unsigned long buttonPressedAt;


int motorcmd;
int sfcmd;
int resetFrameReq;


int pRun = 0;
int cRun = 0;


int opticalPrinter = 0;
int LEDBRIGHTop;
int opShutter = 0;
int opShutterSeq = 0;
unsigned long opShutterTimer = 0;
int pos;
int capperStatus = 0;



//external takeup for OP control//

#define takeup_1 14
#define takeup_2 15
#define sleep 10 






unsigned long serialTimer;

boolean newData = false;
boolean newData1 = false;

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R2, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL, /* data=*/SDA);

typedef u8g2_uint_t u8g_uint_t;

int handshakeS;
int pD = 1;
int cD = 1;


void setup() {
  u8g2.begin();

  Serial2.setRX(9);
  Serial2.setTX(8);
  Serial2.begin(57600);
  // SoftSerial.begin(57600);
  Serial.begin(57600);

  Serial1.setRX(OPRX);
  Serial1.setTX(OPTX);
  Serial1.begin(57600);

  // midi | pipes | midi_usb;

  // midi_usb.begin();
  midi.begin();                 // Initialize the MIDI interface
  midi.setCallbacks(callback);  // Attach the custom callback


  // Wire1.setSDA(18);
  // Wire1.setSCL(19);
  // Wire1.begin();

      pinMode(takeup_1, OUTPUT);
  pinMode(takeup_2, OUTPUT);
   pinMode(sleep, OUTPUT);

  // Turn off motors - Initial state
  analogWrite(takeup_1, 500);
  digitalWrite(takeup_2, LOW);
   pinMode(sleep, HIGH);

}

void loop() {




  // if (calculateValue == 1) {
  //   midiMath();
  //   calculateValue = 0;
  // }
  // midi.update();  // Continuously handle MIDI input
  // midi_usb.update();
  // midiInterpreter();

  recvWithStartEndMarkers();

  if (newData == true) {
    strcpy(tempChars, receivedChars);
    parseData();
    //    showParsedData();
    newData = false;
  }

    graphics();
  auxiliary();
  


  // usbFWD();
  // if (timerUI >= 12) {




  if (cRun >= 0) {
    camera();
  }

  if (opticalPrinter == 1) {
  if (timerUI >= 1250 && pRun == 1 && pD == 1) {
    Serial.println("p");
    pRun = 0;
  } else if (timerUI >= 600 && pRun == 1 && pD == -1) {
    Serial.println("p");
    pRun = 0;
  }
  takeUp();
  }



  // if (debugEnable == 1) {
  //   debug();
  // }
}

void takeUp() {
  if (pRun == 1 && pD == 1) {
      pinMode(sleep, LOW);
  analogWrite(takeup_1, LOW);
  digitalWrite(takeup_2, 300);
  } else if (pRun == 0) {
    pinMode(sleep, HIGH);
  // analogWrite(MOT_A1_PIN, 300);
  // digitalWrite(MOT_A2_PIN, LOW);
  }

}


void camera() {
  if (cRun == 1) {
    Serial1.println('f');
    Serial1.flush();
    Serial.println("SENT");
    cRun = 0;
    timerUI = 0;
  } else if (cRun == 2) {
    Serial1.println('r');
    Serial1.flush();
    //        Serial.println("SENT");
    cRun = 0;
    timerUI = 0;
  }
}


void debug() {
  // Serial.print("channel ");
  // Serial.print(midiCurrentChannel);
  // Serial.print(" pitchBendValue ");
  // Serial.print(midiAdjustedPitchBendVal);
  // Serial.print(" current note ");
  // Serial.print(midiNotesPlayed[midiNotesPlayedIndex]);
  // Serial.print("note index");

  // Serial.print(midiNotesPlayed[1]);
  // Serial.print(midiNotesPlayed[2]);
  // Serial.print(midiNotesPlayed[3]);
  // Serial.print(midiNotesPlayed[4]);
  // Serial.print(midiNotesPlayed[5]);
  // Serial.print(midiNotesPlayed[6]);
  // Serial.print(midiNotesPlayed[7]);
  // Serial.print(midiNotesPlayed[8]);
  // Serial.print(midiNotesPlayed[9]);
  // Serial.print(midiNotesPlayed[10]);
  // Serial.print(averageKeysLoc);
  // Serial.print(" keys being pressed ");
  // Serial.println(midiCurrentKeys);
  // Serial.print("projector speed");
  // Serial.print(CC1ProjSpeed);
  // Serial.print(" projector blade count ");
  // Serial.print(CC2ProjBlades);
  // Serial.print(" projector brightness ");
  // Serial.print(CC3ProjBright);
  // Serial.print(" single frame status ");
  // Serial.print(MSFStatus);
  // Serial.print(" movement position ");
  // Serial.print(moveCMD);
  // Serial.print(" single frame timer count ");
  // Serial.println(SingleTimer);

  Serial.println(moveCMD);
  Serial.println(moveHalt);
}


//  timerUI = 0;


void midiMath() {
  //60 keys total, so certain figures have to be out of 60
  averageKeysLoc = stats.average(midiNotesPlayed, midiNotesPlayedIndex);

  highestNote = stats.maximum(midiNotesPlayed, midiNotesPlayedIndex);

  lowestNote = stats.minimum(midiNotesPlayed, midiNotesPlayedIndex);

  highestNoteAdj = (highestNote - 36) / 60;

  lowestNoteAdj = (lowestNote - 36) / 60;

  midiAdjustedPitchBendVal = map(currentPitchBendVal, 0, 16320, 0, 100);

  int x;

  int y;

  for (x = 0; x < shFlCount; x = x + 1) {
    if ((shFl[x]) == (midiNotesPlayed[midiNotesPlayedIndex])) {
      shFlKeysPlayed = 1;
    }
  }

  for (y = 0; y < naturalsCount; y = y + 1) {
    if ((naturals[y]) == (midiNotesPlayed[midiNotesPlayedIndex])) {
      naturalsKeysPlayed = 1;
    }
  }
}

void midiInterpreter() {
  if (midiCurrentChannel == 4) {
    musicMode = 1;  //setsflag for midi controls exclusivity for both projector and keyboard!
    Serial2.println("@");
  }
  if (MSFStatus == 1) {
    if (noteRecvd == 0) {
      Serial2.println("H");
      Serial.println("SENT");
            noteRecvd = 0;
            MSFStatus = 0;

    } else if (noteRecvd == 1) {
      Serial.println("feedback got");

      noteRecvd = 0;
      MSFStatus = 0;
      if (singleFrameOperation == 1) {
        singleFrameOperation = 0;
      } else {
        midiSong = 0;
      }
      singleFrameOperation = 1;
    } else if (noteRecvd == 2) {
      Serial2.println("H");
      Serial.println("SENT");
      noteRecvd = 0;
                  MSFStatus = 0;



    } else if (noteRecvd == 2) {
      Serial.println("feedback got22222");
      singleFrameOperation = 0;
      noteRecvd = 0;
      MSFStatus = 0;
      midiSong = 0;
    }
  } else if (MSFStatus == -1) {
    if (noteRecvd == 0) {
      Serial2.println("G");
      Serial.println("SENT");
    } else if (noteRecvd == 1) {
      Serial.println("feedback got");
      singleFrameOperation = 1;

      MSFStatus = 0;
    }
  }

  if (moveCMD == 1) {
    if (noteRecvd == 0) {
      Serial2.println("L");
      // Serial.println("SENT");
    } else if (moveRecvd == 1) {
      Serial.println("feedback got");
    }
  } else if (moveCMD == -1) {
    if (noteRecvd == 0) {
      Serial2.println("E");
      // Serial.println("SENT");
    } else if (moveRecvd == 1) {
      // Serial.println("feedback got");
    }
  }

  if (moveHalt == 1) {
    if (moveRecvd == 0) {
      Serial2.println("*");
      // Serial.println("ENDING");

    } else if (moveRecvd == 1) {
      Serial2.println('W');
      moveHalt = 0;
      moveRecvd = 0;
      moveCMD = 0;
      Serial2.println('W');
    }
  }


  if (musicMode == 1) {
    if (midiSong == 1) {

      if (midiKeyTimer > 50) {


        if (midiCurrentKeys == 1) {

          MSFStatus = 1;
        }
        if (singleFrameOperation == 0) {
          if (midiCurrentKeys == 2) {

            MSFStatus = -1;

          } else if (midiCurrentKeys == 3) {
            moveCMD = 1;




          } else if (midiCurrentKeys == 4) {
            moveCMD = -1;
          }
        } else if (singleFrameOperation == 1) {
        }

        midiKeyTimer = 0;
        // sendCommand = 1;
      }





      if (midiCurrentChannel == 1) {  //speed cooperation mode

        if (averageKeysLoc < 58) {
          CC3ProjBright = 25;  //protect film
          CC1ProjSpeed = 25;
        } else if (averageKeysLoc >= 58 && averageKeysLoc < 76) {
          CC3ProjBright = 75;
          CC1ProjSpeed = 50;
        } else if (averageKeysLoc >= 76) {
          CC3ProjBright = 75;
          CC1ProjSpeed = 75;
        }

        if (midiAdjustedPitchBendVal != 0 && midiAdjustedPitchBendVal >= 50) {
          pitchBendReframe = map(midiAdjustedPitchBendVal, 50, 100, 0, 25);
          CC1ProjSpeed = CC1ProjSpeed + pitchBendReframe;
        } else if (midiAdjustedPitchBendVal != 0 && midiAdjustedPitchBendVal <= 50) {
          pitchBendReframe = map(midiAdjustedPitchBendVal, 0, 50, 0, 25);
          CC1ProjSpeed = CC1ProjSpeed + pitchBendReframe;
        }
      }
      if (midiCurrentChannel == 2) {  //speed exclusive mode

        if (midiAdjustedPitchBendVal != 0) {
          if (midiAdjustedPitchBendVal >= 50) {
            pitchBendReframe = map(midiAdjustedPitchBendVal, 50, 100, 50, 100);
            CC1ProjSpeed = pitchBendReframe;
          } else if (midiAdjustedPitchBendVal <= 50) {
            pitchBendReframe = map(midiAdjustedPitchBendVal, 0, 50, 0, 50);  //makes it so that the projector isnt always sitting at 50 percent brightness
            CC1ProjSpeed = pitchBendReframe;
          }
        }

        if (averageKeysLoc < 58) {
          CC2ProjBlades = 10;
        } else if (averageKeysLoc >= 58 && averageKeysLoc < 76) {
          CC2ProjBlades = 50;
        } else if (averageKeysLoc >= 76) {
          CC2ProjBlades = 75;
        } else {
          CC2ProjBlades = 50;
        }

        CC3ProjBright = 75;
      }

      if (midiCurrentChannel == 3) {
        if (averageKeysLoc < 58) {
          CC3ProjBright = 25;
        } else if (averageKeysLoc >= 58 && averageKeysLoc < 76) {
          CC3ProjBright = 50;
        } else if (averageKeysLoc >= 76) {
          CC3ProjBright = 75;
        }
        if (midiAdjustedPitchBendVal != 0) {
          if (midiAdjustedPitchBendVal >= 50) {
            pitchBendReframe = map(midiAdjustedPitchBendVal, 50, 100, 50, 75);
            CC1ProjSpeed = pitchBendReframe;
          } else if (midiAdjustedPitchBendVal <= 50) {
            pitchBendReframe = map(midiAdjustedPitchBendVal, 0, 50, 0, 49);  //makes it so that the projector isnt always sitting at 50 percent brightness
            CC1ProjSpeed = pitchBendReframe;
          }

          //MAJOR BURN RISK IN THIS MODE! RN I DONT HAVE IT SENDING THE PROJ INTO SAFEMODE BECAUSE I WOULD LIKE THE OPTION TO BURN THE FILM --
        }
      }
    }

    if (midiCurrentKeys >= 2 && singleFrameOperation == 1) {
      midiSong = 1;
      MSFStatus = 1;
    }



    // if (SingleTimer >= 75) {
    //   MSFStatus = 3;  //resets shutter to prevent jam or hold on sf mode
    // }


    moveStatus = moveCMD;

    // if (sendCommand == 1) {
    Serial2.print("<");
    Serial2.print(CC1ProjSpeed);
    Serial2.print(",");
    Serial2.print(CC2ProjBlades);
    Serial2.print(",");
    Serial2.print(CC3ProjBright);
    // Serial2.print(",");
    // Serial2.print(moveStatus);
    // Serial2.print(",");
    // Serial2.print(MSFStatus);
    Serial2.println(">");

    // }
  }
  if (midiCurrentKeys == 0) {

    singleFrameOperation = 0;

    // moveHalt = 1;
    moveRecvd = 0;
    MSFStatus = 0;
    midiSong = 0;
    // moveCMD = 0;
    noteRecvd = 0;
    midiNotesPlayedIndex = 0;
    memset(midiNotesPlayed, 0, sizeof(midiNotesPlayed));
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
        receivedChars[ndx] = '\0';
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
  char* strtokIndx;

  strtokIndx = strtok(tempChars, ",");
  FRAMERATE = atoi(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  SHUTTERANGLE = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  LEDBRIGHT = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  BLADECT = atoi(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  machineStatus = atoi(strtokIndx);


  strtokIndx = strtok(NULL, ",");
  frameCount = atoi(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  sfStatus = atoi(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  mCopyStatus = atoi(strtokIndx);



  dataConv();
}

// void usbFWD() {
//   if (Serial.available()) {
//     char c = Serial.read();
//     if (c == 'a') {
//       Serial.println("ok");
//     }
//   }
// }

void dataConv() {
  angle = 100 * SHUTTERANGLE;
  ledPercentage = map(LEDBRIGHT, 0, 4094, 0, 100);
  bladeCount = map(BLADECT, 0, 4094, 1, 4);
}

void graphics() {


  //Serial.println(uiTimer);

  if (opticalPrinter == 0) {

    if (sfStatus == 1 || sfStatus == -1) {
      sfUI = 1;
      buttonPressedAt = millis();
      if (sfStatus == 1) {
        sfDir = 1;
      } else if (sfStatus == -1) {
        sfDir = -1;
      }
      if (millis() - buttonPressedAt <= 20) {
        flicker = flicker + 1;
      }
    }

    if (sfUI == 1) {

      singleframeUI();


    }

    else if (sfUI == 0 && machineStatus == 0) {

      idle();
    }

    //  }
    else if (machineStatus == 1 || machineStatus == -1) {

      active();
    }
  } else if (opticalPrinter == 1 || opticalPrinter == 2) {
    opticalPrinterUI();
  }
}

void auxiliary() {



  while (Serial1.available() > 0) {


    char c = Serial1.read();

    if (c == 'x') {

             


      Serial.println("c");  //mcopy signifier
      //  Serial.println("recvd");
      camFC = camFC + 1;
      //opticalPrinter = 1;
      // Serial.flush();

    } else if (c == 'o') {
      // delay(250);
      Serial.println("c");  //mcopy signifier
      //        Serial.println("recvd");
      camFC = camFC - 1;
      // Serial.flush();
      //opticalPrinter = 1;

    } else if (c == 'f') {
      Serial2.println("g");  //vchanged
      fwdFlag = 1;
      fwdProjTimer = 0;
      //opticalPrinter = 1;
    } else if (c == 'r') {
      Serial2.println("h");
      //opticalPrinter = 1;
    } else if (c == 'A') {
      Serial2.println("B");
      Serial.println("B");
      capperStatus = 2;

    } else if (c == 'B') {
      Serial2.println("A");
      capperStatus = 1;
    }
  }


  while (Serial.available() > 0) {
    // delay(2);
    char c = Serial.read();
    if (c == '9') {
      opticalPrinter = 2;
    } else if (c == 'd') {
      sfcmd = 1;
    } else if (c == 'h') {
      Serial.println('h');
      pD = -1;
    } else if (c == 'g') {
      Serial.println('g');
      pD = 1;
    } else if (c == 'A') {
      Serial2.println("B");
      Serial.println("A");
      Serial1.println("A");
  
    } else if (c == 'B') {
      Serial2.println("A");
      Serial.println("B");
  
      //proj
    } else if (c == 'p' && pD == 1) {
      Serial2.println("g");
      timerUI = 0;
      pRun = 1;
    } else if (c == 'p' && pD == -1) {
      Serial2.println("h");
      timerUI = 0;
      pRun = 1;
    }else if (c == 'H') {
      Serial2.println('H');
      delay(20);
      Serial2.println("A");
      //pD = -1;
    }
    //cam
    else if (c == 'c' && cD == 1) {
      cRun = 1;
      // SoftSerial.println('f');
      // SoftSerial.flush();
      timerUI = 0;
      //       cRun = 1;
      // Serial.println("c");
    } else if (c == 'c' && cD == -1) {
      cRun = 2;
      // SoftSerial.println('r');
      // SoftSerial.flush();
      timerUI = 0;
      //       cRun = 1;
      // Serial.println("c");
    } else if (c == 'e') {
      Serial.println("e");
      cD = 1;
    } else if (c == 'f') {
      Serial.println("f");
      cD = -1;
    } else if (c == 'i') {
      Serial.println("i");
    } else if (c == 'm') {
      Serial.println("q"); //internal ident for mcopy
      opticalPrinter = 1;
      delay(200);

    } else if (c == 'l') {
      Serial.println("l");


    }
  }


  // if (Serial2.available() > 0) {
  //   char c = Serial2.read();
  //   if (c == 'A') {
  //     SoftSerial.println("A");
  //   } else if (c == 'g') {
  //     Serial.println("p");
  //     // Serial.println("g"); //check behavior
  //   } else if (c == 'h') {
  //     Serial.println("p");
  //     // Serial.println("h");
  //   } else if (c == 'B') {
  //     SoftSerial.println("B");
  //   } else if (c == '&') {
  //     if (singleFrameOperation == 0) {
  //       noteRecvd = 1;
  //     } else if (singleFrameOperation == 1 || singleFrameOperation == 2) {
  //       noteRecvd = 2;
  //     }
  //   } else if (c == '$') {
  //     moveRecvd = 1;
  //   }
  // }
  LEDBRIGHTop = LEDBRIGHT * 0.0245;

  // if (mCopyStatus == 1) {
  //   Serial.println("p");
  //   Serial2.println("u");
  //       mCopyStatus = 0;
  // }
  // else if (mCopyStatus == 3) {
  //     Serial.println("A");
  //     Serial2.println("u");
  //     mCopyStatus = 0;
  // } else if (mCopyStatus == 4) {
  //   Serial.println("B");
  //   Serial2.println("u");
  //       mCopyStatus = 0;
  // }

  if (opticalPrinter == 2) {

    Serial.print("Camera frame is ");
    Serial.println(camFC);
    Serial.print("Projector frame is ");
    if (fwdFlag == 0) {
      Serial.println(frameCount);
    } else {
      Serial.println("...");
    }
    Serial.print("Projector brightness is ");
    Serial.println(ledPercentage);
    Serial.print("Shutter is ");
    if (shutterStatus == 1) {
      Serial.println("open");
    } else if (shutterStatus == 0) {
      Serial.println("closed");
    }
  }

  if (fwdProjTimer >= 2000) {
    fwdFlag = 0;
    fwdProjTimer = 0;
  }

  if (resetFrameReq == 1) {
    serialTimer = millis();
    if (millis() - serialTimer >= 20) {
      resetFrameReq = 0;
    }
  }

  if (sfcmd == 1 || sfcmd == -1) {
    serialTimer = millis();
    if (millis() - serialTimer >= 20) {
      sfcmd = 0;
    }
  }
  if (motorcmd == 1 || motorcmd == -1) {
    serialTimer = millis();
    if (millis() - serialTimer >= 20) {
      motorcmd = 0;
    }
  }
}


void handshakeStart() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'i') {
      Serial.println('i');
    } else if (c == 'm') {
      Serial.println("9");
    }
  }
}

void handshake() {

  //for mCopy internal handshake to identify device
  if (handshakeS == 1) {
    Serial.println('i');
  } else if (handshakeS == 2) {
    Serial.println('9');
    handshakeS = 0;
    opticalPrinter = 1;
  }
}

void active() {
  u8g2.clearBuffer();
  u8g2.enableUTF8Print();
  u8g2.setFont(u8g2_font_logisoso26_tr);
  u8g2.setCursor(72, 32);
  u8g2.print(FRAMERATE);
  u8g2.setFont(u8g2_font_lastapprenticebold_tr);
  u8g2.print("FPS");
  //Serial.println(FRAMERATE);

  u8g2.setFont(u8g2_font_helvR10_tf);
  u8g2.setCursor(3, 32);
  u8g2.print(angle);
  u8g2.print("° -- ");
  u8g2.print(bladeCount);
  //Serial.print(angle);
  //Serial.println("°");

  u8g2.setCursor(3, 16);
  u8g2.print(ledPercentage);

  u8g2.sendBuffer();
}

void idle() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_maniac_tr);
  u8g2.setCursor(3, 32);
  u8g2.print("Idle");
  u8g2.setFont(u8g2_font_helvR10_tf);
  u8g2.setCursor(64, 18);
  u8g2.print("frames =");
  u8g2.setCursor(64, 32);
  u8g2.print(frameCount);
  u8g2.sendBuffer();
}

void singleframeUI() {

  unsigned long uiTimer = millis();


  u8g2.clearBuffer();
  u8g2.enableUTF8Print();
  u8g2.setDrawColor(2);
  u8g2.setFont(u8g2_font_helvB24_tf);
  u8g2.setCursor(0, 32);
  if (sfDir == 1) {
    u8g2.print("FWD");
  } else if (sfDir == -1) {
    u8g2.print("REV");
  }


  if (flicker == 0) {
    u8g2.setDrawColor(2);
    u8g2.drawBox(16, 10, 32, 32);
    u8g2.setDrawColor(2);
    u8g2.drawFrame(64, 10, 32, 32);

  } else if (flicker == 1) {
    u8g2.setDrawColor(2);
    u8g2.drawFrame(16, 10, 32, 32);
    u8g2.setDrawColor(2);
    u8g2.drawBox(64, 10, 32, 32);
  }


  if (flicker > 1) {
    flicker = 0;
  }
  u8g2.sendBuffer();

  if (uiTimer - buttonPressedAt >= 750) {

    sfUI = 0;
  }
}

void opticalPrinterUI() {

  if(capperStatus == 1) {
    brightnessReadOut();
  } else if (capperStatus == 2) {
    LEDoff();
  }
}

void brightnessReadOut() {
  u8g2.clearBuffer();
  u8g2.setCursor(3, 32);
    u8g2.setFont(u8g2_font_logisoso18_tr);
  u8g2.print("PBr. = ");
  u8g2.print(ledPercentage);
  u8g2.sendBuffer();
}

void LEDoff() {


  eyesPos = animationTimer;
  if (eyesPos - eyesPosPrev > 400){
        eyesPosKeyF = eyesPosKeyF + 0.25;
        if (eyesPosKeyF < 3) {
          eyesAnimPos = eyesAnimPos + 0.25;
        } else if (eyesPosKeyF > 3) {
          eyesAnimPos = eyesAnimPos - 0.25;
        } 
    if (eyesPosKeyF > 6) {
      eyesPosKeyF = 0;
      eyesAnimPos = 0;
    }
    eyesPosPrev = eyesPos;
  }

  zzzPos = animationTimer;
  if (zzzPos - zzzPosPrev > 2500){
        zzzPosFrame = zzzPosFrame + 1;
    if (zzzPosFrame > 1) {
      zzzPosFrame = 0;
    }
    zzzPosPrev = zzzPos;
  }


  u8g2.clearBuffer();
u8g2.setBitmapMode(1);
// sleepyeyes
u8g2.drawXBM(17, (10 + eyesAnimPos), 61, 15, image_sleepyeyes_bits);
if (zzzPosFrame == 0) {
// zzz1
u8g2.drawXBM(90, 2, 30, 24, image_zzz1_bits);

} else if (zzzPosFrame == 1) {

// zzz2
u8g2.drawXBM(85, 1, 31, 29, image_zzz2_bits);

}
  u8g2.sendBuffer();

}