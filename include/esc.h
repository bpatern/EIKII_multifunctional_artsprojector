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
