// Encoder
#define EncI 17   // encoder pulse Index (AS5047 sensor)
#define EncA 16   // encoder pulse A (AS5047 sensor)
#define EncB 4    // encoder pulse B (AS5047 sensor)
#define EncCSN 5  // encoder SPI CSN Pin (AS5047 sensor) - Library also reserves GPIO 18, 19, 23 for SPI
#define EncMOSI 23
#define EncMISO 19
#define EncCLK 18

// OUTPUT PINS //
#define escPin 22         // PWM output for ESC
#define escProgramPin 21  // serial output to program ESC (needs 10k resistor to 3.3v)
#define ledPin 2          // PWM output for LED (On 38pin HiLetGo ESP32 board, GPIO 2 is the built-in LED)
#define NeoPixelPin 15    // output pin for NeoPixel status LED(s)

// INPUT PINS //
// UI
#define onboardcontrolPin n/a
#define motPotPin ADC_CHANNEL_5         // analog input for motor speed pot
#define motSlewPotPin n/a     // analog input for motor slew rate pot
#define ledPotPin ADC_CHANNEL_7         // analog input for LED dimming pot
#define ledSlewPotPin n/a     // analog input for LED dimming slew rate pot
#define shutBladesPotPin ADC_CHANNEL_3  // analog input for # of shutter blades pot
#define shutAnglePotPin ADC_CHANNEL_0   // analog input for shutter angle pot
#define motDirFwdSwitch 14   // digital input for motor direction switch (forward)
#define motDirBckSwitch 12   // digital input for motor direction switch (backward)
#define buttonApin 26        // digital input for button
#define buttonBpin 25        // digital input for button
#define safeSwitch 13        // switch to enable "safe mode" where lamp brightness is automatically dimmed at slow speeds

// AUX SERIAL PINS
#define auxReceiver 32
#define auxTransmitter 27