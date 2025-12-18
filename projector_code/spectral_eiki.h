// Use this file to define projector-specific configurations, 
// so you don't need to modify the main code for each projector you build

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// ---> PLEASE PLEASE PLEASE DOUBLE CHECK YOUR GPIO ASSIGNMENTS! YOU GENERALLY DO NOT HAVE TO FOLLOW MINE BUT LIKE DONT PUT AN ANALOG PIN INPUT ON A MCU PIN THAT ISNT ANALOG <--- ////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// BASIC SETUP OPTIONS (enable the options based on your hardware choices)
#define enableShutter 1      // 0 = LED stays on all the time (in case physical shutter is installed), 1 = use encoder to blink LED for digital shutter
#define enableShutterPots 1  // 0 = use hard-coded shutterBlades and shutterAngle variables, 1 = use pots to control these functions
#define enableSlewPots 0     // 0 = use hard-coded ledSlewMin and motSlewMin variables, 1 = use pots to control these functions
#define motorSpeedMode 0     // 0 = motor speed is controlled by hard-coded motMinUS & motMaxUS variables, 1 = controlled by closed loop feedback
#define enableMotSwitch 1    // 0 = single pot for motor direction/speed (center = STOP), 1 = use switch for FWD-STOP-BACK & pot for speed
#define motSwitchMode 1       // if enableMotSwitch = 1, how is motor direction controlled? 
                              // 0 = (P26) Use motDirBckSwitch for FWD/REV & motDirFwdSwitch for RUN/STOP
                              // 1 = (Eiki) Use motDirFwdSwitch for RUN FWD, motDirBckSwitch for RUN REV, else STOP
#define enableSafeSwitch 1   // 0 = use hard-coded safeMode variable to limit LED brightness, 1 = use switch to enable/disable safe mode
int safeMode = 0;            // If enableSafeSwitch == 0 then use this variable: 0 = normal, 1 = ledBright is limited by speed and shutter angle to prevent film burns
#define enableButtons 1      // 1 = use buttons A and B (for single frame FORWARD / BACK)
#define enableStatusLed 1    // 1 = use NeoPixel status LED(s) for user feedback

// INPUT PINS //
// UI
#define onboardcontrolPin n/a
#define motPotPin 33         // analog input for motor speed pot
#define motSlewPotPin n/a     // analog input for motor slew rate pot
#define ledPotPin 35         // analog input for LED dimming pot
#define ledSlewPotPin n/a     // analog input for LED dimming slew rate pot
#define shutBladesPotPin 39  // analog input for # of shutter blades pot
#define shutAnglePotPin 36   // analog input for shutter angle pot
#define motDirFwdSwitch 14   // digital input for motor direction switch (forward)
#define motDirBckSwitch 12   // digital input for motor direction switch (backward)
#define buttonApin 26        // digital input for button
#define buttonBpin 25        // digital input for button
#define safeSwitch 13        // switch to enable "safe mode" where lamp brightness is automatically dimmed at slow speeds

#define auxReceiver 32
#define auxTransmitter 27

const int countsPerFrame = 100;   // how many encoder transitions per frame
const int encoderDir = 0;   // Which magnet spin direction matches forward movement? 0 = clockwise, 1 = counter-clockwise
const float FPSmultiplier = 1; // difference between computed and actual FPS (P26 is not 1:1) 

// If (enableShutterPots == 1) then these will be reset later, but we set defaults here
int shutterBlades = 2;     // How many shutter blades: (minimum = 1 so lower values will be constrained to 1)
float shutterAngle = 0.4;  // float shutter angle per blade: 0= LED always off, 1= LED always on, 0.5 = 180d shutter angle

//in order to set the brightness pot to scale properly, the following must first be configured!!! use this to set arbitrary limits or configure the LED driver buck injection constraints

int ledMin = 1373; //set this at zero first, observe voltage where led is completely off OR set to minimum brightness allowed
int ledMax = 2650; //set this at 4096 first, observe voltage where led is on. do not look at led without shades. a scope might be helpful here
const float safeMin = 0.4;       // The minimum brightness in safe mode when running very slowly / stopped

// Debug messages. Use only one. (Warning: debug messages might cause loss of shutter sync. Turn off if not needed.)
int debugEncoder = 0;   // serial messages for encoder count and shutterMap value
int debugUI = 0;        // serial messages for user interface inputs (pots, buttons, switches)/Users/brandonpaterno/Downloads/film_core_8mm_2in_np.stl
int debugFrames = 0;    // serial messages for frame count and FPS
int debugFPSgraph = 0;  // serial messages for only FPSrealAvg (for Arduino IDE serial plotter)
int debugMotor = 0;     // serial messages for motor info
int debugLed = 1;       // serial messages for LED info (LED pot val, safe multiplier, computed brightness, shutter blades and angle)


int motMinUS = 1772.66;       // motor pulse length at -24fps (set this by testing)
int motMaxUS = 1231.04;       // motor pulse length at +24fps (set this by testing)
int minUSoffset = 150;     // used to enforce a minumum speed when running. Otherwise low motor speed = stopped
int minUSoffsetPrinting = 80;
// int motMinUS = 1800;       // motor pulse length at -24fps (set this by testing)
// int motMaxUS = 1210;       // motor pulse length at +24fps (set this by testing)
//int minUSoffset = 80;      // used to enforce a minumum speed when running. Otherwise low motor speed = stopped

int singleFPS = 4;        // what speed to use for film movement in single frame mode