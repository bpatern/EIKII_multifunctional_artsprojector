
// external communication variables and settings//
int motExtSwitch = 0;
volatile int motModeMidi;
int musicMode = 0;

uint64_t ledtimer;

QueueHandle_t outCommanderQueue, customBrightness, q_singleFrame;
QueueHandle_t q_shutterBlade, q_shutterAngle, q_ledBright, q_motSpeed, q_spiRead, q_actualShutterMap;
QueueHandle_t motPot, ledPot;
QueueHandle_t q_intr1_hasRun, q_intr2_hasRun, q_debugShutterPosition;
QueueHandle_t q_motRunFwd, q_motRunBack, q_safemode, q_buttonF, q_buttonR, q_whichbutton;
QueueHandle_t q_shutterMap, q_peekLED;
SemaphoreHandle_t led_iswriting = NULL;

SemaphoreHandle_t controlLock = NULL;
portMUX_TYPE shutterFunctionLock = portMUX_INITIALIZER_UNLOCKED; // create a spinlock to protect the shutter function that is called in the ISR and uses shared variables
portMUX_TYPE shutterMappingLock = portMUX_INITIALIZER_UNLOCKED;  // create a spinlock to protect the shutter mapping function that is called in the shutter map update task and uses shared variables
SemaphoreHandle_t shutterMapping;
SemaphoreHandle_t tx;

SemaphoreHandle_t ledCl = NULL;
SemaphoreHandle_t encoderRead = NULL;
SemaphoreHandle_t motor_isRunning = NULL;
SemaphoreHandle_t singleFraming = NULL;
QueueHandle_t runMsg = NULL;

SemaphoreHandle_t physinput = NULL;
QueueHandle_t ioQ = NULL;

TaskHandle_t ioTASKHANDLE = NULL;
const char *ioTH = "io task handle";

TaskHandle_t motContinuousHandle = NULL;
const char *motCont = "Continuous Motor Function";

TaskHandle_t singleFrame = NULL;
const char *sFF = "Single Frame Function";

TaskHandle_t ledDraw = NULL;
const char *ledDr = "LED controller";

TaskHandle_t shutterPotTranslate = NULL;
const char *shPot = "Shutter Potentiometer";

TaskHandle_t debugPrinter = NULL;
const char *db = "Debug Printer";

TaskHandle_t motorSlewRead = NULL;
const char *slew = "Slewer";

TaskHandle_t FPSactor = NULL;
const char *fpsCalc = "FPS calculator";

TaskHandle_t externalControlParse = NULL;
const char *extCont = "External Control";

TaskHandle_t internalSerialRX = NULL;
const char *iRx = "Internal Serial RX";

TaskHandle_t internalSerialTX = NULL;
const char *iTx = "Internal Serial TX";

TaskHandle_t readControls = NULL;
const char *controlR = "Read LED and motor Pot";

TaskHandle_t fancontrol = NULL;
const char *fancont = "Fan Control unit";

const char *genPurp = "General Purpose";

float shAngF;
float *shAngFptr = &shAngF;

gptimer_handle_t ledtick = NULL;

QueueHandle_t q_actor = NULL;

uint8_t sendingIndividualCommand = 0; // flag to indicate whether we're sending an individual command or the full data string. if 1, we send individual command, if 0 we send full data string. this is to prevent flooding the commander with the full data string when we're just trying to send a single command (like a shutter open/close command from the aux display)
mcpwm_cmpr_handle_t comparator = NULL;
mcpwm_comparator_config_t comparator_config;
mcpwm_oper_handle_t motOper = NULL;
mcpwm_timer_handle_t motPWMTimer = NULL;
mcpwm_gen_handle_t motGenerator = NULL;

int sfRequest;
int runRequest;

int frameCountResetRequest;
int opticalPrinter = 0;
int opAlignment = 0;
int capFlag = 0;
int mCopyStatus = 0;
int pullDownPos = 20;
int scanFlag = 0;

// ext midi variables -- may use CC2 for optical printer
int CC1ProjSpeed;
int CC3ProjBright;
int CC2ProjBlades;
int moveStatus;
int MSFStatus;
int motSingleRun;
int recSpecs;

int receivedRecvdConfirm2;
int receivedRecvdConfirm;

int ledSlewMin = 0;     // the minumum slew value when knob is turned down (msec).
int ledSlewMax = 10000; // the max slew value when knob is turned up (msec).

int motSlewVal = 0;
int motSlewValOld = 0;
int motSlewMin = 0;     // the minumum slew value when knob is turned down (msec).
int motSlewMax = 10000; // the max slew value when knob is turned up (msec).

// static uint32_t IRAM_ATTR shutterMap[150] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//  int *shutterMap = (int *)malloc(sizeof(int) * countsPerFrame);
uint16_t ang;
uint16_t *angle = &ang;

int motDirFwdState = 0;
int motDirBckState = 0;
int buttonApinState = 0;
int *btnA = &buttonApinState;
int buttonBpinState = 0;
int *btnB = &buttonBpinState;

IRAM_ATTR uint32_t shutterMap[1024];

// UI VARS (could also be updated by remote control in future. Put these in a struct for easier radiolib management?)
int motPotFPS = 1; // current requested FPS based on motPotVal and scaling
int motPotFPSOld = 0;
int ledPotValOld;

int motPotVal; // current value of Motor pot (not necessarily the current speed since we might be ramping toward this value)
int ledPotVal; // current value of LED pot (not necessarily the current LED brightness since we might be fading or strobing)
int shutBladesPotVal;
int shutAnglePotVal;
int shutBladesVal;
int shutPotEstimate = 0;
int shutAngEstimate = 0;
  int ledSlewValOld;
  int ledSlewVal;
  int safeModeIs;

// LED VARS //
     int ledSwitch = 0;
     int motSwitch = 0;

int ledB = 0;
int *ledBp = &ledB;
int LedDimMode = 1; // 0 = current-controlled dimming (NOT YET IMPLEMENTED!), 1 = PWM dimming
int LedInvert = 1;  // set to 1 to invert LED output signal so it's active-low (required by H6cc driver board)
// int ledBright = 0;                // current brightness of LED (range depends on Res below. If we're ramping then this will differ from pot value)
float safeSpeedMult = 0.4;   // multiplier to use in "safe mode" to lower lamp brightness at low FPS
#define ledBrightRes 12;     // bits of resolution for LED dimming
#define ledBrightFreq 17000; // PWM frequency (500Hz is published max for H6cc LED driver, 770Hz is closer to shutter segment period, 1000 seems to work best)
#define ledChannel 0;        // ESP32 LEDC channel number. Pairs share settings (0/1, 2/3, 4/5...) so skip one to insure your settings work!

// MOTOR VARS We are using ESP32 LEDC to drive the RC motor ESC via 1000-2000uS PWM @50Hz (standard servo format)
int motMode = 0;              // motor run mode (-1, 0, 1)
int motModePrev = 0;          // previous motor run mode (-1, 0, 1)
int motSingle = 0;            // indicates that we're traveling in single frame mode
int motSinglePrev = 0;        // were we in single frame mode during the last loop?
int motSingleMinTime = 800;   // the minimum movement time (MS) for each single frame move
volatile int motModeReal = 0; // Current running direction detected by encoder (-1, 1)
float motSpeedUS = 1500;      // speed of motor (in pulsewidth uS from 1000-2000)
#define motPWMRes 16          // bits of resolution for extra control (standard servo lib uses 10bit)

#define motPWMChannel 2                  // ESP32 LEDC channel number. Pairs share settings (0/1, 2/3, 4/5...) so skip one to insure your settings work!
#define motPWMFreq 50                    // PWM frequency (50Hz is standard for RC servo / ESC)
int motPWMPeriod = 1000000 / motPWMFreq; // microseconds per pulse

// volatile bool shutterMap[countsPerFrame];  // array holding values for lamp state at each position of digital shutter

volatile bool shutterStateOld = 0; // stores the on/off state of the shutter from previous encoder position

// Machine Status Variables
long frame; // current frame number (frame counter)

long frameOld = 0;           // old frame number for encoder
long frameOldsingle = 0;     // old frame number for encoder
volatile float FPScount = 0; // measured FPS, sourced from encoder counts (100 updates per frame)
volatile float FPSframe = 0; // measured FPS, sourced from frame counts
float FPSreal = 0;           // non-volatile FPS, sourced from encoder counts at slow speeds and frame counts at high speeds
float FPSrealAvg = 0;        // non-volatile measured FPS after averaging for jitter reduction
float FPStarget = 0;         // requested FPS

volatile float myFPScount = FPScount;     // copy volatile FPS to nonvolatile variable so it's safe
volatile float myFPSframe = FPSframe;     // copy volatile FPS to nonvolatile variable so it's safe
volatile int myMotModeReal = motModeReal; // copy volatile FPS to nonvolatile variable so it's safe

// calcFPS variables. esp memory was dumping them before...
bool newData = false;
#define numChars 32
char receivedChars[numChars];
char tempChars[numChars];

#define ESC_WRITE_BIT_TIME_WIDTH 2500 // uSec length of each pulse ("2500" = 400 baud)
char ESC_settings[] = {
    0,               // RPM Throttle Matching enabled
    2,               // 3S LiPo cells
    0,               // "Low" battery cutoff threshold (Low / Med / High)
    1,               // 105C temp cutoff
    0,               // CCW rotation
    0,               // 6V BEC
    0,               // Drag brake force disabled
    3,               // Drag brake rate 5
    3,               // Max reverse force 100%
    5, 0, 4, 2, 0, 0 // extra bytes sniffed with logic analyzer, don't know what they're for
};
