

static char shutMapIRQ[150]; // local variable to hold the shutter map value received from the queue, which will be used to update the LED state in real time in the ISR based on how many blades are actually open at any given moment
static int actualShutterMapPosition;

static int shutterSize = 150;
static int shutterSegments = 3;
static int segmentSize = 40;

static int shutterDiv;
static int shutterOn;

static int ledWriteCount;
static IRAM_ATTR int ledBright;
static IRAM_ATTR int openClOld;

static IRAM_ATTR int ledBrightOld;

static IRAM_ATTR int openCl;
static IRAM_ATTR uint64_t ledWrite_period;

static gptimer_handle_t watchdawg = NULL;

void IRAM_ATTR send_LEDC()
{

    xQueueReceive(ledPot, &ledBright, 1); // receive LED brightness value from LED task via queue

    xQueueReceive(q_shutterMap, &openCl, 1); // receive shutter blade value from UI task via queue

    if (openCl == 0)
    {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    }
    else if (openCl == 1)
    {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, ledBright);
    }
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

}