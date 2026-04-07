inline float constrain(float amt, float low, float high) {
    float constraint = ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
    return constraint;
}

inline float map(float x, float in_min, float in_max, float out_min, float out_max) {
  const float run = in_max - in_min;
  if (run == 0) {
    // log_e("map(): Invalid input range, min == max");
    return -1;  // AVR returns -1, SAM returns 0
  }
  const float rise = out_max - out_min;
  const float delta = x - in_min;
  return (delta * rise) / run + out_min;
} 

double mapf(double x, double in_min, double in_max, double out_min, double out_max);



void externalcontrol();
void midiControlTakeover();
void recvWithStartEndMarkers();
void midiParser();
void uartConfig();
void serialReadTask(void *pvparemeters);
void ESC_send_ACK();
void ESC_ser_write(unsigned char x);

void createTasks();
void serial2RX(void * parameter);
void serial2tx(void * parameter);
void sendConfirmation(char type);
void updateMotor(void *pvParameters);
void updateLed(void *pvParameters);
void as5047MagCheck(void *pvParameters);
void calcFPS(void *pvParameters);
void readUI(void *pvParameters);
void readEncoder(void *pvParameters);
void ESCprogram();
void fixCount();
void shutterQueue(char shutterBlades, float shutterAngle);
void readShutterControls(void *parameter);
// void pressed(Button2& btn);
// void released(Button2& btn);
// void change(Button2& btn);
 void parseIO(void *pvparemeters);
 void updateStatusLED(int x, int R, int G, int B);
void lightSetup();
// prototypes for ISR functions that will be defined in later code
// (prototype must be declared _before_ we attach interrupt because ESP32 requires "IRAM_ATTR" flag which breaks typical Arduino behavior)
// static void IRAM_ATTR send_LEDC(void *arg);
void IRAM_ATTR debugTask(void *pvParameters);
 void IRAM_ATTR singleFrameActor(char fromwhere, uint8_t frombutton, uint8_t level);

void mathConfig();
void motorConfig();


void IRAM_ATTR switchGet(void *arg);
void IRAM_ATTR processVal(void *pvParameters);
void IRAM_ATTR configEncoderSPI(uint16_t coreID);
uint16_t IRAM_ATTR readAngle_raw (uint16_t Addr); 


// static void IRAM_ATTR gpioGet(void *arg);


// 
void as5047Config();
void gpioConfig();

inline uint32_t example_angle_to_compare(int angle);