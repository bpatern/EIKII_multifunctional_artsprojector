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
static void motorCommander(char action, int isStill = 0);
void fixCount();
void shutterQueue(byte shutterBlades, float shutterAngle);
void readShutterControls(void *parameter);
void pressed(Button2& btn);
void released(Button2& btn);
void change(Button2& btn);
static void parseIO(void *pvparemeters);
static void updateStatusLED(int x, int R, int G, int B);
void lightSetup();
// prototypes for ISR functions that will be defined in later code
// (prototype must be declared _before_ we attach interrupt because ESP32 requires "IRAM_ATTR" flag which breaks typical Arduino behavior)
// static void IRAM_ATTR send_LEDC(void *arg);
void IRAM_ATTR debugTask(void *pvParameters);
static void IRAM_ATTR gpioGet(void *arg);
static void IRAM_ATTR shutterRead(spi_transaction_t *val);
void IRAM_ATTR processVal(void *pvParameters);
void IRAM_ATTR configEncoderSPI(uint16_t coreID);
void IRAM_ATTR readAngle_raw (void *pvParams); 


// static void IRAM_ATTR gpioGet(void *arg);

double mapf(double x, double in_min, double in_max, double out_min, double out_max);

static float fscale(float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve);

void as5047Config();
void gpioConfig();

static inline uint32_t example_angle_to_compare(int angle);