/*
DMA SPI driver for AS5047 for asynchronous shutter positioning

ported from ESP-IOT-SOLUTION's simpleFOC fork that has an spi-master based implementation of as5xxx fetching
code is modified for this specific task
requires the definition of a shutter count in main config file

*/

// Volatile Registers Addresses
#define NOP_REG			0x0000
#define ERRFL_REG 		0x0001
#define PROG_REG		0x0003
#define DIAGAGC_REG 	0x3FFC
#define MAG_REG 		0x3FFD
#define ANGLE_REG 		0x3FFE
#define ANGLECOM_REG 	0x3FFF

// Non-Volatile Registers Addresses
#define ZPOSM_REG 		0x0016
#define ZPOSL_REG 		0x0017
#define SETTINGS1_REG 	0x0018
#define SETTINGS2_REG 	0x0019

    static spi_host_device_t enc = SPI3_HOST;
    static spi_device_handle_t encoder;

    static uint8_t rx_buffer[2] = {};

uint8_t IRAM_ATTR calculateParity(uint16_t value)
{
    uint8_t count = 0;
    for (int i = 0; i < 16; i++) {
        if (value & 0x1) {
            count++;
        }
        value >>= 1;
    }
    return count & 0x1;
}

void configEncoderSPI(uint16_t coreID) {
    esp_err_t ret;

    // ret = spi_bus_remove_device(_spi_device);
    // ESP_RETURN_ON_FALSE(ret == ESP_OK,, TAG, "SPI remove device fail");
    // ret = spi_bus_free(_spi_host);
    // ESP_RETURN_ON_FALSE(ret == ESP_OK,, TAG, "SPI free fail");
    // _is_installed = false;

    spi_bus_config_t buscfg = {
        .mosi_io_num = (gpio_num_t)EncMOSI,
        .miso_io_num = (gpio_num_t)EncMISO,
        .sclk_io_num = (gpio_num_t)EncCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1000,
    };

    spi_device_interface_config_t devcfg = {

        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
                .mode = 1,

                .cs_ena_pretrans = 1,
        .clock_speed_hz = 300000,
.input_delay_ns = 350,

        .spics_io_num = (gpio_num_t)EncCSN,
        .queue_size = 1
    };
    ret = spi_bus_initialize(enc, &buscfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);
    ret = spi_bus_add_device(enc, &devcfg, &encoder);
    assert(ret == ESP_OK);

    xTaskCreatePinnedToCore(
        readAngle_raw, 
        "read angle",
        3000,
        NULL,
        configMAX_PRIORITIES - 9,
        NULL,
        coreID
    );


}

void IRAM_ATTR readAngle_raw (void *pvParams) {



    for (;;) {

// if(xSemaphoreTake(encoderRead, 2) == pdTRUE) {
    

    uint16_t command = 1 << 14; // PAR=0 R/W=R (Read command)
    command |= ANGLE_REG; // Command to read angle
    command |= ((uint16_t)calculateParity(command) << 15); // Adding parity bit

    uint8_t cmd_high = (command >> 8) & 0xFF;
    uint8_t cmd_low = command & 0xFF;
    uint8_t tx_buffer[2] = {cmd_high, cmd_low};

    spi_transaction_t t = {};
        memset(&t, 0, sizeof(t));       //Zero out the transaction

    t.length = 16; // 16 bits
    t.tx_buffer = tx_buffer;
    t.rx_buffer = NULL;
    spi_device_transmit(encoder, &t);

        memset(&t, 0, sizeof(t));       //Zero out the transaction

    t.length = 16; // 16 bits
    t.tx_buffer = NULL;
    t.rxlength = 16; // 16 bits
    t.rx_buffer = &rx_buffer;
    spi_device_transmit(encoder, &t);
    // xSemaphoreGive(encoderRead);
    // }
}

}

uint16_t IRAM_ATTR readAngleBuffer (bool getraw)
{
    uint16_t reg_value;
    if (getraw == true) {
     reg_value = (rx_buffer[0] << 8) | rx_buffer[1];

    } else if (getraw == false) {
    }
    return reg_value & ANGLE_REG;

}