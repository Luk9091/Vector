#include "lcd_display.h"
#include "lcd_display_register.h"

#define X_MAX LCD_WIDTH/3
#define Y_MAX LCD_HEIGHT

uint layout[X_MAX][Y_MAX];

#if ANALOG_BRIGHTNESS
uint slice;
#endif

inline void SPI_writeByte(uint8_t data){
    spi_write_blocking(spi1, &data, 1);
}

inline void SPI_writeNByte(uint8_t *data, size_t size){
    spi_write_blocking(spi1, data, size);
}

static void LCD_Reset(void)
{
    gpio_put(LCD_RST_PIN, 1);
    sleep_ms(100);
    gpio_put(LCD_RST_PIN, 0);
    sleep_ms(100);
    gpio_put(LCD_RST_PIN, 1);
    sleep_ms(100);
}

static void sendCommand(uint8_t Reg)
{
    gpio_put(LCD_DC_PIN, 0);
    gpio_put(LCD_CS_PIN, 0);
    SPI_writeByte(Reg);
    gpio_put(LCD_CS_PIN, 1);
}

static void sendData_8b(uint8_t Data)
{
    gpio_put(LCD_DC_PIN, 1);
    gpio_put(LCD_CS_PIN, 0);
    SPI_writeByte(Data);
    gpio_put(LCD_CS_PIN, 1);
}

static void LCD_SendData_16Bit(uint16_t Data)
{
    gpio_put(LCD_DC_PIN, 1);
    gpio_put(LCD_CS_PIN, 0);
    SPI_writeByte((Data >> 8) & 0xFF);
    SPI_writeByte(Data & 0xFF);
    gpio_put(LCD_CS_PIN, 1);
}

static void LCD_InitReg(void)
{
    sendCommand(COLOR_MODE_REG);
    sendData_8b(COLOR_MODE_12bitPerPixel);

    sendCommand(PORT_CONTROL_REG);
    sendData_8b(0x0C);
    sendData_8b(0x0C);
    sendData_8b(0x00);
    sendData_8b(0x33);
    sendData_8b(0x33);

    sendCommand(GATE_CONTROL_REG);
    sendData_8b(0x35);

    sendCommand(VCOM_SETTINGS);
    sendData_8b(0x19);

    sendCommand(LCM_CONTROL);
    sendData_8b(0x2C);

    sendCommand(VDV_VRD_CMD_EN);  //VDV and VRH Command Enable
    sendData_8b(0x01);
    sendCommand(VRH_SET);  //VRH Set
    sendData_8b(0x12);
    sendCommand(VDV_SET);  //VDV Set
    sendData_8b(0x20);

    sendCommand(FRAME_RATE_CONTROL);  //Frame Rate Control in Normal Mode
    sendData_8b(0x0F);
    
    sendCommand(POWER_CONTROL_1);  // Power Control 1
    sendData_8b(0xA4);
    sendData_8b(0xA1);

    sendCommand(POSITIVE_VOLTAGE_GAMMA_CTR);  //Positive Voltage Gamma Control
    sendData_8b(0xD0);
    sendData_8b(0x04);
    sendData_8b(0x0D);
    sendData_8b(0x11);
    sendData_8b(0x13);
    sendData_8b(0x2B);
    sendData_8b(0x3F);
    sendData_8b(0x54);
    sendData_8b(0x4C);
    sendData_8b(0x18);
    sendData_8b(0x0D);
    sendData_8b(0x0B);
    sendData_8b(0x1F);
    sendData_8b(0x23);

    sendCommand(NEGATIVE_VOLTAGE_GAMMA_CTR);  //Negative Voltage Gamma Control
    sendData_8b(0xD0);
    sendData_8b(0x04);
    sendData_8b(0x0C);
    sendData_8b(0x11);
    sendData_8b(0x13);
    sendData_8b(0x2C);
    sendData_8b(0x3F);
    sendData_8b(0x44);
    sendData_8b(0x51);
    sendData_8b(0x2F);
    sendData_8b(0x1F);
    sendData_8b(0x1F);
    sendData_8b(0x20);
    sendData_8b(0x23);

    sendCommand(DISPLAY_INVERSION_ON);  //Display Inversion On

    sendCommand(SLEEP_OUT);  //Sleep Out

    sendCommand(DISPLAY_ON);  //Display On
}

static void LCD_SetAttributes(uint8_t Scan_dir)
{
    uint8_t MemoryAccessReg = 0x00;

    if(Scan_dir == HORIZONTAL) {
        MemoryAccessReg = 0X70;
    } else {
        MemoryAccessReg = 0X00;
    }

    sendCommand(0x36);
    sendData_8b(MemoryAccessReg);
}

void GPIO_Init(){
    gpio_init(LCD_RST_PIN);
    gpio_init(LCD_DC_PIN);
    gpio_init(LCD_CS_PIN);
    
    gpio_set_dir(LCD_RST_PIN, GPIO_OUT);
    gpio_set_dir(LCD_DC_PIN, GPIO_OUT);
    gpio_set_dir(LCD_CS_PIN, GPIO_OUT);

    gpio_put(LCD_CS_PIN, 1);
    gpio_put(LCD_DC_PIN, 1);

#if ANALOG_BRIGHTNESS == false
    gpio_init(LCD_BL_PIN);
    gpio_set_dir(LCD_BL_PIN, GPIO_OUT);
    gpio_put(LCD_BL_PIN, 0);
#else
    // PWM
    gpio_set_function(LCD_BL_PIN, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(LCD_BL_PIN);
    pwm_set_wrap(slice, 100);
    pwm_set_clkdiv_mode(slice, 50);
    pwm_set_chan_level(slice, PWM_CHAN_B, 1);
    pwm_set_enabled(slice, true);
#endif
    
}

void LCD_Init(uint8_t Scan_dir)
{
    GPIO_Init();    
    LCD_Reset();

    LCD_SetAttributes(Scan_dir);
    
    LCD_InitReg();
}

void LCD_brightness(uint value){
#if ANALOG_BRIGHTNESS == false
    gpio_put(LCD_BL_PIN, value);
#else
    if (value > 100){
        value = 100;
    }
    pwm_set_chan_level(slice, PWM_CHAN_B, value);
#endif
}

void LCD_SetWindows(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    //set the X coordinates
    sendCommand(SET_COLUMN_ADDRESS);
    sendData_8b((x0 >> 8) & 0xFF);
    sendData_8b(x0 & 0xFF);
    sendData_8b(((x1  - 1) >> 8) & 0xFF);
    sendData_8b((x1  - 1) & 0xFF);

    //set the Y coordinates
    sendCommand(SET_ROW_ADDRESS);
    sendData_8b((y0 >> 8) & 0xFF);
    sendData_8b(y0 & 0xFF);
    sendData_8b(((y1  - 1) >> 8) & 0xFF);
    sendData_8b((y1  - 1) & 0xFF);

    sendCommand(0X2C);
}


void LCD_Clear(uint8_t Color)
{
    uint red   = (Color >> 0) & 0x03;
    uint green = (Color >> 2) & 0x03;
    uint blue  = (Color >> 4) & 0x03;

    red =   (((red   & 0b10) >> 1) * 0b1100) | ((red   & 0b01) * 0b0011);
    green = (((green & 0b10) >> 1) * 0b1100) | ((green & 0b01) * 0b0011);
    blue =  (((blue  & 0b10) >> 1) * 0b1100) | ((blue  & 0b01) * 0b0011);

    uint16_t data = 0;
    data = red << 8 | green << 4 | blue;
    
    LCD_SetWindows(0, 0, LCD_WIDTH, LCD_HEIGHT);
    gpio_put(LCD_DC_PIN, 1);
    gpio_put(LCD_CS_PIN, 0);
    for (uint y = 0; y < LCD_HEIGHT - 60; y++){
        for (uint x = 0; x < LCD_WIDTH; x++){
    // for (uint y = 0; y < 12; y++){
    //     for (uint x = 0; x < 34; x++){
            data = data << 4;
            switch(x%3){
                case 0:
                    data |= red;
                    break;
                case 1:
                    data |= green;
                    break;
                case 2:
                    data |= blue;
                    break;

                default:
                    data = 0;
                    gpio_put(LCD_CS_PIN, 1);
                    return;
            }

            SPI_writeByte((uint8_t)((data >> 8) & 0xFF));
            SPI_writeByte((uint8_t)(data & 0xFF));
            LCD_DisplayPoint(x, y, Color);
        }
    }
    gpio_put(LCD_CS_PIN, 1);

}
   

void LCD_Display(){
    uint8_t data;
    LCD_SetWindows(0, 0, LCD_WIDTH, LCD_HEIGHT);
    gpio_put(LCD_DC_PIN, 1);
    gpio_put(LCD_CS_PIN, 0);
    for (uint y = 0; y < X_MAX; y++){
        for (uint x = 0; x < Y_MAX; x++){
            for (uint i = i; i < 3; i++){
                data  = 0b1100 * (bool)(layout[x/3][y] >> (17 - 2*i));
                data |= 0b0011 * (bool)(layout[x/3][y] >> (16 - 2*i));
                data = data << 4;
                data |= 0b1100 * (bool)(layout[x/3][y] >> (15 - 2*i));
                data |= 0b0011 * (bool)(layout[x/3][y] >> (14 - 2*i));
                SPI_writeByte(data);
            }
        }
    }
    gpio_put(LCD_CS_PIN, 1);
}

void LCD_DisplayPoint(uint8_t x, uint8_t y, uint8_t color){
    color = color & 0x3F;
    layout[x/3][y] &=~ (0x3F) << (12 - (6 * x%3));
    layout[x/3][y] |=  color  << (12 - (6 * x%3));
}

