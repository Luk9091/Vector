#ifdef __LCD_H

#define COLOR_MODE_REG      0x3A
#define COLOR_MODE_12bitPerPixel 0x03
#define COLOR_MODE_16bitPerPixel 0x05
#define COLOR_MODE_18bitPerPixel 0x06


#define PORT_CONTROL_REG    0xB2
#define GATE_CONTROL_REG    0xB7
#define VCOM_SETTINGS       0xBB
#define LCM_CONTROL         0xC0
#define VDV_VRD_CMD_EN      0xC2
#define VRH_SET             0xC3
#define VDV_SET             0xC4

#define FRAME_RATE_CONTROL  0xC6

#define POWER_CONTROL_1     0xD0
#define POSITIVE_VOLTAGE_GAMMA_CTR 0xE0
#define NEGATIVE_VOLTAGE_GAMMA_CTR 0xE1

#define DISPLAY_INVERSION_ON        0x21
#define DISPLAY_INVERSION_OFF       0x20

#define SLEEP_OUT           0x11
#define SLEEP_IN            0x10

#define DISPLAY_ON          0x29
#define DISPLAY_OFF         0x28


#define SET_COLUMN_ADDRESS  0x2A
#define SET_ROW_ADDRESS     0x2B
#define MEMORY_WRITE        0x2C

#endif