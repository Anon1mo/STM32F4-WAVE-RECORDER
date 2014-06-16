/*
*******************************************************************************
  Copyright (C), 2012-2014, Embest Tech. Co., Ltd.
  FileName     : stm32f4_discovery_lcd.h
  Version      : 1.0.0
  Date         : 2012/05/29
  Description  :
  Function List:
  History      :
  <author>     : lichy
  <time>       : 2012/05/29
  <version >   : 1.0.0
  <desc>       : build this moudle
*******************************************************************************
  This driver was modified by changing some code and adding useful functions.
   Added functions have names started with 'x_' prefix.
  Date of last change: 3 X 2013
  Author of modification: Jan Szemiet
*******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4_DISCOVERY_LCD_H
#define __STM32F4_DISCOVERY_LCD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "fonts.h"

/** @addtogroup Utilities
  * @{
  */

/** @addtogroup STM32F4_DISCOVERY
  * @{
  */

/** @addtogroup STM32F4_DISCOVERY
  * @{
  */

/** @addtogroup STM32F4_DISCOVERY_LCD
  * @{
  */


/** @defgroup STM32F4_DISCOVERY_LCD_Exported_Types
  * @{
  */
typedef struct
{
  int16_t X;
  int16_t Y;
} Point, * pPoint;
/**
  * @}
  */


/**
  * @brief Various internal SD2119 registers name labels
  */
#define SSD2119_DEVICE_CODE_READ_REG  0x00
#define SSD2119_OSC_START_REG         0x00
#define SSD2119_OUTPUT_CTRL_REG       0x01
#define SSD2119_LCD_DRIVE_AC_CTRL_REG 0x02
#define SSD2119_PWR_CTRL_1_REG        0x03
#define SSD2119_DISPLAY_CTRL_REG      0x07
#define SSD2119_FRAME_CYCLE_CTRL_REG  0x0B
#define SSD2119_PWR_CTRL_2_REG        0x0C
#define SSD2119_PWR_CTRL_3_REG        0x0D
#define SSD2119_PWR_CTRL_4_REG        0x0E
#define SSD2119_GATE_SCAN_START_REG   0x0F
#define SSD2119_SLEEP_MODE_1_REG      0x10
#define SSD2119_ENTRY_MODE_REG        0x11
#define SSD2119_SLEEP_MODE_2_REG      0x12
#define SSD2119_GEN_IF_CTRL_REG       0x15
#define SSD2119_PWR_CTRL_5_REG        0x1E
#define SSD2119_RAM_DATA_REG          0x22
#define SSD2119_FRAME_FREQ_REG        0x25
#define SSD2119_ANALOG_SET_REG        0x26
#define SSD2119_VCOM_OTP_1_REG        0x28
#define SSD2119_VCOM_OTP_2_REG        0x29
#define SSD2119_GAMMA_CTRL_1_REG      0x30
#define SSD2119_GAMMA_CTRL_2_REG      0x31
#define SSD2119_GAMMA_CTRL_3_REG      0x32
#define SSD2119_GAMMA_CTRL_4_REG      0x33
#define SSD2119_GAMMA_CTRL_5_REG      0x34
#define SSD2119_GAMMA_CTRL_6_REG      0x35
#define SSD2119_GAMMA_CTRL_7_REG      0x36
#define SSD2119_GAMMA_CTRL_8_REG      0x37
#define SSD2119_GAMMA_CTRL_9_REG      0x3A
#define SSD2119_GAMMA_CTRL_10_REG     0x3B
#define SSD2119_V_RAM_POS_REG         0x44
#define SSD2119_H_RAM_START_REG       0x45
#define SSD2119_H_RAM_END_REG         0x46
#define SSD2119_X_RAM_ADDR_REG        0x4E
#define SSD2119_Y_RAM_ADDR_REG        0x4F

#define ENTRY_MODE_DEFAULT 0x6820//0x6830
#define ENTRY_MODE_BMP 	   0x6810
#define MAKE_ENTRY_MODE(x) ((ENTRY_MODE_DEFAULT & 0xFF00) | (x))

/**
  * @brief The dimensions of the LCD panel.
  */
#define LCD_VERTICAL_MAX   240
#define LCD_HORIZONTAL_MAX 320

/**
  * @brief Various definitions controlling coordinate space mapping and drawing
           direction in the four supported orientations.
  */
#define PORTRAIT

#ifdef PORTRAIT
#define HORIZ_DIRECTION 0x28
#define VERT_DIRECTION 0x20
#define MAPPED_X(x, y) (319 - (y))
#define MAPPED_Y(x, y) (x)
#endif
#ifdef LANDSCAPE
#define HORIZ_DIRECTION 0x00
#define VERT_DIRECTION  0x08
#define MAPPED_X(x, y) (319 - (x))
#define MAPPED_Y(x, y) (239 - (y))
#endif
#ifdef PORTRAIT_FLIP
#define HORIZ_DIRECTION 0x18
#define VERT_DIRECTION 0x10
#define MAPPED_X(x, y) (y)
#define MAPPED_Y(x, y) (239 - (x))
#endif
#ifdef LANDSCAPE_FLIP
#define HORIZ_DIRECTION 0x30
#define VERT_DIRECTION  0x38
#define MAPPED_X(x, y) (x)
#define MAPPED_Y(x, y) (y)
#endif

/** @defgroup STM32F4_DISCOVERY_LCD_Exported_Constants
  * @{
  */

/**
 * @brief Uncomment the line below if you want to use user defined Delay function
 *        (for precise timing), otherwise default _delay_ function defined within
 *         this driver is used (less precise timing).
 */
/* #define USE_Delay */

#ifdef USE_Delay
#include "main.h"
  #define _delay_     Delay  /* !< User can provide more timing precise _delay_ function
                                   (with 10ms time base), using SysTick for example */
#else
  #define _delay_     delay      /* !< Default _delay_ function with less precise timing */
#endif

/**
  * @brief  LCD color
  */
#define LCD_COLOR_WHITE          0xFFFF
#define LCD_COLOR_BLACK          0x0000
#define LCD_COLOR_GREY           0xC618
#define LCD_COLOR_BLUE           0x001F
#define LCD_COLOR_BLUE2          0x051F
#define LCD_COLOR_RED            0xF800
#define LCD_COLOR_MAGENTA        0xF81F
#define LCD_COLOR_GREEN          0x07E0
#define LCD_COLOR_CYAN           0x7FFF
#define LCD_COLOR_YELLOW         0xFFE0

#define White                    LCD_COLOR_WHITE
#define Black                    LCD_COLOR_BLACK
#define Grey                     LCD_COLOR_GREY
#define Blue                     LCD_COLOR_BLUE
#define Blue2                    LCD_COLOR_BLUE2
#define Red                      LCD_COLOR_RED
#define Green                    LCD_COLOR_GREEN
#define Cyan                     LCD_COLOR_CYAN
#define Yellow                   LCD_COLOR_YELLOW
#define Magenta                  LCD_COLOR_MAGENTA

/**
  * @brief  LCD Lines depending on the chosen fonts.
  */
#define LCD_LINE_0               LINE(0)
#define LCD_LINE_1               LINE(1)
#define LCD_LINE_2               LINE(2)
#define LCD_LINE_3               LINE(3)
#define LCD_LINE_4               LINE(4)
#define LCD_LINE_5               LINE(5)
#define LCD_LINE_6               LINE(6)
#define LCD_LINE_7               LINE(7)
#define LCD_LINE_8               LINE(8)
#define LCD_LINE_9               LINE(9)
#define LCD_LINE_10              LINE(10)
#define LCD_LINE_11              LINE(11)
#define LCD_LINE_12              LINE(12)
#define LCD_LINE_13              LINE(13)
#define LCD_LINE_14              LINE(14)
#define LCD_LINE_15              LINE(15)
#define LCD_LINE_16              LINE(16)
#define LCD_LINE_17              LINE(17)
#define LCD_LINE_18              LINE(18)
#define LCD_LINE_19              LINE(19)
#define LCD_LINE_20              LINE(20)
#define LCD_LINE_21              LINE(21)
#define LCD_LINE_22              LINE(22)
#define LCD_LINE_23              LINE(23)
#define LCD_LINE_24              LINE(24)
#define LCD_LINE_25              LINE(25)
#define LCD_LINE_26              LINE(26)
#define LCD_LINE_27              LINE(27)
#define LCD_LINE_28              LINE(28)
#define LCD_LINE_29              LINE(29)

#define Line0			 LCD_LINE_0
#define Line1			 LCD_LINE_1
#define Line2			 LCD_LINE_2
#define Line3			 LCD_LINE_3
#define Line4			 LCD_LINE_4
#define Line5			 LCD_LINE_5
#define Line6			 LCD_LINE_6
#define Line7			 LCD_LINE_7
#define Line8			 LCD_LINE_8
#define Line9			 LCD_LINE_9

/**
  * @brief LCD default font
  */
#define LCD_DEFAULT_FONT         Font16x24

/**
  * @brief  LCD Direction
  */
#define LCD_DIR_HORIZONTAL       0x0000
#define LCD_DIR_VERTICAL         0x0001
#define Horizontal               LCD_DIR_HORIZONTAL
#define Vertical                 LCD_DIR_VERTICAL


#define CenterJustify             0x0
#define LeftJustify               0x1
#define RightJustify              0x2

#define LeftOrientation           0x00
#define NormalOrientation         0x01
#define RightOrientation          0x02
#define UpsideDownOrientation     0x03

/**
  * @brief  LCD Size (Width and Height)
  */
#define LCD_PIXEL_WIDTH          320
#define LCD_PIXEL_HEIGHT         240

/**
  * @}
  */

/** @defgroup STM32F4_DISCOVERY_LCD_Exported_Macros
  * @{
  */
#define ASSEMBLE_RGB(R ,G, B)    ((((R)& 0xF8) << 8) | (((G) & 0xFC) << 3) | (((B) & 0xF8) >> 3))
/**
  * @}
  */

/** @defgroup STM32F4_DISCOVERY_LCD_Exported_Functions
  * @{
  */
/** @defgroup
  * @{
  */
void LCD_DeInit(void);
void STM32f4_Discovery_LCD_Init(void);
void LCD_RGB_Test(void);
void LCD_SetColors(__IO uint16_t _TextColor, __IO uint16_t _BackColor);
void LCD_GetColors(__IO uint16_t *_TextColor, __IO uint16_t *_BackColor);
void LCD_SetTextColor(__IO uint16_t Color);
void LCD_SetBackColor(__IO uint16_t Color);
void LCD_ClearLine(uint16_t Line);
void LCD_Clear(uint16_t Color);

void LCD_SetCursor(uint16_t Xpos, uint8_t Ypos);
void LCD_SetDisplayWindow(uint16_t Xpos, uint8_t Ypos, uint16_t Width, uint8_t Height);

void LCD_DrawChar(uint16_t Xpos, uint16_t Ypos, const uint16_t *c);
void LCD_DisplayChar(uint16_t Line, uint16_t Column, uint8_t Ascii);
void LCD_SetFont(sFONT *fonts);
sFONT *LCD_GetFont(void);
void LCD_DisplayStringLine(uint16_t Line, uint8_t *ptr);

void LCD_WindowModeDisable(void);



void LCD_DrawFilledRectangle(uint16_t Xpos, uint8_t Ypos, uint16_t Width, uint8_t Height, uint16_t Color);
void LCD_DrawLine(uint16_t Xpos, uint8_t Ypos, uint16_t Length, uint8_t Direction);
void LCD_DrawLine_2(uint16_t X1, uint8_t Y1, uint16_t X2, uint8_t Y2);
void LCD_DrawRectangle(uint16_t Xpos, uint8_t Ypos, uint16_t Width, uint8_t Height);
uint16_t LCD_GetBackgroundColor();
uint16_t LCD_GetForegroundColor();
uint16_t LCD_GetTextColor();
void LCD_SetBackgroundColor(uint16_t Color);
void LCD_SetForegroundColor(uint16_t Color);
void LCD_SetTextColor(uint16_t Color);
void LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width);
void LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);
void LCD_DrawMonoPict(const uint32_t *Pict);
void LCD_WriteBMP(uint32_t BmpAddress);
void LCD_DrawUniLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_DrawFullCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);
void LCD_PolyLine(pPoint Points, uint16_t PointCount);
void LCD_PolyLineRelative(pPoint Points, uint16_t PointCount);
void LCD_ClosedPolyLine(pPoint Points, uint16_t PointCount);
void LCD_ClosedPolyLineRelative(pPoint Points, uint16_t PointCount);
void LCD_FillPolyLine(pPoint Points, uint16_t PointCount);


/* The following functions operate in coordinate system as follows:
 * (0,0)
 *  x----->
 *  |   X axis
 *  |
 *  V Y axis
 */
void x_LCD_DrawFilledRectangle(uint16_t Xpos, uint8_t Ypos, uint16_t Width, uint8_t Height, uint16_t Color);
void x_LCD_DrawLine(uint16_t Xpos, uint8_t Ypos, uint16_t Length, uint8_t Direction);
void x_LCD_DrawLine_2(uint16_t Xpos1, uint8_t Ypos1, uint16_t Xpos2, uint8_t Ypos2);
void x_LCD_DrawRectangle(uint16_t Xpos, uint8_t Ypos, uint16_t Length, uint8_t Direction);
void x_LCD_DrawChar(uint16_t Xpos, uint8_t Ypos, char C, uint8_t O);
void x_LCD_DrawText(uint16_t Xpos, uint8_t Ypos, char *S, uint8_t J, uint8_t O);
/**
  * @}
  */

/** @defgroup
  * @{
  */
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue);
uint16_t LCD_ReadReg(uint8_t LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(uint16_t RGB_Code);
uint16_t LCD_ReadRAM(void);
void LCD_PowerOn(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
/**
  * @}
  */

/** @defgroup
  * @{
  */
void LCD_CtrlLinesConfig(void);
void LCD_FSMCConfig(void);
/**
  * @}
  */
/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4_DISCOVERY_LCD_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
