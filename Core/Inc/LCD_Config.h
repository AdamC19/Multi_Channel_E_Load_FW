
#ifndef LCD_CONFIG_H
#define LCD_CONFIG_H 

#define _LCD_USE_FREERTOS 0
#define _LCD_USE_MENU_LIB 0

#define _LCD_COLS         20
#define _LCD_ROWS         4

#define _LCD_RS_PORT      LCD_RS_GPIO_Port
#define _LCD_RS_PIN       LCD_RS_Pin

#define _LCD_E_PORT       LCD_EN_GPIO_Port
#define _LCD_E_PIN        LCD_EN_Pin

#define _LCD_RW_PORT      LCD_RW_GPIO_Port
#define _LCD_RW_PIN       LCD_RW_Pin

#define _LCD_D4_PORT      LCD_DB4_GPIO_Port
#define _LCD_D4_PIN		  LCD_DB4_Pin

#define _LCD_D5_PORT      LCD_DB5_GPIO_Port
#define _LCD_D5_PIN       LCD_DB5_Pin

#define _LCD_D6_PORT      LCD_DB6_GPIO_Port
#define _LCD_D6_PIN       LCD_DB6_Pin

#define _LCD_D7_PORT      LCD_DB7_GPIO_Port
#define _LCD_D7_PIN       LCD_DB7_Pin

#endif

