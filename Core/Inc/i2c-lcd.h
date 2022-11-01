#include "stm32f1xx_hal.h"

/*Expander:
 * P7 P6 P5 P4 P3 P2 P1 P0
 * D7 D6 D5 D4 BL EN RW RS
 *
 * BL=backlight
 * EN=enable
 * RW= R/W_ (1=read 0=write)
 * RS= register selection (1=CGRAM/DDRM 0=instruction)
 *
 * Un mot B7 B6 B5 B4 B3 B2 B1 B0 doit être envoyé en deux parties:
 *  -upper: B7 B6 B5 B4 BL EN RW RS
 *  -lower: B3 B2 B1 B0 BL EN RW RS
 * *
 * Chaque partie doit être envoyée avec EN=1 puis EN=0, donc 4 envois à faire:
 *  -upper1: B7 B6 B5 B4 BL 1 RW RS
 *  -upper2: B7 B6 B5 B4 BL 0 RW RS
 *  -lower1: B3 B2 B1 B0 BL 1 RW RS
 *  -lower2: B3 B2 B1 B0 BL 0 RW RS
 *
 *  BL, RW et RS sont à notre discrétion
 */

// LCD I2C address
#define SLAVE_ADDRESS_LCD (0x27<<1) // change this according to ur setup

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define LCD_En 0x04  // Enable bit
#define LCD_Rw 0x02  // Read/Write bit
#define LCD_Rs 0x01  // Register select bit

enum boolean {false,true};
uint8_t lcd_displaycontrol;
uint8_t shadowLCD[80];

void lcd_init(enum boolean after_power_up);   // initialize lcd
void lcd_send_cmd (char cmd);  // send command to the lcd
void lcd_send_data (char data);  // send data to the lcd
void lcd_send_string (char *str);  // send string to the lcd
void lcd_write (char data);  // send data to the lcd
void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);
void lcd_clear (void); // clear lcd
void lcd_createChar(uint8_t location, uint8_t charmap[]); //create custom characters
void lcd_noDisplay(void);
void lcd_display(void);
void lcd_setBacklight (uint8_t etat); // set backlight
void set_DDRAM_ADR(uint8_t adr); // set adr to DDRAM, equiv to lcd_put_cur
uint8_t read_DDRAM(void); // read current DDRAM adr value
uint8_t read_ADR(void); // Read address counter from DDRAM, Busy Flag cannot be read by I2C
uint8_t read_memory(void); //Compare DDRAM with LCD effective display
