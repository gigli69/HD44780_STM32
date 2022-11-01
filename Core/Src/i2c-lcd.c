
#include "i2c-lcd.h"
extern I2C_HandleTypeDef hi2c2;  // change your handler here accordingly
uint8_t shadowLCD[80];
uint8_t setBacklight=LCD_BACKLIGHT;

void lcd_send_cmd (char cmd) //RS=0
{
	char data_u, data_l;
	uint8_t data_t[5];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = setBacklight;  //en=0, rs=0
	data_t[1] = data_u|setBacklight|LCD_En;  //en=1, rs=0
	data_t[2] = data_u|setBacklight;  //en=0, rs=0
	data_t[3] = data_l|setBacklight|LCD_En;  //en=1, rs=0
	data_t[4] = data_l|setBacklight;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 5, 100);
}

void lcd_send_data (char data) //RS=1
{
	char data_u, data_l;
	uint8_t data_t[5];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = setBacklight|LCD_Rs;  //en=0, rs=1
	data_t[1] = data_u|setBacklight|LCD_Rs|LCD_En;  //en=1, rs=1
	data_t[2] = data_u|setBacklight|LCD_Rs;  //en=0, rs=1
	data_t[3] = data_l|setBacklight|LCD_En|LCD_Rs|LCD_En;  //en=1, rs=1
	data_t[4] = data_l|setBacklight|LCD_Rs;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 5, 100);
	uint8_t addr=read_ADR();
	if(addr==0) {
		addr=0x68;
	}
	if(addr>=0x40) {
		addr-=24;
	}
	shadowLCD[addr-1]=data;
}

void lcd_write(char value)
{
	lcd_send_data(value);
}

void lcd_clear (void)
{
	lcd_send_cmd (LCD_CLEARDISPLAY);
	HAL_Delay(1);
	for(uint8_t i=0;i<80;i++)
		shadowLCD[i]=32;
}

void lcd_put_cur(int row, int col)
{//"Set DDRAM address" function call
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd (col);
}



void lcd_init(enum boolean after_power_up)
{
	// 4 bit initialisation
	if(after_power_up==true) HAL_Delay(50);  // wait for >40ms after power up
	lcd_send_cmd (LCD_FUNCTIONSET|LCD_8BITMODE);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (LCD_FUNCTIONSET|LCD_8BITMODE);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (LCD_FUNCTIONSET|LCD_8BITMODE);
	HAL_Delay(10);
	lcd_send_cmd (LCD_FUNCTIONSET|LCD_4BITMODE);  // 4bit mode
	HAL_Delay(10);

  // display initialisation
	lcd_send_cmd(LCD_FUNCTIONSET|LCD_4BITMODE|LCD_2LINE|LCD_5x8DOTS); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (LCD_DISPLAYCONTROL); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (LCD_CLEARDISPLAY);  // clear display
	HAL_Delay(2);
	lcd_send_cmd (LCD_ENTRYMODESET|LCD_ENTRYLEFT); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	lcd_send_cmd (LCD_DISPLAYCONTROL|lcd_displaycontrol); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)

	for(uint8_t i=0;i<80;i++)
		shadowLCD[i]=32;
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void lcd_createChar(uint8_t location, uint8_t charmap[]) {
	location &= 0x7; // we only have 8 locations 0-7
	lcd_send_cmd (LCD_SETCGRAMADDR | (location << 3));
	for (int i=0; i<8; i++) {
		lcd_send_data(charmap[i]);
	}
}

// Turn the display on/off (quickly)
void lcd_noDisplay(void) {
	lcd_displaycontrol &= ~LCD_DISPLAYON;
	lcd_send_cmd(LCD_DISPLAYCONTROL | lcd_displaycontrol);
}
void lcd_display(void) {
	lcd_displaycontrol |= LCD_DISPLAYON;
	lcd_send_cmd(LCD_DISPLAYCONTROL | lcd_displaycontrol);
}

void lcd_setBacklight (uint8_t etat)
{
	//etat=1 => backlight ON
	//etat=0 => backlight OFF
	setBacklight=(etat==0) ? LCD_NOBACKLIGHT : LCD_BACKLIGHT;
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD, &setBacklight, 1, 100);
	HAL_Delay(2);
}

//Set DDRAM address ADR
void set_DDRAM_ADR(uint8_t adr) { //RW=0, RS=0
	//vérifier les bons intervalles de l'adresse
	uint8_t data_t[4];
	uint8_t cmd=0x80|adr;
	uint8_t data_u = cmd & 0xf0;
	uint8_t data_l = (cmd<<4) & 0xf0;

	data_t[0] = data_u|setBacklight|LCD_En;  //en=1, rs=0, rw=0
	data_t[1] = data_u|setBacklight;  //en=0, rs=0, rw=0
	data_t[2] = data_l|setBacklight|LCD_En;  //en=1, rs=0, rw=0
	data_t[3] = data_l|setBacklight;  //en=0, rs=0, rw=0
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

//Read data from DDRAM current ADR, ADR is incremented after reading
uint8_t read_DDRAM(void) { //RW=1, RS=1
	uint8_t d1 = 0xF0|setBacklight|LCD_Rw|LCD_Rs|LCD_En;  //en=1, rs=1 rw=1
	uint8_t d2 = 0xF0|setBacklight|LCD_Rw|LCD_Rs;  //en=0, rs=1, rw=1
	uint8_t r1,r2;

	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD, &d2, 1, 100); //en=0
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD, &d1, 1, 100); //en=1
	HAL_I2C_Master_Receive(&hi2c2, SLAVE_ADDRESS_LCD, &r1, 1, 100); //lecture des upper bits
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD, &d2, 1, 100); //en=0
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD, &d1, 1, 100); //en=1
	HAL_I2C_Master_Receive(&hi2c2, SLAVE_ADDRESS_LCD, &r2, 1, 100); //lecture des lower bits
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD, &d2, 1, 100); //en=0

	return (r1 & 0xF0)|((r2 & 0xF0)>>4);
}

//Read address counter from DDRAM, Busy Flag cannot be read by I2C
uint8_t read_ADR(void) { //RW=1, RS=0
  	//il n'y a pas d'accès au BUSY_FLAG en I2C
	uint8_t r1,r2;
  	uint8_t d1 = 0xF0|setBacklight|LCD_Rw|LCD_En;  //en=1, rs=0, rw=1
  	uint8_t d2 = 0xF0|setBacklight|LCD_Rw;  //en=0, rs=0, rw=1

  	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD, &d2, 1, 100);
  	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD, &d1, 1, 100); //en=1
  	HAL_I2C_Master_Receive(&hi2c2, SLAVE_ADDRESS_LCD, &r1, 1, 100); //lecture des upper bits
  	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD, &d2, 1, 100); //en=0
  	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD, &d1, 1, 100); //en=1
  	HAL_I2C_Master_Receive(&hi2c2, SLAVE_ADDRESS_LCD, &r2, 1, 100); //lecture des lower bits
  	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD, &d2, 1, 100); //en=0
  	return (r1 & 0xF0)|((r2 & 0xF0)>>4); //Reconstitution de l'ADR
}


//Lecture de la mémoire du LCD et retourne vrai si l'affichage est conforme à l'attendu, false sinon
uint8_t read_memory(void) {
	uint8_t ret[80];
	uint8_t is_egal=true;

	//sauvegarde de l'adresse courante
	uint8_t current_addr=read_ADR();

	//lecture 1ère ligne
	set_DDRAM_ADR(0x00);
	for(uint8_t i=0;i<40;i++) {
		ret[i]=read_DDRAM();
		if(ret[i]!=shadowLCD[i])
			is_egal=false;
	}
	//2ème ligne
	set_DDRAM_ADR(0x40);
	for(uint8_t i=40;i<80;i++) {
		ret[i]=read_DDRAM();
		if(ret[i]!=shadowLCD[i])
			is_egal=false;
	}

	//restauration de l'adresse courante
	set_DDRAM_ADR(current_addr);

	return is_egal;
}
