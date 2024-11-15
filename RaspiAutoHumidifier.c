#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define SW  5      /* GPIO24 */
#define LED 1      /* GPIO18 */

#define MAXTIMINGS 83
#define DHTPIN 7 /* dht11, GPIO4 */

/*4x4 keypad 변수*/
unsigned char row_pins[4] = {25, 24, 23, 22};
unsigned char col_pins[4] = {29, 28, 27, 26};
unsigned char key;
int i = 0;
unsigned char input_key_buffer = 0;
unsigned char lcd_state = 0;
unsigned char humidifier_state = 0;
unsigned char user_hum[2];
float user_hum_integer = 50;
char key_code[16] ={'1', '2', '3', 'A', 
		    '4', '5', '6', 'B', 
		    '7', '8', '9', 'C',
		    'D', '0', 'E', 'F'};
/*dht11 변수*/
int dht11_dat[5] = {0, } ;
int hum;
int tem;
char buffer[20];

/* lcd 관련*/
// Define some device parameters
#define I2C_ADDR   0x27 // I2C device address

// Define some device constants
#define LCD_CHR  1 // Mode - Sending data
#define LCD_CMD  0 // Mode - Sending command

#define LINE1  0x80 // 1st line
#define LINE2  0xC0 // 2nd line

#define LCD_BACKLIGHT   0x08  // On
// LCD_BACKLIGHT = 0x00  # Off

#define ENABLE  0b00000100 // Enable bit
void lcd_init(void);
void lcd_byte(int bits, int mode);
void lcd_toggle_enable(int bits);

// added by Lewis
void typeInt(int i);
void typeFloat(float myFloat);
void lcdLoc(int line); //move cursor
void ClrLcd(void); // clr LCD return home
void typeln(const char *s);
void typeChar(char val);
int fd;  // seen by all subroutines
int notes[] = {
	391, 391, 440, 440, 391, 391, 329, 329.63, 329.63, \
	391, 391, 329.63, 329.63, 293.66, 293.66, 293.66, 0, \
	391, 391, 440, 391, 391, 391, 329.63, 329.63, \
	391, 329.63, 293.66, 329.63, 261.63, 261.63, 261.63, 0
};

/*int musicPlay()
{
	int i;

	softToneCreate(SPKR);

	for (i = 0; i < TOTAL; ++i) {
		softToneWrite(SPKR, notes[i]);
		delay(280);
	}

	return 0;
}*/

/* 4x4 keypad*/
unsigned char keyScan()
{
   int i, j;
   unsigned char idx, scan;

   for(i=0; i<4; i++) {
   	pinMode(row_pins[i], OUTPUT);
	digitalWrite(row_pins[i], HIGH);
	pinMode(col_pins[i], INPUT);
	pullUpDnControl(col_pins[i], PUD_UP);
   }

   for(i=0, idx=0, scan=0xff; i<4; i++) {
	digitalWrite(row_pins[i], LOW);
	delay(10);
	for(j=0; j<4; j++, idx++) {
	    if(digitalRead(col_pins[j]) == LOW) {
	        scan = idx;
	    }
	}

        digitalWrite(row_pins[i], HIGH);
   };

   return scan;
}

int keypad(void)
{
   key = keyScan();
   if(key != 0xff) 
   {
      printf("Scan key code is %d(%c)\n", key, key_code[key]);
      input_key_buffer = key_code[key];
      
   }
}
/*4x4 keypad end*/

/*dht11*/
void read_dht11_dat()

{

  uint8_t laststate = HIGH ;

  uint8_t counter = 0 ;

  uint8_t j = 0, i ;

  uint8_t flag = HIGH ;

  uint8_t state = 0 ;

  float f ;



  dht11_dat[0] = dht11_dat[1] = dht11_dat[2] = dht11_dat[3] = dht11_dat[4] = 0 ;



  pinMode(DHTPIN, OUTPUT) ;

  digitalWrite(DHTPIN, LOW) ;

  delay(18) ;



  digitalWrite(DHTPIN, HIGH) ;

  delayMicroseconds(30) ;

  pinMode(DHTPIN, INPUT) ;



  for (i = 0; i < MAXTIMINGS; i++) {

    counter = 0 ;

    while ( digitalRead(DHTPIN) == laststate) { 

      counter++ ;

      delayMicroseconds(1) ;

      if (counter == 200) break ;

    }

    laststate = digitalRead(DHTPIN) ;

    if (counter == 200) break ; // if while breaked by timer, break for

    if ((i >= 4) && (i % 2 == 0)) {

      dht11_dat[j / 8] <<= 1 ;

      if (counter > 20) dht11_dat[j / 8] |= 1 ;

      j++ ;

    }

  }

  if ((j >= 40) && (dht11_dat[4] == ((dht11_dat[0] + dht11_dat[1] + dht11_dat[2] + dht11_dat[3]) & 0xff))) {

    printf("humidity = %d.%d %% Temperature = %d.%d *C \n", dht11_dat[0], dht11_dat[1], dht11_dat[2], dht11_dat[3]) ;
    hum = dht11_dat[0];
    tem = dht11_dat[2];

  }

  else 
  {
      ;
  }

}


/* dht11 end*/

void input_key_process()
{
   if(input_key_buffer == 'A')
   {
      lcd_state = 1;
   }
   if(input_key_buffer == 'B')
   {
      lcd_state = 2;
      humidifier_state = 2; //가습기 자동모드
      printf("Humidifier Auto\n");
   }
   if(input_key_buffer == 'C')
   {
      lcd_state = 3; 
      humidifier_state = 1; // 가습기 켜짐
      printf("Humidifier On\n");
   }
   if(input_key_buffer == 'F')
   {
      lcd_state = 4;
      humidifier_state = 0; // 가습기 꺼짐
      printf("Humidifier Off\n");
   }
}

void menu()
{
   if(lcd_state == 0)
   {
      ClrLcd();
      lcdLoc(LINE1); typeln("Welcome! This is");
      lcdLoc(LINE2); typeln("Auto Humidifier");
   }
   if(lcd_state == 1)
   {
      ClrLcd();
      lcdLoc(LINE1); typeln("Tem : "); typeInt(tem);
      lcdLoc(LINE2); typeln("Hum : "); typeInt(hum);
   }
   if(lcd_state == 2)
   {
      ClrLcd();
      lcdLoc(LINE1); typeln("Auto Mode");
      lcdLoc(LINE2); typeln("Set Hum L:"); typeFloat(user_hum_integer);
      if(input_key_buffer == '1')
      { user_hum_integer++; input_key_buffer = 'B';}
      if(input_key_buffer == '7')
      { user_hum_integer--; input_key_buffer = 'B';}
      if(input_key_buffer == '2')
      { user_hum_integer += 5; input_key_buffer = 'B';}
      if(input_key_buffer == '8')
      { user_hum_integer-= 5; input_key_buffer = 'B';}
      if(input_key_buffer == '3')
      { user_hum_integer += 10; input_key_buffer = 'B';}
      if(input_key_buffer == '9')
      { user_hum_integer-= 10; input_key_buffer = 'B';}
      if(user_hum_integer > 100) {user_hum_integer = 100;}
      if(user_hum_integer < 0) {user_hum_integer = 0;}
        
    }
    if(lcd_state == 3)
    {
       ClrLcd();
       lcdLoc(LINE1); typeln("Humidifier On"); 
    } 
    if(lcd_state == 4)
    {
       ClrLcd();
       lcdLoc(LINE1); typeln("Humidifier Off");
    }
}

void humidifier_work()
{
   if(humidifier_state == 0)
   {
      digitalWrite(1, LOW);
   }
   if(humidifier_state == 1)
   {
      digitalWrite(1, HIGH);
   }
   if(humidifier_state == 2)
   {
      if(hum >= user_hum_integer)
      {
         digitalWrite(1, LOW);
      }
      if(hum < user_hum_integer)
      {
         digitalWrite(1, HIGH);
      }
   }
}
   




int main(int argc, char** argv)
{
   wiringPiSetup();
  if (wiringPiSetup () == -1) exit (1);

  fd = wiringPiI2CSetup(I2C_ADDR);
  pinMode(1, OUTPUT);
  //printf("fd = %d ", fd);

  lcd_init(); // setup LCD
   while(1) 
   {
        keypad();
        input_key_process();
        menu();
        humidifier_work();
        read_dht11_dat();
        delay(1000);
   }

   return 0 ;
}

// float to string
void typeFloat(float myFloat)   {
  char buffer[20];
  sprintf(buffer, "%4.2f",  myFloat);
  typeln(buffer);
}

// int to string
void typeInt(int i)   {
  char array1[20];
  sprintf(array1, "%d",  i);
  typeln(array1);
}

// clr lcd go home loc 0x80
void ClrLcd(void)   {
  lcd_byte(0x01, LCD_CMD);
  lcd_byte(0x02, LCD_CMD);
}

// go to location on LCD
void lcdLoc(int line)   {
  lcd_byte(line, LCD_CMD);
}

// out char to LCD at current position
void typeChar(char val)   {

  lcd_byte(val, LCD_CHR);
}


// this allows use of any size string
void typeln(const char *s)   {

  while ( *s ) lcd_byte(*(s++), LCD_CHR);

}

void lcd_byte(int bits, int mode)   {

  //Send byte to data pins
  // bits = the data
  // mode = 1 for data, 0 for command
  int bits_high;
  int bits_low;
  // uses the two half byte writes to LCD
  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT ;
  bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT ;

  // High bits
  wiringPiI2CReadReg8(fd, bits_high);
  lcd_toggle_enable(bits_high);

  // Low bits
  wiringPiI2CReadReg8(fd, bits_low);
  lcd_toggle_enable(bits_low);
}

void lcd_toggle_enable(int bits)   {
  // Toggle enable pin on LCD display
  delayMicroseconds(500);
  wiringPiI2CReadReg8(fd, (bits | ENABLE));
  delayMicroseconds(500);
  wiringPiI2CReadReg8(fd, (bits & ~ENABLE));
  delayMicroseconds(500);
}


void lcd_init()   {
  // Initialise display
  lcd_byte(0x33, LCD_CMD); // Initialise
  lcd_byte(0x32, LCD_CMD); // Initialise
  lcd_byte(0x06, LCD_CMD); // Cursor move direction
  lcd_byte(0x0C, LCD_CMD); // 0x0F On, Blink Off
  lcd_byte(0x28, LCD_CMD); // Data length, number of lines, font size
  lcd_byte(0x01, LCD_CMD); // Clear display
  delayMicroseconds(500);
}


