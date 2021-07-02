/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "D:\jobESP\AC Under Voltage\Firmware\ATS V1.0\Inc\M90E32.h"
#include <stdio.h>
#include "ssd1306.h"
#include "fonts.h"
#include "main.h"
#include "stm32f0xx_hal_gpio.h"
#define APP_RX_DATA_SIZE  100
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t SPI_TXbuf[64];
volatile uint8_t SPI2_TXbuf[64];
volatile uint8_t USART_RXbuf[64];
uint8_t VReadbuf[6];
uint8_t Vsend[6];
volatile _Bool Read3911 = 0;
volatile _Bool ReadVolt = 0;
volatile uint8_t ReadCount=0;
uint32_t VAC32BUF[64];

#define timeshowdisplay 1  //100 * 1 ms.
volatile uint8_t ChangshowVolt =0;

volatile uint8_t Send7ment;

volatile uint8_t SysStat = 0;

volatile uint8_t TranferFlag =0;

uint32_t HartbeatCount;

volatile uint32_t BacklightTime = 60000;

uint32_t MaxValue ;
uint32_t MinValue ;
uint32_t VRMS;

double rmsA;
double rmsB;
double rmsC;
double freq;

double currentA;
uint16_t AmpA;

uint16_t VA;
uint16_t VB;
uint16_t VC;
uint16_t FRE;

char va[15];
char vb[15];
char vc[15];
char hz[10];
char amp[15];

uint16_t ADC_raw;

uint16_t Status0;
uint16_t Status1;
volatile _Bool PhaseSequenceerror = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void StopTimer(void);
void StartTimer(void);

uint16_t CommEnergyIC(unsigned char RW, uint16_t address, uint16_t val) ;
void InitEnergyIC(void);
double GetLineVoltageA(void);
double GetLineVoltageB(void);
double GetLineVoltageC(void);
double GetFrequency(void);
double GetLineCurrentA(void);
unsigned short  GetMeterStatus0(void);
unsigned short  GetMeterStatus1(void);
unsigned short GetSysStatus0(void);
unsigned short GetSysStatus1(void);

#define rd(j,k)  HAL_GPIO_ReadPin(j, k)

void VRMS_Read(void);
void buttonRead(void);
void ReadSetting(void);
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
#define false 0
#define true 1
	
#define HIGH GPIO_PIN_SET
#define LOW  GPIO_PIN_RESET
	
uint8_t set_mode = false;  // whether the string is complete
uint8_t phase_correct = false;
uint8_t Under_flag = false;
uint8_t Over_flag = false;
uint8_t UnderRes_flag = false;
uint8_t OverRes_flag = false;
uint8_t FlashLEDUnder = false;
uint8_t FlashLEDOver = false;
uint8_t USART_Send = false;
volatile uint8_t Interrup = false;

uint8_t mmode;
uint16_t V_A,V_B,V_C;
long calVrmsout;
long VRMS_A,VRMS_B,VRMS_C,Status;
volatile  uint16_t V_a,V_b,V_c,V_SUM;
volatile  uint16_t VRMS_a,VRMS_b,VRMS_c,VRMS_SUM;
volatile uint8_t one,ten,hundress,thouson;
volatile uint16_t menuCount;
volatile int32_t flash_dot,healty_count = 255;
volatile int32_t flash_LED_Under,flash_LED_Over;
volatile int32_t displayCount=0;

volatile int32_t hysteresisRLY =0;

volatile int32_t first_mea= 1;

volatile signed char Timer_flag =0;
volatile int32_t ShowLCD = 0;

volatile  uint16_t V_AB = 0;
volatile  uint16_t V_BC = 0;
volatile  uint16_t V_CA = 0;

#define SecMultiply 222

#define OFF_Rly GPIO_PIN_SET
#define ON_Rly GPIO_PIN_RESET

#define Rly_port  GPIOB
#define Under_Rly	GPIO_PIN_2
#define Fault_Rly	GPIO_PIN_9

#define hysteresisTimedefine 500 //10 sec

//#define LED_port   GPIOB
//#define LED_Healty GPIO_PIN_5
//#define LED_Under  GPIO_PIN_9
//#define LED_Over   GPIO_PIN_8

#define Backlight_Port GPIOB
#define Backlight_Pin GPIO_PIN_0
#define OFF_Backlig GPIO_PIN_SET
#define ON_Backlig GPIO_PIN_RESET
//#define ON_Backlight  HAL_GPIO_WritePin(Backlight_Port,Backlight_Pin,ON_Backlig);
//#define OFF_Backlight  HAL_GPIO_WritePin(Backlight_Port,Backlight_Pin,OFF_Backlig);

#define BT_SET_Port  GPIOA
#define BT_UP_Port   GPIOA
#define BT_DW_Port   GPIOA
#define BT_BK_Port   GPIOB

#define BT_SET_Pin  GPIO_PIN_10
#define BT_UP_Pin   GPIO_PIN_8
#define BT_DW_Pin   GPIO_PIN_9
#define BT_BK_Pin   GPIO_PIN_12

#define LED_V1IN_PORT GPIOB
#define LED_V2IN_PORT GPIOB
#define LED_V1ON_PORT GPIOB
#define LED_FAULT_PORT GPIOB
#define LED_V2ON_PORT GPIOC
#define LED_VOUT_PORT GPIOC

#define LED_V1IN_PIN GPIO_PIN_3
#define LED_V2IN_PIN GPIO_PIN_4
#define LED_V1ON_PIN GPIO_PIN_5
#define LED_V2ON_PIN GPIO_PIN_13
#define LED_VOUT_PIN GPIO_PIN_14
#define LED_FAULT_PIN GPIO_PIN_8

#define ON_LED GPIO_PIN_RESET
#define OFF_LED GPIO_PIN_SET

#define IRQ_port GPIOA
#define IRQ1  GPIO_PIN_1

#define MAX_VOLT 220
#define MIN_VOLT 150
#define MAX_TIME 60
#define MIN_TIME 0

#define UnderSet_addr 0x00
#define UnderResSet_addr 0x02
#define UnderTimSet_addr 0x04
#define UnderResTimSet_addr 0x06
#define Source_addr	0x8

#define OffsetTimeError_Un 6 // x25 ms 125
#define OffsetTimeError_UnRe 7 // x25 ms 250
#define OffsetTimeError_Ov 19 // x25 ms 125
#define OffsetTimeError_OvRe 3 // x25 ms 250

//#define Timer1_BaseValue 48248 // 25ms overflow
//#define Timer1_BaseValue 48939 // 30ms overflow
//#define Timer1_BaseValue 1 // 32ms overflow
//#define Timer1_BaseValue 65534 // 16.44ms overflow
//#define Timer1_StartValue 65477 // 20ms overflow

const uint8_t CHR[] ={0xAA, 0xA3,};
enum{State_nor,State_PreUnder,State_Under,State_PreUnderRes,State_UnderRes,State_PreOver,State_Over,State_PreOverRes,State_OverRes};
enum{nor, UnderSet, OverSet,UnderResSet,OverResSet,UnderTimSet,OverTimSet,UnderResTimSet,OverResTimSet};
enum{normal, SetUnder, SetUnderRes,SetUnderTim,SetUnderResTim,SetSource};
volatile int16_t mode = nor,State = State_nor;
volatile int16_t UnderResTimeCount=0, UnderTimeCount =0;
volatile int16_t  OverTimSetValue,UnderResTimSetValue, UnderTimSetValue, OverResTimSetValue;
volatile int16_t UnderValue , OverValue, UnderResValue, OverResValue , SourceValue;
volatile int16_t StartMeasureCount = 5000;

//volatile int16_t UnderValue_temp,UnderResValue_temp, UnderTimSetValue_temp,UnderResTimSetValue_temp;

volatile uint32_t TIMER_Flag =0;

volatile uint32_t Under_Flag =0;
volatile uint32_t UnderRE_Flag =0;

/************ Delay usec (about 800 nanosec)****************/
void delay_us(uint32_t us) { 
	volatile uint32_t counter = 3*us;
	while(counter--);
}

#define BUZZER_PORT				GPIOC
#define BUZZER_PIN				GPIO_PIN_15
#define OFF_BUZZER GPIO_PIN_SET
#define ON_BUZZER GPIO_PIN_RESET

void Beep(void)
{
	HAL_IWDG_Refresh(&hiwdg);
	HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,ON_BUZZER);
	HAL_Delay(100);
	HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,OFF_BUZZER);
}

//*******************LCD************************************//
/* 4 bit mode */
/* Control pins, can be overwritten */
/* RS - Register select pin */

//***************GPIO*************
/**
 * @brief  Sets pin(s) low
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to set pin low
 * @param  GPIO_Pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them low
 * @retval None
 */
#define TM_GPIO_SetPinLow(GPIOx, GPIO_Pin)			((GPIOx)->BSRRH = (GPIO_Pin))

/**
 * @brief  Sets pin(s) high
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to set pin high
 * @param  GPIO_Pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them high
 * @retval None
 */
#define TM_GPIO_SetPinHigh(GPIOx, GPIO_Pin) 		((GPIOx)->BSRRL = (GPIO_Pin))

/**
 * @brief  Sets pin(s) value
 * @note   Defined as macro to get maximum speed using register access
 * @param  GPIOx: GPIOx PORT where you want to set pin value
 * @param  GPIO_Pin: Select GPIO pin(s). You can select more pins with | (OR) operator to set them value
 * @param  val: If parameter is 0 then pin will be low, otherwise high
 * @retval None
 */
//#define TM_GPIO_SetPinValue(GPIOx, GPIO_Pin, val)	((val) ? TM_GPIO_SetPinHigh(GPIOx, GPIO_Pin) : TM_GPIO_SetPinLow(GPIOx, GPIO_Pin))

#define HD44780_RS_PORT				GPIOA
#define HD44780_RS_PIN				GPIO_PIN_6

/* E - Enable pin */
#define HD44780_E_PORT				GPIOA
#define HD44780_E_PIN				GPIO_PIN_7
/* Data pins */
/* D4 - Data 4 pin */
#define HD44780_D4_PORT				GPIOA
#define HD44780_D4_PIN				GPIO_PIN_2
/* D5 - Data 5 pin */
#define HD44780_D5_PORT				GPIOA
#define HD44780_D5_PIN				GPIO_PIN_3
/* D6 - Data 6 pin */
#define HD44780_D6_PORT				GPIOA
#define HD44780_D6_PIN				GPIO_PIN_4
/* D7 - Data 7 pin */
#define HD44780_D7_PORT				GPIOA
#define HD44780_D7_PIN				GPIO_PIN_5

/**
 * @}
 */

/**
 * @defgroup TM_HD44780_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes HD44780 LCD
 * @brief  cols: width of lcd
 * @param  rows: height of lcd
 * @retval None
 */
void TM_HD44780_Init(uint8_t cols, uint8_t rows);

/**
 * @brief  Turn display on
 * @param  None
 * @retval None
 */
void TM_HD44780_DisplayOn(void);

/**
 * @brief  Turn display off
 * @param  None
 * @retval None
 */
void TM_HD44780_DisplayOff(void);

/**
 * @brief  Clears entire LCD
 * @param  None
 * @retval None
 */
void TM_HD44780_Clear(void);

/**
 * @brief  Puts string on lcd
 * @param  x location
 * @param  y location
 * @param  *str: pointer to string to display
 * @retval None
 */
void TM_HD44780_Puts(uint8_t x, uint8_t y, char* str);

/**
 * @brief  Enables cursor blink
 * @param  None
 * @retval None
 */
void TM_HD44780_BlinkOn(void);

/**
 * @brief  Disables cursor blink
 * @param  None
 * @retval None
 */
void TM_HD44780_BlinkOff(void);

/**
 * @brief  Shows cursor
 * @param  None
 * @retval None
 */
void TM_HD44780_CursorOn(void);

/**
 * @brief  Hides cursor
 * @param  None
 * @retval None
 */
void TM_HD44780_CursorOff(void);

/**
 * @brief  Scrolls display to the left
 * @param  None
 * @retval None
 */
void TM_HD44780_ScrollLeft(void);

/**
 * @brief  Scrolls display to the right
 * @param  None
 * @retval None
 */
void TM_HD44780_ScrollRight(void);

/**
 * @brief  Creates custom character
 * @param  location: Location where to save character on LCD. LCD supports up to 8 custom characters, so locations are 0 - 7
 * @param *data: Pointer to 8-bytes of data for one character
 * @retval None
 */
void TM_HD44780_CreateChar(uint8_t location, uint8_t* data);

/**
 * @brief  Puts custom created character on LCD
 * @param  location: Location on LCD where character is stored, 0 - 7
 * @retval None
 */
void TM_HD44780_PutCustom(uint8_t x, uint8_t y, uint8_t location);

/* Private HD44780 structure */
typedef struct {
	uint8_t DisplayControl;
	uint8_t DisplayFunction;
	uint8_t DisplayMode;
	uint8_t Rows;
	uint8_t Cols;
	uint8_t currentX;
	uint8_t currentY;
} HD44780_Options_t;

/* Private functions */
static void TM_HD44780_InitPins(void);
static void TM_HD44780_Cmd(uint8_t cmd);
static void TM_HD44780_Cmd4bit(uint8_t cmd);
static void TM_HD44780_Data(uint8_t data);
static void TM_HD44780_CursorSet(uint8_t col, uint8_t row);

/* Private variable */
static HD44780_Options_t HD44780_Opts;

/* Pin definitions */
#define HD44780_RS_LOW              HAL_GPIO_WritePin(HD44780_RS_PORT, HD44780_RS_PIN,GPIO_PIN_RESET)
#define HD44780_RS_HIGH             HAL_GPIO_WritePin(HD44780_RS_PORT, HD44780_RS_PIN,GPIO_PIN_SET)
#define HD44780_E_LOW               HAL_GPIO_WritePin(HD44780_E_PORT, HD44780_E_PIN,GPIO_PIN_RESET)
#define HD44780_E_HIGH              HAL_GPIO_WritePin(HD44780_E_PORT, HD44780_E_PIN,GPIO_PIN_SET)

#define HD44780_Delay(x)            delay_us(x)

#define HD44780_E_BLINK             HD44780_E_LOW; HD44780_Delay(4);HD44780_E_HIGH; HD44780_Delay(4); HD44780_E_LOW; HD44780_Delay(50)//don't change value in delay time

/* Commands*/
#define HD44780_CLEARDISPLAY        0x01
#define HD44780_RETURNHOME          0x02
#define HD44780_ENTRYMODESET        0x04
#define HD44780_DISPLAYCONTROL      0x08
#define HD44780_CURSORSHIFT         0x10
#define HD44780_FUNCTIONSET         0x20
#define HD44780_SETCGRAMADDR        0x40
#define HD44780_SETDDRAMADDR        0x80

/* Flags for display entry mode */
#define HD44780_ENTRYRIGHT          0x00
#define HD44780_ENTRYLEFT           0x02
#define HD44780_ENTRYSHIFTINCREMENT 0x01
#define HD44780_ENTRYSHIFTDECREMENT 0x00

/* Flags for display on/off control */
#define HD44780_DISPLAYON           0x04
#define HD44780_CURSORON            0x02
#define HD44780_BLINKON             0x01

/* Flags for display/cursor shift */
#define HD44780_DISPLAYMOVE         0x08
#define HD44780_CURSORMOVE          0x00
#define HD44780_MOVERIGHT           0x04
#define HD44780_MOVELEFT            0x00

/* Flags for function set */
#define HD44780_8BITMODE            0x10
#define HD44780_4BITMODE            0x00
#define HD44780_2LINE               0x08
#define HD44780_1LINE               0x00
#define HD44780_5x10DOTS            0x04
#define HD44780_5x8DOTS             0x00

uint8_t customChar[] = {
		0x1F,	/*  xxx 11111 */
		0x11,	/*  xxx 10001 */
		0x11,	/*  xxx 10001 */
		0x11,	/*  xxx 10001 */
		0x11,	/*  xxx 10001 */
		0x11,	/*  xxx 10001 */
		0x11,	/*  xxx 10001 */
		0x1F	/*  xxx 11111 */
};

void TM_HD44780_Init(uint8_t cols, uint8_t rows) {
	/* Initialize delay */
	//TM_DELAY_Init();
	
	/* Init pinout */
	TM_HD44780_InitPins();
	
	/* At least 40ms */
	HD44780_Delay(50000);
	
	/* Set LCD width and height */
	HD44780_Opts.Rows = rows;
	HD44780_Opts.Cols = cols;
	
	/* Set cursor pointer to beginning for LCD */
	HD44780_Opts.currentX = 0;
	HD44780_Opts.currentY = 0;
	
	HD44780_Opts.DisplayFunction = HD44780_4BITMODE | HD44780_5x8DOTS | HD44780_1LINE;
	if (rows > 1) {
		HD44780_Opts.DisplayFunction |= HD44780_2LINE;
	}
	
	/* Try to set 4bit mode */
	TM_HD44780_Cmd4bit(0x03);
	HD44780_Delay(6500);
	
	/* Second try */
	TM_HD44780_Cmd4bit(0x03);
	HD44780_Delay(6500);
	
	/* Third goo! */
	TM_HD44780_Cmd4bit(0x03);
	HD44780_Delay(6500);	
	
	/* Set 4-bit interface */
	TM_HD44780_Cmd4bit(0x02);
	HD44780_Delay(200);
	
	//-----------------jj
	
	TM_HD44780_Cmd4bit(0x28);
	HD44780_Delay(200);
	
	TM_HD44780_Cmd4bit(0x02);
	HD44780_Delay(200);
	
	TM_HD44780_Cmd4bit(0x02);
	HD44780_Delay(200);
	
	TM_HD44780_Cmd4bit(0x02);
	HD44780_Delay(200);
	
	TM_HD44780_Cmd4bit(0x02);
	HD44780_Delay(200);
	
	TM_HD44780_Cmd4bit(0x02);
	HD44780_Delay(200);
	//-----------------jj
	
	/* Set # lines, font size, etc. */
	TM_HD44780_Cmd(HD44780_FUNCTIONSET | HD44780_Opts.DisplayFunction);

	/* Turn the display on with no cursor or blinking default */
	HD44780_Opts.DisplayControl = HD44780_DISPLAYON;
	TM_HD44780_DisplayOn();

	/* Clear lcd */
	TM_HD44780_Clear();

	/* Default font directions */
	HD44780_Opts.DisplayMode = HD44780_ENTRYLEFT | HD44780_ENTRYSHIFTDECREMENT;
	TM_HD44780_Cmd(HD44780_ENTRYMODESET | HD44780_Opts.DisplayMode);

	/* Delay */
	HD44780_Delay(6500);
}

void TM_HD44780_Clear(void) {
	TM_HD44780_Cmd(HD44780_CLEARDISPLAY);
	HD44780_Delay(3500);
}

void TM_HD44780_Puts(uint8_t x, uint8_t y, char* str) {
	TM_HD44780_CursorSet(x, y);
	while (*str) {
		if (HD44780_Opts.currentX >= HD44780_Opts.Cols) {
			HD44780_Opts.currentX = 0;
			HD44780_Opts.currentY++;
			TM_HD44780_CursorSet(HD44780_Opts.currentX, HD44780_Opts.currentY);
		}
		if (*str == '\n') {
			HD44780_Opts.currentY++;
			TM_HD44780_CursorSet(HD44780_Opts.currentX, HD44780_Opts.currentY);
		} else if (*str == '\r') {
			TM_HD44780_CursorSet(0, HD44780_Opts.currentY);
		} else {
			TM_HD44780_Data(*str);
			HD44780_Opts.currentX++;
			//HD44780_Delay(10);
		}
		str++;
	}
}

void TM_HD44780_DisplayOn(void) {
	HD44780_Opts.DisplayControl |= HD44780_DISPLAYON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_DisplayOff(void) {
	HD44780_Opts.DisplayControl &= ~HD44780_DISPLAYON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_BlinkOn(void) {
	HD44780_Opts.DisplayControl |= HD44780_BLINKON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_BlinkOff(void) {
	HD44780_Opts.DisplayControl &= ~HD44780_BLINKON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_CursorOn(void) {
	HD44780_Opts.DisplayControl |= HD44780_CURSORON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_CursorOff(void) {
	HD44780_Opts.DisplayControl &= ~HD44780_CURSORON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_ScrollLeft(void) {
	TM_HD44780_Cmd(HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVELEFT);
}

void TM_HD44780_ScrollRight(void) {
	TM_HD44780_Cmd(HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVERIGHT);
}

void TM_HD44780_CreateChar(uint8_t location, uint8_t *data) {
	uint8_t i;
	/* We have 8 locations available for custom characters */
	location &= 0x07;
	TM_HD44780_Cmd(HD44780_SETCGRAMADDR | (location << 3));
	
	for (i = 0; i < 8; i++) {
		TM_HD44780_Data(data[i]);
	}
}

void TM_HD44780_PutCustom(uint8_t x, uint8_t y, uint8_t location) {
	TM_HD44780_CursorSet(x, y);
	TM_HD44780_Data(location);
}

/* Private functions */
static void TM_HD44780_Cmd(uint8_t cmd) {
	/* Command mode */
	HD44780_RS_LOW;
	HD44780_Delay(1);//must be have
	/* High nibble */
	TM_HD44780_Cmd4bit(cmd >> 4);
	/* Low nibble */
	TM_HD44780_Cmd4bit(cmd & 0x0F);
}

static void TM_HD44780_Data(uint8_t data) {
	/* Data mode */
	HD44780_RS_HIGH;
	HD44780_Delay(1);//must be have
	/* High nibble */
	TM_HD44780_Cmd4bit(data >> 4);
	/* Low nibble */
	TM_HD44780_Cmd4bit(data & 0x0F);
}

static void TM_HD44780_Cmd4bit(uint8_t cmd) {
	/* Set output port */
	uint8_t temp;
	temp = (cmd & 0x08);
	if(temp)
		HAL_GPIO_WritePin(HD44780_D7_PORT, HD44780_D7_PIN,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(HD44780_D7_PORT, HD44780_D7_PIN,GPIO_PIN_RESET);
	temp = (cmd & 0x04);
	if(temp)
		HAL_GPIO_WritePin(HD44780_D6_PORT, HD44780_D6_PIN,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(HD44780_D6_PORT, HD44780_D6_PIN,GPIO_PIN_RESET);
	temp = (cmd & 0x02);
	if(temp)
		HAL_GPIO_WritePin(HD44780_D5_PORT, HD44780_D5_PIN,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(HD44780_D5_PORT, HD44780_D5_PIN,GPIO_PIN_RESET);
	temp = (cmd & 0x01);
	if(temp)
		HAL_GPIO_WritePin(HD44780_D4_PORT, HD44780_D4_PIN,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(HD44780_D4_PORT, HD44780_D4_PIN,GPIO_PIN_RESET);
	
	HD44780_E_BLINK;
}


static void TM_HD44780_CursorSet(uint8_t col, uint8_t row) {
	uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
	
	/* Go to beginning */
	if (row >= HD44780_Opts.Rows) {
		row = 0;
	}
	
	/* Set current column and row */
	HD44780_Opts.currentX = col;
	HD44780_Opts.currentY = row;
	
	/* Set location address */
	TM_HD44780_Cmd(HD44780_SETDDRAMADDR | (col + row_offsets[row]));
	
}

static void TM_HD44780_InitPins(void) {
	/* Init all pins */
	
//	TM_GPIO_Init(HD44780_RS_PORT, HD44780_RS_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
//	TM_GPIO_Init(HD44780_E_PORT, HD44780_E_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
//	TM_GPIO_Init(HD44780_D4_PORT, HD44780_D4_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
//	TM_GPIO_Init(HD44780_D5_PORT, HD44780_D5_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
//	TM_GPIO_Init(HD44780_D6_PORT, HD44780_D6_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
//	TM_GPIO_Init(HD44780_D7_PORT, HD44780_D7_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
	
	/* Set pins low */
	HAL_GPIO_WritePin(HD44780_RS_PORT, HD44780_RS_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HD44780_E_PORT, HD44780_E_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HD44780_D4_PORT, HD44780_D4_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HD44780_D5_PORT, HD44780_D5_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HD44780_D6_PORT, HD44780_D6_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HD44780_D7_PORT, HD44780_D7_PIN,GPIO_PIN_RESET);
}


//******************END LCD ****************************************//


/*        fLASH            */
char Flashdata[50];
#define FLASH_PAGE_START_ADDRESS    0x0801F800
#define FLASH_PAGE_END_ADDRESS      0x0801FFFF
//#define FLASH_PAGE_SIZE             2048
#define FLASH_PAGE_size             2048

uint8_t FlashErase(void);
uint8_t FlashWrite(uint32_t Address, uint8_t *Data, uint32_t Length);

//Interrupt callback routine
/*
//Interrupt TIM Overflow routine
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance ==TIM6)// 1 sec
//	{
//		;
//		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
//		//ReadVolt =1;
//	}
//	else if(htim->Instance ==TIM7)//500 micro sec
//		;//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
//}
*/

//Interrupt SYSTICK

 
void HAL_SYSTICK_Callback()
{	
	static uint16_t count =0;

	if(++count >=100)
	{
		count = 0;
		ReadVolt =1;
	}
	
	if(StartMeasureCount)
	{
		if(--StartMeasureCount<=0)
		{
			StartMeasureCount = 0;
		}
	}
	
	if(hysteresisRLY)
	{
		if(--hysteresisRLY<=0)
		{
			hysteresisRLY = 0;
		}
	}
	
	Send7ment++;	
	if(menuCount)
	{
		menuCount--;
/*
//		if(menuCount ==1)
//		{
//			if(SourceValue ==1)
//			{
//				if(VA >UnderResValue)
//					TM_HD44780_Puts(10, 1, "NORMAL");
//				else{
//					TM_HD44780_Puts(10, 1, "BYPASS");
//					HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,ON_BUZZER);
//				}
//			}
//			if(SourceValue ==2)
//			{
//				if(VB >UnderResValue)
//					TM_HD44780_Puts(10, 1, "NORMAL");
//				else{
//					TM_HD44780_Puts(10, 1, "BYPASS");
//					HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,ON_BUZZER);
//				}
//			}				
//		}	
*/
	}
	else
	{
		mode = nor;
		if(set_mode)
		{
			set_mode = false;
			TM_HD44780_CursorOff();
			TM_HD44780_BlinkOff();
		}
	}
	if(BacklightTime)
	{
		BacklightTime--;
	}
	else
	{
		HAL_GPIO_WritePin(Backlight_Port,Backlight_Pin,OFF_Backlig);
	}
	
	/*     Timer Count      */
	if(TIMER_Flag)
	{	
		/////////////// Under //////////////////////
		if(UnderTimeCount)
		{				
			if(--UnderTimeCount ==0)
			{
				UnderTimeCount = 0;
				if(SourceValue == 1){
					HAL_GPIO_WritePin(Rly_port,Under_Rly,ON_Rly);
					HAL_GPIO_WritePin(LED_V1ON_PORT,LED_V1ON_PIN,OFF_LED);
					HAL_GPIO_WritePin(LED_V2ON_PORT,LED_V2ON_PIN,ON_LED);
					HAL_GPIO_WritePin(LED_FAULT_PORT,LED_FAULT_PIN,ON_LED);
				}
				else{
					HAL_GPIO_WritePin(Rly_port,Under_Rly,OFF_Rly);
					HAL_GPIO_WritePin(LED_V2ON_PORT,LED_V2ON_PIN,OFF_LED);
					HAL_GPIO_WritePin(LED_V1ON_PORT,LED_V1ON_PIN,ON_LED);
					HAL_GPIO_WritePin(LED_FAULT_PORT,LED_FAULT_PIN,ON_LED);
				}
				hysteresisRLY = hysteresisTimedefine;
				
				//TM_HD44780_Puts(10, 1, "BYPASS");
				State = State_Under;				
				
				//=======jj
				HAL_GPIO_WritePin(Backlight_Port,Backlight_Pin,ON_Backlig);
				BacklightTime = 20000;
				
				if(UnderResTimeCount ==0) 
				{
					StopTimer();
				}
			}
		}
		if(UnderResTimeCount)
		{
/*			
			if(flash_LED_Under >=200){
				flash_LED_Under =0;
			}
*/		
			if(--UnderResTimeCount ==0)
			{
				UnderResTimeCount = 0;
				if(SourceValue == 1)
				{
					HAL_GPIO_WritePin(Rly_port,Under_Rly,OFF_Rly);
				}
				else
				{
					HAL_GPIO_WritePin(Rly_port,Under_Rly,ON_Rly);
				}
				
				hysteresisRLY = hysteresisTimedefine;
				//TM_HD44780_Puts(10, 1, "NORMAL");
				HAL_GPIO_WritePin(Backlight_Port,Backlight_Pin,ON_Backlig);
				BacklightTime = 20000;
				HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,OFF_BUZZER);
				
				if(State == State_PreUnderRes)/**/
				{
					State = State_nor;
				}
				//FlashLEDUnder = false;
				if(UnderTimeCount ==0) 
				{
					StopTimer();
				}
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	delay_us(3);
	if(!rd(GPIOA,GPIO_PIN_15))
	{
		TranferFlag =1;
		if(UnderTimSetValue ==0)
		{
				if((GPIO_Pin == GPIO_PIN_15) && (StartMeasureCount ==0))
				{			
					if((SourceValue == 1)&&(VRMS_a >UnderValue))
					{
						if(VRMS_b >UnderResValue){
							hysteresisRLY = hysteresisTimedefine;					
							HAL_GPIO_WritePin(Rly_port,Under_Rly,ON_Rly);
							HAL_GPIO_WritePin(LED_V1ON_PORT,LED_V1ON_PIN,OFF_LED);
							HAL_GPIO_WritePin(LED_V2ON_PORT,LED_V2ON_PIN,ON_LED);
							HAL_GPIO_WritePin(Backlight_Port,Backlight_Pin,ON_Backlig);
							//TM_HD44780_Puts(10, 1, "BYPASS");
							BacklightTime = 20000;
							HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,ON_BUZZER);
							HAL_GPIO_WritePin(LED_FAULT_PORT,LED_FAULT_PIN,ON_LED);
							
							State = State_Under;	
							TIMER_Flag =0;
						}
					}
					if((SourceValue == 2)&&(VRMS_b >UnderValue))
					{
						if(VRMS_a >UnderResValue){
							hysteresisRLY = hysteresisTimedefine;
							HAL_GPIO_WritePin(Rly_port,Under_Rly,OFF_Rly);
							HAL_GPIO_WritePin(Backlight_Port,Backlight_Pin,ON_Backlig);
							HAL_GPIO_WritePin(LED_V2ON_PORT,LED_V2ON_PIN,OFF_LED);
							HAL_GPIO_WritePin(LED_V1ON_PORT,LED_V1ON_PIN,ON_LED);
							//TM_HD44780_Puts(10, 1, "BYPASS");
							BacklightTime = 20000;
							HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,ON_BUZZER);
							HAL_GPIO_WritePin(LED_FAULT_PORT,LED_FAULT_PIN,ON_LED);
							
							State = State_Under;	
							TIMER_Flag =0;
						}
					}	
				}	
		}
/*
		else
		{
			UnderTimeCount = UnderTimSetValue*1000;
			State = State_PreUnder;		
			UnderTimeCount = UnderTimeCount - OffsetTimeError_Un ; //?? ??????
			if(Timer_flag ==0)
			{
			 StartTimer();
			}     	
		}
*/	
	}
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	  char x[20];
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();

  /* USER CODE BEGIN 2 */
	//uint8_t SPIbuf[5];
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE |UART_IT_TXE);
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); //jj
	
	//HAL_GPIO_WritePin(Rly_port,Under_Rly,OFF_Rly);
	
	
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);	//jj debug
		
	HAL_Delay(10);
	InitEnergyIC();
	HAL_Delay(10);
	InitEnergyIC();
	
	
	
	ReadSetting();
	HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,OFF_BUZZER);
	
	HAL_GPIO_WritePin(Rly_port,Fault_Rly,ON_Rly); // Alway ON Fault Relay
	
	if(SourceValue == 1)
	{
		if(rd(GPIOA,GPIO_PIN_15))
		{
			HAL_GPIO_WritePin(Rly_port,Under_Rly,OFF_Rly);
			State = State_nor;
		}
		else
		{
			HAL_GPIO_WritePin(Rly_port,Under_Rly,ON_Rly);
			State = State_Under;
		}	
	}
	else
	{
		if(rd(GPIOA,GPIO_PIN_15))
		{
			HAL_GPIO_WritePin(Rly_port,Under_Rly,ON_Rly);
			State = State_nor;
		}
		else
		{
			HAL_GPIO_WritePin(Rly_port,Under_Rly,OFF_Rly);
			State = State_Under;
		}	
	}
	
	/* Initialize LCD 20 cols x 4 rows */
	TM_HD44780_Init(16, 2);
	
	/* Save custom character on location 0 in LCD */
	//TM_HD44780_CreateChar(0, &customChar[0]);
	
	/* Put string to LCD */	

	TM_HD44780_Clear();
	TM_HD44780_Puts(0, 0,"ATS BY ESPTECHNO");
	
	TM_HD44780_Puts(0, 1,"Please Wait:");
	for(char i =0; i<=100; i++)//wait 3 sec
	{
		sprintf(x, "%d", i);
		TM_HD44780_Puts(12, 1,"   ");
		TM_HD44780_Puts(12, 1, (uint8_t*)x);
		HAL_Delay(20);
		HAL_IWDG_Refresh(&hiwdg);
	}
	
	HAL_GPIO_WritePin(Backlight_Port,Backlight_Pin,ON_Backlig);
	BacklightTime = 60000;
/*
	rmsA = GetLineVoltageA();//a jj?
	rmsA = GetLineVoltageA();//a jj?
	rmsB = GetLineVoltageB();//b jj?
	rmsB = GetLineVoltageB();//b jj?	
	
	if(rmsA < 5.0)
		rmsA = 0.0;
	if(rmsB < 5.0)
		rmsB = 0.0;
	VRMS_a = (uint16_t)rmsA;
	VRMS_b = (uint16_t)rmsB;	
	
	if(SourceValue ==1)
	{
		VRMS_SUM = VRMS_a;
	}
	else
	{
		VRMS_SUM = VRMS_b;
	}
*/	
	ShowLCD = timeshowdisplay;
	
	//VRMS_Read();
/*        Setting    */
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_IWDG_Refresh(&hiwdg);
		displayCount++;
		if(displayCount >=4500001)
		{
			displayCount = 0;
		}
		
		if((ReadVolt)&&(mode == nor))
		{
			ReadVolt =0;
			if((++ShowLCD >= timeshowdisplay)&&(hysteresisRLY ==0))
			{
				
				/*    Current Measurmemt */
				/*
				currentA = GetLineCurrentA();
				sprintf(amp, "%f", currentA);
				if(currentA > 10.0){
					amp[2]=' ';
					amp[3]='A';
					amp[4]=' ';
					amp[5]='\0';
				}	
				else{
					amp[3]='A';
					amp[4]=' ';
					amp[5]='\0';
				}
				*/
				/* Current Measurmemt */
				
				ShowLCD =0;	
				if(++ChangshowVolt >=15)
				{
					ChangshowVolt =0;
					VA = VRMS_a;
					VB = VRMS_b;	
				}
					
				sprintf(va, "%d", VA);
				if(VA >100)
				{					
					va[3]='V';
					va[4]=' ';
					va[5]='\0';				
				}
				else if((VA < 100)&&(VA >10))
				{
					va[2]='V';
					va[3]=' ';
					va[4]=' ';
					va[5]='\0';
				}
				else if(VA <10)
				{
					va[1]=va[0];
					va[2]='V';
					va[0]=' ';
					va[3]=' ';
				  va[4]=' ';
				  va[5]='\0';	
				}		
				sprintf(vb, "%d", VB);
				if(VB >100)
				{					
					vb[3]='V';
					vb[4]=' ';
					vb[5]='\0';				
				}
				else if((VB < 100)&&(VB >10))
				{
					vb[2]='V';
					vb[3]=' ';
					vb[4]=' ';
					vb[5]='\0';
				}
				else if(VB <10)
				{
					vb[1]=vb[0];
					vb[2]='V';
					vb[0]=' ';
					vb[3]=' ';
				  vb[4]=' ';
				  vb[5]='\0';	
				}
				
				if(TranferFlag)
				{
					TranferFlag =0;
					UserRxBufferFS[0] = 'A';
					CDC_Transmit_FS((uint8_t*)UserRxBufferFS, 0x1); //1 CHAR SEND
				}
				
				
				
				TM_HD44780_Puts(0, 0," V1   V2  STATUS");
				//TM_HD44780_Puts(0, 0," V1   A   STATUS");	
				
				TM_HD44780_Puts(0, 1, (uint8_t*)va);
				
				TM_HD44780_Puts(5, 1, (uint8_t*)vb);
				//TM_HD44780_Puts(5, 1, (uint8_t*)amp);
				
				//if((SourceValue ==1)&&((State == State_nor)||(State == State_Under)))
				if((SourceValue ==1)&&(State == State_nor))
				{
					if(VA >UnderResValue)
					{
						TM_HD44780_Puts(10, 1, "NORMAL");
						HAL_GPIO_WritePin(LED_V1ON_PORT,LED_V1ON_PIN,ON_LED);
						HAL_GPIO_WritePin(LED_V2ON_PORT,LED_V2ON_PIN,OFF_LED);
						HAL_GPIO_WritePin(LED_FAULT_PORT,LED_FAULT_PIN,OFF_LED);
						//HAL_GPIO_WritePin(Rly_port,Fault_Rly,OFF_Rly);
						//HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,OFF_BUZZER);//off buzzer
					}
					/*
					else
					{
						TM_HD44780_Puts(10, 1, "BYPASS");
						HAL_GPIO_WritePin(LED_V1ON_PORT,LED_V1ON_PIN,OFF_LED);
						HAL_GPIO_WritePin(LED_V2ON_PORT,LED_V2ON_PIN,ON_LED);
						HAL_GPIO_WritePin(LED_FAULT_PORT,LED_FAULT_PIN,ON_LED);
						//HAL_GPIO_WritePin(Rly_port,Fault_Rly,ON_Rly);
					}
					*/
				}
				//if((SourceValue ==2)&&((State == State_nor)||(State == State_Under)))
				if((SourceValue ==2)&&(State == State_nor))
				{
					if(VB >UnderResValue)
					{
						TM_HD44780_Puts(10, 1, "NORMAL");
						HAL_GPIO_WritePin(LED_V1ON_PORT,LED_V1ON_PIN,OFF_LED);
						HAL_GPIO_WritePin(LED_V2ON_PORT,LED_V2ON_PIN,ON_LED);
						HAL_GPIO_WritePin(LED_FAULT_PORT,LED_FAULT_PIN,OFF_LED);
						//HAL_GPIO_WritePin(Rly_port,Fault_Rly,OFF_Rly);
						//HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,OFF_BUZZER);//off buzzer
					}
					/*
					else
					{
						TM_HD44780_Puts(10, 1, "BYPASS");
						HAL_GPIO_WritePin(LED_V1ON_PORT,LED_V1ON_PIN,ON_LED);
						HAL_GPIO_WritePin(LED_V2ON_PORT,LED_V2ON_PIN,OFF_LED);
						HAL_GPIO_WritePin(LED_FAULT_PORT,LED_FAULT_PIN,ON_LED);
						//HAL_GPIO_WritePin(Rly_port,Fault_Rly,ON_Rly);
					}
					*/
				}
				/* Show LED Status */
				//if(VA > UnderValue){
					//HAL_GPIO_WritePin(LED_V1IN_PORT,LED_V1IN_PIN,ON_LED);
				//}				
				//else{
				if(VA <= UnderValue){
					HAL_GPIO_WritePin(LED_V1IN_PORT,LED_V1IN_PIN,OFF_LED);
					if(SourceValue ==1){
						HAL_GPIO_WritePin(LED_V1ON_PORT,LED_V1ON_PIN,OFF_LED);
						TM_HD44780_Puts(10, 1, "BYPASS");
					}
				}							
				//if(VB > UnderValue){
					//HAL_GPIO_WritePin(LED_V2IN_PORT,LED_V2IN_PIN,ON_LED);
				//}
				//else{
				if(VB <= UnderValue){
					HAL_GPIO_WritePin(LED_V2IN_PORT,LED_V2IN_PIN,OFF_LED);
					if(SourceValue ==2){
						HAL_GPIO_WritePin(LED_V2ON_PORT,LED_V2ON_PIN,OFF_LED);
						TM_HD44780_Puts(10, 1, "BYPASS");
					}
				}

				if(VA > UnderResValue)
					HAL_GPIO_WritePin(LED_V1IN_PORT,LED_V1IN_PIN,ON_LED);
				if(VB > UnderResValue)
					HAL_GPIO_WritePin(LED_V2IN_PORT,LED_V2IN_PIN,ON_LED);
				
				
				/* Alway ON LED_VOUT  */
				HAL_GPIO_WritePin(LED_VOUT_PORT,LED_VOUT_PIN,ON_LED); 			
			}
			
			VRMS_Read();

			
		}// End if(ReadVolt)
			
		buttonRead();  

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
  }
  /* USER CODE END 3 */

}

/* System Clock Configuration
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
    /* Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1024;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xB71A;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 16;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0x05DB;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_11 
                          |SPI2_CS_Pin|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5 
                           PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB11 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 SPI2_CS_Pin PB3 PB4 
                           PB5 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|SPI2_CS_Pin|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

void StartTimer(void)
{
	TIMER_Flag = 1;
}
void StopTimer(void)
{
	TIMER_Flag = 0;
}

////////////////M90E26///////////////////////////
uint16_t CommEnergyIC(unsigned char RW, uint16_t address, uint16_t val) {
	uint8_t SPIbuf[5];
	//unsigned char* data=(unsigned char*)&val;
	//unsigned short output;
	uint16_t Result;
  //SPI interface rate is 200 to 160k bps. It Will need to be slowed down for EnergyIC
  //switch MSB and LSB of value

  //Set read write flag
	if(RW)
	{
		address |= 0x8000;
	}
	else
	{
		address &= ~0x8000;
	}
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
  //Write address byte by byte
	delay_us(1);
  SPIbuf[1] = (uint8_t)address;	//lsb
	address = address>>8;
	SPIbuf[0] = (uint8_t)address;	//msb
	
	HAL_SPI_Transmit(&hspi2, (uint8_t*)SPIbuf, 0x02, 0x05); 
  /* Must wait 4 us for data to become valid */
  delay_us(5);

  if(RW)
  {
    HAL_SPI_Receive(&hspi2, (uint8_t*)SPIbuf, 0x02, 0x05); 
		Result = SPIbuf[0];	//msb
		Result = Result<<8;
		Result |= SPIbuf[1];	//lsb
  }
  else
  {
		SPIbuf[1] = (uint8_t)val;	//lsb
		val = val>>8;
		SPIbuf[0] = (uint8_t)val;	//msb
		HAL_SPI_Transmit(&hspi2, (uint8_t*)SPIbuf, 0x02, 0x05);
  }

  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
	//delayUS(1);

  return Result;
}
double  GetLineVoltageA() {
  unsigned short voltage = CommEnergyIC(1, UrmsA, 0xFFFF);
  return (double)voltage / 238.5;
}

double  GetLineVoltageB() {
  unsigned short voltage = CommEnergyIC(1, UrmsB, 0xFFFF);
  return (double)voltage / 238.5;
}

double  GetLineVoltageC() {
  unsigned short voltage = CommEnergyIC(1, UrmsC, 0xFFFF);
  return (double)voltage / 238.5;
}

unsigned short  GetMeterStatus0() {
  return CommEnergyIC(1, EnStatus0, 0xFFFF);
}

unsigned short  GetMeterStatus1() {
  return CommEnergyIC(1, EnStatus1, 0xFFFF);
}

double GetLineCurrentA() {
  unsigned short current = CommEnergyIC(1, IrmsA, 0xFFFF);
  return (double)current * 7.13 / 1000;
}

double GetLineCurrentB() {
  unsigned short current = CommEnergyIC(1, IrmsB, 0xFFFF);
  return (double)current * 7.13 / 1000;
}

double GetLineCurrentC() {
  unsigned short current = CommEnergyIC(1, IrmsC, 0xFFFF);
  return (double)current * 7.13 / 1000;
}

double GetActivePowerA() {
  short int apower = (short int)CommEnergyIC(1, PmeanA, 0xFFFF); //Complement, MSB is signed bit
  return (double)apower * 2.94;
}

double GetFrequency() {
  unsigned short freq = CommEnergyIC(1, Freq, 0xFFFF);
  return (double)freq / 100;
}

double GetPowerFactor() {
  short int pf = (short int)CommEnergyIC(1, PFmeanT, 0xFFFF); //MSB is signed bit
  //if negative
  if (pf & 0x8000) {
    pf = (pf & 0x7FFF) * -1;
  }
  return (double)pf / 1000;
}

double GetImportEnergy() {
  //Register is cleared after reading
  unsigned short ienergy = CommEnergyIC(1, APenergyA, 0xFFFF);
  return (double)ienergy / 10 / 1000; //returns kWh if PL constant set to 1000imp/kWh
}

double GetExportEnergy() {
  //Register is cleared after reading
  unsigned short eenergy = CommEnergyIC(1, ANenergyT, 0xFFFF);
  return (double)eenergy / 10 / 1000; //returns kWh if PL constant set to 1000imp/kWh
}

unsigned short GetSysStatus0() {
  return CommEnergyIC(1, EMMState0, 0xFFFF);
}

unsigned short GetSysStatus1() {
  return CommEnergyIC(1, EMMState1, 0xFFFF);
}

void InitEnergyIC() {
  //Serial.println("Initialising:");
  unsigned short systemstatus0;
	
	CommEnergyIC(0, MeterEn, 0xFFFF); //Perform EnMeter ---jj
	
	CommEnergyIC(0, SoftReset, 0x789A); //Perform soft reset
  //CommEnergyIC(0, FuncEn0, 0x0030); //Voltage sag irq=1, report on warnout pin=1, energy dir change irq=0
  //CommEnergyIC(0, FuncEn1, 0x0030); //Voltage sag irq=1, report on warnout pin=1, energy dir change irq=0
  CommEnergyIC(0, SagTh, 0x1F2F); //Voltage sag threshhold
  
  //Set metering config values
  CommEnergyIC(0, ConfigStart, 0x5678); //Metering calibration startup command. Register 31 to 3B need to be set
  CommEnergyIC(0, PLconstH, 0x00B9); //PL Constant MSB
  CommEnergyIC(0, PLconstL, 0xC1F3); //PL Constant LSB
  CommEnergyIC(0, MMode0, 0x0087); //Metering Mode Configuration. All defaults. See pg 58 of datasheet.
  CommEnergyIC(0, MMode1, 0x5555); //PGA Gain Configuration. x2 for DPGA and PGA. See pg 59 of datasheet
  CommEnergyIC(0, PStartTh, 0x08BD); //Active Startup Power Threshold
  CommEnergyIC(0, QStartTh, 0x0AEC); //Reactive Startup Power Threshold
  CommEnergyIC(0, CSZero, 0x5F59); //Write CSOne, as self calculated
  
  //Serial.print("Checksum 0:");
  //Serial.println(CommEnergyIC(1, CSZero, 0x0000), HEX); //Checksum 0. Needs to be calculated based off the above values.
  CommEnergyIC(1, CSZero, 0x0000);
	
  //Set metering calibration values
  CommEnergyIC(0, CalStart, 0x5678); //Metering calibration startup command. Register 41 to 4D need to be set
  CommEnergyIC(0, GainA, 0x1D39); //Line calibration gain
  CommEnergyIC(0, PhiA, 0x0000); //Line calibration angle
  CommEnergyIC(0, GainB, 0x1D39); //Line calibration gain
  CommEnergyIC(0, PhiB, 0x0000); //Line calibration angle
  CommEnergyIC(0, GainC, 0x1D39); //Line calibration gain
  CommEnergyIC(0, PhiC, 0x0000); //Line calibration angle
  CommEnergyIC(0, PoffsetA, 0x0000); //A line active power offset
  CommEnergyIC(0, QoffsetA, 0x0000); //A line reactive power offset
  CommEnergyIC(0, PoffsetB, 0x0000); //B line active power offset
  CommEnergyIC(0, QoffsetB, 0x0000); //B line reactive power offset
  CommEnergyIC(0, PoffsetC, 0x0000); //C line active power offset
  CommEnergyIC(0, QoffsetC, 0x0000); //C line reactive power offset
  CommEnergyIC(0, CSOne, 0x2402); //Write CSOne, as self calculated
  
  //Serial.print("Checksum 1:");
  //Serial.println(CommEnergyIC(1, CSOne, 0x0000), HEX); //Checksum 1. Needs to be calculated based off the above values.
	CommEnergyIC(1, CSOne, 0x0000);

  //Set measurement calibration values
  CommEnergyIC(0, AdjStart, 0x5678); //Measurement calibration startup command, registers 61-6F
  CommEnergyIC(0, UgainA, 0xD8E9);  //A SVoltage rms gain
  CommEnergyIC(0, IgainA, 0x1BC9); //A line current gain
  CommEnergyIC(0, UoffsetA, 0x0000); //A Voltage offset
  CommEnergyIC(0, IoffsetA, 0x0000); //A line current offset
  CommEnergyIC(0, UgainB, 0xD8E9);  //B Voltage rms gain
  CommEnergyIC(0, IgainB, 0x1BC9); //B line current gain
  CommEnergyIC(0, UoffsetB, 0x0000); //B Voltage offset
  CommEnergyIC(0, IoffsetB, 0x0000); //B line current offset
  CommEnergyIC(0, UgainC, 0xD8E9);  //C Voltage rms gain
  CommEnergyIC(0, IgainC, 0x1BC9); //C line current gain
  CommEnergyIC(0, UoffsetC, 0x0000); //C Voltage offset
  CommEnergyIC(0, IoffsetC, 0x0000); //C line current offset
  CommEnergyIC(0, CSThree, 0xA694); //Write CSThree, as self calculated

  //Serial.print("Checksum 3:");
  //Serial.println(CommEnergyIC(1, CSThree, 0x0000), HEX); //Checksum 3. Needs to be calculated based off the above values.
	CommEnergyIC(1, CSThree, 0x0000);
	
  CommEnergyIC(0, ConfigStart, 0x8765); //Checks correctness of 31-3B registers and starts normal metering if ok
  CommEnergyIC(0, CalStart, 0x8765); //Checks correctness of 41-4D registers and starts normal metering if ok
  CommEnergyIC(0, AdjStart, 0x8765); //Checks correct ness of 61-6F registers and starts normal measurement  if ok

  CommEnergyIC(0, EMMIntEn1, 0x200); // PhaseA,B Phase loss INT Enable.
	
	systemstatus0 = GetSysStatus0();
	
}
//--------------- FLASH fUNCTION ---------------------------------//
uint8_t FlashErase(void)
{
  uint8_t ret = 1;
  uint32_t Address;
  
  /* Unlock the Flash to enable the flash control register access *************/ 
	HAL_FLASH_Unlock();
  
  /* Erase the user Flash area ***********/

  /* Clear pending flags (if any) */  
  FLASH->SR = (FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
  
  for(Address = FLASH_PAGE_START_ADDRESS; Address < FLASH_PAGE_END_ADDRESS; Address += FLASH_PAGE_size)
  {
    /* Wait for last operation to be completed */
    while((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY);
    
    if((FLASH->SR & (uint32_t)FLASH_FLAG_WRPERR)!= (uint32_t)0x00)
    {
      /* Write protected error */
      ret = 0;
      break;
    }
    
    if((FLASH->SR & (uint32_t)(FLASH_SR_PGERR)) != (uint32_t)0x00)
    {
      /* Programming error */
      ret = 0;
      break;
    }
    
    /* If the previous operation is completed, proceed to erase the page */
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = Address;
    FLASH->CR |= FLASH_CR_STRT;
      
    /* Wait for last operation to be completed */
    while((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY);
    
    if((FLASH->SR & (uint32_t)FLASH_FLAG_WRPERR)!= (uint32_t)0x00)
    {
      /* Write protected error */
      ret = 0;
      break;
    }
    
    if((FLASH->SR & (uint32_t)(FLASH_SR_PGERR)) != (uint32_t)0x00)
    {
      /* Programming error */
      ret = 0;
      break;
    }
      
    /* Disable the PER Bit */
    FLASH->CR &= ~FLASH_CR_PER;
  }
  
  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  /* Set the LOCK Bit to lock the FLASH control register and program memory access */
  FLASH->CR |= FLASH_CR_LOCK;
  
  return ret;
}

uint8_t FlashWrite(uint32_t Address, uint8_t *Data, uint32_t Length)
{
  uint8_t ret = 1;
  uint16_t TmpData;
  
  if(Address >= FLASH_PAGE_START_ADDRESS && Address <= FLASH_PAGE_END_ADDRESS)
  {
    /* Unlock the Flash to enable the flash control register access *************/ 
    HAL_FLASH_Unlock();
    
    /* Clear pending flags (if any) */  
    FLASH->SR = (FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    
    while(Length > 0)
    {
      if(Length == 1)
      {
        TmpData = Data[0] | (0x00 << 8 );
        Data = Data + 1;
        Length = Length - 1;
      }
      else
      {
        TmpData = Data[0] | (Data[1] << 8 );
        Data = Data + 2;
        Length = Length - 2;
      }
      
      /* Wait for last operation to be completed */
      while((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY);
      
      if((FLASH->SR & (uint32_t)FLASH_FLAG_WRPERR)!= (uint32_t)0x00)
      {
        /* Write protected error */
        ret = 0;
        break;
      }
      
      if((FLASH->SR & (uint32_t)(FLASH_SR_PGERR)) != (uint32_t)0x00)
      {
        /* Programming error */
        ret = 0;
        break;
      }
      
      /* If the previous operation is completed, proceed to program the new data */
      FLASH->CR |= FLASH_CR_PG;
      
      *(__IO uint16_t*)Address = TmpData;
      
      /* Wait for last operation to be completed */
      while((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY);
      
      if((FLASH->SR & (uint32_t)FLASH_FLAG_WRPERR)!= (uint32_t)0x00)
      {
        /* Write protected error */
        ret = 0;
        break;
      }
      
      if((FLASH->SR & (uint32_t)(FLASH_SR_PGERR)) != (uint32_t)0x00)
      {
        /* Programming error */
        ret = 0;
        break;
      }
      
      /* Disable the PG Bit */
      FLASH->CR &= ~FLASH_CR_PG;
      
      /* Next address */
      Address = Address + 2;
    }
    
    /* Lock the Flash to disable the flash control register access (recommended
    to protect the FLASH memory against possible unwanted operation) *********/
    /* Set the LOCK Bit to lock the FLASH control register and program memory access */
    FLASH->CR |= FLASH_CR_LOCK;
  }
  else
    ret = 0;
  
  return ret;
}

/* $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  */
void ReadSetting(void)
{
	UnderValue = *(uint8_t *)0x801F800;
	UnderValue = UnderValue<<8;
	UnderValue |= *(uint8_t *)0x801F801;
	
	UnderResValue = *(uint8_t *)0x801F802;
	UnderResValue = UnderResValue<<8;
	UnderResValue |= *(uint8_t *)0x801F803;
		
	UnderTimSetValue = *(uint8_t *)0x801F804;
	UnderTimSetValue = UnderTimSetValue<<8;
	UnderTimSetValue |= *(uint8_t *)0x801F805;
		
	UnderResTimSetValue = *(uint8_t *)0x801F806;
	UnderResTimSetValue = UnderResTimSetValue<<8;
	UnderResTimSetValue |= *(uint8_t *)0x801F807;
	
	SourceValue = *(uint8_t *)0x801F808;
	SourceValue = SourceValue<<8;
	SourceValue |= *(uint8_t *)0x801F809;
	
	if((UnderValue >600)||(UnderValue <100))
		UnderValue = 200;
	
	if((UnderResValue >600)||(UnderResValue <100))
		UnderResValue = 210;
	
	if((UnderTimSetValue >60)||(UnderTimSetValue <0))
		UnderTimSetValue = 5;
	
	if((UnderResTimSetValue >60)||(UnderResTimSetValue <0))
		UnderResTimSetValue = 5;
	
	if((SourceValue >2)||(SourceValue <1))
		SourceValue = 1;
	
	if(SourceValue == 1)
		HAL_GPIO_WritePin(Rly_port,Under_Rly,OFF_Rly);
	else
		HAL_GPIO_WritePin(Rly_port,Under_Rly,ON_Rly);
	
	if(SourceValue ==1)
	{
		if(VA >UnderResValue){
			TM_HD44780_Puts(10, 1, "NORMAL");
			HAL_GPIO_WritePin(Rly_port,Under_Rly,OFF_Rly);
		}
		else{
			TM_HD44780_Puts(10, 1, "BYPASS");
			HAL_GPIO_WritePin(Rly_port,Under_Rly,ON_Rly);
			HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,ON_BUZZER);
		}
	}
	if(SourceValue ==2)
	{
		if(VB >UnderResValue){
			TM_HD44780_Puts(10, 1, "NORMAL");
			HAL_GPIO_WritePin(Rly_port,Under_Rly,ON_Rly);
		}
		else{
			TM_HD44780_Puts(10, 1, "BYPASS");
			HAL_GPIO_WritePin(Rly_port,Under_Rly,OFF_Rly);
			HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,ON_BUZZER);
		}
	}						
	
}

/* $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  */
//--------------------
void VRMS_Read(void)
//--------------------
{
	rmsA = GetLineVoltageA();//a jj?
	rmsA = GetLineVoltageA();//a jj?
	rmsB = GetLineVoltageB();//b jj?
	rmsB = GetLineVoltageB();//b jj?
	
	CommEnergyIC(0, EMMIntState1, 0xFFFF);
	
	if(rmsA < 50.0)
		rmsA = 0.0;
	if(rmsB < 50.0)
		rmsB = 0.0;
	VRMS_a = (uint16_t)rmsA;
	VRMS_b = (uint16_t)rmsB;
	
	FRE = (uint16_t)freq;
	if(FRE ==49)FRE =50;
	
	if(SourceValue ==1)
	{
		VRMS_SUM = VRMS_a;
	}
	else
	{
		VRMS_SUM = VRMS_b;
	}
	if(!(StartMeasureCount))
  {
			/*****************UNDER**********************/
		if((VRMS_SUM <= UnderValue)&&(State == State_nor))
		{
			UnderTimeCount = UnderTimSetValue*1000;
			State = State_PreUnder;
			if(UnderTimeCount ==0)
			{ 
				if((SourceValue == 1)&&(VRMS_b >UnderResValue))
				{
					hysteresisRLY = hysteresisTimedefine;					
					HAL_GPIO_WritePin(Rly_port,Under_Rly,ON_Rly);
					HAL_GPIO_WritePin(Backlight_Port,Backlight_Pin,ON_Backlig);
					HAL_GPIO_WritePin(LED_V1ON_PORT,LED_V1ON_PIN,OFF_LED);
					HAL_GPIO_WritePin(LED_V2ON_PORT,LED_V2ON_PIN,ON_LED);
					//TM_HD44780_Puts(10, 1, "BYPASS");
					BacklightTime = 20000;
					HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,ON_BUZZER);
					HAL_GPIO_WritePin(LED_FAULT_PORT,LED_FAULT_PIN,ON_LED);
				}
				if((SourceValue == 2)&&(VRMS_a >UnderResValue))
				{
					hysteresisRLY = hysteresisTimedefine;
					HAL_GPIO_WritePin(Rly_port,Under_Rly,OFF_Rly);
					HAL_GPIO_WritePin(Backlight_Port,Backlight_Pin,ON_Backlig);
					HAL_GPIO_WritePin(LED_V2ON_PORT,LED_V2ON_PIN,OFF_LED);
					HAL_GPIO_WritePin(LED_V1ON_PORT,LED_V1ON_PIN,ON_LED);
					//TM_HD44780_Puts(10, 1, "BYPASS");
					BacklightTime = 20000;
					HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,ON_BUZZER);
					HAL_GPIO_WritePin(LED_FAULT_PORT,LED_FAULT_PIN,ON_LED);
				}
				State = State_Under;	
				TIMER_Flag =0;
			}
			else
			{
				UnderTimeCount = UnderTimeCount - OffsetTimeError_Un ; //?? ??????
				if(Timer_flag ==0)
				{
					StartTimer();
				}
			}      
		}
		if((VRMS_SUM >= UnderValue) && (State == State_PreUnder))
		{
			UnderTimeCount = 0;
			State = State_nor;
			StopTimer();
		}
		if((VRMS_SUM >= UnderResValue) && (State == State_Under))
		{
			UnderResTimeCount = UnderResTimSetValue*1000; //1000*1ms = 1 Sec
			State = State_PreUnderRes;
			if(UnderResTimeCount == 0)
			{
				hysteresisRLY = hysteresisTimedefine;
				HAL_GPIO_WritePin(Rly_port,Under_Rly,OFF_Rly);
				if(State == State_PreUnderRes)/**/
				{
					State = State_nor;
				}
			}
			else
			{
				UnderResTimeCount = UnderResTimeCount - OffsetTimeError_UnRe; //?? ??????
				if(Timer_flag ==0)
				{
					StartTimer();
				}
			}	      
		}
		if((VRMS_SUM <= UnderResValue)&&(State == State_PreUnderRes))
		{
			State = State_Under;
			UnderResTimeCount = 0;
			StopTimer();
		} 
		/*****************OVER**********************/		
	}
					
}
/* $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  */

/* $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  */
/* $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  */
void EEPROMWriteInt(uint32_t addr, uint16_t Value)
{
	Flashdata[1 + addr] = (uint8_t)Value;
	Value = Value>>8;
	Flashdata[0 + addr] = (uint8_t)Value;
}

/* $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  */
void buttonRead(void)
{
	enum{st1, st2, st3, st4,st5};
  static uint32_t state,deb; //deb==Debount
  uint16_t tempValue;
	char datalcd[10];
	
  if(rd(BT_SET_Port,BT_SET_Pin)&&rd(BT_UP_Port,BT_UP_Pin)&&rd(BT_DW_Port,BT_DW_Pin)&&rd(BT_BK_Port,BT_BK_Pin))
  {    
    state = st1;  
    return;
  }

  if((!rd(BT_SET_Port,BT_SET_Pin)||!rd(BT_UP_Port,BT_UP_Pin)||!rd(BT_DW_Port,BT_DW_Pin)||!rd(BT_BK_Port,BT_BK_Pin))&&state == st1)
  {
    state = st2;
    deb = 20000;
    return;
  }

  if((!rd(BT_SET_Port,BT_SET_Pin)||!rd(BT_UP_Port,BT_UP_Pin)||!rd(BT_DW_Port,BT_DW_Pin)||!rd(BT_BK_Port,BT_BK_Pin))&&state == st2)
  {
    if(deb)
    {
      deb--;
      return;
    }
    else
    {	
			HAL_GPIO_WritePin(Backlight_Port,Backlight_Pin,ON_Backlig);
			BacklightTime = 50000;
			Beep();
			
			// IF LCD ERROR Press UP and Down
			if(!rd(BT_UP_Port,BT_UP_Pin)&&!rd(BT_DW_Port,BT_DW_Pin))
			{
				TM_HD44780_Init(16, 2);
				TM_HD44780_Clear();
				HAL_Delay(20);
				return;
			}
			
      if(!rd(BT_SET_Port,BT_SET_Pin))
      {
				TM_HD44780_Clear();		
				StopTimer();
				if(++mode > SetSource)
				{
					mode = normal;
					ReadVolt = 1;
					ShowLCD = timeshowdisplay-1;
				}			
				/////////////////////////
				/////////////////////////
				if(mode == nor)
				{
					if(set_mode)
					{
						set_mode = false;
						EEPROMWriteInt(Source_addr,SourceValue);
						
						/* Save data to Flash */
						FlashErase();
						FlashWrite(FLASH_PAGE_START_ADDRESS, (uint8_t*)Flashdata, 16);
						
						if(SourceValue == 1)
						{
							HAL_GPIO_WritePin(Rly_port,Under_Rly,OFF_Rly);
						}
						else
						{
							HAL_GPIO_WritePin(Rly_port,Under_Rly,ON_Rly);
						}
						
						char x[10];				
						TM_HD44780_Clear();
						TM_HD44780_CursorOff();
						TM_HD44780_BlinkOff();
						TM_HD44780_Puts(0, 0,"System Saving");					
						TM_HD44780_Puts(0, 1,"Please Wait:");
						for(char i =0; i<=100; i++)//wait 3 sec
						{
							sprintf(x, "%d", i);
							TM_HD44780_Puts(12, 1,"   ");
							TM_HD44780_Puts(12, 1, (uint8_t*)x);
							HAL_Delay(20);
							HAL_IWDG_Refresh(&hiwdg);
						}
					}
				}
				else if(mode == SetUnder)
				{
					if(set_mode)
					{
						
					}
				}
				else if(mode == SetUnderRes)
				{
					menuCount =30000;
					if(set_mode)
					{
						EEPROMWriteInt(UnderSet_addr,UnderValue); 
					}
				}
				else if(mode == SetUnderTim)
				{
					if(set_mode)
					{
						EEPROMWriteInt(UnderResSet_addr,UnderResValue); 
					}
				}
				else if(mode == SetUnderResTim)
				{
					if(set_mode)
					{
						EEPROMWriteInt(UnderTimSet_addr,UnderTimSetValue); 
					}
				}
				else if(mode == SetSource)
				{
					if(set_mode)
					{
						EEPROMWriteInt(UnderResTimSet_addr,UnderResTimSetValue); 
					}
				}
						
        menuCount =30000;
				BacklightTime = 50000;
        deb = 5*30000; //3sec
        state = st3;
      }
     
      if((!rd(BT_UP_Port,BT_UP_Pin))&&set_mode == true)
      { 
        switch(mode)
        {
          case SetUnder:
          tempValue = UnderValue;
          UnderValue++;
          if((UnderValue > MAX_VOLT)||(UnderValue > UnderResValue-5))UnderValue = tempValue;
          break;

          case SetUnderRes:
           tempValue = UnderResValue;
           UnderResValue++;
          if(UnderResValue > MAX_VOLT)UnderResValue = tempValue;
          break;
        
           case SetUnderTim:
           tempValue = UnderTimSetValue;
           UnderTimSetValue++;
          if(UnderTimSetValue > MAX_TIME)UnderTimSetValue = tempValue;
          break;

          case SetUnderResTim:
          tempValue = UnderResTimSetValue;
          UnderResTimSetValue++;
          if(UnderResTimSetValue > MAX_TIME)UnderResTimSetValue = tempValue;
          break;

          case SetSource:
          tempValue = SourceValue;
          SourceValue++;
          if(SourceValue > 2)SourceValue = 1;
          break;
					
        }  
        state = st4;
        menuCount =30000;
				BacklightTime = 50000;
      }

      if((!rd(BT_DW_Port,BT_DW_Pin))&&set_mode == true)
      {
        switch(mode)
        {
          case SetUnder:
          tempValue = UnderValue;
          UnderValue--;
          if(UnderValue < MIN_VOLT)UnderValue = tempValue;
          break;

          case SetUnderRes:
           tempValue = UnderResValue;
           UnderResValue--;
          if((UnderResValue < MIN_VOLT) || (UnderResValue < UnderValue +5))  UnderResValue = tempValue;
          break;

           case SetUnderTim:
           tempValue = UnderTimSetValue;
           UnderTimSetValue--;
          if(UnderTimSetValue < MIN_TIME)UnderTimSetValue = tempValue;
          break;

          case SetUnderResTim:
          tempValue = UnderResTimSetValue;
          UnderResTimSetValue--;
          if(UnderResTimSetValue < MIN_TIME)UnderResTimSetValue = tempValue;
          break;

          case SetSource:
          tempValue = SourceValue;
          SourceValue--;
          if(SourceValue < 1)SourceValue = 2;
          break;
        }
        state = st4;
        menuCount =30000;
				BacklightTime = 50000;
      }
			if(!rd(BT_BK_Port,BT_BK_Pin))
      {
				menuCount =30000;
				TM_HD44780_CursorOff();
				TM_HD44780_BlinkOff();
				delay_us(20);
				if(set_mode)
				{
					set_mode = false;
					ReadSetting();
				}
				/*
				else
				{
					if(SourceValue ==1)
					{
						if(VA >UnderResValue)
							TM_HD44780_Puts(10, 1, "NORMAL");
						else
							TM_HD44780_Puts(10, 1, "BYPASS");
					}
					if(SourceValue ==2)
					{
						if(VB >UnderResValue)
							TM_HD44780_Puts(10, 1, "NORMAL");
						else
							TM_HD44780_Puts(10, 1, "BYPASS");
					}						
				}
				*/
				mode = normal;
				ReadVolt = 1;
				ShowLCD = timeshowdisplay-1;
			}
			//++++++++++MENU+++++++++++++++++++//
			
			//TM_HD44780_Clear();
			switch(mode)
      {
				case SetUnder:
				TM_HD44780_Puts(0, 0,"   MENU[1/5]    ");
				TM_HD44780_Puts(0, 1,"Volt Under: ");
				
				sprintf(datalcd, "%d", UnderValue);					
				va[3]='\0';	
				TM_HD44780_Puts(12, 1, "   ");
				TM_HD44780_Puts(12, 1, (uint8_t*)datalcd);
				TM_HD44780_CursorSet(14, 1);
				break;

				case SetUnderRes:
				TM_HD44780_Puts(0, 0,"   MENU[2/5]    ");
				TM_HD44780_Puts(0, 1,"Volt Un_Re: "); 
				sprintf(datalcd, "%d", UnderResValue);					
				va[3]='\0';	
				TM_HD44780_Puts(12, 1, "   ");
				TM_HD44780_Puts(12, 1, (uint8_t*)datalcd);
				TM_HD44780_CursorSet(14, 1);
				break;

				case SetUnderTim:
				TM_HD44780_Puts(0, 0,"   MENU[3/5]    ");
				TM_HD44780_Puts(0, 1,"Time Under: ");
				sprintf(datalcd, "%d", UnderTimSetValue);					
				va[2]='\0';	
				TM_HD44780_Puts(12, 1, "   ");
				TM_HD44780_Puts(12, 1, (uint8_t*)datalcd);
				if(UnderTimSetValue <10)
					TM_HD44780_CursorSet(12, 1);
				else
					TM_HD44780_CursorSet(13, 1);
				break;

				case SetUnderResTim:
				TM_HD44780_Puts(0, 0,"   MENU[4/5]    ");
				TM_HD44780_Puts(0, 1,"Time Un_Re: ");
				sprintf(datalcd, "%d", UnderResTimSetValue);					
				va[2]='\0';	
				TM_HD44780_Puts(12, 1, "   ");
				TM_HD44780_Puts(12, 1, (uint8_t*)datalcd);
				if(UnderResTimSetValue <10)
					TM_HD44780_CursorSet(12, 1);
				else
					TM_HD44780_CursorSet(13, 1);
				break;

				case SetSource:
				TM_HD44780_Puts(0, 0,"   MENU[5/5]    ");
				TM_HD44780_Puts(0, 1,"Source Select:V");
				sprintf(datalcd, "%d", SourceValue);					
				va[1]='\0';	
				TM_HD44780_Puts(15, 1, (uint8_t*)datalcd);
				TM_HD44780_CursorSet(15, 1);
				break;

      }
			//++++++++++END MENU+++++++++++++++++++//
    }
  }

  if((!rd(BT_SET_Port,BT_SET_Pin)||!rd(BT_UP_Port,BT_UP_Pin)||!rd(BT_DW_Port,BT_DW_Pin)||!rd(BT_BK_Port,BT_BK_Pin))&&state == st3)
  {
    if(deb)
    {
      deb--;
      return;
    }
    else
    {
      if(set_mode==false)flash_dot =20;     
      set_mode = true;
      mode = SetUnder;
			TM_HD44780_Clear();
			TM_HD44780_Puts(0, 0,"   MENU[1/5]    ");
			TM_HD44780_Puts(0, 1,"Volt Under: ");
				
			sprintf(datalcd, "%d", UnderValue);					
			va[3]='\0';	
			TM_HD44780_Puts(12, 1, (uint8_t*)datalcd);
			TM_HD44780_CursorSet(14, 1);
			TM_HD44780_CursorOn();
			TM_HD44780_BlinkOn();
			for(char i =0; i<10; i++)//wait 3 sec
			{
				HAL_Delay(30);
				HAL_IWDG_Refresh(&hiwdg);
			}
			
    }
    menuCount =30000;
		BacklightTime = 50000;
  }
	
}
/* $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  */

/* $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  */
//*******************************************

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
