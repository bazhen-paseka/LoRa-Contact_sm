/**
* \version 1.0
* \author bazhen.paseka
* \13-July-2022
*************************************************************************************
* \copyright	Bazhen Paseka
* \copyright	Brovary
* \copyright	Ukraine
*************************************************************************************
*/

/*
**************************************************************************
*							INCLUDE FILES
**************************************************************************
*/
	#include "main.h"
	#include "spi.h"
	#include "usart.h"
	//#include "gpio.h"
	#include "LoRa-Contact_SM.h"
/*
**************************************************************************
*							LOCAL DEFINES
**************************************************************************
*/


/*
**************************************************************************
*							LOCAL CONSTANTS
**************************************************************************
*/


/*
**************************************************************************
*						    LOCAL DATA TYPES
**************************************************************************
*/


/*
**************************************************************************
*							  LOCAL TABLES
**************************************************************************
*/

/*
**************************************************************************
*								 MACRO'S
**************************************************************************
*/

/*
**************************************************************************
*                       LOCAL FUNCTION PROTOTYPES
**************************************************************************
*/
	void Command_button_pressed (void) ;
	void LoraMain_TX(void) ;
	void LoraMain_RX(void) ;
	void Slave_Answer(void);
/*
**************************************************************************
*						 LOCAL GLOBAL VARIABLES
**************************************************************************
*/
	SX1278_hw_t SX1278_hw;
	SX1278_t 	SX1278;

	int master;
	int ret;

	char buffer[64];

	int message;
	int message_length;
	int answer_cnt = 0 ;

	char DataChar[0xFF];
	volatile uint32_t ch1_u32 = 0;
	volatile uint32_t ch2_u32 = 0;
	volatile uint32_t ch3_u32 = 0;
	volatile uint32_t ch4_u32 = 0;
	volatile uint32_t ch5_u32 = 0;
/*
**************************************************************************
*                        LOCAL FUNCTION PROTOTYPES
**************************************************************************
*/

/*
**************************************************************************
*                           GLOBAL FUNCTIONS
**************************************************************************
*/
void LoRa_Contact_Init (void){
	int soft_version_arr_int[3];
	soft_version_arr_int[0] = ((SOFT_VERSION) / 100)     ;
	soft_version_arr_int[1] = ((SOFT_VERSION) /  10) %10 ;
	soft_version_arr_int[2] = ((SOFT_VERSION)      ) %10 ;

	sprintf(DataChar,"\r\n\r\n\tLoRa over sx1278 v%d.%d.%d \r\nUART1 for debug on speed 115200 \r\n",
			soft_version_arr_int[0] ,
			soft_version_arr_int[1] ,
			soft_version_arr_int[2] ) ;
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	#define 	DATE_as_int_str 	(__DATE__)
	#define 	TIME_as_int_str 	(__TIME__)
	sprintf(DataChar,"Build: %s. Time: %s.\r\n" ,
			DATE_as_int_str ,
			TIME_as_int_str ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

	master = MASTER ;
	if (master == 1) {
		sprintf(DataChar, "Mode: Master\r\n" );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	} else {
		sprintf(DataChar, "Mode: Slave\r\n" );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	}

	//initialize LoRa module
	SX1278_hw.dio0.port		= DIO0_GPIO_Port;
	SX1278_hw.dio0.pin 		= DIO0_Pin;
	SX1278_hw.nss.port 		= NSS_GPIO_Port;
	SX1278_hw.nss.pin 		= NSS_Pin;
	SX1278_hw.reset.port	= RESET_GPIO_Port;
	SX1278_hw.reset.pin 	= RESET_Pin;
	SX1278_hw.spi 			= &hspi1;

	SX1278.hw = &SX1278_hw;

	sprintf(DataChar, "Configuring LoRa module ... " );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	SX1278_begin(	&SX1278					,
					SX1278_433MHZ			,
					SX1278_POWER_17DBM		,
					SX1278_LORA_SF_8		,
					SX1278_LORA_BW_20_8KHZ	,
					10						);

	sprintf(DataChar, " done.\r\n" );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	if (master == 1) {
		ret = SX1278_LoRaEntryTx(&SX1278, 16, 2000);
	} else {
		ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);
	}
} //***************************************************************************

void LoRa_Contact_Main (void){
	if (master == 1) {
		if (	(ch1_u32 == 1)
			||	(ch2_u32 == 1)
			||	(ch3_u32 == 1)
			||	(ch4_u32 == 1)
			||	(ch5_u32 == 1)) {
			ret = SX1278_LoRaEntryTx(&SX1278, 16, 2000);
			sprintf(DataChar, "set Master: %d\r\n", ret );
			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
			HAL_Delay(2500);

			Command_button_pressed();
			//LoraMain_TX();

			ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);
			sprintf(DataChar, "set Slave: %d\r\n", ret );
			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
			HAL_Delay(2500);
			HAL_Delay(10000);
		}
	} else {
		LoraMain_RX();
	}
} //***************************************************************************

/***************************************************************************
*                           LOCAL FUNCTIONS
***************************************************************************/

void LoraMain_TX(void) {
	message_length = sprintf(buffer, "connect-%d", message);
	ret = SX1278_LoRaEntryTx(	&SX1278			,
								message_length	,
								2000			);
	sprintf(DataChar, "entry: %d, ", ret );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	sprintf(DataChar, "send: %s, ", buffer );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	ret = SX1278_LoRaTxPacket(	&SX1278				,
								(uint8_t *) buffer	,
								message_length		,
								2000				);
	message++;

	sprintf(DataChar, "trans: %d.\r\n", ret );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	HAL_Delay(800);
} //***************************************************************************

void LoraMain_RX(void) {
	ret = SX1278_LoRaRxPacket(&SX1278);
	sprintf(DataChar, "Rx%d: %d ", SLAVE_NUMBER, ret );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	if (ret > 0) {
		SX1278_read(&SX1278, (uint8_t *) buffer, ret);
		sprintf(DataChar, "\"%s\"", buffer );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		if (buffer[4] == SLAVE_NUMBER + '0' ) {
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			sprintf(DataChar, "\t\t- - - BINGO - - - - \r\n" );
			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
			Slave_Answer();
		} else {
			sprintf(DataChar, "\r\n" );
			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		}
	} else {
		sprintf(DataChar, " \r\n" );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	}
	HAL_Delay(1000);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
} //***************************************************************************

void Slave_Answer(void){
	sprintf(DataChar, "set master...\r\n" );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	ret = SX1278_LoRaEntryTx(&SX1278, 16, 2000);	// Master
	HAL_Delay(1000);

	message_length = sprintf(buffer, "BOX-5");
	ret = SX1278_LoRaEntryTx(	&SX1278			,
								message_length	,
								2000			);
	sprintf(DataChar, "entry answer: %d, ", ret );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	sprintf(DataChar, "send answer: %s, ", buffer );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	ret = SX1278_LoRaTxPacket(	&SX1278				,
								(uint8_t *) buffer	,
								message_length		,
								2000				);
	answer_cnt++;
	sprintf(DataChar, "trans answert: %d.\r\n", ret );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	HAL_Delay(1000);

	sprintf(DataChar, "set Slave...\r\n" );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);	// Slave
	HAL_Delay(1000);
} //***************************************************************************

void Command_button_pressed(void) {
	if (ch1_u32 == 1 ) {
		message_length = sprintf(buffer, "Box-1");
		ret = SX1278_LoRaEntryTx(	&SX1278			,
									message_length	,
									2000			);
		sprintf(DataChar, "entry: %d, ", ret );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

		sprintf(DataChar, "send: %s,", buffer );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		ret = SX1278_LoRaTxPacket(	&SX1278				,
									(uint8_t *) buffer	,
									message_length		,
									2000				);
		sprintf(DataChar, "trans: %d.\r\n", ret );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		HAL_Delay(2500);
		ch1_u32 = 0 ;
	}	//---------------------------------------------------------------------

	if (ch2_u32 == 1 ) {
		message_length = sprintf(buffer, "Box-2");
		ret = SX1278_LoRaEntryTx(	&SX1278			,
									message_length	,
									2000			);
		sprintf(DataChar, "entry: %d, ", ret );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

		sprintf(DataChar, "send: %s,", buffer );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		ret = SX1278_LoRaTxPacket(	&SX1278				,
									(uint8_t *) buffer	,
									message_length		,
									2000				);
		sprintf(DataChar, "trans: %d.\r\n", ret );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		HAL_Delay(2500);
		ch2_u32 = 0 ;
	}	//---------------------------------------------------------------------

	if (ch3_u32 == 1 ) {
		message_length = sprintf(buffer, "BOX-3");
		ret = SX1278_LoRaEntryTx(	&SX1278			,
									message_length	,
									2000			);
		sprintf(DataChar, "entry: %d, ", ret );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

		sprintf(DataChar, "send: %s, ", buffer );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		ret = SX1278_LoRaTxPacket(	&SX1278				,
									(uint8_t *) buffer	,
									message_length		,
									2000				);
		sprintf(DataChar, "trans: %d.\r\n", ret );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		HAL_Delay(2500);
		ch3_u32 = 0 ;
	}	//---------------------------------------------------------------------

	if (ch4_u32 == 1 ) {
		message_length = sprintf(buffer, "BOX-4");
		ret = SX1278_LoRaEntryTx(	&SX1278			,
									message_length	,
									2000			);
		sprintf(DataChar, "entry: %d, ", ret );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

		sprintf(DataChar, "send: %s, ", buffer );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		ret = SX1278_LoRaTxPacket(	&SX1278				,
									(uint8_t *) buffer	,
									message_length		,
									2000				);
		sprintf(DataChar, "trans: %d.\r\n", ret );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		HAL_Delay(2500);
		ch4_u32 = 0 ;
	}	//---------------------------------------------------------------------

	if (ch5_u32 == 1 ) {
		message_length = sprintf(buffer, "Box-5");
		ret = SX1278_LoRaEntryTx(	&SX1278			,
									message_length	,
									2000			);
		sprintf(DataChar, "entry: %d, ", ret );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

		sprintf(DataChar, "send: %s, ", buffer );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		ret = SX1278_LoRaTxPacket(	&SX1278				,
									(uint8_t *) buffer	,
									message_length		,
									2000				);
		sprintf(DataChar, "trans: %d.\r\n", ret );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		HAL_Delay(2500);
		ch5_u32 = 0 ;
	} //---------------------------------------------------------------------
} //***************************************************************************

//***************************************************************************
//***************************************************************************

