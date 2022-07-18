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
	#include "iwdg.h"
	#include "rng.h"
	#include "LoRa-Contact_SM.h"
/*
**************************************************************************
*							LOCAL DEFINES
**************************************************************************
*/
	#define	TX_TIME		500
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
	void Command_button_pressed (int _box_number) ;
	void LoraMain_TX(void) ;
	void LoraMain_RX(void) ;
	void LoraMaster_RX(void) ;
	void Slave_Answer(void);
	void Get_UID_96bit(uint32_t *UID) ;
/*
**************************************************************************
*						 LOCAL GLOBAL VARIABLES
**************************************************************************
*/
	SX1278_hw_t SX1278_hw;
	SX1278_t 	SX1278;

	int ret;
	char buffer[64];
	int message;
	int message_length;

	char DataChar[0xFF];
	volatile uint32_t ch_u32[5] = { 0 };
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

	sprintf(DataChar,"\r\n\r\n\tLoRa-Contact-L053 over sx1278 v%d.%d.%d \r\nUART1 for debug on speed 115200 \r\n",
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
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	uint32_t myUID[3] = { 0 };
	Get_UID_96bit(myUID); // Unique device ID register (96 bits)
	sprintf(DataChar, "UID: %010lu %010lu %010lu\r\n", myUID[0], myUID[1], myUID[2]);
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	uint32_t myRNG_u32 = HAL_RNG_GetRandomNumber(&hrng);
	sprintf(DataChar, "rng: %010lu\r\n", myRNG_u32 );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

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

#if (MASTER == 1)
	ret = SX1278_LoRaEntryTx(&SX1278, 16, 2000);
	sprintf(DataChar, "mode Master=%d\r\n", ret);
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
#elif
	ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);
	sprintf(DataChar, "mode Slave=%d\r\n", ret );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
#endif

} //***************************************************************************

void LoRa_Contact_Main (void){
#if MASTER == 1
	for (int box_number = 0; box_number < SLAVE_QNT; box_number++) {
		if (ch_u32[box_number] == 1) {
			ret = SX1278_LoRaEntryTx(&SX1278, 16, 2000);
			Command_button_pressed(box_number);
			//LoraMain_TX();
			ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);
			HAL_IWDG_Refresh(&hiwdg);
		}
	}
	LoraMaster_RX();
	HAL_IWDG_Refresh(&hiwdg);
#elif
	LoraMain_RX();
	HAL_IWDG_Refresh(&hiwdg);
#endif
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
	sprintf(DataChar, "Rx%d %d byte: ", SLAVE_NUMBER, ret );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	if (ret > 0) {
		SX1278_read(&SX1278, (uint8_t *) buffer, ret);
			sprintf(DataChar, "\t\"%s\"", buffer );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		if (buffer[4] == SLAVE_NUMBER + '0' ) {
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			HAL_Delay(1100);
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
	ret = SX1278_LoRaEntryTx(&SX1278, 16, 2000);	// Master
	message_length = sprintf( buffer, "Confirm-%d", SLAVE_NUMBER);
	ret = SX1278_LoRaEntryTx( &SX1278, message_length, 2000 ) ;

	sprintf(DataChar, " \t Answer: \"%s\"\r\n", buffer );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	ret = SX1278_LoRaTxPacket(	&SX1278, (uint8_t *) buffer, message_length, 2000 ) ;
	ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);	// Slave
} //***************************************************************************

void Command_button_pressed(int _box_number) {
	message_length = sprintf(buffer, "BOX-%d", _box_number+1 );
	ret = SX1278_LoRaEntryTx ( &SX1278, message_length, 2000 ) ;
	ret = SX1278_LoRaTxPacket( &SX1278, (uint8_t *) buffer, message_length, 2000 ) ;
	sprintf(DataChar, "send: %s\r\n", buffer );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	HAL_Delay(TX_TIME);
	ch_u32[_box_number] = 0 ;
} //***************************************************************************

void LoraMaster_RX(void) {
	int beeper = 0;
	ret = SX1278_LoRaRxPacket(&SX1278);
	sprintf(DataChar, "MasterRx %d byte: ", ret );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	if (ret > 0) {
		SX1278_read(&SX1278, (uint8_t *) buffer, ret);
		sprintf(DataChar, "\t\"%s\"\r\n", buffer );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		beeper = buffer[8] - '0';
		sprintf(DataChar, "Beeper=%d\r\n", beeper );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

		for (int b=0; b<beeper; b++) {
			HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			HAL_Delay(200);
		}
	} else {
		sprintf(DataChar, " \r\n" );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	}
	HAL_Delay(1001-beeper*200);
} //***************************************************************************

void Get_UID_96bit(uint32_t *UID) {	// Unique device ID register (96 bits)
  UID[0] = (uint32_t)(READ_REG(*((uint32_t *)UID_BASE)));
  UID[1] = (uint32_t)(READ_REG(*((uint32_t *)(UID_BASE + 4U))));
  UID[2] = (uint32_t)(READ_REG(*((uint32_t *)(UID_BASE + 8U))));
} //***************************************************************************


//***************************************************************************

