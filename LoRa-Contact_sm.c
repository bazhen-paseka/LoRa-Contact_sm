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
	#include "Lora_local_config.h"
#ifdef L053
	#include "rng.h"
#endif
	#include "crc.h"
	#include "LoRa-Contact_SM.h"
	#include "Lora_local_config.h"
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

	#define	LORA_SIZE	80
	int ret;
	char buffer[LORA_SIZE];
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

#ifdef L053
	uint32_t myRNG_u32 = HAL_RNG_GetRandomNumber(&hrng);
	sprintf(DataChar, "rng: %010lu\r\n", myRNG_u32 );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
#endif

	//CRC Start
	sprintf(DataChar, "CRC Start... \r\n" );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	__IO uint32_t uwCRCValue = 0;
	#define CRC_BUFFER_SIZE    114

	static const uint32_t CRCDataBuffer[ CRC_BUFFER_SIZE ] = {
		0x00001021, 0x20423063, 0x408450a5, 0x60c670e7, 0x9129a14a, 0xb16bc18c,
		0xd1ade1ce, 0xf1ef1231, 0x32732252, 0x52b54294, 0x72f762d6, 0x93398318,
		0xa35ad3bd, 0xc39cf3ff, 0xe3de2462, 0x34430420, 0x64e674c7, 0x44a45485,
		0xa56ab54b, 0x85289509, 0xf5cfc5ac, 0xd58d3653, 0x26721611, 0x063076d7,
		0x569546b4, 0xb75ba77a, 0x97198738, 0xf7dfe7fe, 0xc7bc48c4, 0x58e56886,
		0x78a70840, 0x18612802, 0xc9ccd9ed, 0xe98ef9af, 0x89489969, 0xa90ab92b,
		0x4ad47ab7, 0x6a961a71, 0x0a503a33, 0x2a12dbfd, 0xfbbfeb9e, 0x9b798b58,
		0xbb3bab1a, 0x6ca67c87, 0x5cc52c22, 0x3c030c60, 0x1c41edae, 0xfd8fcdec,
		0xad2abd0b, 0x8d689d49, 0x7e976eb6, 0x5ed54ef4, 0x2e321e51, 0x0e70ff9f,
		0xefbedfdd, 0xcffcbf1b, 0x9f598f78, 0x918881a9, 0xb1caa1eb, 0xd10cc12d,
		0xe16f1080, 0x00a130c2, 0x20e35004, 0x40257046, 0x83b99398, 0xa3fbb3da,
		0xc33dd31c, 0xe37ff35e, 0x129022f3, 0x32d24235, 0x52146277, 0x7256b5ea,
		0x95a88589, 0xf56ee54f, 0xd52cc50d, 0x34e224c3, 0x04817466, 0x64475424,
		0x4405a7db, 0xb7fa8799, 0xe75ff77e, 0xc71dd73c, 0x26d336f2, 0x069116b0,
		0x76764615, 0x5634d94c, 0xc96df90e, 0xe92f99c8, 0xb98aa9ab, 0x58444865,
		0x78066827, 0x18c008e1, 0x28a3cb7d, 0xdb5ceb3f, 0xfb1e8bf9, 0x9bd8abbb,
		0x4a755a54, 0x6a377a16, 0x0af11ad0, 0x2ab33a92, 0xed0fdd6c, 0xcd4dbdaa,
		0xad8b9de8, 0x8dc97c26, 0x5c644c45, 0x3ca22c83, 0x1ce00cc1, 0xef1fff3e,
		0xdf7caf9b, 0xbfba8fd9, 0x9ff86e17, 0x7e364e55, 0x2e933eb2, 0x0ed11ef0
	};

	for(int i=0; i<3; i++){
		uwCRCValue = HAL_CRC_Calculate(&hcrc, (uint32_t *)CRCDataBuffer, CRC_BUFFER_SIZE);
		uint32_t uwExpectedCRCValue = 0x379E9F06;
		if(uwCRCValue != uwExpectedCRCValue) {
			sprintf(DataChar, "CRC calc - ERROR. \r\n" );
			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		} else {
			sprintf(DataChar, "CRC calc - Ok. \r\n" );
			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		}
	}

//// AES Start ****************************************************************************
//	uint32_t	AES_Data_u32[64]	= { 0 } ;
//	struct 		AES_ctx 			my_AES;
//	uint8_t AES_KEY[16] = {0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF};
//	//static const uint8_t AES_IV[16]  = {0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA};
//	uint8_t AES_IV[16]  = { 0 };
//
//	for (int i=0; i<64; i++) {
//		AES_Data_u32[i] = 1111000000 + i ;	//	generate New data
//	}
//
//	for (int i=0; i<64; i++) {
//		if (i%8 == 0) {
//			sprintf(DataChar,"\r\n");
//			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
//		}
//		sprintf(DataChar," %010lu ", AES_Data_u32[i] ) ;	// print new data
//		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
//
//	}
//	sprintf(DataChar,"\r\n" );
//	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
//
//#ifdef L053
//	for (int i=0; i<16; i++) {
//		AES_IV[i] = (uint8_t)((0x0000000000001111)&(HAL_RNG_GetRandomNumber(&hrng)));	//	Create random AES_IV
//	}
//#endif
//
//	AES_init_ctx_iv(&my_AES, AES_KEY, AES_IV);
//	AES_CBC_encrypt_buffer(&my_AES, (uint8_t *)&AES_Data_u32, 256 );		//	encryption
//
//	for (int i=0; i<64; i++) {
//		if (i%8 == 0) {
//			sprintf(DataChar,"\r\n");
//			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
//		}
//		sprintf(DataChar," %010lu ", AES_Data_u32[i]  ) ;
//		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
//	}
//	sprintf(DataChar,"\r\n" );
//	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
//
//	AES_init_ctx_iv(&my_AES, AES_KEY, AES_IV);
//	AES_CBC_decrypt_buffer(&my_AES, (uint8_t *)&AES_Data_u32, 256);		//	decryption:
//
//	for (int i=0; i<64; i++) {
//		if (i%8 == 0) {
//			sprintf(DataChar,"\r\n");
//			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
//		}
//		sprintf(DataChar," %010lu ", AES_Data_u32[i] ) ;	// print decryption data
//		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
//	}
//	sprintf(DataChar,"\r\n" );
//	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

// initialize LoRa module
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
#else
		ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);
		sprintf(DataChar, "mode Slave=%d\r\n", ret );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
#endif
} //***************************************************************************

void LoRa_Contact_Main (void){
#if (MASTER == 1)
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
#else
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
	uint8_t RX_buffer_u8[90] = {0};
	ret = SX1278_LoRaRxPacket(&SX1278);
	sprintf(DataChar, "Rx%d %d byte: ", SLAVE_NUMBER, ret );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	if (ret > 0) {
		SX1278_read(&SX1278, (uint8_t *) RX_buffer_u8, ret);
		sprintf(DataChar, "%s", RX_buffer_u8 );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

		HAL_CRC_DeInit(&hcrc);
		HAL_CRC_Init(&hcrc);

		__IO uint32_t CRC_Value_u32 = 0;
		CRC_Value_u32 = HAL_CRC_Accumulate(&hcrc, (uint32_t *)&RX_buffer_u8, 20 );
		sprintf(DataChar, "RX-CRC: %08lX (397FD0B6)\r\n", CRC_Value_u32 );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
//		char bufferCRC[90] = {0};


		#define CRC_BUFFER_SIZE    114

		static const uint32_t CRCDataBuffer[ CRC_BUFFER_SIZE ] = {
			0x00001021, 0x20423063, 0x408450a5, 0x60c670e7, 0x9129a14a, 0xb16bc18c,
			0xd1ade1ce, 0xf1ef1231, 0x32732252, 0x52b54294, 0x72f762d6, 0x93398318,
			0xa35ad3bd, 0xc39cf3ff, 0xe3de2462, 0x34430420, 0x64e674c7, 0x44a45485,
			0xa56ab54b, 0x85289509, 0xf5cfc5ac, 0xd58d3653, 0x26721611, 0x063076d7,
			0x569546b4, 0xb75ba77a, 0x97198738, 0xf7dfe7fe, 0xc7bc48c4, 0x58e56886,
			0x78a70840, 0x18612802, 0xc9ccd9ed, 0xe98ef9af, 0x89489969, 0xa90ab92b,
			0x4ad47ab7, 0x6a961a71, 0x0a503a33, 0x2a12dbfd, 0xfbbfeb9e, 0x9b798b58,
			0xbb3bab1a, 0x6ca67c87, 0x5cc52c22, 0x3c030c60, 0x1c41edae, 0xfd8fcdec,
			0xad2abd0b, 0x8d689d49, 0x7e976eb6, 0x5ed54ef4, 0x2e321e51, 0x0e70ff9f,
			0xefbedfdd, 0xcffcbf1b, 0x9f598f78, 0x918881a9, 0xb1caa1eb, 0xd10cc12d,
			0xe16f1080, 0x00a130c2, 0x20e35004, 0x40257046, 0x83b99398, 0xa3fbb3da,
			0xc33dd31c, 0xe37ff35e, 0x129022f3, 0x32d24235, 0x52146277, 0x7256b5ea,
			0x95a88589, 0xf56ee54f, 0xd52cc50d, 0x34e224c3, 0x04817466, 0x64475424,
			0x4405a7db, 0xb7fa8799, 0xe75ff77e, 0xc71dd73c, 0x26d336f2, 0x069116b0,
			0x76764615, 0x5634d94c, 0xc96df90e, 0xe92f99c8, 0xb98aa9ab, 0x58444865,
			0x78066827, 0x18c008e1, 0x28a3cb7d, 0xdb5ceb3f, 0xfb1e8bf9, 0x9bd8abbb,
			0x4a755a54, 0x6a377a16, 0x0af11ad0, 0x2ab33a92, 0xed0fdd6c, 0xcd4dbdaa,
			0xad8b9de8, 0x8dc97c26, 0x5c644c45, 0x3ca22c83, 0x1ce00cc1, 0xef1fff3e,
			0xdf7caf9b, 0xbfba8fd9, 0x9ff86e17, 0x7e364e55, 0x2e933eb2, 0x0ed11ef0
		};

		__IO uint32_t testCRCValue = 0;
		testCRCValue = HAL_CRC_Accumulate(&hcrc, (uint32_t *)CRCDataBuffer, CRC_BUFFER_SIZE);
	//	uint32_t uwExpectedCRCValue = 0x379E9F06;

		sprintf(DataChar, "\r\nTestCRC: %08lX (379E9F06)\r\n", testCRCValue );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

//		struct AES_ctx	my_AES;
//		uint8_t AES_KEY[16] = {0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF};
//		uint8_t AES_IV[16]  = {0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA};
//		AES_init_ctx_iv(&my_AES, AES_KEY, AES_IV);
//		AES_CBC_decrypt_buffer(&my_AES, (uint8_t *)&buffer, LORA_SIZE);		//	decryption:
//		sprintf(DataChar, "\r\ndecryption:\r\n%s\r\n", buffer );
//		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

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
	//message_length = sprintf(buffer, "BOX-%d", _box_number+1 );

	uint8_t buffer_tx_u8[LORA_SIZE] = { 0 };
	for (int i=0; i<(LORA_SIZE-1); i++) {
		buffer_tx_u8[i] = (char)(i+40);
	}
	buffer_tx_u8[LORA_SIZE]='\0';
	message_length = LORA_SIZE;

//	struct 		AES_ctx 			my_AES;
//	uint8_t AES_KEY[16] = {0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF};
//	uint8_t AES_IV[16]  = {0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA};
//	AES_init_ctx_iv(&my_AES, AES_KEY, AES_IV);
//	AES_CBC_encrypt_buffer(&my_AES, (uint8_t *)&buffer, LORA_SIZE );		//	encryption

	HAL_CRC_DeInit(&hcrc);
	HAL_CRC_Init(&hcrc);

	__IO uint32_t CRC_Value_u32 = 0;
	CRC_Value_u32 = HAL_CRC_Accumulate(&hcrc, (uint32_t *)&buffer_tx_u8, LORA_SIZE/4 );
	sprintf(DataChar, "CRC: %08lX (397FD0B6)\r\n", CRC_Value_u32 );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	char bufferCRC[90] = {0};

	sprintf(bufferCRC, "%s%lX", buffer_tx_u8, CRC_Value_u32 );

//	memcpy((uint8_t*)&bufferCRC, (uint8_t*)&buffer_tx_u8, LORA_SIZE);
//	bufferCRC[80] = (uint8_t)((0x000000FF )&( CRC_Value_u32    )) ;
//	bufferCRC[81] = (uint8_t)((  0x0000FF )&( CRC_Value_u32>>4 )) ;
//	bufferCRC[82] = (uint8_t)((    0x00FF )&( CRC_Value_u32>>8 )) ;
//	bufferCRC[83] = (uint8_t)((      0xFF )&( CRC_Value_u32>>12)) ;
//	bufferCRC[84] = '\0';


	ret = SX1278_LoRaEntryTx ( &SX1278, 88, 2000 ) ;
	ret = SX1278_LoRaTxPacket( &SX1278, (uint8_t *) bufferCRC, 88, 2000 ) ;
	sprintf(DataChar, "send: %s;\r\n", bufferCRC );
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

