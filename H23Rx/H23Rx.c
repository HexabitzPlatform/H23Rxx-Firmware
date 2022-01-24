/*
    BitzOS (BOS) V0.2.5 - Copyright (C) 2017-2021 Hexabitz
    All rights reserved

    File Name     : H23Rx.c
    Description   : Source code for module H23R0/H23R1.
				 	module (BT800/BT900)
*/

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"


/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Module exported parameters ------------------------------------------------*/
module_param_t modParam[NUM_MODULE_PARAMS] = {{.paramPtr=NULL, .paramFormat=FMT_FLOAT, .paramName=""}};

/* Private variables ---------------------------------------------------------*/

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;


EventGroupHandle_t handleUartTerminal = NULL;





/* BT900 variables and Data Structures: -------------------------------------------*/


uint8_t BT_User_Buffer_Length = 192;


uint8_t BT_Devices_Name[BT_Max_Number_Of_Devices][BT_Max_Device_Name_Length] = {0};
uint8_t BT_Devices_Address[BT_Max_Number_Of_Devices][BT_Device_Address_Length]= {0};
uint8_t BT_Devices_Index=0;
uint8_t BT_Rx=0;
uint8_t BT_User_Buffer[192] = {0};
uint8_t BT_User_Buffer_Index = 0;
uint8_t BT_Commands_Buffer[BT_Command_Buffer_Length] = {0};
uint8_t BT_Commands_Buffer_Index = 0;
uint8_t BT_BOS_Index = 0;


uint8_t* BT_User_Buffer_ptr = BT_User_Buffer;
uint8_t* BT_User_Buffer_beginning_ptr = BT_User_Buffer;
uint8_t* BT_User_Buffer_Index_ptr = &BT_User_Buffer_Index;

uint8_t BT_To_User_Buffer_flag = 0;
//1: Bluetooth To User_Buffer
//0: Bluetooth To BOS Messaging Buffer

uint8_t BT_Connection_flag = 0;
//1: Connected
//0: Disconnected


uint8_t BT_SPP_Mode = 0;
//0: SPP_Command_Mode
//1: SPP_Bridge_Mode

uint8_t BT_delete_connecting_char_flag = 0;
uint8_t BT_delete_disconnecting_char_flag = 0;

uint8_t BT_boot = 1; //flag for sending name to BT900 Module on startup
//--------------------------------------------------------------------------------------


/* BT900 Private functions -----------------------------------------------*/

Module_Status BT_RUN_SCRIPT_MODE(void){

	Module_Status result = H23Rx_OK;

	if(BT_Connection_flag == 0)
	{
	    writePxMutex(PORT_BTC_CONN,(char*)"\r",1,cmd50ms, HAL_MAX_DELAY);
	    Delay_ms_no_rtos(200);
	    writePxMutex(PORT_BTC_CONN,(char*)"at+run \"$autorun$\"\r",19,cmd50ms, HAL_MAX_DELAY);
	    Delay_ms_no_rtos(200);
	}
	else
	{
		result = H23Rx_ERROR;

	}
	return result;

}
//-------------------------------------------------------------------------------


Module_Status BT_Switch_To_SPP_Command_Mode(void){
	Module_Status result = H23Rx_OK;

	if(BT_Connection_flag == 1) //Switching from Bridge mode to Command mode
	{
		BT_SPP_Mode = 0; //SPP_Command_Mode
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		Delay_ms_no_rtos(200);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

	}
	else //Bluetooth is not connected, so we are in Command mode already
	{
	    result = H23Rx_ERROR;
	}
	return result;
}
//---------------------------------------------------------------------
Module_Status BT_Switch_To_SPP_Bridge_Mode(void){
	Module_Status result = H23Rx_OK;

	if(BT_Connection_flag == 1) //Switching from Command mode to Bridge mode
	{
		BT_SPP_Mode = 1; //SPP_Bridge_Mode
		writePxMutex(PORT_BTC_CONN,(char*)"ato\r",4,cmd50ms, HAL_MAX_DELAY);
	}
	else //Bluetooth is not connected, so we can't switch to Bridge mode
	{
	    result = H23Rx_ERROR;
	}
	return result;
}
//----------------------------------------------------------------------------------------

void BT_Delete_Script(void){
    writePxMutex(PORT_BTC_CONN,(char*)"\r",1,cmd50ms, HAL_MAX_DELAY);
    Delay_ms_no_rtos(200);
    writePxMutex(PORT_BTC_CONN,(char*)"exit\r",5,cmd50ms, HAL_MAX_DELAY);
    Delay_ms_no_rtos(200);
    writePxMutex(PORT_BTC_CONN,(char*)"\r",1,cmd50ms, HAL_MAX_DELAY);
    Delay_ms_no_rtos(200);
	writePxMutex(PORT_BTC_CONN, (char *) "at&f 1\r", 7, cmd50ms, HAL_MAX_DELAY);
    Delay_ms_no_rtos(200);
}
/*-------------------------------------------------------------------*/
void BT_Download_Script(void){
	/* Change baudrate to 115200 to match BT900 UART */
		UpdateBaudrate(PcPort, 115200);
		/* setup DMA stream */
		StartScastDMAStream(PORT_BTC_CONN, myID, PcPort, myID, BIDIRECTIONAL, H23Rx_MAX_NUMBER_OF_DATA_DMA, 0xFFFFFFFF, false);
}
/*-------------------------------------------------------------------*/


/* Create CLI commands --------------------------------------------------------*/

static portBASE_TYPE btClearUserBuffer( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btSendData( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btDisconnect( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btDeleteScriptCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btDownloadScriptCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btConnectCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btScanCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btSetNameCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btSetDiscoverableCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btStreamToPort( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );




//
/* CLI command structure : bt-clear-user-buffer */
const CLI_Command_Definition_t btClearUserBufferCommandDefinition =
{
	( const int8_t * ) "bt-clear-user-buffer", /* The command string to type. */
	( const int8_t * ) "bt-clear-user-buffer:\r\n Clear BT900 User Data Buffer\r\n\r\n",
	btClearUserBuffer, /* The function to run. */
	0/* No Parameters are expected. */
};
//--------------------------------------------------------------------------
/* CLI command structure : bt-send-data */
const CLI_Command_Definition_t btSendDataCommandDefinition =
{
	( const int8_t * ) "bt-send-data", /* The command string to type. */
	( const int8_t * ) "bt-send-data:\r\n Send Data over BT900 Bluetooth Module \r\n\r\n",
	btSendData, /* The function to run. */
	1/* One Parameter is expected. */
};
//--------------------------------------------------------------------------
/* CLI command structure : bt-disconnect */
const CLI_Command_Definition_t btDisconnectCommandDefinition =
{
	( const int8_t * ) "bt-disconnect", /* The command string to type. */
	( const int8_t * ) "bt-disconnect:\r\n Disconnect BT900 module\r\n\r\n",
	btDisconnect, /* The function to run. */
	0/* No Parameters are expected. */
};
//--------------------------------------------------------------------------
/* CLI command structure : bt-delete-script */
const CLI_Command_Definition_t btDeleteScriptCommandDefinition =
{
	( const int8_t * ) "bt-delete-script", /* The command string to type. */
	( const int8_t * ) "bt-delete-script:\r\n Delete current smartBASIC script on BT900. This should be called before writing a new script on the module\r\n\r\n",
	btDeleteScriptCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
//--------------------------------------------------------------------------
/* CLI command structure : bt-download-script */
const CLI_Command_Definition_t btDownloadScriptCommandDefinition =
{
	( const int8_t * ) "bt-download-script", /* The command string to type. */
	( const int8_t * ) "bt-download-script:\r\n Download new $autorun$ script to BT900 module (1st parameter):uart\r\n\r\n",
	btDownloadScriptCommand, /* The function to run. */
	1 /* No parameters are expected. */
};
//--------------------------------------------------------------------------
/* CLI command structure : connect */
const CLI_Command_Definition_t btConnectCommandDefinition =
{
	( const int8_t * ) "bt-connect", /* The command string to type. */
	( const int8_t * ) "bt-connect:\r\n Connect to another bluetooth device \r\n\r\n",
	btConnectCommand, /* The function to run. */
	1 /* One parameter is expected. */
};
//-----------------------------------------------------------------------------
/* CLI command structure : scan */
const CLI_Command_Definition_t btScanCommandDefinition =
{
	( const int8_t * ) "bt-scan", /* The command string to type. */
	( const int8_t * ) "bt-scan:\r\n Scan nearby BLE devices and display them in a list along with their SSIDs\r\n\r\n",
	btScanCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
//--------------------------------------------------------------------------------
/* CLI command structure : Set Name */
const CLI_Command_Definition_t btSetNameCommandDefinition =
{
	( const int8_t * ) "bt-set-name", /* The command string to type. */
	( const int8_t * ) "bt-set-name:\r\n Setting BT900 Module Name\r\n\r\n",
	btSetNameCommand, /* The function to run. */
	1 /* One parameter is expected. */
};
//--------------------------------------------------------------------------------
/* CLI command structure : Set Discoverable */
const CLI_Command_Definition_t btSetDiscoverableCommandDefinition =
{
	( const int8_t * ) "bt-set-discoverable", /* The command string to type. */
	( const int8_t * ) "bt-set-discoverable:\r\n Setting BT900 Module discoverable\r\n\r\n",
	btSetDiscoverableCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
//--------------------------------------------------------------------------------
/* CLI command structure : bt-stream-to-port */
const CLI_Command_Definition_t btStreamToPortDefinition =
{
	( const int8_t * ) "bt-stream-to-port", /* The command string to type. */
	( const int8_t * ) "bt-stream-to-port:\r\n Streaming BT900 data to another port\r\n\r\n",
	btStreamToPort, /* The function to run. */
	1/* One Parameter is expected. */
};
//--------------------------------------------------------------------------------





/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   -----------------------------------------------------------------------
*/

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PREDIV                         = 1
  *            PLLMUL                         = 6
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	
	__HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV32;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	

	__SYSCFG_CLK_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	
}
/* --- Save array topology and Command Snippets in Flash RO --- 
*/
uint8_t SaveToRO(void)
{
	BOS_Status result = BOS_OK; 
	HAL_StatusTypeDef FlashStatus = HAL_OK;
	uint16_t add = 2, temp = 0;
	uint8_t snipBuffer[sizeof(snippet_t)+1] = {0};
	
	HAL_FLASH_Unlock();
	
	/* Erase RO area */
	FLASH_PageErase(RO_START_ADDRESS);
	FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE); 
	if(FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	} else {			
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	}	
	
	/* Save number of modules and myID */
	if (myID)
	{
		temp = (uint16_t) (N<<8) + myID;
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, RO_START_ADDRESS, temp);
		FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE); 
		if (FlashStatus != HAL_OK) {
			return pFlash.ErrorCode;
		} else {
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
		}			
	
	/* Save topology */
		for(uint8_t i=1 ; i<=N ; i++)
		{
			for(uint8_t j=0 ; j<=MaxNumOfPorts ; j++)
			{
				if (array[i-1][0]) {
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, RO_START_ADDRESS+add, array[i-1][j]);
					add += 2;
					FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE); 
					if (FlashStatus != HAL_OK) {
						return pFlash.ErrorCode;
					} else {
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					}		
				}				
			}
		}
	}
	
	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for(uint8_t s=0 ; s<numOfRecordedSnippets ; s++) 
	{
		if (snippets[s].cond.conditionType) 
		{
			snipBuffer[0] = 0xFE;		// A marker to separate Snippets
			memcpy( (uint8_t *)&snipBuffer[1], (uint8_t *)&snippets[s], sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j=0 ; j<(sizeof(snippet_t)/2) ; j++)
			{		
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, currentAdd, *(uint16_t *)&snipBuffer[j*2]);
				FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE); 
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 2;
				}				
			}			
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j=0 ; j<((strlen(snippets[s].cmd)+1)/2) ; j++)
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, currentAdd, *(uint16_t *)(snippets[s].cmd+j*2));
				FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE); 
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 2;
				}				
			}				
		}	
	}
	
	HAL_FLASH_Lock();
	
	return result;
}

/* --- Clear array topology in SRAM and Flash RO --- 
*/
uint8_t ClearROtopology(void)
{
	// Clear the array 
	memset(array, 0, sizeof(array));
	N = 1; myID = 0;
	
	return SaveToRO();
}



/* --- H23R0 module initialization.
*/
void Module_Peripheral_Init(void)
{


	/* clean message buffer */
	memset(cMessage[PORT_BTC_CONN-1], 0, MAX_MESSAGE_SIZE);
	messageLength[PORT_BTC_CONN-1] = 0;

	/* Array ports */
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART4_UART_Init();
  MX_USART5_UART_Init();
  MX_USART6_UART_Init();

	/* FT234XD/BT900 UART */
  MX_USART3_UART_Init();

	
//	/* BT800/BT900 EN_RST */
#ifdef H23R1
	BT_MODE_GPIO_Init();
#endif
  /* clear global data for list */
  //cleanListBtcDevices();
  /* setting baudrate */
  UpdateBaudrate(PORT_BTC_CONN, 115200); /* Normal baudrate for BT900 */
  /* clean global variable */
  /* create a event group for UART port */

  //Sending Bluetooth Name to BT900 Module:
  Delay_ms_no_rtos(2000); //Waiting for BT900 Module to get initialized
  uint8_t BT_Module_Name[30] = {0};
  uint16_t BT_Name_Length = 0;
  uint16_t temp;
  EE_ReadVariable(_EE_H23x_Name_Length_Address, &BT_Name_Length);
  for(uint16_t i=0;i<BT_Name_Length;i++)
  {
	  EE_ReadVariable(_EE_H23x_Name_Beginning_Address + i, &temp);
	  BT_Module_Name[i] = (uint8_t)temp;
  }

  BT_Set_Name(BT_Module_Name, (uint8_t)BT_Name_Length);

}

/*-----------------------------------------------------------*/


/* --- H23R0 message processing task
*/
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift)
{
	Module_Status result = H23Rx_OK;

	switch (code)
	{
		case CODE_H23Rx_SCAN_INQUIRE:
			result = BT_Scan();
			break;

		case CODE_H23Rx_CONNECT_INQUIRE:
			result = BT_Connect(cMessage[port-1][4]); //send device index as a parameter in the first byte
			break;

		case CODE_H23Rx_DISCONNECT_INQUIRE:
			BT_Disconnect();
			break;

		case CODE_H23Rx_CLEAR_USER_BUFFER:
			result = BT_Clear_User_Buffer();
			break;

		case CODE_H23Rx_SEND_DATA:
			//The first parameter is data length ( cMessage[port-1][4] )
			result = BT_Send_Data(&cMessage[port-1][5], cMessage[port-1][4]);
			break;

		case CODE_H23Rx_SET_NAME:
			//The first parameter is name length ( cMessage[port-1][4] )
			result = BT_Set_Name(&cMessage[port-1][5], cMessage[port-1][4]);
			break;

		case CODE_H23Rx_SET_DISCOVERABLE:
			result = BT_Set_Discoverable();
			break;

		case CODE_H23Rx_STREAM_TO_PORT:
			result = BT_Stream_To_Port(cMessage[port-1][4]);
			break;


		default:
			result = H23Rx_ERR_UnknownMessage;
			break;

	}

	return result;
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands
*/
void RegisterModuleCLICommands(void)
{
	FreeRTOS_CLIRegisterCommand (&btClearUserBufferCommandDefinition);
	FreeRTOS_CLIRegisterCommand (&btSendDataCommandDefinition);
	FreeRTOS_CLIRegisterCommand (&btDisconnectCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &btDeleteScriptCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &btDownloadScriptCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &btConnectCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &btScanCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &btSetNameCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &btSetDiscoverableCommandDefinition);
	FreeRTOS_CLIRegisterCommand (&btStreamToPortDefinition);



}

/*-----------------------------------------------------------*/

/* --- Get the port for a given UART.
*/
uint8_t GetPort(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART4)
			return P1;
	else if (huart->Instance == USART2)
			return P2;
	else if (huart->Instance == USART6)
			return P3;
	else if (huart->Instance == USART1)
			return P4;
	else if (huart->Instance == USART5)
			return P5;
	else if (huart->Instance == USART3)
			return P6;

	return 0;
}

/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   -----------------------------------------------------------------------
*/

void BT_RESET_MODULE(void){

	BT_Disconnect();
	Delay_ms(1000);
	writePxMutex(PORT_BTC_CONN,(char*)"\r",1,cmd50ms, HAL_MAX_DELAY);
	Delay_ms_no_rtos(20);
	writePxMutex(PORT_BTC_CONN,(char*)"exit\r",5,cmd50ms, HAL_MAX_DELAY);
	Delay_ms_no_rtos(500);
	writePxMutex(PORT_BTC_CONN,(char*)"\r",1,cmd50ms, HAL_MAX_DELAY);
	Delay_ms_no_rtos(100);
	writePxMutex(PORT_BTC_CONN,(char*)"ATZ\r",4,cmd50ms, HAL_MAX_DELAY);
	Delay_ms(2000);

}
//------------------------------------------

void BT_Receive_Data_To_BOS(void){

	BT_To_User_Buffer_flag = 0;

}
//------------------------------------------
Module_Status BT_Receive_Data(uint8_t* buffer,uint8_t size){
	Module_Status result = H23Rx_OK;
	if(buffer!= NULL && size!=0)
	{
		BT_To_User_Buffer_flag = 1;
		BT_User_Buffer_beginning_ptr = buffer;
		BT_User_Buffer_ptr = buffer;
		BT_User_Buffer_Length = size;
		*BT_User_Buffer_Index_ptr = 0;

	}
	else
	{
		result = H23Rx_ERROR;
	}
	return result;

}
//------------------------------------------

 void BT_Disconnect(void){
		BT_Connection_flag = 0;
		BT_SPP_Mode = 0; //SPP_Command_Mode
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		Delay_ms_no_rtos(1000);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

}
/*-------------------------------------------------------------------*/

Module_Status BT_Scan(void){
	Module_Status result = H23Rx_OK;

	if(BT_Connection_flag == 0) //Scanning Bluetooth devices:
	{

		writePxMutex(PORT_BTC_CONN, (char *) "setdiscoverable 1 0\r", 20, cmd50ms, HAL_MAX_DELAY);
	    Delay_ms(500);
		writePxMutex(PORT_BTC_CONN, (char *) "setpairable 1\r", 14, cmd50ms, HAL_MAX_DELAY);
	    Delay_ms(500);
		memset(BT_Commands_Buffer,0,sizeof(BT_Commands_Buffer));
		memset(BT_Devices_Name,0,sizeof(BT_Devices_Name[0][0])*300);
		memset(BT_Devices_Address,0,sizeof(BT_Devices_Address[0][0])*120);
		BT_Devices_Index=0;
		BT_Commands_Buffer_Index=0;
		writePxMutex(PORT_BTC_CONN, (char *) "at+bti\r", 7, cmd50ms, HAL_MAX_DELAY);
	    Delay_ms(12000);
	    int i=0,dev_counter=0,j=0;
	    for(i;i<BT_Commands_Buffer_Index;i++){
			if(BT_Commands_Buffer[i]== 'I' ||
			   BT_Commands_Buffer[i]=='N' ||
			   BT_Commands_Buffer[i]=='Q') dev_counter++;
			else dev_counter=0;

			if(dev_counter==3){
				dev_counter=0;
				i+=3;
				for(j=0;j<12;j++){
					BT_Devices_Address[BT_Devices_Index][j]=BT_Commands_Buffer[i];
					i++;
				}
				i+=5;
				j=0;
				while(BT_Commands_Buffer[i]!='\n'){
					BT_Devices_Name[BT_Devices_Index][j]=BT_Commands_Buffer[i];
					j++;
					i++;
				}
				BT_Devices_Index++;
			}

	    }


	}
	else //Bluetooth is Connected, so we can't do scan
	{
	    result = H23Rx_ERROR;
	}
	return result;
}
/*-------------------------------------------------------------------*/

 Module_Status BT_Connect(uint8_t BT_Device){
	Module_Status result = H23Rx_OK;

	if(BT_Connection_flag == 0 && BT_Devices_Index!=0 && BT_Device < BT_Devices_Index && BT_Device>=0)
	{
		uint8_t connect_command[20]= {0};
		memcpy((char *)&connect_command[0], (char *)"at+btw ", 7);
		memcpy((char *)&connect_command[7], (char *)&BT_Devices_Address[BT_Device-1], 12);
		memcpy((char *)&connect_command[19], (char *)"\r", 1);
		writePxMutex(PORT_BTC_CONN, (char *)&connect_command[0], 20, cmd50ms, HAL_MAX_DELAY);

		Delay_ms(12000);

		memset(connect_command,0,sizeof(connect_command));
		memcpy((char *)&connect_command[0], (char *)"atd ", 4);
		memcpy((char *)&connect_command[4], (char *)&BT_Devices_Address[BT_Device], 12);
		memcpy((char *)&connect_command[16], (char *)"\r", 1);
		writePxMutex(PORT_BTC_CONN, (char *)&connect_command[0], 17, cmd50ms, HAL_MAX_DELAY);
		Delay_ms(3000);



	}
	else
	{
	    result = H23Rx_ERROR;
	}

	return result;

}
/*-------------------------------------------------------------------*/
 Module_Status BT_Clear_User_Buffer(void){

	 Module_Status result = H23Rx_OK;


	 if(BT_To_User_Buffer_flag == 1)
	 {
		memset(BT_User_Buffer_beginning_ptr,0,BT_User_Buffer_Length);
		BT_User_Buffer_ptr = BT_User_Buffer_beginning_ptr;
		*BT_User_Buffer_Index_ptr = 0;
	 }
	 else
	 {
		 result = H23Rx_ERROR;
	 }

	 return result;


 }
 /*-------------------------------------------------------------------*/
  Module_Status BT_Send_Data(uint8_t* BT_Data,uint8_t length){

	Module_Status result = H23Rx_OK;
	if(BT_Connection_flag == 1 && length!=0 && BT_Data!=NULL)
	{
		for(int i=0;i<length;i++)
		{
			writePxMutex(PORT_BTC_CONN, (char *)&BT_Data[0+i], 1, cmd50ms, HAL_MAX_DELAY);
		}

	}
	else
	{
		if(BT_Connection_flag == 1) result = H23Rx_ERR_WrongParams;
		else result = H23Rx_ERROR;
	}

	return result;

 }
 /*-------------------------------------------------------------------*/
  Module_Status BT_Send_Message(uint8_t dst,uint16_t code,uint16_t numberOfParams){

	Module_Status result = H23Rx_OK;
	if(BT_Connection_flag == 1)
	{
		SendMessageToModule(dst,code,numberOfParams);
	}
	else
	{
	    result = H23Rx_ERROR;
	}

	return result;



  }
  /*-------------------------------------------------------------------*/
  Module_Status BT_Set_Discoverable(void){
	  Module_Status result = H23Rx_OK;

	  if(BT_Connection_flag == 0)
	  	{
		  	 writePxMutex(PORT_BTC_CONN, (char *) "setdiscoverable 1 0\r", 20, cmd50ms, HAL_MAX_DELAY);
	  	}

	  else //Bluetooth is Connected, so we can't set it as Discoverable
		{
		  	 result = H23Rx_ERROR;
		}

		return result;
  }
  /*-------------------------------------------------------------------*/
  Module_Status BT_Set_Name(uint8_t* name, uint8_t length){
	  Module_Status result = H23Rx_OK;



	  if(BT_Connection_flag == 0)
	  	{
		  if(length > 30) length=30; //Max length: 30 character
		  else if(length <= 0) length = 1; //Min length: 1 character
		  	 uint8_t command [30] = "setname ";
		 	 memcpy(&command[8],name, length);
		 	 memcpy(&command[8+length],"\r",1);
		 	 length+=9;
		 	 if(BT_boot == 1)
		 	 {
			  	 HAL_UART_Transmit(P6uart, (uint8_t *)command, length, HAL_MAX_DELAY);
			  	 BT_boot = 0;
		 	 }
		 	 else
		 	 {
			  	 writePxMutex(PORT_BTC_CONN, (char *) command, length, cmd50ms, HAL_MAX_DELAY);
		 	 }


		  	 //Storing new name in EEPROM:
		  	 length-=9;
		  	 EE_WriteVariable(_EE_H23x_Name_Length_Address, length);
		  	 for(uint8_t i=0;i<length;i++)
		  	 {
		  		 EE_WriteVariable(_EE_H23x_Name_Beginning_Address + i, name[i]);

		  	 }


	  	}

	  else //Bluetooth is Connected, so we can't set it as Discoverable
		{
		  	 result = H23Rx_ERROR;
		}

		return result;
  }
  /*-------------------------------------------------------------------*/
  Module_Status BT_Stream_To_Port(uint8_t port_number){
  	Module_Status result = H23Rx_OK;
  	if(port_number >= 1 && port_number<6)
  	{
  		if(BT_Connection_flag == 1)
  		{
  			//UpdateBaudrate(port_number, 115200);
  			StartScastDMAStream(PORT_BTC_CONN, myID, port_number, myID, BIDIRECTIONAL, 0xFFFFFFFF, 0xFFFFFFFF, false);
  		}
  		else
  		{
  			result = H23Rx_ERROR;
  		}
  	}
  	else
  	{
  		result = H23Rx_ERR_WrongParams;
  	}
  	return result;
  }
  /*-------------------------------------------------------------------*/







/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|															Commands																 	|
   -----------------------------------------------------------------------
*/
static portBASE_TYPE btClearUserBuffer( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	BT_Clear_User_Buffer();

	writePxMutex(PcPort, "Clearing BT900 User Data Buffer\r\n", 33, cmd50ms, HAL_MAX_DELAY);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}
//--------------------------------------------------------------------------------------------------------

static portBASE_TYPE btSendData( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{

	Module_Status result = H23Rx_OK;
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Obtain the 1st parameter string: Data to be sent*/
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);

	result = BT_Send_Data(pcParameterString1, xParameterStringLength1);

	if(result == H23Rx_ERR_WrongParams)
	{
		writePxMutex(PcPort, "Wrong input parameter (No data to be sent)\r\n", 44, cmd50ms, HAL_MAX_DELAY);
	}
	else if(result == H23Rx_ERROR)
	{
		writePxMutex(PcPort, "BT900 module is Not connected to any device\r\n", 45, cmd50ms, HAL_MAX_DELAY);
	}
	else
	{
		writePxMutex(PcPort,"Done! (Data has been sent)\r\n", 28, cmd50ms, HAL_MAX_DELAY);
	}
	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}
//--------------------------------------------------------------------------------------------------------

static portBASE_TYPE btDisconnect( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if(BT_Connection_flag == 1)
	{
		BT_Disconnect();
		sprintf( ( char * ) pcWriteBuffer, "BT900 Disconnected\r\n");
	}
	else
	{
		sprintf( ( char * ) pcWriteBuffer, "BT900 is Not Connected\r\n");

	}


	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}
//--------------------------------------------------------------------------------------------------------
static portBASE_TYPE btDeleteScriptCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );


	sprintf( ( char * ) pcWriteBuffer, "Current smartBASIC script deleted successfuly\r\n");


	BT_Delete_Script();


	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}
//----------------------------------------------------------------------------------------------

static portBASE_TYPE btDownloadScriptCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	Module_Status result = H23Rx_OK;
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Obtain the 1st parameter string */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);

	if (!strncmp((const char *)pcParameterString1, "uart", 4))
	{
		writePxMutex(PcPort, "Downloading new smartBASIC program to BT900 via UART. Please set baudrate to 115200 then open this port again and load the smartBASIC file\r\n", 140, cmd50ms, HAL_MAX_DELAY);
		BT_Download_Script();
	}
	else
	{
		result = H23Rx_ERR_WrongParams;
	}

	if (H23Rx_ERR_WrongParams == result)
	{
		writePxMutex(PcPort, "Wrong input parameter\r\n",23, cmd50ms, HAL_MAX_DELAY);
	}

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}
//----------------------------------------------------------------------------------------------
static portBASE_TYPE btConnectCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;
	uint8_t lenPar = 0;
	Module_Status result = H23Rx_OK;
	uint8_t temp[5];
	uint8_t device;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if (BT_Devices_Index != 0)
	{
		/* Obtain the 1st parameter string */
		pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);


		lenPar = strlen((char *)pcParameterString1);
		if(lenPar < 3)
		{
			sprintf((char*)pcWriteBuffer, "Error\r\n");
		}
		else if ( ('[' == pcParameterString1[0]) && (']' == pcParameterString1[lenPar - 1]) )
		{
			memcpy(&temp[0], &pcParameterString1[1], lenPar - 2);
			device = (uint8_t)atol((char*)temp);
			sprintf((char *)pcWriteBuffer, "Connecting to bluetooth device %s:\r\n", pcParameterString1);
			writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
			/* clean terminal output */
			memset((char *)pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
			result = BT_Connect(device);

			if(result == H23Rx_OK)
			{
			sprintf((char *)pcWriteBuffer, "Connected\r\n");
			}

			else
			{
			sprintf((char*)pcWriteBuffer, "Error\r\n");
			}

		}
		else
		{
			sprintf((char *)pcWriteBuffer, "Wrong input parameter\r\n");
		}
	}
	else
	{
		sprintf((char *)pcWriteBuffer, "Must call \"scan\" command first\r\n");
	}

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}
//-------------------------------------------------------------------------------------
static portBASE_TYPE btScanCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	Module_Status result = H23Rx_OK;
	uint8_t name_length=0;

	/* Scan */
  sprintf( (char *)pcWriteBuffer, "Scanning nearby bluetooth devices..\r\n\nIndex\tName\r\n\r\n");
  writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
  /* clean terminal output */
  memset((char *)pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
  result = BT_Scan();

  if (result != H23Rx_ERROR)
  {
	  if(BT_Devices_Index ==0)
	  {
		  writePxMutex(PcPort, "\r\n\r\nNo Devices found\r\n", 22, cmd50ms, HAL_MAX_DELAY);
	  }
	  else //Printing Devices to terminal:
	  {
		  for(int i=0;i<BT_Devices_Index;i++)
		  {
			  sprintf( (char *)pcWriteBuffer, "%d\t",i);
			  writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
			  name_length = strlen((char *)&BT_Devices_Name[i][0]);
			  if(name_length!=0)
			  {
				  writePxMutex(PcPort, (char *)&BT_Devices_Name[i][0], name_length, cmd50ms, HAL_MAX_DELAY);
			  }
			  else
			  {
				  writePxMutex(PcPort, "[Hidden Device]", 15, cmd50ms, HAL_MAX_DELAY);
			  }
			 writePxMutex(PcPort, "\r\n", 2, cmd50ms, HAL_MAX_DELAY);
		  }
		  writePxMutex(PcPort, "\nDone\r\n", 7, cmd50ms, HAL_MAX_DELAY);


	  }
  }
  else
  {
	  writePxMutex(PcPort, "Bluetooth is already Connected, so we can't execute scan command\r\n", 66, cmd50ms, HAL_MAX_DELAY);

  }

  	memset((char *)pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}


/*-----------------------------------------------------------*/
static portBASE_TYPE btSetNameCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	Module_Status result = H23Rx_OK;
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Obtain the 1st parameter string: New BT900 Module name */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);


	if(xParameterStringLength1 == 0 ) result = H23Rx_ERR_WrongParams;
	else
	{
		result = BT_Set_Name(pcParameterString1,xParameterStringLength1);
	}


	if (H23Rx_ERR_WrongParams == result)
	{
		writePxMutex(PcPort, "Wrong input parameter\r\n",23, cmd50ms, HAL_MAX_DELAY);
	}
	else if(H23Rx_ERROR == result)
	{
		writePxMutex(PcPort, "BT900 is Connected to another device, we can't change its name\r\n",64, cmd50ms, HAL_MAX_DELAY);
	}
	else
	{
		writePxMutex(PcPort, "Done!\r\n",7, cmd50ms, HAL_MAX_DELAY);
	}

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}
//----------------------------------------------------------------------------------------------
static portBASE_TYPE btSetDiscoverableCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{

	Module_Status result = H23Rx_OK;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	result = BT_Set_Discoverable();

	if(result ==  H23Rx_ERROR)
	{
		writePxMutex(PcPort, "BT900 is Connected to another device, we can't set it as discoverable\r\n",71, cmd50ms, HAL_MAX_DELAY);
	}
	else
	{
		writePxMutex(PcPort, "Done!\r\n",7, cmd50ms, HAL_MAX_DELAY);
	}


	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}
//----------------------------------------------------------------------------------------------
static portBASE_TYPE btStreamToPort( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{

	Module_Status result = H23Rx_OK;
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;
	uint8_t lenPar = 0;
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Obtain the 1st parameter string: Data to be sent*/
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);

	lenPar = strlen((char *)pcParameterString1);
	if(lenPar < 2)
	{
		result = H23Rx_ERR_WrongParams;
	}
	else if ( ('p' == pcParameterString1[0]) || ('P' == pcParameterString1[0]) )
	{
		result = BT_Stream_To_Port(pcParameterString1[1] - '0');
	}


	if(result == H23Rx_ERR_WrongParams)
	{
		writePxMutex(PcPort, "Wrong input parameter (Wrong Port)\r\n", 36, cmd50ms, HAL_MAX_DELAY);
	}
	else if(result == H23Rx_ERROR)
	{
		writePxMutex(PcPort, "Can't Start Stream, BT900 module is Not connected to any device\r\n", 65, cmd50ms, HAL_MAX_DELAY);
	}
	else
	{
		writePxMutex(PcPort,"Streaming BT900 Data to ", 24, cmd50ms, HAL_MAX_DELAY);
		writePxMutex(PcPort,pcParameterString1, lenPar, cmd50ms, HAL_MAX_DELAY);
		writePxMutex(PcPort,", baudrate: 115200\r\n", 20, cmd50ms, HAL_MAX_DELAY);


	}
	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}
//----------------------------------------------------------------------------------------------



/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
