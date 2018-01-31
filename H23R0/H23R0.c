/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved

    File Name     : H23R0.c
    Description   : Source code for module H23R0.
										Bluetooth module (BT800/BT900)

		Required MCU resources :

			>> USARTs 1,2,4,5,6 for module ports.
			>> USART 3 for FT234XD connected to BT800 USB / for BT900 UART.
			>> PB15 for BT800/BT900 EN_RST.
			>> PB2 for BT900 RST.
			>> PA7 for BT900 BT_MODE.
			>> PB14 for BT900 BT_VSP.
			>> PB12 for BT900 Host_Wkup.

		                                  | BT_MODE (nAutoRUN) |    BT_VSP (SIO_19 ) |
		| Self-contained Run mode         |           0        |          0          |
		| Interactive / development mode  |           1        |          0          |
		| Over the Air (OTA)              |           0        |          0          |
		| VSP Bridge-to-UART mode         |           1        |          0          |
		| VSP Command mode                |           0        |          0          |
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


/* Private variables ---------------------------------------------------------*/

TaskHandle_t ControlBluetoothTaskHandle = NULL;

/* Private function prototypes -----------------------------------------------*/
void ControlBluetoothTask(void * argument);
void btEnableHandshakeUart(void);
void btDisableHandshakeUart(void);
void btSendMsgToTerminal(uint8_t *pStr);
void btShowMsgOnTerminal(uint8_t *pStr1, uint8_t *pStr2);
HAL_StatusTypeDef btSendCommandToBtc(uint8_t *command);
void btResetBt900Module(void);
void btUpdateScript(void);
void btRunScript(void);
Module_Status btVspMode(int8_t inputVspMode);

/* Create CLI commands --------------------------------------------------------*/

static portBASE_TYPE btGetInfoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btUpdateScriptCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btRunScriptCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btVspModeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btSetBaudrateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btScanCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btConnectCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* CLI command structure : bt-info */
const CLI_Command_Definition_t btGetInfoCommandDefinition =
{
	( const int8_t * ) "bt-info", /* The command string to type. */
	( const int8_t * ) "bt-info:\r\n Command to get information of BT900 module\r\n\r\n",
	btGetInfoCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

/* CLI command structure : bt-update-script */
const CLI_Command_Definition_t btUpdateScriptCommandDefinition =
{
	( const int8_t * ) "bt-update-script", /* The command string to type. */
	( const int8_t * ) "bt-update-script:\r\n Command to update new $autorun$ script\r\n\r\n",
	btUpdateScriptCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

/* CLI command structure : bt-run-script */
const CLI_Command_Definition_t btRunScriptCommandDefinition =
{
	( const int8_t * ) "bt-run-script", /* The command string to type. */
	( const int8_t * ) "bt-run-script:\r\n Command to restart BT900 with $autorun$ script\r\n\r\n",
	btRunScriptCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

/* CLI command structure : bt-vsp-mode */
const CLI_Command_Definition_t btVspModeCommandDefinition =
{
	( const int8_t * ) "bt-vsp-mode", /* The command string to type. */
	( const int8_t * ) "bt-vsp-mode:\r\n Set VSP mode for bluetooth module (BT900) (1st parameter): command or bridge\r\n\r\n",
	btVspModeCommand, /* The function to run. */
	1 /* No parameters are expected. */
};

/* CLI command structure : bt-set-baudrate */
const CLI_Command_Definition_t btSetBaudrateCommandDefinition =
{
	( const int8_t * ) "bt-set-baudrate", /* The command string to type. */
	( const int8_t * ) "bt-set-baudrate:\r\n Set connection speed between MCU and Bluetooth module \r\n\r\n",
	btSetBaudrateCommand, /* The function to run. */
	1 /* One parameter is expected. */
};

/* CLI command structure : scan */
const CLI_Command_Definition_t btScanCommandDefinition =
{
	( const int8_t * ) "scan", /* The command string to type. */
	( const int8_t * ) "scan:\r\n Scan nearby ble devices and display them in a list along with their SSIDs and RSSI levels\r\n\r\n",
	btScanCommand, /* The function to run. */
	0 /* One parameter is expected. */
};

/* CLI command structure : connect */
const CLI_Command_Definition_t btConnectCommandDefinition =
{
	( const int8_t * ) "connect", /* The command string to type. */
	( const int8_t * ) "connect:\r\n Set connection with other bluetooth device \r\n\r\n",
	btConnectCommand, /* The function to run. */
	1 /* One parameter is expected. */
};



/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   -----------------------------------------------------------------------
*/

/* --- H23R0 module initialization.
*/
void Module_Init(void)
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

	/* BT800/BT900 EN_RST */
	BT_RST_GPIO_Init();
#ifdef H23R1
	BT_VSP_GPIO_Init();
	BT_MODE_GPIO_Init();
	BT_HOST_WKUP_GPIO_Init();
	BT_CTS_RTS_GPIO_Init();
#endif
  /* setting baudrate */
  UpdateBaudrate(PORT_BTC_CONN, 115200); /* Normal baudrate for BT900 */
	/* Create the Bluetooth module task */
	xTaskCreate(ControlBluetoothTask, (const char *) "ControlBluetooth", (2*configMINIMAL_STACK_SIZE), NULL, osPriorityNormal, &ControlBluetoothTaskHandle);
	/* In default, the BT900 will run in the "Self-contained Run mode" */
	/* btRunScript(); */
	btVspMode(H23R0_RUN_VspBridgeToUartMode);
}

/*-----------------------------------------------------------*/

/* ControlBluetoothTask function
*/
void ControlBluetoothTask(void * argument)
{
	static uint16_t code_field = 0;
  uint8_t tMessage[MAX_MESSAGE_SIZE] = {0};
  int8_t *pcOutputString;


	/* Infinite loop */
	for(;;)
	{
		/* Wait forever until a message is received from the Bluetooth module */
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    /* code field of message */
    code_field = cMessage[PORT_BTC_CONN-1][4] - 0x30;
    code_field += (cMessage[PORT_BTC_CONN-1][3] - 0x30) * 10;
    code_field += (cMessage[PORT_BTC_CONN-1][2] - 0x30) * 100;
    code_field += (cMessage[PORT_BTC_CONN-1][1] - 0x30) * 1000;
    switch(code_field)
    {
      case CODE_H23R0_EVBTC_SPPCONN:
        break;

      case CODE_H23R0_EVBTC_SPPDISCON:
        break;

      case CODE_H23R0_EVBTC_PAIR_REQUEST:
        break;

      case CODE_H23R0_EVBTC_PIN_REQUEST:
        break;

      case CODE_H23R0_EVBTC_PAIR_RESULT:
        break;

      case CODE_H23R0_EVBTC_AUTHREQ:
        break;

      case CODE_H23R0_EVBTC_PASSKEY:
        break;

      case CODE_H23R0_LED_STATUS_ON:
        IND_ON();
        break;

      case CODE_H23R0_LED_STATUS_OFF:
        IND_OFF();
        break;

      case CODE_H23R0_SHOW_DEBUG_INFO:
      	break;

      case CODE_H23R0_SCAN_RESPONSE:
      	/* Obtain the address of the output buffer */
				pcOutputString = FreeRTOS_CLIGetOutputBuffer();
				memcpy(pcOutputString, &cMessage[PORT_BTC_CONN-1][5], (size_t)(messageLength[PORT_BTC_CONN-1]-4));
				/*btShowMsgOnTerminal((uint8_t *)"\r\n", tMessage); */
				/*writePxMutex(PcPort, pcOutputString, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);*/
				osDelay(10);
        break;

      default:
        break;
    }
		/* reset value */
		memset(tMessage, 0, (size_t) MAX_MESSAGE_SIZE);
		memset(cMessage[PORT_BTC_CONN-1], 0, (size_t) MAX_MESSAGE_SIZE);
		messageLength[PORT_BTC_CONN-1] = 0;

    if ((portStatus[PORT_BTC_CONN] != STREAM) &&
        (portStatus[PORT_BTC_CONN] != CLI) &&
        (portStatus[PORT_BTC_CONN] != PORTBUTTON))
    {
      /* Free the port */
      portStatus[PORT_BTC_CONN] = FREE;
      /* Read this port again */
      HAL_UART_Receive_IT(GetUart(PORT_BTC_CONN), (uint8_t *)&cRxedChar, 1);
    }
    taskYIELD();
	}
}

/*-----------------------------------------------------------*/

/* --- H23R0 message processing task
*/
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst)
{
	Module_Status result = H23R0_OK;

	switch (code)
	{
		case CODE_H23R0_GET_INFO:
			break;

		case CODE_H23R0_OTA_MODE:
			btUpdateScript();
			break;

		case CODE_H23R0_RUN_MODE:
			btRunScript();
			break;

		case CODE_H23R0_VSP_COMMAND_MODE:
			btVspMode(H23R0_RUN_VspCommandMode);
			break;

		case CODE_H23R0_VSP_BRIDGE_MODE:
			btVspMode(H23R0_RUN_VspBridgeToUartMode);
			break;

		case CODE_H23R0_LED_STATUS_ON:
			IND_ON();
			break;

		case CODE_H23R0_LED_STATUS_OFF:
			IND_OFF();
      break;

		default:
			result = H23R0_ERR_UnknownMessage;
			break;
	}

	return result;
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands
*/
void RegisterModuleCLICommands(void)
{
	FreeRTOS_CLIRegisterCommand( &btUpdateScriptCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &btRunScriptCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &btVspModeCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &btSetBaudrateCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &btScanCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &btConnectCommandDefinition);
}

/*-----------------------------------------------------------*/

/* --- Enable CTS/RTS flow control
*/
void btEnableHandshakeUart(void)
{
	__HAL_UART_HWCONTROL_CTS_ENABLE(GetUart(PORT_BTC_CONN));
	__HAL_UART_HWCONTROL_RTS_ENABLE(GetUart(PORT_BTC_CONN));
}

/*-----------------------------------------------------------*/

/* --- Disable CTS/RTS flow control
*/
void btDisableHandshakeUart(void)
{
	__HAL_UART_HWCONTROL_CTS_DISABLE(GetUart(PORT_BTC_CONN));
	__HAL_UART_HWCONTROL_RTS_DISABLE(GetUart(PORT_BTC_CONN));
}

/*-----------------------------------------------------------*/

/* --- Setting connection to send a message into Terminal app
*/
void btSendMsgToTerminal(uint8_t *pStr)
{
  #if (H23R0_SHOW_DEBUG_INFO_TERMINAL == H23R0_ENABLE_DEBUG_BTC)
	writePxMutex( H23R0_UART_DEBUG_PORT,
                (char *) pStr,
                strlen((char *) pStr),
                cmd50ms,
                HAL_MAX_DELAY);
  #else
	writePxMutex(PcPort, (char *) pStr, strlen((char *) pStr), cmd50ms, HAL_MAX_DELAY);
  #endif
}

/*-----------------------------------------------------------*/

/* --- Debug function to show message contain in Terminal app
*/
void btShowMsgOnTerminal(uint8_t *pStr1, uint8_t *pStr2)
{
  btSendMsgToTerminal(pStr1);
  btSendMsgToTerminal(pStr2);
}

/*-----------------------------------------------------------*/

/* --- Send command from MCU to Bluetooth module its setting/control
*/
HAL_StatusTypeDef btSendCommandToBtc(uint8_t *command)
{
	HAL_StatusTypeDef result;

	result = writePxMutex(PORT_BTC_CONN, (char *) command, strlen((char *) command), cmd50ms, HAL_MAX_DELAY);
	return result;
}

/*-----------------------------------------------------------*/

/* --- Setting pins to reset bluetooth module
*/
void btResetBt900Module(void)
{
  BT_CLEAR_RST_PIN();
  Delay_ms_no_rtos(10);
  BT_SET_RST_PIN();
  Delay_ms_no_rtos(1600); /* The BT900 module start-up time is ~1.6 seconds */
}

/*-----------------------------------------------------------*/

/* --- Setting pins to update new script on bluetooth module
*/
void btUpdateScript(void)
{
  BT_CLEAR_VSP_PIN();
  BT_CLEAR_MODE_PIN();
  btResetBt900Module();
}

/*-----------------------------------------------------------*/

/* --- Setting pins to run automatically $autorun$ script in bluetooth module
*/
void btRunScript(void)
{
	btDisableHandshakeUart();
	//UpdateBaudrate(PORT_BTC_CONN, 921600); /* Normal baudrate for BT900 */
	BT_CLEAR_VSP_PIN();
	BT_CLEAR_MODE_PIN();
	btResetBt900Module();
}

/*-----------------------------------------------------------*/

/* --- Setting pins to run VSP mode
*/
Module_Status btVspMode(int8_t inputVspMode)
{
	Module_Status result = H23R0_OK;

	btEnableHandshakeUart();
	//UpdateBaudrate(PORT_BTC_CONN, 115200); /* default baudrate of BT900 */
	if (H23R0_RUN_VspCommandMode == inputVspMode)
	{
		BT_CLEAR_VSP_PIN();
		BT_CLEAR_MODE_PIN();
		btResetBt900Module();
	}
	else if (H23R0_RUN_VspBridgeToUartMode == inputVspMode)
	{
		BT_CLEAR_VSP_PIN();
		BT_SET_MODE_PIN();
		btResetBt900Module();
	}
	else
	{
		result = H23R0_ERR_WrongParams;
	}
	return result;
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

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|															Commands																 	|
   -----------------------------------------------------------------------
*/
static portBASE_TYPE btGetInfoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Get information from the BT900 */

	sprintf( ( char * ) pcWriteBuffer, "Get information from BT900 module\r\n");

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE btUpdateScriptCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Do something to update script command on the BT900 */
	btUpdateScript();
	sprintf( ( char * ) pcWriteBuffer, "Updating new bt script via OTA ...\r\n");

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;

}

static portBASE_TYPE btRunScriptCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* The BT900 will be restarted and run with $autorun$ script */
	btRunScript();
	sprintf( ( char * ) pcWriteBuffer, "Run $autorun$ script in the BT900 board");

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;

}

static portBASE_TYPE btVspModeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	Module_Status result = H23R0_OK;

	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;
	static const int8_t *pcMessageOK = ( int8_t * ) "Started %s\r\n";
	static const int8_t *pcMessageWrongParam = ( int8_t * ) "Wrong parameter!\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Obtain the 1st parameter string define VSP mode on BT900 */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	if (!strncmp((const char *)pcParameterString1, "command", 7))
	{
		result = btVspMode(H23R0_RUN_VspCommandMode);
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, "VSP command mode\r\n");
	}
	else if (!strncmp((const char *)pcParameterString1, "bridge", 11))
	{
		result = btVspMode(H23R0_RUN_VspBridgeToUartMode);
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, "VSP Bridge-to-UART mode\r\n");
	}
	else
	{
		result = H23R0_ERR_WrongParams;
	}

	/* Respond to the command */
	if (H23R0_ERR_WrongParams == result)
	{
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongParam );
	}

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE btSetBaudrateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	Module_Status result = H23R0_OK;

	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;
	static const int8_t *pcMessageOK = ( int8_t * ) "Started %s\r\n";
	static const int8_t *pcMessageWrongParam = ( int8_t * ) "Wrong parameter!\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Obtain the 1st parameter string define VSP mode on BT900 */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	if (!strncmp((const char *)pcParameterString1, "921600", 7))
	{
		UpdateBaudrate(PORT_BTC_CONN, 921600); /* Normal baudrate for BT900 */
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, "Baudrate: 921600\r\n");
	}
	else if (!strncmp((const char *)pcParameterString1, "115200", 11))
	{
		UpdateBaudrate(PORT_BTC_CONN, 115200); /* Normal baudrate for BT900 */
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, "Baudrate: 115200\r\n");
	}
	else
	{
		result = H23R0_ERR_WrongParams;
	}

	/* Respond to the command */
	if (H23R0_ERR_WrongParams == result)
	{
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongParam );
	}

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE btScanCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Scan */

	/* sprintf( ( char * ) pcWriteBuffer, "Get information from BT900 module\r\n"); */

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE btConnectCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Connect */

	/* sprintf( ( char * ) pcWriteBuffer, "Get information from BT900 module\r\n"); */

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
