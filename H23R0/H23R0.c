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
		| Self-contained Run mode         |           0        |          1          |
		| Interactive / development mode  |           1        |          1          |
		| VSP Bridge-to-UART mode         |           1        |          0          |
		| VSP Command mode                |           0        |          0          |
		| Over the Air (OTA)              |           0        |          0          |

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

/* Show state of transmission from BT900 module to MCU
 * 0 - Nothing
 * 1 - Have just received a new message from BT900
 * 2 - Finished transmission
 */
uint8_t stateTransmitBtToMcu = 0;

/* state of call scan instruction
 * 0 - did not yet call "scan"
 * 1 - "scan" have been called
 */
uint8_t stateScanDevices = 0;

uint16_t lenStrTerminal = 0;
uint8_t dstModule = 0;

EventGroupHandle_t handleUartTerminal = NULL;

/* Private variables ---------------------------------------------------------*/

TaskHandle_t ControlBluetoothTaskHandle = NULL;

/* Private function prototypes -----------------------------------------------*/
void ControlBluetoothTask(void * argument);
void btEnableHandshakeUart(void);
void btDisableHandshakeUart(void);
void btSendMsgToTerminal(uint8_t *pStr, uint8_t lenStr);
void btWaitEventFinishTransmission(void);
void btSendMsgToModule(uint8_t dst, uint16_t lenStr);
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
  /* clean global variable */
  stateTransmitBtToMcu = 0;
  stateScanDevices = 0;
  lenStrTerminal = 0;
  dstModule = 0;
  /* create a event group for UART port */
  handleUartTerminal = xEventGroupCreate();
	/* Create the Bluetooth module task */
	xTaskCreate(ControlBluetoothTask, (const char *) "ControlBluetooth", (2*configMINIMAL_STACK_SIZE), NULL, osPriorityNormal, &ControlBluetoothTaskHandle);
	/* In default, the BT900 will run in the "Self-contained Run mode" */
	btRunScript();
	/* btVspMode(H23R0_RUN_VspBridgeToUartMode); */
}

/*-----------------------------------------------------------*/

/* ControlBluetoothTask function
*/
void ControlBluetoothTask(void * argument)
{
	static uint16_t code_field = 0;
  uint8_t tMessage[MAX_MESSAGE_SIZE] = {0};


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

      case CODE_H23R0_SCAN_RESPOND:
      	stateTransmitBtToMcu = 0;
      	btSendMsgToTerminal(&cMessage[PORT_BTC_CONN-1][5], messageLength[PORT_BTC_CONN-1]-4);
        break;

      case CODE_H23R0_CONNECT_RESPOND:
      	stateTransmitBtToMcu = 0;
      	if ('0' == cMessage[PORT_BTC_CONN-1][5])
      	{
					sprintf((char *)&tMessage[0], "Connected success ...\r\n");
      	}
      	else
      	{
					sprintf((char *)&tMessage[0], "Connected failure ...\r\n");
      	}
      	btSendMsgToTerminal(tMessage, strlen((char *)tMessage));
        break;

      case CODE_H23R0_FINISHED_TRANS:
        if (PC != PcPort)
        {
          stateTransmitBtToMcu = H23R0_BTC_CLOSE_CONNECTION;
          xEventGroupSetBits(handleUartTerminal, EVENT_CLOSE_CONNECTION_BIT);
        }
        else
        {
          btSendMsgToModule(dstModule, lenStrTerminal);
        }
        lenStrTerminal = 0;
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
	static const int8_t *pcMessageWrongParam = ( int8_t * ) "Wrong parameter!\r\n";
	static const int8_t *pcMessageMustScan = ( int8_t * ) "Must call \"scan\" command first\r\n";
	uint8_t lenPar = 0;

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

		case CODE_H23R0_SCAN_REQUIRE:
			/* Send a control message to BT900 to run inquiry new bluetooth devices */
			SendMessageFromPort(PORT_BTC_CONN, 0, 0, CODE_H23R0_SCAN_REQUIRE, 0);
      dstModule = src;
			stateScanDevices = 1;
      break;

		case CODE_H23R0_CONNECT_REQUIRE:
      if (1 == stateScanDevices)
      {
        /* dst - 1 byte | src - 1 byte | code - 2 bytes | crc - 1 byte */
        lenPar = messageLength[port-1] - 5;
        if ( ('[' == cMessage[port-1][5]) && (']' == cMessage[port-1][messageLength[port-1] - 2]) )
        {
          /* Send a control message to BT900 to run inquiry new bluetooth devices */
          memcpy((char *)&messageParams[0], (char *)&cMessage[port-1][5], lenPar - 2);
          SendMessageFromPort(PORT_BTC_CONN, 0, 0, CODE_H23R0_CONNECT_REQUIRE, lenPar - 2);
        }
        else
        {
          memcpy((char *)&messageParams[0], (char *)pcMessageWrongParam, strlen((char *)pcMessageWrongParam));
          /* Send response */
          SendMessageToModule(src, CODE_CLI_response, strlen((char *)pcMessageWrongParam));
        }
      }
      else
      {
        memcpy((char *)&messageParams[0], (char *)pcMessageMustScan, strlen((char *)pcMessageMustScan));
        /* Send response */
        SendMessageToModule(src, CODE_CLI_response, strlen((char *)pcMessageMustScan));
      }
      dstModule = src;
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
	FreeRTOS_CLIRegisterCommand( &btGetInfoCommandDefinition);
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
void btSendMsgToTerminal(uint8_t *pStr, uint8_t lenStr)
{
  int8_t *tOutput;

	/* Obtain the address of the output buffer */
	tOutput = FreeRTOS_CLIGetOutputBuffer();
	memcpy(&tOutput[0]+lenStrTerminal, (char *)pStr, (size_t)(lenStr));
	lenStrTerminal = lenStrTerminal + lenStr;
}

/*-----------------------------------------------------------*/

/* --- Wait event finish transmission that is sent from bluetooth module
*/
void btWaitEventFinishTransmission(void)
{
	EventBits_t tEvBits;
	do {
		tEvBits = xEventGroupWaitBits(handleUartTerminal, EVENT_CLOSE_CONNECTION_BIT, pdTRUE, pdFALSE, cmd50ms);
		if ((tEvBits & EVENT_CLOSE_CONNECTION_BIT) != EVENT_CLOSE_CONNECTION_BIT)
		{
			continue;
		}
	} while(H23R0_BTC_CLOSE_CONNECTION != stateTransmitBtToMcu);
	stateTransmitBtToMcu = 0;
}

/*-----------------------------------------------------------*/

/* --- Send message that have been received from BT900 to other MCU
*/
void btSendMsgToModule(uint8_t dst, uint16_t lenStr)
{
  int8_t *tOutputCli;
  uint16_t tCodeField = CODE_CLI_response;
  uint16_t tLenStr = 0;

	/* Obtain the address of the output buffer */
	tOutputCli = FreeRTOS_CLIGetOutputBuffer();
  do {
    if (lenStr > (tLenStr + MAX_MESSAGE_SIZE - 5))
    {
      memcpy(messageParams, tOutputCli+tLenStr, (size_t)(MAX_MESSAGE_SIZE - 5));
      tLenStr = tLenStr + MAX_MESSAGE_SIZE - 5;
      tCodeField = CODE_CLI_response | (1 << 15); /* long message */
			/* Send response */
			SendMessageToModule(dst, tCodeField, (size_t)(MAX_MESSAGE_SIZE - 5));
    }
    else
    {
      memcpy(messageParams, tOutputCli+tLenStr, (size_t)(lenStr - tLenStr));
      tLenStr = lenStr;
      tCodeField = CODE_CLI_response; /* long message */
			/* Send response */
			SendMessageToModule(dst, tCodeField, (size_t)(lenStr - tLenStr));
    }
  } while (tLenStr < lenStr);
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
  Delay_ms_no_rtos(2900); /* The BT900 module start-up time is ~1.6 seconds + ~1.3 s for radio initialisation */
}

/*-----------------------------------------------------------*/

/* --- Setting pins to update new script on bluetooth module
*/
void btUpdateScript(void)
{
	btDisableHandshakeUart();
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
	BT_SET_VSP_PIN();
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
	sprintf( (char *)pcWriteBuffer, "List scanning bluetooth devices:\r\nIndex\tRSSI\tName devices\r\n\r\n");
  writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
  /* clean terminal output */
  memset((char *)pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
	/* Send a control message to BT900 to run inquiry new bluetooth devices */
	SendMessageFromPort(PORT_BTC_CONN, 0, 0, CODE_H23R0_SCAN_REQUIRE, 0);
  /* waiting event finish transmission */
	btWaitEventFinishTransmission();
	/* print all datas in output buffer of Terminal */
	writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
  /* clean terminal output */
  memset((char *)pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
	sprintf( ( char * ) pcWriteBuffer, "\r\n");
	stateScanDevices = 1;

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE btConnectCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;
	uint8_t lenPar = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if (1 == stateScanDevices)
	{
		/* Obtain the 1st parameter string define VSP mode on BT900 */
		pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
		sprintf((char *)pcWriteBuffer, "Connecting bluetooth device %s:\r\n", pcParameterString1);
		writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
	  /* clean terminal output */
	  memset((char *)pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

		lenPar = strlen((char *)pcParameterString1);
		if ( ('[' == pcParameterString1[0]) && (']' == pcParameterString1[lenPar - 1]) )
		{
			/* Send a control message to BT900 to run inquiry new bluetooth devices */
			memcpy(&messageParams[0], &pcParameterString1[1], lenPar - 2);
			SendMessageFromPort(PORT_BTC_CONN, 0, 0, CODE_H23R0_CONNECT_REQUIRE, lenPar - 2);
			/* waiting event finish transmission */
			btWaitEventFinishTransmission();
			/* print all datas in output buffer of Terminal */
			writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
      /* clean terminal output */
      memset((char *)pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
			sprintf((char *)pcWriteBuffer, "\r\n");
		}
		else
		{
			sprintf((char *)pcWriteBuffer, "Wrong value of input parameter\r\n");
		}
	}
	else
	{
		sprintf((char *)pcWriteBuffer, "Must call \"scan\" command first\r\n");
	}

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
