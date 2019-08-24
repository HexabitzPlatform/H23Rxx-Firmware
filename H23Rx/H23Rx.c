/*
    BitzOS (BOS) V0.1.6 - Copyright (C) 2017-2019 Hexabitz
    All rights reserved

    File Name     : H23Rx.c
    Description   : Source code for module H23R0/H23R1.
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

/* Module exported parameters ------------------------------------------------*/
module_param_t modParam[NUM_MODULE_PARAMS] = {{.paramPtr=NULL, .paramFormat=FMT_FLOAT, .paramName=""}};

/* Private variables ---------------------------------------------------------*/

/* Transmission state from BT900 module to MCU
 * 0 - Nothing
 * 1 - Have just received a new message from BT900
 * 2 - Finished transmission
 */
static uint8_t stateTransmitBtToMcu = 0;

/* "scan" command state
 * 0 - did not yet call "scan"
 * 1 - "scan" have been called
 */
static uint8_t stateScanDevices = 0;
static uint8_t dstModule = 0;
static uint8_t listBtcDevices[MAX_SCAN_NUMBER_DEVICES][MAX_SSID_SIZE];
static uint8_t indexBtcDevice = 0;
static uint8_t scriptPort = 0;

EventGroupHandle_t handleUartTerminal = NULL;
TaskHandle_t ControlBluetoothTaskHandle = NULL;

/* Private function prototypes -----------------------------------------------*/
void ControlBluetoothTask(void * argument);
void btEnableHandshakeUart(void);
void btDisableHandshakeUart(void);
void btSendMsgToTerminal(uint8_t *pStr, uint8_t lenStr);
void btWaitEventFinishTransmission(void);
void btSendMsgToModule(uint8_t dst, uint8_t *pStr, uint8_t lenStr);
HAL_StatusTypeDef btSendCommandToBtc(const uint8_t *command);
void btResetBt900Module(void);
Module_Status btDownloadScript(Module_Status method, uint8_t port);
void btRunScript(void);
void btDevelopmentMode(void);
Module_Status btVspMode(Module_Status inputVspMode);

/* Create CLI commands --------------------------------------------------------*/

static portBASE_TYPE btGetInfoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btResetCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btDownloadScriptCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btRunScriptCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btVspModeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btDeleteScriptCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btScanCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE btConnectCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* CLI command structure : bt-info */
const CLI_Command_Definition_t btGetInfoCommandDefinition =
{
	( const int8_t * ) "bt-info", /* The command string to type. */
	( const int8_t * ) "bt-info:\r\n Get BT900 module information\r\n\r\n",
	btGetInfoCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

/* CLI command structure : bt-reset */
const CLI_Command_Definition_t btResetCommandDefinition =
{
	( const int8_t * ) "bt-reset", /* The command string to type. */
	( const int8_t * ) "bt-reset:\r\n Reset BT900 module\r\n\r\n",
	btResetCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

/* CLI command structure : bt-download-script */
const CLI_Command_Definition_t btDownloadScriptCommandDefinition =
{
	( const int8_t * ) "bt-download-script", /* The command string to type. */
	( const int8_t * ) "bt-download-script:\r\n Download new $autorun$ script to BT900 module (1st parameter): ota or uart\r\n\r\n",
	btDownloadScriptCommand, /* The function to run. */
	1 /* No parameters are expected. */
};

/* CLI command structure : bt-run-script */
const CLI_Command_Definition_t btRunScriptCommandDefinition =
{
	( const int8_t * ) "bt-run-script", /* The command string to type. */
	( const int8_t * ) "bt-run-script:\r\n Restart BT900 with $autorun$ script\r\n\r\n",
	btRunScriptCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

/* CLI command structure : bt-vsp-mode */
const CLI_Command_Definition_t btVspModeCommandDefinition =
{
	( const int8_t * ) "bt-vsp-mode", /* The command string to type. */
	( const int8_t * ) "bt-vsp-mode:\r\n Set VSP mode for BT900 module (1st parameter): command or bridge\r\n\r\n",
	btVspModeCommand, /* The function to run. */
	1 /* No parameters are expected. */
};

/* CLI command structure : bt-delete-script */
const CLI_Command_Definition_t btDeleteScriptCommandDefinition =
{
	( const int8_t * ) "bt-delete-script", /* The command string to type. */
	( const int8_t * ) "bt-delete-script:\r\n Delete current smartBASIC script on BT900. This should be called before writing a new script on the module\r\n\r\n",
	btDeleteScriptCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

/* CLI command structure : scan */
const CLI_Command_Definition_t btScanCommandDefinition =
{
	( const int8_t * ) "scan", /* The command string to type. */
	( const int8_t * ) "scan:\r\n Scan nearby BLE devices and display them in a list along with their SSIDs and RSSI levels\r\n\r\n",
	btScanCommand, /* The function to run. */
	0 /* One parameter is expected. */
};

/* CLI command structure : connect */
const CLI_Command_Definition_t btConnectCommandDefinition =
{
	( const int8_t * ) "connect", /* The command string to type. */
	( const int8_t * ) "connect:\r\n Connect to another bluetooth device \r\n\r\n",
	btConnectCommand, /* The function to run. */
	1 /* One parameter is expected. */
};


/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   -----------------------------------------------------------------------
*/

/* --- clear "listBtcDevices" buffer
*/
void cleanListBtcDevices(void)
{
  uint8_t i;

  /* clear */
  for (i = 0; i < MAX_SCAN_NUMBER_DEVICES; i++)
  {
    memset(listBtcDevices[i], 0, MAX_SSID_SIZE);
  }
  indexBtcDevice = 0;
}

/* --- Copy SSID name to "listBtcDevices" buffer
*/
void copyDataToListBtcDevice(uint8_t *pStr, uint8_t lenStr)
{
	memcpy(listBtcDevices[indexBtcDevice], (char *)pStr, (size_t)(lenStr));
  indexBtcDevice++;
}

/* --- Print/Send list of bluetooth devices available
*/
void sendListBtcDevices(uint8_t type, uint8_t dst)
{
  uint8_t i;

  for(i = 0; i < indexBtcDevice; i++)
  {
    if (H23Rx_SEND_TO_TERMINAL_APP == type)
    {
      btSendMsgToTerminal(listBtcDevices[i], strlen((char *)listBtcDevices[i]));
    }
    else if (H23Rx_SEND_TO_OTHER_DEVICES == type)
    {
      btSendMsgToModule(dst, listBtcDevices[i], strlen((char *)listBtcDevices[i]));
    }
    else
    {
      /* nothing to do here */
    }
  }
}

/* --- H23R0 module initialization.
*/
void Module_Init(void)
{
	uint16_t vsp_mode;
	
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

	/* Read VSP mode from EEPROM */
	EE_ReadVariable(_EE_H23xVSP, &vsp_mode);
	
	/* BT800/BT900 EN_RST */
	BT_RST_GPIO_Init();
#ifdef H23R1
	BT_VSP_GPIO_Init();
	BT_HOST_WKUP_GPIO_Init();
	BT_CTS_RTS_GPIO_Init();	
	// YOLO for now. If VSP mode is Command, don't initialize mode pin GPIO
	if (vsp_mode != H23Rx_RUN_VspCommandMode)
		BT_MODE_GPIO_Init();
#endif
  /* clear global data for list */
  cleanListBtcDevices();
  /* setting baudrate */
  UpdateBaudrate(PORT_BTC_CONN, 115200); /* Normal baudrate for BT900 */
  /* clean global variable */
  stateTransmitBtToMcu = 0;
  stateScanDevices = 0;
  dstModule = 0;
  /* create a event group for UART port */
  handleUartTerminal = xEventGroupCreate();
	/* Create the Bluetooth module task */
	xTaskCreate(ControlBluetoothTask, (const char *) "ControlBluetooth", (2*configMINIMAL_STACK_SIZE), NULL, osPriorityNormal-osPriorityIdle, &ControlBluetoothTaskHandle);
	/* By default, the BT900 will run in the "Self-contained Run mode" */
	btRunScript();
	/* btVspMode(H23Rx_RUN_VspBridgeToUartMode); */
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
    /* code field of message - convert text to binary */
    code_field = cMessage[PORT_BTC_CONN-1][4] - 0x30;
    code_field += (cMessage[PORT_BTC_CONN-1][3] - 0x30) * 10;
    code_field += (cMessage[PORT_BTC_CONN-1][2] - 0x30) * 100;
    code_field += (cMessage[PORT_BTC_CONN-1][1] - 0x30) * 1000;
    switch(code_field)
    {
      case CODE_H23Rx_EVBTC_SPPCONN:
        break;

      case CODE_H23Rx_EVBTC_SPPDISCON:
        break;

      case CODE_H23Rx_EVBTC_PAIR_REQUEST:
        break;

      case CODE_H23Rx_EVBTC_PIN_REQUEST:
        break;

      case CODE_H23Rx_EVBTC_PAIR_RESULT:
        break;

      case CODE_H23Rx_EVBTC_AUTHREQ:
        break;

      case CODE_H23Rx_EVBTC_PASSKEY:
        break;

      case CODE_H23Rx_LED_STATUS_ON:
        IND_ON();
        break;

      case CODE_H23Rx_LED_STATUS_OFF:
        IND_OFF();
        break;

      case CODE_H23Rx_SHOW_DEBUG_INFO:
      	stateTransmitBtToMcu = 0;
      	btSendMsgToTerminal(&cMessage[PORT_BTC_CONN-1][5], messageLength[PORT_BTC_CONN-1]-4);
      	break;

      case CODE_H23Rx_SCAN_RESPOND:
        stateScanDevices = 1;
      	stateTransmitBtToMcu = 0;
      	copyDataToListBtcDevice(&cMessage[PORT_BTC_CONN-1][5], messageLength[PORT_BTC_CONN-1]-4);
        break;

      case CODE_H23Rx_SCAN_RESPOND_ERR:
        stateScanDevices = 0;
      	stateTransmitBtToMcu = 0;
      	btSendMsgToTerminal(&cMessage[PORT_BTC_CONN-1][5], messageLength[PORT_BTC_CONN-1]-4);
        break;

      case CODE_H23Rx_CONNECT_RESPOND:
      	stateTransmitBtToMcu = 0;
      	if ('0' == cMessage[PORT_BTC_CONN-1][5])
      	{
					sprintf((char *)&tMessage[0], "Connection succeeded ...\r\n");
      	}
      	else
      	{
					sprintf((char *)&tMessage[0], "Connection failed ...\r\n");
      	}
        if (CLI == portStatus[PcPort])
        {
          btSendMsgToTerminal(tMessage, strlen((char *)tMessage));
          stateTransmitBtToMcu = H23R0_BTC_CLOSE_CONNECTION;
          xEventGroupSetBits(handleUartTerminal, EVENT_CLOSE_CONNECTION_BIT);
        }
        else
        {
          btSendMsgToModule(dstModule, &cMessage[PORT_BTC_CONN-1][5], 1);
        }
        break;

      case CODE_H23Rx_FINISHED_SCAN:
        if (CLI == portStatus[PcPort])
        {
          sendListBtcDevices(H23Rx_SEND_TO_TERMINAL_APP, 0);
          stateTransmitBtToMcu = H23R0_BTC_CLOSE_CONNECTION;
          xEventGroupSetBits(handleUartTerminal, EVENT_CLOSE_CONNECTION_BIT);
        }
        else
        {
          sendListBtcDevices(H23Rx_SEND_TO_OTHER_DEVICES, dstModule);
        }
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
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift)
{
	Module_Status result = H23Rx_OK;
	static const int8_t *pcMessageWrongParam = ( int8_t * ) "Wrong parameter!\r\n";
	static const int8_t *pcMessageMustScan = ( int8_t * ) "Must call \"scan\" command first\r\n";
	uint8_t lenPar = 0;

	switch (code)
	{
		case CODE_H23Rx_GET_INFO:
			break;

		case CODE_H23Rx_DOWNLOAD_SCRIPT_OTA:
			btDownloadScript(H23Rx_RUN_DownloadScriptViaOta, src);
			break;

		case CODE_H23Rx_DOWNLOAD_SCRIPT_UART:
			btDownloadScript(H23Rx_RUN_DownloadScriptViaUart, src);
			break;

		case CODE_H23Rx_RUN_AUTORUN_SCRIPT:
			btRunScript();
			break;

		case CODE_H23Rx_VSP_COMMAND_MODE:
			btVspMode(H23Rx_RUN_VspCommandMode);
			break;

		case CODE_H23Rx_VSP_BRIDGE_MODE:
			btVspMode(H23Rx_RUN_VspBridgeToUartMode);
			break;

		case CODE_H23Rx_LED_STATUS_ON:
			IND_ON();
			break;

		case CODE_H23Rx_LED_STATUS_OFF:
			IND_OFF();
      break;

		case CODE_H23Rx_SCAN_INQUIRE:
      stateScanDevices = 0;
      cleanListBtcDevices();
			/* Send a control message to BT900 to inquire about new bluetooth devices */
			SendMessageFromPort(PORT_BTC_CONN, 0, 0, CODE_H23Rx_SCAN_INQUIRE, 0);
      dstModule = src;
      break;

		case CODE_H23Rx_CONNECT_INQUIRE:
      if (1 == stateScanDevices)
      {
        /* dst - 1 byte | src - 1 byte | code - 2 bytes | crc - 1 byte */
        lenPar = messageLength[port-1] - shift;
        if ( ('[' == cMessage[port-1][1+shift]) && (']' == cMessage[port-1][messageLength[port-1] - 2]) )
        {
          /* Send a control message to BT900 to run inquiry new bluetooth devices */
          memcpy((char *)&messageParams[0], (char *)&cMessage[port-1][1 + shift], lenPar - 2);
          SendMessageFromPort(PORT_BTC_CONN, 0, 0, CODE_H23Rx_CONNECT_INQUIRE, lenPar - 2);
        }
        else
        {
          memcpy((char *)&messageParams[0], (char *)pcMessageWrongParam, strlen((char *)pcMessageWrongParam));
          /* Send response */
          SendMessageToModule(src, CODE_CLI_RESPONSE, strlen((char *)pcMessageWrongParam));
        }
      }
      else
      {
        memcpy((char *)&messageParams[0], (char *)pcMessageMustScan, strlen((char *)pcMessageMustScan));
        /* Send response */
        SendMessageToModule(src, CODE_CLI_RESPONSE, strlen((char *)pcMessageMustScan));
      }
      dstModule = src;
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
	FreeRTOS_CLIRegisterCommand( &btGetInfoCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &btResetCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &btDownloadScriptCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &btRunScriptCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &btVspModeCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &btDeleteScriptCommandDefinition);
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

/* --- DMA stream timer callback that will be called after
 * 		 timeout event in btDownloadScript() function
*/
void btcDmaStreamDownloadScript(TimerHandle_t xTimer)
{
	uint32_t tid = 0;

	/* close DMA stream */
	tid = ( uint32_t ) pvTimerGetTimerID( xTimer );
	if (23 == tid)
	{
		StopStreamDMA(PORT_BTC_CONN);
		StopStreamDMA(scriptPort);
	}

	/* create event to close update script command */
	stateTransmitBtToMcu = H23R0_BTC_CLOSE_CONNECTION;
	xEventGroupSetBits(handleUartTerminal, EVENT_CLOSE_CONNECTION_BIT);
}

/*-----------------------------------------------------------*/

/* --- Wait until script file is transmitted to Bluetooth module with a 30 second timeout
*/
void btWaitEventFinishTransmission(void)
{
	EventBits_t tEvBits;
	do {
		tEvBits = xEventGroupWaitBits(handleUartTerminal, EVENT_CLOSE_CONNECTION_BIT, pdTRUE, pdFALSE, 10000);
		if ((tEvBits & EVENT_CLOSE_CONNECTION_BIT) != EVENT_CLOSE_CONNECTION_BIT)
		{
			continue;
		}
	} while(H23R0_BTC_CLOSE_CONNECTION != stateTransmitBtToMcu);
	stateTransmitBtToMcu = 0;
}

/*-----------------------------------------------------------*/

/* --- Send a message to thet erminal
*/
void btSendMsgToTerminal(uint8_t *pStr, uint8_t lenStr)
{
  int8_t *tOutput;

	/* Obtain output buffer address */
	tOutput = FreeRTOS_CLIGetOutputBuffer();
	memcpy(tOutput, (char *)pStr, (size_t)(lenStr));
  /* print all data in Terminal output buffer */
  writePxMutex(PcPort, (char *)tOutput, strlen((char *)tOutput), cmd50ms, HAL_MAX_DELAY);
  /* clean terminal output */
  memset((char *)tOutput, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
}

/*-----------------------------------------------------------*/

/* --- Send message that have been received from BT900 to the MCU
*/
void btSendMsgToModule(uint8_t dst, uint8_t *pStr, uint8_t lenStr)
{
	if (dst) {
		memcpy(messageParams, (char *)pStr, (size_t)lenStr);
		/* Send response */
		SendMessageToModule(dst, CODE_CLI_RESPONSE, (size_t)lenStr);
	}
}

/*-----------------------------------------------------------*/

/* --- Send command from MCU to the Bluetooth module
*/
HAL_StatusTypeDef btSendCommandToBtc(const uint8_t *command)
{
	HAL_StatusTypeDef result;

	result = writePxMutex(PORT_BTC_CONN, (char *) command, strlen((char *)command), cmd50ms, HAL_MAX_DELAY);
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

/* --- Setting pins to run VSP mode
*/
Module_Status btVspMode(Module_Status inputVspMode)
{
	Module_Status result = H23Rx_OK;

	btEnableHandshakeUart();
	//UpdateBaudrate(PORT_BTC_CONN, 115200); /* default baudrate of BT900 */
	if (H23Rx_RUN_VspCommandMode == inputVspMode)
	{
		BT_CLEAR_VSP_PIN();
		BT_CLEAR_MODE_PIN();
		btResetBt900Module();
	}
	else if (H23Rx_RUN_VspBridgeToUartMode == inputVspMode)
	{
		BT_CLEAR_VSP_PIN();
		BT_SET_MODE_PIN();
		btResetBt900Module();
	}
	else
	{
		result = H23Rx_ERR_WrongParams;
	}
	return result;
}

/*-----------------------------------------------------------*/

/* --- Setting pins to update new script on bluetooth module
*/
Module_Status btDownloadScript(Module_Status method, uint8_t port)
{
	Module_Status result = H23Rx_OK;
	TimerHandle_t xTimer = NULL;

	/*btDisableHandshakeUart();*/
    btEnableHandshakeUart();
	if (H23Rx_RUN_DownloadScriptViaOta == method)
	{
		/* update new smartBASIC script via ota method */
		/*btVspMode(H23Rx_RUN_VspCommandMode);*/
		btVspMode(H23Rx_RUN_VspBridgeToUartMode);
	}
	else if (H23Rx_RUN_DownloadScriptViaUart == method)
	{
		scriptPort = port;
		/* update new smartBASIC script via uart method */
		stateTransmitBtToMcu = 0;
		/* setup pin to control BT900 module */
		/*btVspMode(H23Rx_RUN_VspCommandMode);*/
		btVspMode(H23Rx_RUN_VspBridgeToUartMode);
		/* Save VSP mode to EEPROM */
		EE_WriteVariable(_EE_H23xVSP, H23Rx_RUN_VspBridgeToUartMode);
		
		/* Change baudrate to 115200 to match BT900 UART */
		UpdateBaudrate(PcPort, 115200);

		/* setup DMA stream */
		StartDMAstream(GetUart(PORT_BTC_CONN), GetUart(scriptPort), H23Rx_MAX_NUMBER_OF_DATA_DMA);
		StartDMAstream(GetUart(scriptPort), GetUart(PORT_BTC_CONN), H23Rx_MAX_NUMBER_OF_DATA_DMA);

		/* Create a timeout timer */
		xTimer = xTimerCreate( "StreamTimer", pdMS_TO_TICKS(30000), pdFALSE, ( void * ) 23, btcDmaStreamDownloadScript );
		/* Start the timeout timer */
		xTimerStart( xTimer, portMAX_DELAY );
	}
	else
	{
		result = H23Rx_ERR_WrongParams;
	}
  return result;
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

/* --- Setting pins to put the bluetooth module in interactive / development mode
*/
void btDevelopmentMode(void)
{
	btDisableHandshakeUart();
	//UpdateBaudrate(PORT_BTC_CONN, 921600); /* Normal baudrate for BT900 */
	BT_SET_VSP_PIN();
	BT_SET_MODE_PIN();
	btResetBt900Module();
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

	sprintf( ( char * ) pcWriteBuffer, "Get BT900 module information \r\n");


	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/* -----------------------------------------------------------------------
	|															Commands																 	|
   -----------------------------------------------------------------------
*/
static portBASE_TYPE btResetCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Reset BT900 module */

	sprintf( ( char * ) pcWriteBuffer, "Reset BT900 module\r\n");
	
	btResetBt900Module();

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

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

	/* Obtain the 1st parameter string define VSP mode on BT900 */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	if (!strncmp((const char *)pcParameterString1, "ota", 3))
	{
		sprintf( ( char * ) pcWriteBuffer, "Downloading new smartBASIC program to BT900 via OTA ...\r\n");
		writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
		/* Do something to update script command on the BT900 */
		result = btDownloadScript(H23Rx_RUN_DownloadScriptViaOta, PcPort);
	}
	else if (!strncmp((const char *)pcParameterString1, "uart", 4))
	{
		sprintf( ( char * ) pcWriteBuffer, "Downloading new smartBASIC program to BT900 via UART. Please set baudrate to 115200 then open this port again and load the smartBASIC file\r\n");
		writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
		result = btDownloadScript(H23Rx_RUN_DownloadScriptViaUart, PcPort);
		/* Wait until the baudrate is changed and file is transmitted with 30-sec timeout */
		btWaitEventFinishTransmission();
	}
	else
	{
		result = H23Rx_ERR_WrongParams;
	}

	if (H23Rx_ERR_WrongParams == result)
	{
		sprintf( ( char * ) pcWriteBuffer, "Wrong input parameter\r\n");
		writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
	}

	sprintf( ( char * ) pcWriteBuffer, "\r\nDone\r\n");
	writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
	/* clean terminal output */
	memset((char *)pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
	sprintf((char *)pcWriteBuffer, "\r\n");

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
	Module_Status result = H23Rx_OK;

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
		result = btVspMode(H23Rx_RUN_VspCommandMode);
		/* Save VSP mode to EEPROM */
		EE_WriteVariable(_EE_H23xVSP, H23Rx_RUN_VspCommandMode);
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, "VSP command mode\r\n");
	}
	else if (!strncmp((const char *)pcParameterString1, "bridge", 6))
	{
		result = btVspMode(H23Rx_RUN_VspBridgeToUartMode);
		/* Save VSP mode to EEPROM */
		EE_WriteVariable(_EE_H23xVSP, H23Rx_RUN_VspBridgeToUartMode);
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, "VSP Bridge-to-UART mode\r\n");
	}
	else
	{
		result = H23Rx_ERR_WrongParams;
	}

	/* Respond to the command */
	if (H23Rx_ERR_WrongParams == result)
	{
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongParam );
	}

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE btDeleteScriptCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	Module_Status result = H23Rx_OK;

	static const uint8_t *pcMsgDelFirmware = ( uint8_t * ) "at&f*\r\n";
	static const int8_t *pcMessageWrongParam = ( int8_t * ) "Failed to delete current smartBASIC script\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	sprintf( ( char * ) pcWriteBuffer, "Current smartBASIC script deleted successfuly\r\n");

	/* Save VSP mode to EEPROM */
	EE_WriteVariable(_EE_H23xVSP, H23Rx_RUN_VspCommandMode);
	/* waiting BT900 reset */
	Delay_ms(100);

	/* Respond to the command */
	if (H23Rx_ERR_WrongParams == result)
	{
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongParam );
	}
	else
	{
		btSendCommandToBtc(pcMsgDelFirmware);
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
  stateScanDevices = 0;
  cleanListBtcDevices();
	sprintf( (char *)pcWriteBuffer, "Scanning nearby bluetooth devices..\r\n\nIndex\tRSSI\tName\r\n\r\n");
  writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
  /* clean terminal output */
  memset((char *)pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
	/* Send a control message to BT900 to run inquiry new bluetooth devices */
	SendMessageFromPort(PORT_BTC_CONN, 0, 0, CODE_H23Rx_SCAN_INQUIRE, 0);
  /* waiting event finish transmission */
	btWaitEventFinishTransmission();
  /* clean terminal output */
  memset((char *)pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
	sprintf( ( char * ) pcWriteBuffer, "\r\n");

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
		sprintf((char *)pcWriteBuffer, "Connecting to bluetooth device %s:\r\n", pcParameterString1);
		writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
	  /* clean terminal output */
	  memset((char *)pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

		lenPar = strlen((char *)pcParameterString1);
		if ( ('[' == pcParameterString1[0]) && (']' == pcParameterString1[lenPar - 1]) )
		{
			/* Send a control message to BT900 to run inquiry new bluetooth devices */
			memcpy(&messageParams[0], &pcParameterString1[1], lenPar - 2);
			SendMessageFromPort(PORT_BTC_CONN, 0, 0, CODE_H23Rx_CONNECT_INQUIRE, lenPar - 2);
			/* waiting event finish transmission */
			btWaitEventFinishTransmission();
      /* clean terminal output */
      memset((char *)pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
			sprintf((char *)pcWriteBuffer, "\r\n");
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
/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
