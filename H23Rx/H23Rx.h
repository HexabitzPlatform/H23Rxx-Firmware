/*
    BitzOS (BOS) V0.2.4 - Copyright (C) 2017-2021 Hexabitz
    All rights reserved

    File Name     : H23Rx.h
    Description   : Header file for module H23R0/H23R1.
					Bluetooth module (BT800/BT900)
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H23Rx_H
#define H23Rx_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H23Rx_MemoryMap.h"
#include "H23Rx_uart.h"
#include "H23Rx_gpio.h"
#include "H23Rx_dma.h"


/* H23R0_Status Type Definition */
typedef enum
{
  H23Rx_OK = 0,
	H23Rx_ERR_UnknownMessage = 1,
	H23Rx_ERR_WrongParams,
	H23Rx_RUN_VspCommandMode,
	H23Rx_RUN_VspBridgeToUartMode,
	H23Rx_RUN_DownloadScriptViaOta,
	H23Rx_RUN_DownloadScriptViaUart,
	H23Rx_ERROR = 255
} Module_Status;



//BT900 constants:

#define BT_Max_Number_Of_Devices 10
#define BT_Max_Device_Name_Length 30
#define BT_Device_Address_Length 12

#define BT_Command_Buffer_Length 500


/* BT900 variables and Data Structures: -------------------------------------------*/

extern uint8_t BT_User_Buffer_Length;


extern uint8_t BT_Devices_Name[BT_Max_Number_Of_Devices][BT_Max_Device_Name_Length];
extern uint8_t BT_Devices_Address[BT_Max_Number_Of_Devices][BT_Device_Address_Length];
extern uint8_t BT_Devices_Index;
extern uint8_t BT_Rx;
extern uint8_t BT_User_Buffer[192];
extern uint8_t BT_User_Buffer_Index;
extern uint8_t BT_Commands_Buffer[BT_Command_Buffer_Length];
extern uint8_t BT_Commands_Buffer_Index;
extern uint8_t BT_BOS_Index;

extern uint8_t* BT_User_Buffer_ptr;
extern uint8_t* BT_User_Buffer_beginning_ptr;
extern uint8_t* BT_User_Buffer_Index_ptr;


extern uint8_t BT_To_User_Buffer_flag;
//1: Bluetooth To User_Buffer
//0: Bluetooth To BOS Messaging Buffer

extern uint8_t BT_Connection_flag;
//1: Connected
//0: Disconnected

extern uint8_t BT_SPP_Mode;
//0: SPP_Command_Mode
//1: SPP_Bridge_Mode

extern uint8_t BT_delete_connecting_char_flag;
extern uint8_t BT_delete_disconnecting_char_flag;

extern uint8_t BT_boot; //flag for sending name to BT900 Module on startup

/*-------------------------------------------------------------------------------------*/


/* BT900 Private Function Prototypes-------------------------------------------*/


extern Module_Status BT_RUN_SCRIPT_MODE(void);
extern Module_Status BT_Switch_To_SPP_Command_Mode(void);
extern Module_Status BT_Switch_To_SPP_Bridge_Mode(void);


/*-------------------------------------------------------------------*/

/* Exported definitions -------------------------------------------------------*/

#ifdef H23R0
	#define	modulePN		_H23R0
#endif
#ifdef H23R1
	#define	modulePN		_H23R1
#endif

/* Port-related definitions */
#define	NumOfPorts		6
#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5
#define _P6

/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
#define _Usart4 1
#define _Usart5 1
#define _Usart6 1

/* Port-UART mapping */
#define P1uart &huart4
#define P2uart &huart2
#define P3uart &huart6
#define P4uart &huart1
#define P5uart &huart5
#define P6uart &huart3

/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT	GPIOA
#define	USART1_RX_PORT	GPIOA
#define	USART1_AF				GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT	GPIOA
#define	USART2_RX_PORT	GPIOA
#define	USART2_AF				GPIO_AF1_USART2

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT	GPIOA
#define	USART4_RX_PORT	GPIOA
#define	USART4_AF				GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_4
#define	USART5_TX_PORT	GPIOB
#define	USART5_RX_PORT	GPIOB
#define	USART5_AF				GPIO_AF4_USART5

#define	USART6_TX_PIN		GPIO_PIN_4
#define	USART6_RX_PIN		GPIO_PIN_5
#define	USART6_TX_PORT	GPIOA
#define	USART6_RX_PORT	GPIOA
#define	USART6_AF				GPIO_AF5_USART6

/* Module-specific Definitions */
#define	USART3_TX_PIN			GPIO_PIN_10
#define	USART3_RX_PIN			GPIO_PIN_11
#define	USART3_RTS_PIN			GPIO_PIN_1
#define	USART3_CTS_PIN			GPIO_PIN_6
#define	USART3_TX_PORT			GPIOB
#define	USART3_RX_PORT			GPIOB
#define	USART3_RTS_PORT			GPIOB
#define	USART3_CTS_PORT			GPIOA
#define	USART3_AF				GPIO_AF4_USART3
#ifdef H23R0
	#define	_BT_RST_PIN			GPIO_PIN_15
	#define	_BT_RST_PORT		GPIOB
  /* to be defined in future */
	#define	_BT_VSP_PIN			GPIO_PIN_14
	#define	_BT_VSP_PORT		GPIOB
	#define	_BT_MODE_PIN		GPIO_PIN_7
	#define	_BT_MODE_PORT		GPIOA
	#define	_BT_HOST_WKUP_PIN	GPIO_PIN_12
	#define	_BT_HOST_WKUP_PORT	GPIOB
#endif
#ifdef H23R1
	#define	_BT_RST_PIN							GPIO_PIN_2
	#define	_BT_RST_PORT						GPIOB
	#define	_BT_VSP_PIN							GPIO_PIN_14
	#define	_BT_VSP_PORT						GPIOB
	#define	_BT_MODE_PIN						GPIO_PIN_7
	#define	_BT_MODE_PORT						GPIOA
	#define	_BT_HOST_WKUP_PIN				    GPIO_PIN_12
	#define	_BT_HOST_WKUP_PORT			        GPIOB
#endif

#define NUM_MODULE_PARAMS		1

/* Module EEPROM Variables */

// Module Addressing Space 500 - 599

#define _EE_H23x_Name_Length_Address			500		//H23Rx Bluetooth module Name Length
#define _EE_H23x_Name_Beginning_Address			501
/*Max name length: 30 character
 * first name character Address: 501
 * last name character Address: 530
 */






/* Indicator LED */
#define _IND_LED_PORT										GPIOA
#define _IND_LED_PIN										GPIO_PIN_11

/* Port-related Bluetooth - BT900 */
#define PORT_BTC_CONN										P6

/* Macros define to enable/disable debug information will be shown
 * on Terminal Application for Bluetooth
 */
#define H23Rx_ENABLE_DEBUG_BTC          1
#define H23Rx_DISABLE_DEBUG_BTC         0

#define H23Rx_UART_DEBUG_PORT           P2
#define H23Rx_SHOW_DEBUG_INFO_TERMINAL  0

/* Macros define bit event group to trigger terminal uart port */
#define EVENT_CLOSE_CONNECTION_BIT      ( 1 << 0 ) /* close connection event */
#define H23R0_BTC_CLOSE_CONNECTION      0xFF

/* Macros define maximum number of data to transfer (0 up to 65535)*/
#define H23Rx_MAX_NUMBER_OF_DATA_DMA    40000

/* define list of bluetooth available */
#define MAX_SSID_SIZE                     20
#define MAX_SCAN_NUMBER_DEVICES           10

#define H23Rx_SEND_TO_TERMINAL_APP        0x00
#define H23Rx_SEND_TO_OTHER_DEVICES       0x01

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);

/* handler for control bluetooth module task */
extern TaskHandle_t ControlBluetoothTaskHandle;


/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   -----------------------------------------------------------------------
*/

#if defined(H23R1) || defined(H23R0)
//	#define BT_SET_RST_PIN()				HAL_GPIO_WritePin(_BT_RST_PORT,_BT_RST_PIN,GPIO_PIN_SET)
//	#define BT_CLEAR_RST_PIN()			    HAL_GPIO_WritePin(_BT_RST_PORT,_BT_RST_PIN,GPIO_PIN_RESET)
//
//	#define BT_SET_VSP_PIN()				HAL_GPIO_WritePin(_BT_VSP_PORT,_BT_VSP_PIN,GPIO_PIN_SET)
//	#define BT_CLEAR_VSP_PIN()			    HAL_GPIO_WritePin(_BT_VSP_PORT,_BT_VSP_PIN,GPIO_PIN_RESET)

	#define BT_SET_MODE_PIN()				HAL_GPIO_WritePin(_BT_MODE_PORT,_BT_MODE_PIN,GPIO_PIN_SET)
	#define BT_CLEAR_MODE_PIN()			    HAL_GPIO_WritePin(_BT_MODE_PORT,_BT_MODE_PIN,GPIO_PIN_RESET)
#endif

extern void BT_RESET_MODULE(void);
extern void BT_Receive_Data_To_BOS(void);
extern Module_Status BT_Receive_Data(uint8_t* buffer,uint8_t size);
extern void BT_Disconnect(void);
extern void BT_Delete_Script(void);
extern void BT_Download_Script(void);
extern Module_Status BT_Scan(void);
extern Module_Status BT_Connect(uint8_t BT_Device);
extern Module_Status BT_Clear_User_Buffer(void);
extern Module_Status BT_Send_Data(uint8_t* BT_Data, uint8_t length);
extern Module_Status BT_Send_Message(uint8_t dst,uint16_t code,uint16_t numberOfParams);
/*BT_Send_Message() function is the same as SendMessageToModule(),
but it insures that BT900 is connected to Bluetooth device before sending any Message*/
extern Module_Status BT_Set_Discoverable(void);
extern Module_Status BT_Set_Name(uint8_t* name, uint8_t length);
extern Module_Status BT_Stream_To_Port(uint8_t port_number);



/* -----------------------------------------------------------------------
	|															Commands																 	|
   -----------------------------------------------------------------------
*/

extern const CLI_Command_Definition_t btClearUserBufferCommandDefinition;
extern const CLI_Command_Definition_t btSendDataCommandDefinition;
extern const CLI_Command_Definition_t btDisconnectCommandDefinition;
extern const CLI_Command_Definition_t btDeleteScriptCommandDefinition;
extern const CLI_Command_Definition_t btDownloadScriptCommandDefinition;
extern const CLI_Command_Definition_t btConnectCommandDefinition;
extern const CLI_Command_Definition_t btScanCommandDefinition;
extern const CLI_Command_Definition_t btSetNameCommandDefinition;
extern const CLI_Command_Definition_t btSetDiscoverableCommandDefinition;
extern const CLI_Command_Definition_t btStreamToPortCommandDefinition;




#endif /* H23Rx_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
