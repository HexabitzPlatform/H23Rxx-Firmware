/*
 BitzOS (BOS) V0.2.5 - Copyright (C) 2017-2021 Hexabitz
 All rights reserved

 File Name     : H23Rx_gpio.h
 Description   : Header file contains all the functions prototypes for
 the GPIO .

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __H23Rx_gpio_H
#define __H23Rx_gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"


extern void GPIO_Init(void);
extern void IND_LED_Init(void);
	 
extern void BT_RST_GPIO_Init(void);
#ifdef H23R1	 
extern void BT_VSP_GPIO_Init(void);
extern void BT_MODE_GPIO_Init(void);
extern void BT_HOST_WKUP_GPIO_Init(void);
extern void BT_CTS_RTS_GPIO_Init(void);
#endif


#ifdef __cplusplus
}
#endif
#endif /*__H23Rx_gpio_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
