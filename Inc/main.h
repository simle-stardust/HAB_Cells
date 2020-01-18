/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Button_Pin GPIO_PIN_13
#define Button_GPIO_Port GPIOC
#define LTC1_RST_Pin GPIO_PIN_14
#define LTC1_RST_GPIO_Port GPIOC
#define LTC1_CS_Pin GPIO_PIN_15
#define LTC1_CS_GPIO_Port GPIOC
#define ADC_Thermal_Int_Pin GPIO_PIN_0
#define ADC_Thermal_Int_GPIO_Port GPIOC
#define ADC_Ele_Pin GPIO_PIN_1
#define ADC_Ele_GPIO_Port GPIOC
#define ADC_Thermal_Ext_Pin GPIO_PIN_2
#define ADC_Thermal_Ext_GPIO_Port GPIOC
#define LTC2_CS_Pin GPIO_PIN_3
#define LTC2_CS_GPIO_Port GPIOC
#define LowerHeater1_Pin GPIO_PIN_0
#define LowerHeater1_GPIO_Port GPIOA
#define UpperHeater1_Pin GPIO_PIN_1
#define UpperHeater1_GPIO_Port GPIOA
#define LowerHeater2_Pin GPIO_PIN_2
#define LowerHeater2_GPIO_Port GPIOA
#define UpperHeater2_Pin GPIO_PIN_3
#define UpperHeater2_GPIO_Port GPIOA
#define LTC2_RST_Pin GPIO_PIN_4
#define LTC2_RST_GPIO_Port GPIOA
#define LTC3_RST_Pin GPIO_PIN_4
#define LTC3_RST_GPIO_Port GPIOC
#define LTC3_CS_Pin GPIO_PIN_5
#define LTC3_CS_GPIO_Port GPIOC
#define LowerHeater3_Pin GPIO_PIN_0
#define LowerHeater3_GPIO_Port GPIOB
#define UpperHeater3_Pin GPIO_PIN_1
#define UpperHeater3_GPIO_Port GPIOB
#define LTC4_CS_Pin GPIO_PIN_2
#define LTC4_CS_GPIO_Port GPIOB
#define LTC4_RST_Pin GPIO_PIN_10
#define LTC4_RST_GPIO_Port GPIOB
#define LowerHeater4_Pin GPIO_PIN_11
#define LowerHeater4_GPIO_Port GPIOB
#define UpperHeater4_Pin GPIO_PIN_12
#define UpperHeater4_GPIO_Port GPIOB
#define LTC5_CS_Pin GPIO_PIN_13
#define LTC5_CS_GPIO_Port GPIOB
#define LowerHeater5_Pin GPIO_PIN_14
#define LowerHeater5_GPIO_Port GPIOB
#define UpperHeater5_Pin GPIO_PIN_15
#define UpperHeater5_GPIO_Port GPIOB
#define LowerHeater6_Pin GPIO_PIN_6
#define LowerHeater6_GPIO_Port GPIOC
#define UpperHeater6_Pin GPIO_PIN_7
#define UpperHeater6_GPIO_Port GPIOC
#define LTC5_RST_Pin GPIO_PIN_9
#define LTC5_RST_GPIO_Port GPIOC
#define LTC6_CS_Pin GPIO_PIN_8
#define LTC6_CS_GPIO_Port GPIOA
#define LTC6_RST_Pin GPIO_PIN_9
#define LTC6_RST_GPIO_Port GPIOA
#define WiFi_GPIO0_Pin GPIO_PIN_3
#define WiFi_GPIO0_GPIO_Port GPIOB
#define WiFI_CH_PD_Pin GPIO_PIN_4
#define WiFI_CH_PD_GPIO_Port GPIOB
#define WiFi_RST_Pin GPIO_PIN_5
#define WiFi_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
