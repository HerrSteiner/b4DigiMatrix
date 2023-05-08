/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Eg1FilterVolPE2_Pin GPIO_PIN_2
#define Eg1FilterVolPE2_GPIO_Port GPIOE
#define Eg1ProcPE3_Pin GPIO_PIN_3
#define Eg1ProcPE3_GPIO_Port GPIOE
#define Eg2Osc1FMPE4_Pin GPIO_PIN_4
#define Eg2Osc1FMPE4_GPIO_Port GPIOE
#define Eg2Osc1WavPE5_Pin GPIO_PIN_5
#define Eg2Osc1WavPE5_GPIO_Port GPIOE
#define Eg2Osc1VolPE6_Pin GPIO_PIN_6
#define Eg2Osc1VolPE6_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define Eg2ProcPF0_Pin GPIO_PIN_0
#define Eg2ProcPF0_GPIO_Port GPIOF
#define AnalogInOsc1FMPF1_Pin GPIO_PIN_1
#define AnalogInOsc1FMPF1_GPIO_Port GPIOF
#define AnalogInOsc2FMPF2_Pin GPIO_PIN_2
#define AnalogInOsc2FMPF2_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define Osc2TablePB2_Pin GPIO_PIN_2
#define Osc2TablePB2_GPIO_Port GPIOB
#define AnalogInOsc3FMPF11_Pin GPIO_PIN_11
#define AnalogInOsc3FMPF11_GPIO_Port GPIOF
#define AnalogInCutPF12_Pin GPIO_PIN_12
#define AnalogInCutPF12_GPIO_Port GPIOF
#define Osc1FMOsc1VolPF13_Pin GPIO_PIN_13
#define Osc1FMOsc1VolPF13_GPIO_Port GPIOF
#define Osc1FMOsc2FMPF14_Pin GPIO_PIN_14
#define Osc1FMOsc2FMPF14_GPIO_Port GPIOF
#define Osc1FMOsc2WavPF15_Pin GPIO_PIN_15
#define Osc1FMOsc2WavPF15_GPIO_Port GPIOF
#define Osc1FMOsc2VolPG0_Pin GPIO_PIN_0
#define Osc1FMOsc2VolPG0_GPIO_Port GPIOG
#define Osc1FMOsc3FMPG1_Pin GPIO_PIN_1
#define Osc1FMOsc3FMPG1_GPIO_Port GPIOG
#define Eg2Osc2FMPE7_Pin GPIO_PIN_7
#define Eg2Osc2FMPE7_GPIO_Port GPIOE
#define Eg2Osc2WavPE8_Pin GPIO_PIN_8
#define Eg2Osc2WavPE8_GPIO_Port GPIOE
#define Eg2Osc2VolPE9_Pin GPIO_PIN_9
#define Eg2Osc2VolPE9_GPIO_Port GPIOE
#define Eg2Osc3FMPE10_Pin GPIO_PIN_10
#define Eg2Osc3FMPE10_GPIO_Port GPIOE
#define Eg2Osc3WavPE11_Pin GPIO_PIN_11
#define Eg2Osc3WavPE11_GPIO_Port GPIOE
#define Eg2Osc3VolPE12_Pin GPIO_PIN_12
#define Eg2Osc3VolPE12_GPIO_Port GPIOE
#define Eg2CutPE13_Pin GPIO_PIN_13
#define Eg2CutPE13_GPIO_Port GPIOE
#define Eg2MorphPE14_Pin GPIO_PIN_14
#define Eg2MorphPE14_GPIO_Port GPIOE
#define Eg2FilterVolPE15_Pin GPIO_PIN_15
#define Eg2FilterVolPE15_GPIO_Port GPIOE
#define EG2TriggerPB10_Pin GPIO_PIN_10
#define EG2TriggerPB10_GPIO_Port GPIOB
#define Osc1VolOut1PB11_Pin GPIO_PIN_11
#define Osc1VolOut1PB11_GPIO_Port GPIOB
#define Osc1VolOut2PB12_Pin GPIO_PIN_12
#define Osc1VolOut2PB12_GPIO_Port GPIOB
#define Osc1FMOsc1FMPB13_Pin GPIO_PIN_13
#define Osc1FMOsc1FMPB13_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define Osc1FMOsc1WavPB15_Pin GPIO_PIN_15
#define Osc1FMOsc1WavPB15_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define Eg1Osc2FMPD10_Pin GPIO_PIN_10
#define Eg1Osc2FMPD10_GPIO_Port GPIOD
#define Eg1Osc2WavPD11_Pin GPIO_PIN_11
#define Eg1Osc2WavPD11_GPIO_Port GPIOD
#define Eg1Osc2VolPD12_Pin GPIO_PIN_12
#define Eg1Osc2VolPD12_GPIO_Port GPIOD
#define Eg1Osc3FMPD13_Pin GPIO_PIN_13
#define Eg1Osc3FMPD13_GPIO_Port GPIOD
#define Eg1Osc3WavPD14_Pin GPIO_PIN_14
#define Eg1Osc3WavPD14_GPIO_Port GPIOD
#define Eg1Osc3VolPD15_Pin GPIO_PIN_15
#define Eg1Osc3VolPD15_GPIO_Port GPIOD
#define Osc1FMOsc3WavPG2_Pin GPIO_PIN_2
#define Osc1FMOsc3WavPG2_GPIO_Port GPIOG
#define Osc1FMOsc3VolPG3_Pin GPIO_PIN_3
#define Osc1FMOsc3VolPG3_GPIO_Port GPIOG
#define Osc1FMCutPG4_Pin GPIO_PIN_4
#define Osc1FMCutPG4_GPIO_Port GPIOG
#define Osc1FMMorphPG5_Pin GPIO_PIN_5
#define Osc1FMMorphPG5_GPIO_Port GPIOG
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define Osc1FMProcPG8_Pin GPIO_PIN_8
#define Osc1FMProcPG8_GPIO_Port GPIOG
#define Osc1VolFilterPC6_Pin GPIO_PIN_6
#define Osc1VolFilterPC6_GPIO_Port GPIOC
#define Osc2VolOut1PC7_Pin GPIO_PIN_7
#define Osc2VolOut1PC7_GPIO_Port GPIOC
#define Osc2VolOut2PC8_Pin GPIO_PIN_8
#define Osc2VolOut2PC8_GPIO_Port GPIOC
#define Osc2VolFilterPC9_Pin GPIO_PIN_9
#define Osc2VolFilterPC9_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Osc1TablePA15_Pin GPIO_PIN_15
#define Osc1TablePA15_GPIO_Port GPIOA
#define Osc3VolOut1PC10_Pin GPIO_PIN_10
#define Osc3VolOut1PC10_GPIO_Port GPIOC
#define Osc3VolOut2PC11_Pin GPIO_PIN_11
#define Osc3VolOut2PC11_GPIO_Port GPIOC
#define Osc3VolFilterPC12_Pin GPIO_PIN_12
#define Osc3VolFilterPC12_GPIO_Port GPIOC
#define FilterOut1PD0_Pin GPIO_PIN_0
#define FilterOut1PD0_GPIO_Port GPIOD
#define FilterOut2PD1_Pin GPIO_PIN_1
#define FilterOut2PD1_GPIO_Port GPIOD
#define ProcOut1PD2_Pin GPIO_PIN_2
#define ProcOut1PD2_GPIO_Port GPIOD
#define ProcOut2PD3_Pin GPIO_PIN_3
#define ProcOut2PD3_GPIO_Port GPIOD
#define ProcFilterPD4_Pin GPIO_PIN_4
#define ProcFilterPD4_GPIO_Port GPIOD
#define Eg1Osc1FMPD5_Pin GPIO_PIN_5
#define Eg1Osc1FMPD5_GPIO_Port GPIOD
#define Eg1Osc1WavPD6_Pin GPIO_PIN_6
#define Eg1Osc1WavPD6_GPIO_Port GPIOD
#define Eg1Osc1VolPD7_Pin GPIO_PIN_7
#define Eg1Osc1VolPD7_GPIO_Port GPIOD
#define Osc2VolOsc1FMPG9_Pin GPIO_PIN_9
#define Osc2VolOsc1FMPG9_GPIO_Port GPIOG
#define Osc2VolOsc2FMPG10_Pin GPIO_PIN_10
#define Osc2VolOsc2FMPG10_GPIO_Port GPIOG
#define Osc2VolOsc3FMPG11_Pin GPIO_PIN_11
#define Osc2VolOsc3FMPG11_GPIO_Port GPIOG
#define Osc2VolCutPG12_Pin GPIO_PIN_12
#define Osc2VolCutPG12_GPIO_Port GPIOG
#define Osc3VolOsc1FMPG13_Pin GPIO_PIN_13
#define Osc3VolOsc1FMPG13_GPIO_Port GPIOG
#define Osc3VolOsc2FMPG14_Pin GPIO_PIN_14
#define Osc3VolOsc2FMPG14_GPIO_Port GPIOG
#define Osc3VolProcPG15_Pin GPIO_PIN_15
#define Osc3VolProcPG15_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Osc3TablePB4_Pin GPIO_PIN_4
#define Osc3TablePB4_GPIO_Port GPIOB
#define FilterModePB5_Pin GPIO_PIN_5
#define FilterModePB5_GPIO_Port GPIOB
#define EG1LoopPB6_Pin GPIO_PIN_6
#define EG1LoopPB6_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define EG1TriggerPB8_Pin GPIO_PIN_8
#define EG1TriggerPB8_GPIO_Port GPIOB
#define EG2LoopPB9_Pin GPIO_PIN_9
#define EG2LoopPB9_GPIO_Port GPIOB
#define Eg1CutPE0_Pin GPIO_PIN_0
#define Eg1CutPE0_GPIO_Port GPIOE
#define Eg1MorphPE1_Pin GPIO_PIN_1
#define Eg1MorphPE1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
