/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wavetables.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DAC_RANGE 4095.0f
#define SR 96000.f
#define WAVE_TABLE_SIZE 4096.f

struct OscData {
  // everything phase related
  float inc;
  float phase_accumulator;

  // the rest
  /*
  uint32_t waveform1 = maxAnalogIn;
  uint32_t waveform2;
  */
  float crossFMint;
  float crossFM;
  float volume;
  float tableIndex;

  GPIO_PinState waveChange; // chooses alternative wavetable
  float output1;
  float output2;
  float outputFilter;
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint16_t adc1Buffer[9];
uint16_t adc2Buffer[5];
uint16_t adc3Buffer[8];
volatile uint8_t adc1Completed = 0;
volatile uint8_t adc2Completed = 0;
volatile uint8_t adc3Completed = 0;

float analogIn = 0.0f;
float aInOsc1, aInOsc2, aInOsc3, aInFilter;
float cutoff = 0.f;
float reso = 0.1f;
float low=0.f;
float high=0.f;
float band=0.f;
float notch = 0.f;
float filterIndex = 0.f;
float filterVolume = 0.f;
float *filterStates[5]={&low,&high,&band,&notch,&low};
float filterOutput1,filterOutput2;
float dacLUT[4096];
float cutLUT[4096];

float eg1Attack;
float eg1Decay;
GPIO_PinState eg1Loop;
GPIO_PinState eg1Trigger;
float eg1Value = 0.f;
float eg1Inc = 0.f;
float eg1Dec = 0.f;
float eg1Amount = 0.f;
float eg1Destinations[12];
typedef enum {
	stop,
	attack,
	sustain,
	decay
} egStates;

egStates eg1State = stop;
egStates eg2State = stop;

uint32_t DACData;
volatile struct OscData osc1 = {.inc = WAVE_TABLE_SIZE * 100.f / SR, .phase_accumulator = 0.f,.tableIndex = 0};
volatile struct OscData osc2 = {.inc = WAVE_TABLE_SIZE * 101.f / SR, .phase_accumulator = 0.f,.tableIndex = 0};
volatile struct OscData osc3 = {.inc = WAVE_TABLE_SIZE * 99.f / SR, .phase_accumulator = 0.f,.tableIndex = 0};

// wavetableset definitions, each one is made of several individual single cycle wave tables
const float *waveset1[6]={sintable,tritable,sawtable,squaretable,randomtable,sintable};
const float *waveset2[6]={sintable,sinsawtable,sawtable,squaretable,sinsquaretable,sintable};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_DAC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if (hadc == &hadc1){
		adc1Completed = 1;
		return;
	}
	if (hadc == &hadc2){
		adc2Completed = 1;
		return;
	}
	if (hadc == &hadc3){
		adc3Completed = 1;
		return;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// since we only use one timer, there is no need to check the timer

	// calculate envelope generator 1

	if (eg1State == attack) {
		if (eg1Loop == GPIO_PIN_SET || eg1Trigger == GPIO_PIN_RESET){
			eg1Value += eg1Inc;
			if (eg1Value >= 1.f) {
				eg1Value = 1.f;
				eg1State=sustain;
			}
		}
		else {
			eg1State = decay;
			HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
		}
	}
	else if (eg1State == sustain){
		if (eg1Trigger == GPIO_PIN_SET){
			eg1State = decay;
			HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
		}
	}
	else if (eg1State == decay){
		eg1Value -= eg1Dec;
		if (eg1Value <= 0.f){
			eg1Value = 0.f;
			eg1State = stop;
		}
	}
	else if (eg1State == stop){
		if (eg1Loop == GPIO_PIN_SET || eg1Trigger == GPIO_PIN_RESET){
			eg1State = attack;
			HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
		}
	}

	//float env1Value = eg1Value * eg1Value; // making the envelope snappy
	float env1Value = eg1Value * (eg1Amount*200.f);


	float sample;
	float sample2;
	float sample3;

	float accumulator;

	// calculate oscillator 1
	accumulator = osc1.phase_accumulator;
	accumulator += osc1.inc;
	if (accumulator >= WAVE_TABLE_SIZE) {
	    accumulator -= WAVE_TABLE_SIZE;
	}
	// apply FM

	accumulator += analogIn * aInOsc1 + eg1Destinations[0] * (env1Value * 200.f);//osc1.crossFM * sample2;

	if (accumulator >= WAVE_TABLE_SIZE) {
		    accumulator -= WAVE_TABLE_SIZE;
	}
	else if (accumulator < 0) {
	    accumulator += WAVE_TABLE_SIZE;
	}
	// store accumulator
	osc1.phase_accumulator = accumulator;

	// get wavetable
	float tableIndex = osc1.tableIndex + eg1Destinations[1]* (env1Value * 8.f);
	uint16_t tindex = (uint16_t)tableIndex;
	if (tindex>4)tindex = 4;
	uint16_t tindexPlus = tindex + 1;


	// interpolated table value
	uint16_t index = (uint16_t)accumulator;
	uint16_t indexPlus = index + 1;
	float sampleA,sampleB;

	float tableFactor1 = osc1.waveChange == GPIO_PIN_RESET ? 1.f:0.f;
	sampleA = tableFactor1 * ((indexPlus - accumulator) * *(waveset1[tindex]+index) + (accumulator-index) * *(waveset1[tindex]+indexPlus));
	sampleB = tableFactor1 * ((indexPlus - accumulator) * *(waveset1[tindexPlus]+index) + (accumulator-index) * *(waveset1[tindexPlus]+indexPlus));

	float tableFactor2 = osc1.waveChange == GPIO_PIN_SET ? 1.f:0.f;
	sampleA += tableFactor2 * ((indexPlus - accumulator) * *(waveset2[tindex]+index) + (accumulator-index) * *(waveset2[tindex]+indexPlus));
	sampleB += tableFactor2 * ((indexPlus - accumulator) * *(waveset2[tindexPlus]+index) + (accumulator-index) * *(waveset2[tindexPlus]+indexPlus));

	sample = (((tindexPlus - tableIndex) * sampleA + (tableIndex-tindex)*sampleB) * osc1.volume);// * env1Value;

	// calculate oscillator 2
	accumulator = osc2.phase_accumulator;
	accumulator += osc2.inc;
	if (accumulator >= WAVE_TABLE_SIZE) {
		accumulator -= WAVE_TABLE_SIZE;
	}
	accumulator += analogIn * aInOsc2;

	if (accumulator >= WAVE_TABLE_SIZE) {
		accumulator -= WAVE_TABLE_SIZE;
	}
	else if (accumulator < 0) {
		accumulator += WAVE_TABLE_SIZE;
	}
	osc2.phase_accumulator = accumulator;

	// get wavetable
	tableIndex = osc2.tableIndex;
	tindex = (uint16_t)tableIndex;
	tindexPlus = tindex + 1;


	// interpolated table value
	index = (uint16_t)accumulator;
	indexPlus = index + 1;

	tableFactor1 = osc2.waveChange == GPIO_PIN_RESET ? 1.f:0.f;
	tableFactor2 = osc2.waveChange == GPIO_PIN_SET ? 1.f:0.f;

	sampleA = tableFactor1 * ((indexPlus - accumulator) * *(waveset1[tindex]+index) + (accumulator-index) * *(waveset1[tindex]+indexPlus));
	sampleB = tableFactor1 * ((indexPlus - accumulator) * *(waveset1[tindexPlus]+index) + (accumulator-index) * *(waveset1[tindexPlus]+indexPlus));


	sampleA += tableFactor2 *((indexPlus - accumulator) * *(waveset2[tindex]+index) + (accumulator-index) * *(waveset2[tindex]+indexPlus));
	sampleB += tableFactor2 * ((indexPlus - accumulator) * *(waveset2[tindexPlus]+index) + (accumulator-index) * *(waveset2[tindexPlus]+indexPlus));

	sample2 = ((tindexPlus - tableIndex) * sampleA + (tableIndex-tindex)*sampleB) * osc2.volume;

	// calculate oscillator 3
	accumulator = osc3.phase_accumulator;
	accumulator += osc3.inc;
	if (accumulator >= WAVE_TABLE_SIZE) {
		accumulator -= WAVE_TABLE_SIZE;
	}
	accumulator += analogIn * aInOsc3;

	if (accumulator >= WAVE_TABLE_SIZE) {
		accumulator -= WAVE_TABLE_SIZE;
	}
	else if (accumulator < 0) {
		accumulator += WAVE_TABLE_SIZE;
	}
	osc3.phase_accumulator = accumulator;

	// get wavetable
	tableIndex = osc3.tableIndex;
	tindex = (uint16_t)tableIndex;
	tindexPlus = tindex + 1;


	// interpolated table value
	index = (uint16_t)accumulator;
	indexPlus = index + 1;

	tableFactor1 = osc3.waveChange == GPIO_PIN_RESET ? 1.f:0.f;
	tableFactor2 = osc3.waveChange == GPIO_PIN_SET ? 1.f:0.f;

	sampleA = tableFactor1 * ((indexPlus - accumulator) * *(waveset1[tindex]+index) + (accumulator-index) * *(waveset1[tindex]+indexPlus));
	sampleB = tableFactor1 * ((indexPlus - accumulator) * *(waveset1[tindexPlus]+index) + (accumulator-index) * *(waveset1[tindexPlus]+indexPlus));


	sampleA += tableFactor2 * ((indexPlus - accumulator) * *(waveset2[tindex]+index) + (accumulator-index) * *(waveset2[tindex]+indexPlus));
	sampleB += tableFactor2 * ((indexPlus - accumulator) * *(waveset2[tindexPlus]+index) + (accumulator-index) * *(waveset2[tindexPlus]+indexPlus));

	sample3 = ((tindexPlus - tableIndex) * sampleA + (tableIndex-tindex)*sampleB) * osc3.volume;

	// filter
	//cutoff = cutoff freq in Hz
	//fs = sampling frequency //(e.g. 44100Hz)
	//f = 2 sin (pi * cutoff / fs) //[approximately]
	//q = resonance/bandwidth [0 < q <= 1]  most res: q=1, less: q=0

	// adding the oscillators
	float filterInput = (sample * osc1.outputFilter + sample2 *osc2.outputFilter + sample3 * osc3.outputFilter) ;/// 2.0f;

	low = low + cutoff * band;
	high = reso * filterInput - low - reso*band;
	band = cutoff * high + band;
	notch = high + low;
	float compensation = 1.f;
	float antiReso = 1.f - reso;
	//if (reso < 0.4f)
	compensation = 4.f*antiReso*antiReso*antiReso*antiReso*antiReso*antiReso*antiReso * antiReso * antiReso * antiReso * antiReso + 1.f;
	// filter blend
	index = (uint16_t)filterIndex;
	indexPlus = index + 1;
	float filterOutput = ((indexPlus - filterIndex) * *filterStates[index] + (filterIndex - index) * *filterStates[indexPlus])*filterVolume * compensation;

	float output1 = (filterOutput*filterOutput1 + sample * osc1.output1 + sample2 * osc2.output1 + sample3 * osc3.output1) * 0.222f;// 4.5f;
	float output2 = (filterOutput*filterOutput2 + sample * osc1.output2 + sample2 * osc2.output2 + sample3 * osc3.output2) * 0.222f;// 4.5f;
	DACData = ((uint32_t) (((output1 + 1.0f)*0.5 * DAC_RANGE)* 0.5) );
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DACData);

	DACData = ((uint32_t) (((output2 + 1.0f)*0.5 * DAC_RANGE)* 0.5) );
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DACData);

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// init wavetable
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  for (int i=0;i<4096;i++){
	  dacLUT[i] = ((float)i) / 4095.0f;
	  cutLUT[i] = (2.f * sinf (3.14159265359f *  (dacLUT[i]*11000.f + 20.f) / SR));
  }
  float eg1AmountReadings[32];
  int eg1AmountIndex = 0;

  float eg1AttackReadings[8];
  int eg1AttackIndex = 0;
  float eg1DecayReadings[8];
  int eg1DecayIndex = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_DAC_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  // start peripherals

  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim6); // start the timer in interrupt mode
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1Buffer, 9);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2Buffer, 5);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3Buffer, 8);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float freqFactor = WAVE_TABLE_SIZE / SR;
  //float SRFactor = 1. / SR;
  while (1)
  {

	  while(!adc1Completed && !adc2Completed && !adc3Completed){
		  HAL_Delay(2);
	  }
	  if (adc1Completed){
		  HAL_ADC_Stop_DMA(&hadc1);
		  adc1Completed = 0;

		  osc1.inc = freqFactor * (dacLUT[adc1Buffer[0]]*2095.f + 0.01f);
		  osc1.tableIndex = dacLUT[adc1Buffer[1]] * 4.f;
		  osc1.volume = dacLUT[adc1Buffer[2]];
		  osc1.crossFM = dacLUT[adc1Buffer[3]]*1000.f;

		  osc2.inc = freqFactor * (dacLUT[adc1Buffer[4]]*2095.f + 0.01f);
		  osc2.tableIndex = dacLUT[adc1Buffer[5]]* 4.f;
		  osc2.volume = dacLUT[adc1Buffer[6]];

		  osc3.inc = freqFactor * (dacLUT[adc1Buffer[7]]*2095.f + 0.01f);
		  osc3.tableIndex = dacLUT[adc1Buffer[8]]* 4.f;


		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1Buffer, 9);
	  }

	  if (adc2Completed){
		  HAL_ADC_Stop_DMA(&hadc2);
		  adc2Completed = 0;
		  osc3.volume = dacLUT[adc2Buffer[0]];
		 //10 13
		  cutoff = cutLUT[adc2Buffer[1]];//(2.f * sinf (3.14159265359f *  (dacLUT[adc2Buffer[1]]*11000.f + 20.f + analogIn*aInFilter*100.f) * SRFactor));
		  reso =  dacLUT[adc2Buffer[2]] + 0.01f;
		  filterIndex = dacLUT[adc2Buffer[3]] * 3.f;
		  filterVolume = dacLUT[adc2Buffer[4]];

		  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2Buffer, 5);
	  }

	  if (adc3Completed){
		  HAL_ADC_Stop_DMA(&hadc3);
		  adc3Completed = 0;

		  analogIn = adc3Buffer[3];

		  eg1AttackReadings[eg1AttackIndex] = dacLUT[adc3Buffer[5]];
		  eg1AttackIndex++;
		  if (eg1AttackIndex == 8) {
			  float eAtt = (eg1AttackReadings[0]+eg1AttackReadings[1]+eg1AttackReadings[2]+eg1AttackReadings[3]+eg1AttackReadings[4]+eg1AttackReadings[5]+eg1AttackReadings[6]+eg1AttackReadings[7]) / 8.f;
			  eg1Inc = eAtt * 0.125f + 0.00001f;
			  eg1AttackIndex = 0;
		  }

		  eg1DecayReadings[eg1DecayIndex] = dacLUT[adc3Buffer[6]];
		  eg1DecayIndex++;
		  if (eg1DecayIndex == 8){
			  float eDec = (eg1DecayReadings[0]+eg1DecayReadings[1]+eg1DecayReadings[2]+eg1DecayReadings[3]+eg1DecayReadings[4]+eg1DecayReadings[5]+eg1DecayReadings[6]+eg1DecayReadings[7]) / 8.f;
			  eg1Dec = eDec * 0.125f + 0.00001f;
			  eg1DecayIndex = 0;
		  }


		  eg1AmountReadings[eg1AmountIndex] = dacLUT[adc3Buffer[7]];
		  eg1AmountIndex++;
		  if (eg1AmountIndex == 32){
			  eg1AmountIndex = 0;
			  eg1Amount = (eg1AmountReadings[0] + eg1AmountReadings[1] + eg1AmountReadings[2] + eg1AmountReadings[3]
			+ eg1AmountReadings[4] + eg1AmountReadings[5] + eg1AmountReadings[6] + eg1AmountReadings[7]
			+ eg1AmountReadings[8] + eg1AmountReadings[9] + eg1AmountReadings[10] + eg1AmountReadings[11]
			+ eg1AmountReadings[12] + eg1AmountReadings[13] + eg1AmountReadings[14] + eg1AmountReadings[15]
			+ eg1AmountReadings[16] + eg1AmountReadings[17] + eg1AmountReadings[18] + eg1AmountReadings[19]
			+ eg1AmountReadings[20] + eg1AmountReadings[21] + eg1AmountReadings[22] + eg1AmountReadings[23]
			+ eg1AmountReadings[24] + eg1AmountReadings[25] + eg1AmountReadings[26] + eg1AmountReadings[27]
		    + eg1AmountReadings[28] + eg1AmountReadings[29] + eg1AmountReadings[30] + eg1AmountReadings[31]
			  ) / 32.f;
		  }

		  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3Buffer, 8);
	  }

	  // read GPIO
	  osc1.waveChange = HAL_GPIO_ReadPin(Osc1TablePA15_GPIO_Port, Osc1TablePA15_Pin);
	  osc2.waveChange = HAL_GPIO_ReadPin(GPIOB, Osc2TablePB2_Pin);
	  osc3.waveChange = HAL_GPIO_ReadPin(GPIOB, Osc3TablePB4_Pin);

	  osc1.output1 = HAL_GPIO_ReadPin(GPIOB, Osc1VolOut1PB11_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;
	  osc1.output2 = HAL_GPIO_ReadPin(GPIOB, Osc1VolOut2PB12_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;
	  osc1.outputFilter = HAL_GPIO_ReadPin(GPIOC, Osc1VolFilterPC6_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;

	  osc2.output1 = HAL_GPIO_ReadPin(GPIOC, Osc2VolOut1PC7_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;
	  osc2.output2 = HAL_GPIO_ReadPin(GPIOC, Osc2VolOut2PC8_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;
	  osc2.outputFilter =  HAL_GPIO_ReadPin(GPIOC, Osc2VolFilterPC9_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;

	  osc3.output1 = HAL_GPIO_ReadPin(GPIOC, Osc3VolOut1PC10_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;
	  osc3.output2 = HAL_GPIO_ReadPin(GPIOC, Osc3VolOut2PC11_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;
	  osc3.outputFilter = HAL_GPIO_ReadPin(GPIOC, Osc3VolFilterPC12_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;

	  filterOutput1 = HAL_GPIO_ReadPin(GPIOD, FilterOut1PD0_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;
	  filterOutput2 = HAL_GPIO_ReadPin(GPIOD, FilterOut2PD1_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;


	  aInOsc1 = HAL_GPIO_ReadPin(GPIOF, AnalogInOsc1FMPF1_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;
	  aInOsc2 = HAL_GPIO_ReadPin(GPIOF, AnalogInOsc2FMPF2_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;
	  aInOsc3 = HAL_GPIO_ReadPin(GPIOF, AnalogInOsc3FMPF11_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;
	  aInFilter = HAL_GPIO_ReadPin(GPIOF, AnalogInCutPF12_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;

	  eg1Loop = HAL_GPIO_ReadPin(GPIOB, EG1LoopPB6_Pin);// == GPIO_PIN_RESET ? 1.0f:0.f;
	  eg1Trigger = HAL_GPIO_ReadPin(GPIOB, EG1TriggerPB8_Pin);// == GPIO_PIN_RESET ? 1.0f:0.f;

	  eg1Destinations[0] = HAL_GPIO_ReadPin(GPIOD, Eg1Osc1FMPD5_Pin) == GPIO_PIN_RESET ? 1.0f:0.f;
	  eg1Destinations[1] = HAL_GPIO_ReadPin(GPIOD, Eg1Osc1WavPD6_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
	  eg1Destinations[2] = HAL_GPIO_ReadPin(GPIOD, Eg1Osc1VolPD7_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
	  eg1Destinations[3] = HAL_GPIO_ReadPin(GPIOD, Eg1Osc2FMPD10_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
	  eg1Destinations[4] = HAL_GPIO_ReadPin(GPIOD, Eg1Osc2WavPD11_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
	  eg1Destinations[5] = HAL_GPIO_ReadPin(GPIOD, Eg1Osc2VolPD12_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
	  eg1Destinations[6] = HAL_GPIO_ReadPin(GPIOD, Eg1Osc3FMPD13_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
	  eg1Destinations[7] = HAL_GPIO_ReadPin(GPIOD, Eg1Osc3WavPD14_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
	  eg1Destinations[8] = HAL_GPIO_ReadPin(GPIOD, Eg1Osc3VolPD15_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
	  eg1Destinations[9] = HAL_GPIO_ReadPin(GPIOE, Eg1CutPE0_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
	  eg1Destinations[10] = HAL_GPIO_ReadPin(GPIOE, Eg1FilterVolPE2_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;
	  eg1Destinations[11] = HAL_GPIO_ReadPin(GPIOE, Eg1MorphPE1_Pin)== GPIO_PIN_RESET ? 1.0f:0.f;

	  HAL_Delay(2);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 5;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 8;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 216000000/96000 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Eg1FilterVolPE2_Pin Eg1ProcPE3_Pin Eg2Osc1FMPE4_Pin Eg2Osc1WavPE5_Pin
                           Eg2Osc1VolPE6_Pin Eg2Osc2FMPE7_Pin Eg2Osc2WavPE8_Pin Eg2Osc2VolPE9_Pin
                           Eg2Osc3FMPE10_Pin Eg2Osc3WavPE11_Pin Eg2Osc3VolPE12_Pin Eg2CutPE13_Pin
                           Eg2MorphPE14_Pin Eg2FilterVolPE15_Pin Eg1CutPE0_Pin Eg1MorphPE1_Pin */
  GPIO_InitStruct.Pin = Eg1FilterVolPE2_Pin|Eg1ProcPE3_Pin|Eg2Osc1FMPE4_Pin|Eg2Osc1WavPE5_Pin
                          |Eg2Osc1VolPE6_Pin|Eg2Osc2FMPE7_Pin|Eg2Osc2WavPE8_Pin|Eg2Osc2VolPE9_Pin
                          |Eg2Osc3FMPE10_Pin|Eg2Osc3WavPE11_Pin|Eg2Osc3VolPE12_Pin|Eg2CutPE13_Pin
                          |Eg2MorphPE14_Pin|Eg2FilterVolPE15_Pin|Eg1CutPE0_Pin|Eg1MorphPE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Eg2ProcPF0_Pin AnalogInOsc1FMPF1_Pin AnalogInOsc2FMPF2_Pin AnalogInOsc3FMPF11_Pin
                           AnalogInCutPF12_Pin Osc1FMOsc1VolPF13_Pin Osc1FMOsc2FMPF14_Pin Osc1FMOsc2WavPF15_Pin */
  GPIO_InitStruct.Pin = Eg2ProcPF0_Pin|AnalogInOsc1FMPF1_Pin|AnalogInOsc2FMPF2_Pin|AnalogInOsc3FMPF11_Pin
                          |AnalogInCutPF12_Pin|Osc1FMOsc1VolPF13_Pin|Osc1FMOsc2FMPF14_Pin|Osc1FMOsc2WavPF15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : Osc2TablePB2_Pin EG2TriggerPB10_Pin Osc1VolOut1PB11_Pin Osc1VolOut2PB12_Pin
                           Osc1FMOsc1FMPB13_Pin Osc1FMOsc1WavPB15_Pin Osc3TablePB4_Pin FilterModePB5_Pin
                           EG1LoopPB6_Pin EG1TriggerPB8_Pin EG2LoopPB9_Pin */
  GPIO_InitStruct.Pin = Osc2TablePB2_Pin|EG2TriggerPB10_Pin|Osc1VolOut1PB11_Pin|Osc1VolOut2PB12_Pin
                          |Osc1FMOsc1FMPB13_Pin|Osc1FMOsc1WavPB15_Pin|Osc3TablePB4_Pin|FilterModePB5_Pin
                          |EG1LoopPB6_Pin|EG1TriggerPB8_Pin|EG2LoopPB9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Osc1FMOsc2VolPG0_Pin Osc1FMOsc3FMPG1_Pin Osc1FMOsc3WavPG2_Pin Osc1FMOsc3VolPG3_Pin
                           Osc1FMCutPG4_Pin Osc1FMMorphPG5_Pin Osc1FMProcPG8_Pin Osc2VolOsc1FMPG9_Pin
                           Osc2VolOsc2FMPG10_Pin Osc2VolOsc3FMPG11_Pin Osc2VolCutPG12_Pin Osc3VolOsc1FMPG13_Pin
                           Osc3VolOsc2FMPG14_Pin Osc3VolOsc3FMPG15_Pin */
  GPIO_InitStruct.Pin = Osc1FMOsc2VolPG0_Pin|Osc1FMOsc3FMPG1_Pin|Osc1FMOsc3WavPG2_Pin|Osc1FMOsc3VolPG3_Pin
                          |Osc1FMCutPG4_Pin|Osc1FMMorphPG5_Pin|Osc1FMProcPG8_Pin|Osc2VolOsc1FMPG9_Pin
                          |Osc2VolOsc2FMPG10_Pin|Osc2VolOsc3FMPG11_Pin|Osc2VolCutPG12_Pin|Osc3VolOsc1FMPG13_Pin
                          |Osc3VolOsc2FMPG14_Pin|Osc3VolOsc3FMPG15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Eg1Osc2FMPD10_Pin Eg1Osc2WavPD11_Pin Eg1Osc2VolPD12_Pin Eg1Osc3FMPD13_Pin
                           Eg1Osc3WavPD14_Pin Eg1Osc3VolPD15_Pin FilterOut1PD0_Pin FilterOut2PD1_Pin
                           ProcOut1PD2_Pin ProcOut2PD3_Pin Osc3VolCutPD4_Pin Eg1Osc1FMPD5_Pin
                           Eg1Osc1WavPD6_Pin Eg1Osc1VolPD7_Pin */
  GPIO_InitStruct.Pin = Eg1Osc2FMPD10_Pin|Eg1Osc2WavPD11_Pin|Eg1Osc2VolPD12_Pin|Eg1Osc3FMPD13_Pin
                          |Eg1Osc3WavPD14_Pin|Eg1Osc3VolPD15_Pin|FilterOut1PD0_Pin|FilterOut2PD1_Pin
                          |ProcOut1PD2_Pin|ProcOut2PD3_Pin|Osc3VolCutPD4_Pin|Eg1Osc1FMPD5_Pin
                          |Eg1Osc1WavPD6_Pin|Eg1Osc1VolPD7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Osc1VolFilterPC6_Pin Osc2VolOut1PC7_Pin Osc2VolOut2PC8_Pin Osc2VolFilterPC9_Pin
                           Osc3VolOut1PC10_Pin Osc3VolOut2PC11_Pin Osc3VolFilterPC12_Pin */
  GPIO_InitStruct.Pin = Osc1VolFilterPC6_Pin|Osc2VolOut1PC7_Pin|Osc2VolOut2PC8_Pin|Osc2VolFilterPC9_Pin
                          |Osc3VolOut1PC10_Pin|Osc3VolOut2PC11_Pin|Osc3VolFilterPC12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Osc1TablePA15_Pin */
  GPIO_InitStruct.Pin = Osc1TablePA15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Osc1TablePA15_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
