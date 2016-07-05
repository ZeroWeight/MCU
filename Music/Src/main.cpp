/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
enum stairs{
	END=-1,
	N=0,
	A1=1455,
	B1b=1373,
	B1=1296,
	C1=1223,
	C1s=1154,
	D1=1090,
	E1b=1029,
	E1=971,
	F1=916,
	F1s=865,
	G1=816,
	G1s=771,
	A2=727,
	B2b=686,
	B2=648,
	C2=612,
	C2s=577,
	D2=545,
	E2b=514,
	E2=485,
	F2=458,
	F2s=432,
	G2=408,
	G2s=385,
	A3=364,
	B3b=343,
	B3=324,
	C3=306,
	C3s=289,
	D3=272,
	E3b=257,
	E3=243,
	F3=229,
	F3s=216,
	G3=216,
	G3s=193,
	A4=182,
	B4b=172,
	B4=162,
	C4=153,
	C4s=144,
	D4=136,
	E4b=129,
	E4=121,
	F4=115,
	F4s=108,
	G4=102,
	G4s=96,
	A5=91,
	B5b=86,
	B5=81,
	C5=76,
	C5s=72,
	D5=68,
	D5s=64,
	E5b=64,
	E5=61,
	F5=57,
	F5s=54,
	G5=51,
	G5s=48,
	A6=46,
	B6b=43,
	B6=40,
	C6=38,
	C6s=36,
	D6=34,
	E6b=32,
	E6=30,
	F6=29,
	F6s=27,
	G6=26,
	G6s=24,
	A7=23,
	B7b=21,
	B7=20,
	C7=19,
	C7s=18,
	D7=17,
	E7b=16,
	E7=15,
	F7=14,
	F7s=14,
	G7=13,
	G7s=12,
	A8=11,
	B8b=11,
	B8=10,
	C8=10,
};
const int map_1[]={E4b,E4b,E4b,G4,B5b,B5b,B5b,B5b,C5,C5,E5b,C5,B5b,B5b,B5b,B5b,G4,G4,G4,G4,B5b,G4,E4b,E4b,C4,C4,E4b,G4,B5b,B5b,B5b,B5b,
C5,C5,C5,C5,C5,E5b,B5b,B5b,G4,G4,F4,F4,G4,F4,E4b,E4b,F4,F4,B5b,B5b,B5b,A5,B5b,B5b,C5,C5,C5,C5,D5,C5,B5b,B5b,E5b,E5b,E5b,E5b,C5,C5,E5b,E5b,
B5b,B5b,B5b,C5,B5b,B5b,B5b,B5b,C5,C5,C5,C5,B5b,B5b,G4,G4,F4,F4,F4,G4,B5b,B5b,B5b,B5b,E4b,E4b,E4b,E4b,E4b,E4b,G4,G4,F4,F4,G4,F4,E4b,E4b,E4b,E4b,
C5,C5,C5,C5,C5,C5,G4,G4,F4,F4,F4,G4,E4b,E4b,E4b,E4b,E5b,E5b,E5b,E5b,E5b,E5b,N,N,C5,C5,C5,C5,C5,C5,N,N,B5b,B5b,B5b,B5b,C5,C5,B5b,B5b,
F4,F4,G4,G4,B5b,B5b,B5b,B5b,E5b,E5b,E5b,E5b,E5b,E5b,N,N,C5,C5,C5,C5,C5,C5,N,N,B5b,B5b,B5b,B5b,C5,C5,B5b,B5b,F4,F4,G4,F4,E4b,E4b,E4b,E4b,END};
const int map_2[]={N,N,N,E5,E5,G5s,B6,C6s,C6s,C6s,N,C6s,C6s,B6,C6s,B6,B6,F5s,F5s,F5s,F5s,E5,F5s,G5s,
G5s,G5s,N,G5s,G5s,F5s,G5s,F5s,F5s,E5,E5,E5,E5,F5s,G5s,A6,A6,A6,N,A6,A6,G5s,A6,G5s,G5s,F5s,F5s,F5s,F5s,E5,F5s,G5s,
G5s,G5s,G5s,G5s,N,N,N,N,N,N,N,G5s,G5s,G5s,B6,C6s,C6s,C6s,N,C6s,C6s,B6,C6s,B6,B6,F5s,F5s,F5s,F5s,E5,F5s,G5s,
G5s,G5s,N,G5s,G5s,F5s,G5s,F5s,F5s,E5,E5,E5,E5,F5s,G5s,A6,A6,A6,N,A6,A6,G5s,A6,B6,B6,F5s,F5s,F5s,F5s,E5,F5s,E5,
E5,D5s,C5s,C5s,C5s,C5s,C5s,C5s,END
};
const int map_3[]={F6,F6,F6,F6,F6,F6,F6,F6,E6b,E6b,F6,F6,G6s,G6s,E6b,E6b,
	F6,F6,F6,F6,C6s,C6s,E6b,E6b,G5s,G5s,G5s,G5s,G5s,G5s,G5s,G5s,
	F6,F6,F6,F6,F6,F6,F6,F6,E6b,E6b,F6,F6,G6s,G6s,E6b,E6b,
	F6,F6,F6,F6,C6s,C6s,C6s,C6s,E6b,E6b,E6b,E6b,F6,F6,F6,F6,
	E5b,E5b,F5,E5b,E5b,F5,F5,F5,G5s,G5s,F5,E5b,E5b,F5,F5,F5,
	E5b,E5b,F5,E5b,E5b,E5b,B5b,B5b,C5s,C5s,C5s,C5s,C5s,C5s,C5s,C5s,
	E5b,E5b,F5,E5b,E5b,F5,F5,F5,G5s,G5s,F5,E5b,E5b,E5b,B5b,B5b,
	B5b,B5b,F5,F5,F5,F5,F5,F5,N,N,N,N,N,N,N,N,
	E5b,E5b,F5,E5b,E5b,E5b,F5,F5,G5s,G5s,F5,E5b,F5,F5,F5,F5,
	E5b,E5b,F5,E5b,E5b,E5b,B5b,B5b,B5b,B5b,B5b,B5b,N,N,B5b,C5s,
	E5b,E5b,F5,E5b,E5b,B5b,B5b,B5b,E5b,E5b,F5,F5,E5b,E5b,G4s,G4s,
	G4s,G4s,B5b,B5b,B5b,B5b,B5b,B5b,B5b,B5b,B5b,B5b,N,N,N,N,
	E5b,E5b,F5,E5b,E5b,F5,F5,F5,G5s,G5s,F5,E5b,E5b,F5,F5,F5,
	E5b,E5b,F5,E5b,E5b,E5b,B5b,B5b,C5s,C5s,C5s,C5s,C5s,C5s,C5s,C5s,
	E5b,E5b,F5,E5b,E5b,F5,F5,F5,G5s,G5s,F5,E5b,E5b,E5b,B5b,B5b,
	B5b,B5b,F5,F5,F5,F5,F5,F5,N,N,N,N,N,N,N,N,
	E5b,E5b,F5,E5b,E5b,E5b,F5,F5,G5s,G5s,F5,E5b,F5,F5,F5,F5,
	E5b,E5b,F5,E5b,E5b,E5b,B5b,B5b,B5b,B5b,B5b,B5b,N,N,B5b,C5s,
	E5b,E5b,F5,E5b,E5b,B5b,B5b,B5b,E5b,E5b,F5,F5,E5b,E5b,G4s,G4s,
	B5b,B5b,B5b,B5b,B5b,B5b,B5b,N,N,N,N,N,N,F5,G5s,
	B6b,B6b,G5s,B6b,B6b,C6s,C6s,C6s,B6b,B6b,G5s,G5s,G5s,E5b,E5b,E5b,
	F5,F5,E5b,F5,F5,G5s,G5s,G5s,B5b,B5b,B5b,B5b,N,N,F5,G5s,
	B6b,B6b,G5s,B6b,B6b,C6s,C6s,C6s,B6b,B6b,B6b,G5s,G5s,G5s,G5s,G5s,
	E5b,E5b,E5b,E5b,F5,F5,F5,F5,F5,F5,F5,F5,N,N,F5,G5s,
	B6b,B6b,G5s,B6b,B6b,C6s,C6s,C6s,C6,C6,C6s,E6b,G5s,G5s,G5s,G5s,
	F6,F6,G6s,E6b,E6b,E6b,F6,F6,C6s,C6s,C6s,C6s,N,N,B6b,F6,
	E6b,E6b,F6,B6b,N,N,B6b,F6,E6b,E6b,F6,E6b,E6b,E6b,G5s,G5s,
	G5s,G5s,B6b,B6b,B6b,B6b,B6b,B6b,B6b,B6b,B6b,B6b,N,N,B6b,F6,
	E6b,E6b,F6,B6b,N,N,B6b,F6,E6b,E6b,F6,E6b,G5s,G5s,G5s,G5s,
	B6b,B6b,B6b,B6b,B6b,B6b,B6b,B6b,B6b,B6b,B6b,B6b,B6b,B6b,B6b,B6b,END};
const int map_4[]={C4,D4,E4,F4,G4,A5,B5,C5,D5,E5,F5,G5,A6,B6,C6,D6,END};
const int* map[]={map_1,map_2,map_3,map_4};
int music;
int iterator;
int enable;
int count;
int time1;
int time2;
int T=8000;
int buffer;
int lbutton1;
int lbutton2;
int lbutton3;
int lbutton4;
int pause;
int SHIFT;
const int l=10000;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim){
	if (!pause){
		++time1;
		++time2;
	}
	if(time1==T){
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		 ++iterator;
		//if (map[music][iterator-1]!=map[music][iterator-1])
			time2=0;
		if(map[music][iterator]==END){
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
			iterator=0;
			enable=0;
			HAL_TIM_Base_Stop_IT(&htim3);
			return;
		}
		time1=0;
	}
	else if(map[music][iterator]==N) return;
	else if (time2==(map[music][iterator]>>1)){
		time2=0;
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
	}
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)==GPIO_PIN_SET) lbutton1++;
	else lbutton1=0;
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)==GPIO_PIN_SET) lbutton2++;
	else lbutton2=0;
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==GPIO_PIN_SET) lbutton3++;
	else lbutton3=0;
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3)==GPIO_PIN_SET) lbutton4++;
	else lbutton4=0;
	if(lbutton1==l&&(!SHIFT)) {
		SHIFT=1;
		//replay
		lbutton1=0;
		iterator=0;
		for(int i=0;i<(1<<19);i++);
	}
	if(lbutton2==l&&(!SHIFT)) {
		//pause
		SHIFT=1;
		lbutton2=0;
		pause=1;
	}
	if(lbutton3==l&&(!SHIFT)) {
		SHIFT=1;
		lbutton3=0;
		pause=0;
	}
	if(lbutton4==l&&(!SHIFT)) {
		SHIFT=1;
		lbutton4=0;
		T*=1.1;
		time1=0;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
	if (SHIFT){
		SHIFT=0;
		return;
	}
	if(GPIO_PIN==GPIO_PIN_0&&(!SHIFT)){
		T=12000;
		pause=0;
		if(enable){
			if(music==0){
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				HAL_TIM_Base_Stop_IT(&htim3);
				enable=0;
				iterator=0;
			}
			else{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				HAL_TIM_Base_Stop_IT(&htim3);
				for(int i=0;i<(1<<19);i++);
				music=0;
				enable=1;
				iterator=0;
				time1=time2=0;
				HAL_TIM_Base_Start_IT(&htim3);
			}
		}
		else{
			music=0;
			enable=1;
			iterator=0;
			time1=time2=0;
			HAL_TIM_Base_Start_IT(&htim3);
		}
	}
	else if(GPIO_PIN==GPIO_PIN_1&&(!SHIFT)){
		pause=0;
		T=12000;
		if(enable){
			if(music==1){
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				HAL_TIM_Base_Stop_IT(&htim3);
				enable=0;
				iterator=0;
			}
			else{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				HAL_TIM_Base_Stop_IT(&htim3);
				for(int i=0;i<(1<<19);i++);
				music=1;
				enable=1;
				iterator=0;
				time1=time2=0;
				HAL_TIM_Base_Start_IT(&htim3);
			}
		}
		else{
			music=1;
			enable=1;
			iterator=0;
			time1=time2=0;
			HAL_TIM_Base_Start_IT(&htim3);
		}
	}
	else if(GPIO_PIN==GPIO_PIN_2&&(!SHIFT)){
		pause=0;
		T=8000;
		if(enable){
			if(music==2){
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				HAL_TIM_Base_Stop_IT(&htim3);
				enable=0;
				iterator=0;
			}
			else{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				HAL_TIM_Base_Stop_IT(&htim3);
				for(int i=0;i<(1<<19);i++);
				music=2;
				enable=1;
				iterator=0;
				time1=time2=0;
				HAL_TIM_Base_Start_IT(&htim3);
			}
		}
		else{
			music=2;
			enable=1;
			iterator=0;
			time1=time2=0;
			HAL_TIM_Base_Start_IT(&htim3);
		}
	}
	else if (GPIO_PIN==GPIO_PIN_3&&(!SHIFT)) {
		time1=0;
		T/=1.1;
	}
	
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
