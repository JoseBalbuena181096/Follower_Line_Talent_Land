/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int flag=0;
int  MAX_SPEED=0;//596-686
int  MED_SPEED=0;//314---400
float Kp=0;//106-135
float Ki=0;//
float Kd=0;//216-275
float x[3]={0,0,0};
signed int u=0;
enum {STOP,STARTING,RUN} state=STOP;
enum {CENTER,RIGHT,LEFT}out_state=CENTER;
uint16_t lectura_adc[11];
int b[11];
int error=0;
int line=0;
signed int speed_1=0;
signed int speed_2=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void Motor_R (signed int speed);
void Motor_L (signed int speed);
void ControllerPID();
void StateMachine();
void ReadSensors();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)lectura_adc,11);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  char buffer[80];

	   	 b[0]=(lectura_adc[0]);
	   	 b[1]=(lectura_adc[1]);
	   	 b[2]=(lectura_adc[2]);
	   	 b[3]=(lectura_adc[3]);
	   	 b[4]=(lectura_adc[4]);
	   	 b[5]=(lectura_adc[5]);
	   	 b[6]=(lectura_adc[6]);
	   	 b[7]=(lectura_adc[7]);
	   	 b[8]=(lectura_adc[8]);
	   	 b[9]=(lectura_adc[4]);
	   	 b[10]=(lectura_adc[10]);
	       sprintf(buffer,"%d %d  %d %d %d %d %d %d %d %d %d \n\n\n\r",b[10],b[8],b[6],b[4],b[2],b[0],b[1],b[3],b[5],b[7],b[9]);
	       HAL_UART_Transmit(&huart3,(uint8_t*)buffer,(uint16_t)strlen(buffer),(uint32_t)100);
	//StateMachine();
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void Motor_R (signed int speed)
{
   if(!speed) {
	   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
   }
   else {
      speed = (speed >= MAX_SPEED) ? MAX_SPEED : speed;
      if (speed >=1){
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,speed);
      HAL_GPIO_WritePin(GPIOE,D2_Pin,GPIO_PIN_RESET);
      }
      else {
      speed *= (0-1);
      speed = (speed >= MAX_SPEED) ? MAX_SPEED : speed;
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,speed);
      HAL_GPIO_WritePin(GPIOE,D2_Pin,GPIO_PIN_SET);
      }
   }
   return;
}

///Function to set speed of left motor///
void Motor_L (signed int speed)
{
   if(!speed) {
	   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
   }
   else {
      speed = (speed >= MAX_SPEED) ? MAX_SPEED : speed;
      if (speed >=1){
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,speed);
      HAL_GPIO_WritePin(GPIOE,D1_Pin,GPIO_PIN_SET);
      }
      else {
      speed *= (0-1);
      speed = (speed >= MAX_SPEED) ? MAX_SPEED : speed;
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,speed);
      HAL_GPIO_WritePin(GPIOE,D1_Pin,GPIO_PIN_RESET);
      }
   }
   return;
}
void ControllerPID(){
    if(line) {
     //PID Control
    x[0]=error;
    x[2]=x[0]+x[2];
    u=Kp*x[0]+Ki*(x[2])+Kd*(x[0]-x[1]);
    speed_1=MED_SPEED-u;
    speed_2=MED_SPEED+u;
    Motor_R(speed_2);
    Motor_L(speed_1);
    x[1]=x[0];
  }
   else{
      switch (out_state){
      case CENTER:
           speed_1 = MED_SPEED;
           speed_2 = MED_SPEED;
           break;
      case LEFT:
           speed_1 = MAX_SPEED;
           speed_2 = (0-MAX_SPEED);
           break;
      case RIGHT:
           speed_1 = (0-MAX_SPEED);
           speed_2 = MAX_SPEED;
           break;
     }
    }
  }
void StateMachine(){
   switch(state){
    case STOP:
           __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
           __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
           if((flag==0)&& !(HAL_GPIO_ReadPin(GPIOD,SW2_Pin))){
        	   /*
        	   MAX_SPEED=650;//
        	   MED_SPEED=565;//
        	   Kp=145;//
        	   Ki=0;//
        	   Kd=327;//
*/
          	   MAX_SPEED=455;//
          	   MED_SPEED=0;//
          	   Kp=102;//
          	   Ki=0;//
          	   Kd=229;//
          	   state=STARTING;
               flag++;
           }
           else if((flag==0)&& !(HAL_GPIO_ReadPin(GPIOC,SW1_Pin))){
        	   MAX_SPEED=650;//
        	   MED_SPEED=0;//
        	   Kp=145;//
        	   Ki=0;//
        	   Kd=327;//
               state=STARTING;
               flag++;
             }
            break;
    case STARTING:
            state=RUN;
            break;
    case RUN:
    	    ReadSensors();
    	    ControllerPID();
            break;
     }
  }
void ReadSensors(){

	 b[0]=(lectura_adc[0]>4000) ? 1 : 0;
	 b[1]=(lectura_adc[1]>4000) ? 1 : 0;
	 b[2]=(lectura_adc[2]>4000) ? 1 : 0;
	 b[3]=(lectura_adc[3]>4000) ? 1 : 0;
	 b[4]=(lectura_adc[4]>4000) ? 1 : 0;
	 b[5]=(lectura_adc[5]>4000) ? 1 : 0;
	 b[6]=(lectura_adc[6]>4000) ? 1 : 0;
	 b[7]=(lectura_adc[7]>4000) ? 1 : 0;
	 b[8]=(lectura_adc[8]>4000) ? 1 : 0;
	 b[9]=(lectura_adc[9]>4000) ? 1 : 0;
	 b[10]=(lectura_adc[10]>4000)? 1 : 0;

/*
	 b[0]=(lectura_adc[0]);
	 b[1]=(lectura_adc[1]);
	 b[2]=(lectura_adc[2]);
	 b[3]=(lectura_adc[3]);
	 b[4]=(lectura_adc[4]);
	 b[5]=(lectura_adc[5]);
	 b[6]=(lectura_adc[6]);
	 b[7]=(lectura_adc[7]);
	 b[8]=(lectura_adc[8]);
	 b[9]=(lectura_adc[4]);
	 b[10]=(lectura_adc[10]);
 */
    if (b[0]||b[1]||b[2]||b[3]||b[4]||b[5]||b[6]||b[7]||b[8]||b[9]||b[10]) {
        error = (b[1])         ? (0-2) : error;
        error = (b[3])         ? (0-4) : error;
        error = (b[5])         ? (0-6) : error;
        error = (b[7])         ? (0-8) : error;
        error = (b[9])         ? (0-10) : error;
        error = (b[1] && b[3])   ? (0-3) : error;
        error = (b[3] && b[5])   ? (0-5) : error;
        error = (b[5] && b[7])   ? (0-7) : error;
        error = (b[7] && b[9])   ? (0-9) : error;
        /*Positive right sensor*/
        error = (b[2])         ? 2 : error;
        error = (b[4])         ? 4 : error;
        error = (b[6])         ? 6 : error;
        error = (b[8])         ? 8 : error;
        error = (b[10])        ? 10 : error;
        error = (b[2] && b[4])   ? 3 : error;
        error = (b[4] && b[6])   ? 5 : error;
        error = (b[6] && b[8])   ? 7 : error;
        error = (b[8] && b[10])  ? 9 : error;
        /*Neutral middle sensor*/
        error = (b[0])         ? 0 : error;
        error = (b[0] && b[1])   ? (0-1) : error;
        error = (b[0] && b[2])   ? 1 : error;
        out_state = ((error <= 4)&&(error >=(0-4)))  ? CENTER : out_state;
        out_state = ((error >= 5)&&(error <=10))        ? LEFT : out_state;
        out_state = ((error <=(0-5))&&(error >=(0-10))) ? RIGHT: out_state;
        line=1;
    }
    else{
        line=0;
      }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
