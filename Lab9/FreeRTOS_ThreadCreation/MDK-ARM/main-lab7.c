/**
 ******************************************************************************
 * @file    FreeRTOS\FreeRTOS_ThreadCreation\Src\main.c
 * @author  MCD Application Team
 * @version V1.0.1
 * @date    26-February-2014
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"

//201102452 서정원
//201102520 최지인


/* Private typedef -----------------------------------------------------------*/
#define  PERIOD_VALUE       0xFFFF  /* Period Value  */
#define  PULSE1_VALUE       0xFFFF        /* Capture Compare 1 Value  */
#define  PULSE2_VALUE       900         /* Capture Compare 2 Value  */
#define  PULSE3_VALUE       600         /* Capture Compare 3 Value  */
#define  PULSE4_VALUE       450         /* Capture Compare 4 Value  */
/* Private define ------------------------------------------------------------*/
TIM_HandleTypeDef TimHandle1, TimHandle2, TimHandle3, TimHandle4;;
TIM_OC_InitTypeDef sConfig, sConfig1, sConfig2;
TIM_Encoder_InitTypeDef sENConfig;
/* Timer Input Capture Configuration Structure declaration */
TIM_IC_InitTypeDef sICConfig;

/* Captured Values */
uint32_t uwIC2Value1 = 0;
uint32_t uwIC2Value2 = 0;
uint32_t uwDiffCapture1 = 0;

uint32_t uwIC2Value3 = 0;
uint32_t uwIC2Value4 = 0;
uint32_t uwDiffCapture2 = 0;

uint32_t uwIC2Value5 = 0;
uint32_t uwIC2Value6= 0;
uint32_t uwDiffCapture3 = 0;


/*
 * 최초 ubrain을 동작 시켰을 때 sonar의 값은
 * 0이 측정되므로 이 값은 무시해야 한다.
 * 따라서 충돌의 위험을 제거하기 위해
 * 초기 값을 충분히 큰 값으로 설정하고 시작한다.
 */
uint32_t distance1 = 99999;
uint32_t distance2 = 99999;
uint32_t distance3 = 99999;

/*
 * 좌전방, 우전방 적외선 센서의 값을 저장할
 * 변수이다.
 */
uint32_t rSensor;
uint32_t lSensor;

/*
 * 방향을 나타내는 변수
 * 정면 : 0
 * 왼쪽으로 돈 상황 : 음수
 * 오른쪽으로 돈 상황 : 양수
 */
int direction = 0;


/* Capture index */
uint32_t uhCaptureIndex = 0;

/* Frequency Value */
uint32_t uwFrequency = 0;

uint32_t firstInput = 0;
uint32_t secondInput = 0;
uint32_t mydiff = 0;

uint32_t uwPrescalerValue = 0;
uint32_t uwPrescalerValue1 = 0;
uint32_t uwPrescalerValue2 = 0;
uint16_t motorInterrupt1 = 0;
uint16_t motorInterrupt2 = 0;

uint8_t encoder_right = READY;
uint8_t encoder_left  = READY;

static void Error_Handler(void);

void TIM3_IRQHandler(void)
{
								HAL_TIM_IRQHandler(&TimHandle1);
}

xTaskHandle xHandle;


#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
		#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
		#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
								/* Place your implementation of fputc here */
								/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
								HAL_UART_Transmit(&UartHandle1, (uint8_t *)&ch, 1, 0xFFFF);

								return ch;
}

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t ch;
uint16_t Encoder1 = 0;
uint16_t Encoder2 = 0;
GPIO_InitTypeDef GPIO_InitStruct;

/* ADC handler declaration */
ADC_HandleTypeDef AdcHandle1, AdcHandle2, AdcHandle3;
ADC_ChannelConfTypeDef adcConfig1, adcConfig2, adcConfig3;
ADC_ChannelConfTypeDef sConfigADC;

/* Variable used to get converted value */
__IO uint32_t uhADCxRight;
__IO uint32_t uhADCxForward;
__IO uint32_t uhADCxLeft;

osThreadId LEDThread1Handle, LEDThread2Handle;
/* Private function prototypes -----------------------------------------------*/

static void SystemClock_Config(void);
static void prvuBrainInit(void);
void Motor_Forward(void);
void Motor_Backward(void);
void Motor_Left(void);
void Motor_Right(void);
void Motor_Stop(void);
void Motor_Speed_Up_Config(void);
void Motor_Speed_Down_Config(void);
void Motor_Init(void);
static void EXTILine_Config(void);
static void Error_Handler(void);
/* Private functions ---------------------------------------------------------*/

/*
 * myTask는 ubrain의 이동간 충돌 감지와
 * 방향을 결정하는 task이다.
 * 이 task에서 주행알고리즘의 대부분을 차지하고
 * 있다.
 * 상황 판단은 3가지로 나누었다.
 * 1. 모든 진행에 있어서 전방에 장애물이 출현한 경우
 * 2. 장애물을 피해 왼쪽으로 돈 경우
 * 3. 장애물을 피해 오른쪽으로 돈 경우
 */
void myTask(void *params) {
		int i;
		osDelay(20); //20ms Delay
		Motor_Forward(); // 매번 앞으로 전진 시킴
		for (;; ) { // 무한 loop을 이용해 매번 상황을 인지함
										/*
										 * 전방 25cm 이내와 좌전방 우전방 적외선 센서값이 10000이상일 경우
										 * 장애물이 있다고 판단한다.
										 */
										if (distance2 < 25 || uhADCxLeft > 1000 || uhADCxRight > 1000 ) {
																		// 주행을 멈추고
																		Motor_Stop();

																		/* 좌 우 초음파 센서의 값을 비교해 더 먼 쪽으로 회전한다.
																		 * 1. 오른쪽 센서값이 더 클경우
																		 * 오른쪽으로 회전
																		 */
																		if (( distance1 > distance3 )) {
																										motorInterrupt2 = 1; // 바퀴 회전수를 계산하기 위해 초기화
																										Motor_Right();

																										direction++; // 오른쪽 회전의 경우 방향 설정 값을 증가시킨다

																										while(motorInterrupt2 < 30) { // 1회 회전시 바퀴 회전수 30만큼 회전한다.
																																		vTaskDelay(1/portTICK_RATE_MS); // motorInterrupt2 값을 읽어오기 위한 딜레이
																										}
																		}

																		/* 좌 우 초음파 센서의 값을 비교해 더 먼 쪽으로 회전한다.
																		 * 2. 왼쪽 센서값이 더 클경우
																		 * 왼쪽으로 회전
																		 */
																		if (( distance1 < distance3 )) {
																										motorInterrupt1 = 1;// 바퀴 회전수를 계산하기 위해 초기화
																										Motor_Left();

																										direction--; // 왼쪽 회전의 경우 방향 설정 값을 감소시킨다

																										while(motorInterrupt1 < 30) { // 1회 회전시 바퀴 회전수 30만큼 회전한다.
																																		vTaskDelay(1/portTICK_RATE_MS); // motorInterrupt2 값을 읽어오기 위한 딜레이
																										}
																		}
										}
										/*
										 * 방향이 양수일 경우 오른쪽으로 회전했다고 판단
										 * 즉, ubrain이 최종 진행 방향 대비 오른쪽으로 치우쳐져 있다.
										 * 따라서 장애물이 ubrain의 왼쪽에 있으므로 왼쪽 센서값을 읽어
										 * 50cm가 넘을 경우 장애물을 통과했다고 판단
										 */
										else if (  ( direction > 2 ) && ( distance3 > 50 ) ) {
																		Motor_Stop();

																		Motor_Left(); // 왼쪽에 공간이 생겼으므로 왼쪽으로 회전

																		direction--; // 왼쪽으로 회전했으니 방향 설정 값을 감소시킨다.

																		motorInterrupt1 = 1; // 바퀴 회전수를 계산하기 위해 초기화

																		while(motorInterrupt1 < 30) { // 1회 회전시 바퀴 회전수 30만큼 회전한다.
																										vTaskDelay(1/portTICK_RATE_MS); // motorInterrupt1 값을 읽어오기 위한 딜레이
																		}
										}

										/*
										 * 방향이 음일 경우 왼쪽으로 회전했다고 판단
										 * 즉, ubrain이 최종 진행 방향 대비 왼쪽으로 치우쳐져 있다.
										 * 따라서 장애물이 ubrain의 오른쪽에 있으므로 오른쪽 센서값을 읽어
										 * 50cm가 넘을 경우 장애물을 통과했다고 판단
										 */
										else if (  ( direction < -2 ) && ( distance1 > 50 ) ) {
																		Motor_Stop();

																		Motor_Right(); // 왼쪽에 공간이 생겼으므로 왼쪽으로 회전

																		direction++; // 왼쪽으로 회전했으니 방향 설정 값을 감소시킨다.

																		motorInterrupt2 = 1; // 바퀴 회전수를 계산하기 위해 초기화

																		while(motorInterrupt2 < 30) { // 1회 회전시 바퀴 회전수 30만큼 회전한다.
																										vTaskDelay(1/portTICK_RATE_MS); // motorInterrupt2 값을 읽어오기 위한 딜레이
																		}
										}
										vTaskDelay(3/portTICK_RATE_MS); // ubrain이 진행하는 동안 센서값을 각각 읽어와야 하므로 딜레이를 준다.
										Motor_Stop();
										Motor_Forward(); // 다시 전진 시킨다.
		}
}

/*
 * myTask2 초음파 센서값을 읽어와 저장하기 위한 task
 */
void myTask2 (void)  {
								osDelay(500); //500ms Delay : 최초 uwDiffCapture 들의 값이 0으로 읽어오는 것을 방지하기 위한 딜레이

								while (1) {
																/* Capture computation */
																osDelay(100); // 100ms Delay : 센서값을 읽어오기 위한 딜레이
																distance1 = uwDiffCapture1/58; // 오른쪽 초음파 센서값을 cm로 변환한 값
																distance2 = uwDiffCapture2/58; // 앞쪽 초음파 센서값을 cm로 변환한 값
																distance3 = uwDiffCapture3/58; // 왼쪽 초음파 센서값을 cm로 변환한 값

								}
}

/*
 * myTask3 적외선 센서값을 읽어와 저장하기 위한 task
 */
void myTask3(void) {
								//##-1- Configure the ADC peripheral #######################################*/
								AdcHandle1.Instance          = ADC3;// ADC 3????

								AdcHandle1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
								AdcHandle1.Init.Resolution = ADC_RESOLUTION12b;
								AdcHandle1.Init.ScanConvMode = DISABLE;
								// Mode ????
								AdcHandle1.Init.ContinuousConvMode = DISABLE;
								AdcHandle1.Init.DiscontinuousConvMode = DISABLE;
								AdcHandle1.Init.NbrOfDiscConversion = 0;
								AdcHandle1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
								AdcHandle1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
								AdcHandle1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
								AdcHandle1.Init.NbrOfConversion = 1;
								//DMA(Direct Memory Access)
								AdcHandle1.Init.DMAContinuousRequests = DISABLE;
								AdcHandle1.Init.EOCSelection = DISABLE;

								HAL_ADC_Init(&AdcHandle1);//ADC Initialized

								/*##-2- Configure ADC regular channel ######################################*/
								adcConfig1.Channel = ADC_CHANNEL_11; //a?? ????
								adcConfig1.Rank = 1;
								adcConfig1.SamplingTime = ADC_SAMPLETIME_480CYCLES; //???��? ??? ????
								adcConfig1.Offset = 0;

								HAL_ADC_ConfigChannel(&AdcHandle1, &adcConfig1);

								AdcHandle2.Instance          = ADC2;// ADC?��?

								AdcHandle2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
								AdcHandle2.Init.Resolution = ADC_RESOLUTION12b;
								AdcHandle2.Init.ScanConvMode = DISABLE;
								AdcHandle2.Init.ContinuousConvMode = DISABLE;
								AdcHandle2.Init.DiscontinuousConvMode = DISABLE;
								AdcHandle2.Init.NbrOfDiscConversion = 0;
								AdcHandle2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
								AdcHandle2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
								AdcHandle2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
								AdcHandle2.Init.NbrOfConversion = 1;
								AdcHandle2.Init.DMAContinuousRequests = DISABLE;
								AdcHandle2.Init.EOCSelection = DISABLE;

								HAL_ADC_Init(&AdcHandle2);

								/*##-2- Configure ADC regular channel ######################################*/
								adcConfig2.Channel = ADC_CHANNEL_14;
								adcConfig2.Rank = 1;
								adcConfig2.SamplingTime = ADC_SAMPLETIME_480CYCLES;
								adcConfig2.Offset = 0;

								HAL_ADC_ConfigChannel(&AdcHandle2, &adcConfig2);

								AdcHandle3.Instance          = ADC1;// ADC?��?

								AdcHandle3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
								AdcHandle3.Init.Resolution = ADC_RESOLUTION12b;
								AdcHandle3.Init.ScanConvMode = DISABLE;
								AdcHandle3.Init.ContinuousConvMode = DISABLE;
								AdcHandle3.Init.DiscontinuousConvMode = DISABLE;
								AdcHandle3.Init.NbrOfDiscConversion = 0;
								AdcHandle3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
								AdcHandle3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
								AdcHandle3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
								AdcHandle3.Init.NbrOfConversion = 1;
								AdcHandle3.Init.DMAContinuousRequests = DISABLE;
								AdcHandle3.Init.EOCSelection = DISABLE;

								HAL_ADC_Init(&AdcHandle3);

								/*##-2- Configure ADC regular channel ######################################*/
								adcConfig3.Channel = ADC_CHANNEL_15;
								adcConfig3.Rank = 1;
								adcConfig3.SamplingTime = ADC_SAMPLETIME_480CYCLES;
								adcConfig3.Offset = 0;
								HAL_ADC_ConfigChannel(&AdcHandle3, &adcConfig3);

								/* Infinite loop */
								while(1)
								{
																HAL_ADC_Start(&AdcHandle3);
																uhADCxForward = HAL_ADC_GetValue(&AdcHandle3);
																HAL_ADC_PollForConversion(&AdcHandle3, 0xFF);
																if(uhADCxForward >2000) uhADCxForward= 2000;
																else if(uhADCxForward<100) uhADCxForward = 100;

																// 우전방 적외선 센서 값 측정 후 저장
																HAL_ADC_Start(&AdcHandle1);
																uhADCxLeft = HAL_ADC_GetValue(&AdcHandle1);
																HAL_ADC_PollForConversion(&AdcHandle1, 0xFF);
																if(uhADCxLeft >2000) uhADCxLeft= 2000;
																else if(uhADCxLeft<100) uhADCxLeft = 100;
																rSensor = uhADCxRight;

																// 좌전방 적외선 센서 값 측정 후 저장
																HAL_ADC_Start(&AdcHandle2);
																uhADCxRight = HAL_ADC_GetValue(&AdcHandle2);
																HAL_ADC_PollForConversion(&AdcHandle2, 0xFF);
																if(uhADCxRight >2000) uhADCxRight= 2000;
																else if(uhADCxRight<100) uhADCxRight = 100;
																lSensor = uhADCxLeft;

																vTaskDelay(30 / portTICK_RATE_MS);
								}

}



/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{
								uint8_t ch;
								uint16_t Encoder1 = 0;
								uint16_t Encoder2 = 0;
								GPIO_InitTypeDef GPIO_InitStruct;

								HAL_Init();
								prvuBrainInit();

								printf("\r\n Lab 7. FreeRTOS by CNU : "); // Serial port initialized with 115200 bps, 2 stop bit, 8 bit char

								uint32_t uwPrescalerValue = 0;
								SystemClock_Config();

								BSP_COM1_Init();
								uwPrescalerValue = ((SystemCoreClock / 2) / 1000000) - 1;
								TimHandle1.Instance = TIM3;
								TimHandle1.Init.Period        = 0xFFFF;
								TimHandle1.Init.Prescaler     = uwPrescalerValue;
								TimHandle1.Init.ClockDivision = 0;
								TimHandle1.Init.CounterMode   = TIM_COUNTERMODE_UP;

								if(HAL_TIM_IC_Init(&TimHandle1) != HAL_OK) { Error_Handler(); }
								sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
								sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
								sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
								sICConfig.ICFilter    = 0;

								HAL_TIM_IC_ConfigChannel(&TimHandle1, &sICConfig, TIM_CHANNEL_1);
								HAL_TIM_IC_ConfigChannel(&TimHandle1, &sICConfig, TIM_CHANNEL_2);
								HAL_TIM_IC_ConfigChannel(&TimHandle1, &sICConfig, TIM_CHANNEL_3);
								HAL_TIM_IC_ConfigChannel(&TimHandle1, &sICConfig, TIM_CHANNEL_4);

								HAL_TIM_IC_Start_IT(&TimHandle1, TIM_CHANNEL_2);
								HAL_TIM_IC_Start_IT(&TimHandle1, TIM_CHANNEL_3);
								HAL_TIM_IC_Start_IT(&TimHandle1, TIM_CHANNEL_4);


								uwPrescalerValue = (SystemCoreClock / 2 / 131099) - 1;

								uwPrescalerValue = (SystemCoreClock / 2 / 131099) - 1;

								TimHandle2.Instance = TIM10;

								TimHandle2.Init.Prescaler     = uwPrescalerValue;
								TimHandle2.Init.Period        = 0xFFFF;
								TimHandle2.Init.ClockDivision = 0;
								TimHandle2.Init.CounterMode   = TIM_COUNTERMODE_UP;
								HAL_TIM_PWM_Init(&TimHandle2);
								//printf("\r\n 6 :%d",uwPrescalerValue/1);

								/*##-2- Configure the PWM channels #########################################*/
								/* Common configuration for all channels */
								sConfig.OCMode     = TIM_OCMODE_PWM1;
								sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
								sConfig.OCFastMode = TIM_OCFAST_DISABLE;

								/* Set the pulse value for channel 1 */
								sConfig.Pulse = 2;
								HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig, TIM_CHANNEL_1);


								/*##-3- Start PWM signals generation #######################################*/
								/* Start channel 3 */
								HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);

/*##-3- Start PWM signals generation #######################################*/
								/* Start channel 3 */
								HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);

								// PB2 ???? ???? ????? ???? GPIO ????
								__GPIOB_CLK_ENABLE();

								GPIO_InitStruct.Pin = GPIO_PIN_2;
								GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
								GPIO_InitStruct.Pull = GPIO_NOPULL;
								GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

								HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // MC_EN(PB2) ???? ????


								/*##-1- Configure the TIM peripheral #######################################*/
								/* Initialize TIMx peripheral as follow:
								 + Prescaler = (SystemCoreClock/2)/18000000
								 + Period = 1800  (to have an output frequency equal to 10 KHz)
								 + ClockDivision = 0
								 + Counter direction = Up
								 */
								sConfig1.OCMode     = TIM_OCMODE_PWM1;
								sConfig1.OCPolarity = TIM_OCPOLARITY_HIGH;
								sConfig1.OCFastMode = TIM_OCFAST_DISABLE;
								sConfig1.Pulse = 20000;

								Motor_Init();
								EXTILine_Config(); // Encoder Interrupt Setting

								Motor_Forward();

								// Task 등록 및 스케줄러 실행
								osThreadDef(myTask, myTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
								osThreadCreate(osThread(myTask), NULL);

								osThreadDef(myTask2, myTask2, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
								osThreadCreate(osThread(myTask2), NULL);

								osThreadDef(myTask3, myTask3, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
								osThreadCreate(osThread(myTask3), NULL);

								osKernelStart(NULL,NULL);

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
								if(htim->Instance == TIM3)
								{
																if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
																{
																								if((TIM3->CCER & TIM_CCER_CC2P) == 0)
																								{
																																/* Get the 1st Input Capture value */
																																uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
																																TIM3->CCER |= TIM_CCER_CC2P;
																								}
																								else if((TIM3->CCER & TIM_CCER_CC2P) == TIM_CCER_CC2P)
																								{
																																/* Get the 2nd Input Capture value */
																																uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

																																/* Capture computation */
																																if (uwIC2Value2 > uwIC2Value1)
																																{
																																								uwDiffCapture1 = (uwIC2Value2 - uwIC2Value1);
																																}
																																else if (uwIC2Value2 < uwIC2Value1)
																																{
																																								uwDiffCapture1 = ((0xFFFF - uwIC2Value1) + uwIC2Value2);
																																}
																																else
																																{
																																								uwDiffCapture1 = 0;
																																}
																																//printf("\r\n Value Right : %d cm", uwDiffCapture1/58);

																																uwFrequency = (2*HAL_RCC_GetPCLK1Freq()) / uwDiffCapture1;
																																TIM3->CCER &= ~TIM_CCER_CC2P;
																								}
																}

																if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
																{
																								if((TIM3->CCER & TIM_CCER_CC3P) == 0)
																								{
																																/* Get the 1st Input Capture value */
																																uwIC2Value3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
																																TIM3->CCER |= TIM_CCER_CC3P;
																								}
																								else if((TIM3->CCER & TIM_CCER_CC3P) == TIM_CCER_CC3P)
																								{
																																/* Get the 2nd Input Capture value */
																																uwIC2Value4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

																																/* Capture computation */
																																if (uwIC2Value4 > uwIC2Value3)
																																{
																																								uwDiffCapture2 = (uwIC2Value4 - uwIC2Value3);
																																}
																																else if (uwIC2Value4 < uwIC2Value3)
																																{
																																								uwDiffCapture2 = ((0xFFFF - uwIC2Value3) + uwIC2Value4);
																																}
																																else
																																{
																																								uwDiffCapture2 = 0;
																																}
																																//printf("\r\n Value Forward : %d cm", uwDiffCapture2/58);


																																uwFrequency = (2*HAL_RCC_GetPCLK1Freq()) / uwDiffCapture2;
																																TIM3->CCER &= ~TIM_CCER_CC3P;
																								}
																}

																if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
																{
																								if((TIM3->CCER & TIM_CCER_CC4P) == 0)
																								{
																																/* Get the 1st Input Capture value */
																																uwIC2Value5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
																																TIM3->CCER |= TIM_CCER_CC4P;
																								}
																								else if((TIM3->CCER & TIM_CCER_CC4P) == TIM_CCER_CC4P)
																								{
																																/* Get the 2nd Input Capture value */
																																uwIC2Value6 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

																																/* Capture computation */
																																if (uwIC2Value6 > uwIC2Value5)
																																{
																																								uwDiffCapture3 = (uwIC2Value6 - uwIC2Value5);
																																}
																																else if (uwIC2Value6 < uwIC2Value5)
																																{
																																								uwDiffCapture3 = ((0xFFFF - uwIC2Value5) + uwIC2Value6);
																																}
																																else
																																{
																																								uwDiffCapture3 = 0;
																																}
																																//printf("\r\n Value Left: %d cm", uwDiffCapture3/58);

																																uwFrequency = (2*HAL_RCC_GetPCLK1Freq()) / uwDiffCapture3;
																																TIM3->CCER &= ~TIM_CCER_CC4P;
																								}
																}
								}
}

static void Error_Handler(void)
{
								/* Turn LED3 on */
								BSP_LED_On(LED3);
								while(1)
								{
								}
}


/*

   Function : prvuBrainInit ( )
   Description : move old ubrain init codes here for Lab 7.
   Date : 13, May. 2015
   Author : HSK

 */

void prvuBrainInit(void)
{
								HAL_Init();

								/* Configure the system clock to have a system clock = 180 Mhz */
								SystemClock_Config();

								/* Initialize LEDs */
								BSP_LED_Init(LED1);
								BSP_LED_Init(LED2);
								BSP_LED_Init(LED3);


								BSP_COM1_Init(); // USART3 ???? ???

}


/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 180000000
 *            HCLK(Hz)                       = 180000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 25000000
 *            PLL_M                          = 25
 *            PLL_N                          = 360
 *            PLL_P                          = 2
 *            PLL_Q                          = 7
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 5
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void)
{
								RCC_ClkInitTypeDef RCC_ClkInitStruct;
								RCC_OscInitTypeDef RCC_OscInitStruct;

								/* Enable Power Control clock */
								__PWR_CLK_ENABLE();

								/* The voltage scaling allows optimizing the power consumption when the device is
								   clocked below the maximum system frequency, to update the voltage scaling value
								   regarding system frequency refer to product datasheet.  */
								__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

								/* Enable HSE Oscillator and activate PLL with HSE as source */
								RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
								RCC_OscInitStruct.HSEState = RCC_HSE_ON;
								RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
								RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
								RCC_OscInitStruct.PLL.PLLM = 25;
								RCC_OscInitStruct.PLL.PLLN = 360;
								RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
								RCC_OscInitStruct.PLL.PLLQ = 7;
								HAL_RCC_OscConfig(&RCC_OscInitStruct);

								HAL_PWREx_ActivateOverDrive();

								/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
								   clocks dividers */
								RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
								RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
								RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
								RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
								RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
								HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}


void Motor_Init(void)
{
								GPIO_InitTypeDef GPIO_InitStruct;

								__GPIOB_CLK_ENABLE();

								GPIO_InitStruct.Pin = GPIO_PIN_2;
								GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
								GPIO_InitStruct.Pull = GPIO_NOPULL;
								GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

								HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

//  --------------------------------------- Motor ----------------------------------------------------START
/* Compute the prescaler value to have TIM4 counter clock equal to 18 MHz */
								uwPrescalerValue1 = (SystemCoreClock / 18000000) - 1;

/* Compute the prescaler value to have TIM4 counter clock equal to 18 MHz */
								uwPrescalerValue2 = ((SystemCoreClock/2) / 18000000) - 1;

								TimHandle3.Instance = TIM8;
								TimHandle3.Init.Period        = PULSE1_VALUE;
								TimHandle3.Init.Prescaler     = uwPrescalerValue1;
								TimHandle3.Init.ClockDivision = 0;
								TimHandle3.Init.CounterMode   = TIM_COUNTERMODE_UP;
								HAL_TIM_PWM_Init(&TimHandle3);


/*## Configure the PWM channels #########################################*/
/* Common configuration for all channels */
								sConfig1.OCMode     = TIM_OCMODE_PWM1;
								sConfig1.OCPolarity = TIM_OCPOLARITY_HIGH;
								sConfig1.OCFastMode = TIM_OCFAST_DISABLE;
								sConfig1.Pulse = PULSE1_VALUE;
								HAL_TIM_PWM_ConfigChannel(&TimHandle3, &sConfig1, TIM_CHANNEL_1);
								HAL_TIM_PWM_ConfigChannel(&TimHandle3, &sConfig1, TIM_CHANNEL_2);

								TimHandle4.Instance = TIM4;
								TimHandle4.Init.Period        = PULSE1_VALUE;
								TimHandle4.Init.Prescaler     = uwPrescalerValue2;
								TimHandle4.Init.ClockDivision = 0;
								TimHandle4.Init.CounterMode   = TIM_COUNTERMODE_UP;
								HAL_TIM_PWM_Init(&TimHandle4);

/*## Configure the PWM channels #########################################*/
/* Common configuration for all channels */
								sConfig2.OCMode     = TIM_OCMODE_PWM1;
								sConfig2.OCPolarity = TIM_OCPOLARITY_HIGH;
								sConfig2.OCFastMode = TIM_OCFAST_DISABLE;
								sConfig2.Pulse = PULSE1_VALUE;
								HAL_TIM_PWM_ConfigChannel(&TimHandle4, &sConfig2, TIM_CHANNEL_1);
								HAL_TIM_PWM_ConfigChannel(&TimHandle4, &sConfig2, TIM_CHANNEL_2);
}



void Motor_Forward(void)
{
								HAL_TIM_PWM_Start(&TimHandle3, TIM_CHANNEL_1);
								HAL_TIM_PWM_Start(&TimHandle4, TIM_CHANNEL_2);
}

void Motor_Backward(void)
{
								HAL_TIM_PWM_Start(&TimHandle3, TIM_CHANNEL_2);
								HAL_TIM_PWM_Start(&TimHandle4, TIM_CHANNEL_1);
}

void Motor_Left(void)
{
								HAL_TIM_PWM_Start(&TimHandle3, TIM_CHANNEL_2);
								HAL_TIM_PWM_Start(&TimHandle4, TIM_CHANNEL_2);
}

void Motor_Right(void)
{
								HAL_TIM_PWM_Start(&TimHandle3, TIM_CHANNEL_1);
								HAL_TIM_PWM_Start(&TimHandle4, TIM_CHANNEL_1);
}

void Motor_Stop(void)
{
								HAL_TIM_PWM_Stop(&TimHandle3, TIM_CHANNEL_1);
								HAL_TIM_PWM_Stop(&TimHandle3, TIM_CHANNEL_2);
								HAL_TIM_PWM_Stop(&TimHandle4, TIM_CHANNEL_1);
								HAL_TIM_PWM_Stop(&TimHandle4, TIM_CHANNEL_2);
}

void Motor_Speed_Up_Config(void)
{
								sConfig1.Pulse  = sConfig1.Pulse + 100;
								sConfig2.Pulse  = sConfig2.Pulse + 100;
								TIM3->CCR1 = sConfig1.Pulse;
								TIM3->CCR2 = sConfig1.Pulse;
								TIM4->CCR1 = sConfig2.Pulse;
								TIM4->CCR2 = sConfig2.Pulse;
}

void Motor_Speed_Down_Config(void)
{
								sConfig1.Pulse  = sConfig1.Pulse - 100;
								sConfig2.Pulse  = sConfig2.Pulse - 100;
								TIM3->CCR1 = sConfig1.Pulse;
								TIM3->CCR2 = sConfig1.Pulse;
								TIM4->CCR1 = sConfig2.Pulse;
								TIM4->CCR2 = sConfig2.Pulse;
}
/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 180000000
 *            HCLK(Hz)                       = 180000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 25000000
 *            PLL_M                          = 25
 *            PLL_N                          = 360
 *            PLL_P                          = 2
 *            PLL_Q                          = 7
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 5
 * @param  None
 * @retval None
 */


#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
								/* User can add his own implementation to report the file name and line number,
								   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

								/* Infinite loop */
								while (1)
								{
								}
}

#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
								switch(GPIO_Pin)
								{
								case GPIO_PIN_15:
																encoder_right = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3);
																if(encoder_right == 0)
																{
																								if(motorInterrupt1==0)
																																motorInterrupt1=20000;
																								motorInterrupt1--;
																								encoder_right = 3;

																}
																else if(encoder_right == 1)
																{
																								motorInterrupt1++;
																								encoder_right = 3;
																								if(motorInterrupt1==20000)
																																motorInterrupt1=0;
																}
																break;

								case GPIO_PIN_4:
																encoder_left = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);
																if(encoder_left == 0)
																{
																								motorInterrupt2++;
																								encoder_left =3;
																								if(motorInterrupt2==20000)
																																motorInterrupt2=0;
																}
																else if(encoder_left == 1)
																{
																								if(motorInterrupt2==0)
																																motorInterrupt2=20000;
																								motorInterrupt2--;
																								encoder_left =3;

																}
																break;
								}
}


/**
 * @brief  Configures EXTI Line (connected to PA15, PB3, PB4, PB5 pin) in interrupt mode
 * @param  None
 * @retval None
 */
static void EXTILine_Config(void)
{
								GPIO_InitTypeDef GPIO_InitStructure;

								/* Enable GPIOA clock */
								__GPIOA_CLK_ENABLE();

								/* Configure PA0 pin as input floating */
								GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
								GPIO_InitStructure.Pull = GPIO_NOPULL;
								GPIO_InitStructure.Pin = GPIO_PIN_15;
								HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

/* Enable and set EXTI Line0 Interrupt to the lowest priority */
								HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
								HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


/* Enable GPIOB clock */
								__GPIOB_CLK_ENABLE();

								/* Configure PA0 pin as input floating */
								GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
								GPIO_InitStructure.Pull = GPIO_NOPULL;
								GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
								HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

								/* Enable and set EXTI Line0 Interrupt to the lowest priority */
								HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
								HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}
#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
								/* User can add his own implementation to report the file name and line number,
								   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

								/* Infinite loop */
								while (1)
								{}
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
