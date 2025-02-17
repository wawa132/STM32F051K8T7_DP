/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_NUM 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Read_Sensor_ID(void);
void ADC_CT_PRO(uint8_t i);
void ADC_TM_PRO(void);
void ADC_DP_PRO(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char rxData, rxBuff[30], usrData, usrBuff[30], txBuff[30], Buff[255];
uint8_t adcFlag, ctFlag, tmpDpFlag, txFlag, I2CBuff[16], ReadI2C[16], senID[4];
uint16_t adcCnt = 300, adcRaw[40], adcData[4], adcAvg[4], adcSqrt[2], adcSqrAvg[2];
uint16_t ctCnt[2];
uint32_t adcRawSum[4], adcSum[4], adcSqr[2], adcSqrSum[2];
int32_t adcValue[4];

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

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
    MX_ADC_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();
    MX_USART1_UART_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */
    Read_Sensor_ID();
    HAL_ADCEx_Calibration_Start(&hadc);
    HAL_ADC_Start_DMA(&hadc, (uint32_t *)&adcRaw, 40);
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&rxData, 1);
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&usrData, 1);
    HAL_TIM_Base_Start_IT(&htim3);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if (adcFlag)
        {
            adcRawSum[0] = adcRaw[0] + adcRaw[4] + adcRaw[8] + adcRaw[12] + adcRaw[16] + adcRaw[20] + adcRaw[24] + adcRaw[28] + adcRaw[32] + adcRaw[36];
            adcRawSum[1] = adcRaw[1] + adcRaw[5] + adcRaw[9] + adcRaw[13] + adcRaw[17] + adcRaw[21] + adcRaw[25] + adcRaw[29] + adcRaw[33] + adcRaw[37];
            adcRawSum[2] = adcRaw[2] + adcRaw[6] + adcRaw[10] + adcRaw[14] + adcRaw[18] + adcRaw[22] + adcRaw[26] + adcRaw[30] + adcRaw[34] + adcRaw[38];
            adcRawSum[3] = adcRaw[3] + adcRaw[7] + adcRaw[11] + adcRaw[15] + adcRaw[19] + adcRaw[23] + adcRaw[27] + adcRaw[31] + adcRaw[35] + adcRaw[39];

            adcData[0] = adcRawSum[0] / 10;
            adcData[1] = adcRawSum[1] / 10;
            adcData[2] = adcRawSum[2] / 10;
            adcData[3] = adcRawSum[3] / 10;

            adcRawSum[0] = 0;
            adcRawSum[1] = 0;
            adcRawSum[2] = 0;
            adcRawSum[3] = 0;

            adcSum[0] += adcData[0];
            adcSum[1] += adcData[1];
            adcSum[2] += adcData[2];
            adcSum[3] += adcData[3];

            if (adcCnt > 0)
                adcCnt--;

            if (adcCnt == 0)
            {
                adcAvg[0] = adcSum[0] / 300;
                adcAvg[1] = adcSum[1] / 300;
                adcAvg[2] = adcSum[2] / 300;
                adcAvg[3] = adcSum[3] / 300;

                adcSum[0] = 0;
                adcSum[1] = 0;
                adcSum[2] = 0;
                adcSum[3] = 0;

                tmpDpFlag = 1;

                adcCnt = 300;
            }

            adcFlag = 0;
            ctFlag = 1;
        }

        if (ctFlag)
        {
            ADC_CT_PRO(0); // ct1
            ADC_CT_PRO(1); // ct2
            ctFlag = 0;
        }

        if (tmpDpFlag)
        {
            ADC_TM_PRO(); // tmp
            ADC_DP_PRO(); // dP

            tmpDpFlag = 0;
        }

        if (txFlag)
        { // ADC별 ID와 데이터 전송
            snprintf(Buff, sizeof(Buff),
                     "ADC1(ID:%2d): %3ld, ADC2(ID:%2d): %3ld, ADC3(ID:%2d): %3ld, ADC4(ID:%2d): %3ld\r\n",
                     senID[0], adcValue[0], senID[1], adcValue[1], senID[2], adcValue[2], senID[3], adcValue[3]);
            HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&Buff, strlen(Buff));

            txFlag = 0;
        }
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
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI14CalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        static uint16_t cnt;
        // 1msec 간격 ADC
        adcFlag = 1;

        cnt++;

        if (cnt >= 1000)
        {
            txFlag = 1;
            cnt = 0;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        rxBuff[0] = rxBuff[1];
        rxBuff[1] = rxBuff[2];
        rxBuff[2] = rxBuff[3];
        rxBuff[3] = rxBuff[4];
        rxBuff[4] = rxData;

        // 센서데이터 요청 ex.R01(아이디 01번 센서 데이터 요청)
        if (rxBuff[0] == 'R' && rxBuff[3] == 0x0D && rxBuff[4] == 0x0A)
        {
            uint8_t ID;

            ID = (rxBuff[1] - '0') * 10;
            ID += rxBuff[2] - '0';

            for (uint8_t i = 0; i < ADC_NUM; i++)
            {
                if (senID[i] == ID)
                {
                    uint16_t checksum;

                    if (adcValue[i] < 0)
                    {
                        uint16_t data = abs(adcValue[i]);
                        snprintf(txBuff, sizeof(txBuff), "A%02d-%02d", senID[i], data);
                    }
                    else
                        snprintf(txBuff, sizeof(txBuff), "A%02d%03ld", senID[i], adcValue[i]);

                    checksum = txBuff[0] + txBuff[1] + txBuff[2] + txBuff[3] + txBuff[4] + txBuff[5];

                    txBuff[6] = checksum >> 8;
                    txBuff[7] = checksum & 0xFF;
                    txBuff[8] = 0x0D;
                    txBuff[9] = 0x0A;

                    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&txBuff, 10);
                    break;
                }
            }
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&rxData, 1);
    }

    if (huart->Instance == USART1)
    {
        char tmpBuff[50];

        usrBuff[0] = usrBuff[1];
        usrBuff[1] = usrBuff[2];
        usrBuff[2] = usrBuff[3];
        usrBuff[3] = usrBuff[4];
        usrBuff[4] = usrBuff[5];
        usrBuff[5] = usrData;

        // ADC별 ID 부여 ex.A101(ADC1 = 01번) or A215(ADC2 = 15번)
        if (usrBuff[0] == 'A' && usrBuff[4] == 0x0D && usrBuff[5] == 0x0A)
        {
            I2CBuff[0] = (usrBuff[2] - '0') * 10;
            I2CBuff[0] += (usrBuff[3] - '0');

            if (usrBuff[1] == '1')
            {
                senID[0] = I2CBuff[0];
                HAL_I2C_Mem_Write(&hi2c1, 0xA0, 0x40, I2C_MEMADD_SIZE_8BIT, &I2CBuff[0], 1, 1);
                snprintf(tmpBuff, sizeof(tmpBuff), "ADC1 ID: %2d\r\n", I2CBuff[0]);
            }
            else if (usrBuff[1] == '2')
            {
                senID[1] = I2CBuff[0];
                HAL_I2C_Mem_Write(&hi2c1, 0xA0, 0x42, I2C_MEMADD_SIZE_8BIT, &I2CBuff[0], 1, 1);
                snprintf(tmpBuff, sizeof(tmpBuff), "ADC2 ID: %2d\r\n", I2CBuff[0]);
            }
            else if (usrBuff[1] == '3')
            {
                senID[2] = I2CBuff[0];
                HAL_I2C_Mem_Write(&hi2c1, 0xA0, 0x44, I2C_MEMADD_SIZE_8BIT, &I2CBuff[0], 1, 1);
                snprintf(tmpBuff, sizeof(tmpBuff), "ADC3 ID: %2d\r\n", I2CBuff[0]);
            }
            else if (usrBuff[1] == '4')
            {
                senID[3] = I2CBuff[0];
                HAL_I2C_Mem_Write(&hi2c1, 0xA0, 0x46, I2C_MEMADD_SIZE_8BIT, &I2CBuff[0], 1, 1);
                snprintf(tmpBuff, sizeof(tmpBuff), "ADC4 ID: %2d\r\n", I2CBuff[0]);
            }
            else
            {
                snprintf(tmpBuff, sizeof(tmpBuff), "Wrong Protocol: %s", usrBuff);
            }

            HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tmpBuff, strlen(tmpBuff));
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&usrData, 1);
    }
}

void Read_Sensor_ID(void)
{
    HAL_I2C_Mem_Read(&hi2c1, 0xA0, 0x40, I2C_MEMADD_SIZE_8BIT, &ReadI2C[0], 16, 1);

    for (uint8_t i = 0; i < ADC_NUM; i++)
    {
        senID[i] = ReadI2C[i * 2];
    }
}

void ADC_CT_PRO(uint8_t i)
{ // 전류 연산

    ctCnt[i]++;

    if (adcData[i] >= adcAvg[i])
        adcSqr[i] = adcData[i] - adcAvg[i];
    else
        adcSqr[i] = adcAvg[i] - adcData[i];

    adcSqr[i] *= adcSqr[i];
    adcSqrSum[i] += adcSqr[i];

    if (ctCnt[i] >= 300)
    {
        adcSqrAvg[i] = adcSqrSum[i] / 300;
        adcSqrSum[i] = 0;

        adcSqrt[i] = sqrt(adcSqrAvg[i]);
        adcValue[i] = adcSqrt[i] * 100;
        adcValue[i] /= 150;

        if (adcValue[i] > 999)
            adcValue[i] = 999;
        else if (adcValue[i] < -99)
            adcValue[i] = -99;

        // 환경공단 테스트 설정 ADC1: 배출시설(E0101) 보조, ADC2: 없음.
        // 0으로 설정
        adcValue[i] = 0;

        ctCnt[i] = 0;
    }
}

void ADC_TM_PRO(void)
{ // 온도 연산
    adcValue[2] = (adcAvg[2] - 600) * 1400;
    adcValue[2] /= 2420;
    adcValue[2] -= 425;
    adcValue[2] /= 10;

    if (adcValue[2] < -99)
        adcValue[2] = -99;
    else if (adcValue[2] > 999)
        adcValue[2] = 99;

    // 환경공단 테스트 설정 ADC3: 방지시설(P0301) 온도
    // 0으로 설정
    adcValue[2] = 0;
}

void ADC_DP_PRO(void)
{ // 차압 연산
    adcValue[3] = (adcAvg[3] - 2048) * 3000;
    adcValue[3] /= 4656;

    if (adcValue[3] > 999)
        adcValue[3] = 999;
    else if (adcValue[3] < -99)
        adcValue[3] = -99;

    // 환경공단 테스트 설정 ADC4: 방지시설(P0301) 차압
    // 0으로 설정
    adcValue[3] = 0;
}
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

#ifdef USE_FULL_ASSERT
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
    printf("Wrong parameters value: file %s on line %ld\r\n", file, line);
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
