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
#include "gpio.h"
#include "mecanum/mecanum.h"
#include "ps2/PS2.h"
#include "string.h"
#include "tim.h"
#include "usart.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct
{
    GPIO_TypeDef *IN1_Port;
    uint16_t IN1_Pin;
    GPIO_TypeDef *IN2_Port;
    uint16_t IN2_Pin;
    TIM_HandleTypeDef *htim;
    uint32_t Channel;
} Motor_t;
Motor_t motor[4] = {
    {LF_1_AIN2_GPIO_Port, LF_1_AIN2_Pin, LF_1_AIN1_GPIO_Port, LF_1_AIN1_Pin, &htim2, TIM_CHANNEL_2},
    {RF_1_BIN1_GPIO_Port, RF_1_BIN1_Pin, RF_1_BIN2_GPIO_Port, RF_1_BIN2_Pin, &htim2, TIM_CHANNEL_1},
    {LB_2_BIN2_GPIO_Port, LB_2_BIN2_Pin, LB_2_BIN1_GPIO_Port, LB_2_BIN1_Pin, &htim1, TIM_CHANNEL_4},
    {RB_2_AIN1_GPIO_Port, RB_2_AIN1_Pin, RB_2_AIN2_GPIO_Port, RB_2_AIN2_Pin, &htim1, TIM_CHANNEL_3},
};

void Motor_Init(void)
{
}
void Motor_SetSpeed(uint8_t id, int16_t pwm)
{
    if (id > 3)
        return;
    if (pwm > 0)
    {
        if (HAL_GPIO_ReadPin(motor[id].IN1_Port, motor[id].IN1_Pin) == GPIO_PIN_RESET)
        {
            HAL_GPIO_WritePin(motor[id].IN1_Port, motor[id].IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor[id].IN2_Port, motor[id].IN2_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(motor[id].htim, motor[id].Channel, pwm);
        }
        
    }
    else if (pwm < 0)
    {
        if (HAL_GPIO_ReadPin(motor[id].IN1_Port, motor[id].IN1_Pin) == GPIO_PIN_SET)
        {
            HAL_GPIO_WritePin(motor[id].IN1_Port, motor[id].IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor[id].IN2_Port, motor[id].IN2_Pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(motor[id].htim, motor[id].Channel, -pwm);
        }
        
    }

    else
    {
        if (HAL_GPIO_ReadPin(motor[id].IN1_Port, motor[id].IN1_Pin) != GPIO_PIN_RESET || HAL_GPIO_ReadPin(motor[id].IN2_Port, motor[id].IN2_Pin) != GPIO_PIN_RESET)
        {
            HAL_GPIO_WritePin(motor[id].IN1_Port, motor[id].IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor[id].IN2_Port, motor[id].IN2_Pin, GPIO_PIN_RESET);
             __HAL_TIM_SET_COMPARE(motor[id].htim, motor[id].Channel, 0);
        }
       
    }
}
double positionChangeSpeed = 0.2;
double UpDownPosition = 20;
double LeftRightPosition = 15;
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
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART3_UART_Init();
    PS2Buttons PS2;
    PS2_Init(&htim3, &PS2);

    /* USER CODE BEGIN 2 */

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // Update PS2 controller state
        PS2_Update();

        if (UpDownPosition > 100)
        {
            UpDownPosition = 100;
        }
        if (UpDownPosition < 0)
        {
            UpDownPosition = 0;
        }
        if (LeftRightPosition > 100)
        {
            LeftRightPosition = 100;
        }
        if (LeftRightPosition < 0)
        {
            LeftRightPosition = 0;
        }

        // Convert PS2 analog stick values (0-255) to normalized stick values (-1 to 1)
        // LX, LY control left stick (movement)
        // RX, RY control right stick (rotation, LY also affects forward/backward)
        float lx = ((float)PS2.LX - 128.00) / 128.0f; // Left stick X: -1 to 1 (left/right strafe)
        float ly = (128.00 - (float)PS2.LY) / 128.0f; // Left stick Y: -1 to 1 (forward/backward)
        float rx = ((float)PS2.RX - 128.00) / 128.0f; // Right stick X: -1 to 1 (rotation)

        // Deadzone
        if (fabsf(lx) < 0.1f)
            lx = 0.0f;
        if (fabsf(ly) < 0.1f)
            ly = 0.0f;
        if (fabsf(rx) < 0.1f)
            rx = 0.0f;

        // Scale velocities based on stick deflection magnitude
        // Adjust these coefficients to tune speed responsiveness
        float vx = ly * 600.0f;     // Forward velocity: max ~30
        float vy = lx * 600.0f;     // Lateral velocity: max ~30
        float omega = rx * 4000.0f; // Angular velocity: max ~150 (Increased to match translation speed)

        // Calculate motor PWM values using mecanum wheel kinematics
        // Fixed values for testing
        // float vx = 3.0f;
        // float vy = 3.0f;
        // float omega = 3.0f;
        int16_t motor_pwm[4];
        Mecanum_Calc(vx, vy, omega, motor_pwm);

        // Set motor speeds (this function handles the sign and applies PWM)
        
        
            Motor_SetSpeed(0, motor_pwm[0]);
            Motor_SetSpeed(1, motor_pwm[1]);
            Motor_SetSpeed(2, motor_pwm[2]);
            Motor_SetSpeed(3, motor_pwm[3]);    
        
        // HAL_GPIO_WritePin(LF_1_AIN2_GPIO_Port, LF_1_AIN2_Pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(LF_1_AIN1_GPIO_Port, LF_1_AIN1_Pin, GPIO_PIN_RESET);
        // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 35560);

        double dutyUpDown = ((10.00 * UpDownPosition / 100) + 2.50) / 100.00 * 20000.00;
        double dutyLeftRight = ((10.00 * LeftRightPosition / 100) + 2.50) / 100.00 * 20000.00;
        Motor_Init();
        if (PS2.UP)
        {
            UpDownPosition += positionChangeSpeed;
        }
        else if (PS2.DOWN)
        {
            UpDownPosition -= positionChangeSpeed;
        }
        if (PS2.LEFT)
        {
            LeftRightPosition -= positionChangeSpeed;
        }
        else if (PS2.RIGHT)
        {
            LeftRightPosition += positionChangeSpeed;
        }

        // if (PS2.CIRCLE)
        // {
        //     Motor_SetSpeed(1, 35580);
        // }else{
        //     Motor_SetSpeed(1, 0);
        // }

        if (PS2.SQUARE)
        {
            UpDownPosition = 20;
            LeftRightPosition = 15;
        }

        if (PS2.R2)
        {
            HAL_GPIO_WritePin(BOOST_GPIO_Port, BOOST_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(BOOST_GPIO_Port, BOOST_Pin, GPIO_PIN_RESET);
        }

        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, dutyUpDown);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, dutyLeftRight);

        // Control loop frequency: 50ms = 20Hz
        // Adjust if needed for different control responsiveness
        HAL_Delay(5);
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

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
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
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
