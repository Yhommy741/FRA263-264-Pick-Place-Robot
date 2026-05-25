/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @brief   Robot library — RobotConfig.h driven, Gripper integrated
 *
 * All commands and getters are in OUTPUT shaft space.
 * With N=5: Robot_Move(2π) → output shaft moves 2π rad, motor moves 10π rad.
 *
 * Live-Expression Debug Commands (Motor):
 *   dbg_cmd        — write command ID (see table below)
 *   dbg_target     — target angle  [rad, OUTPUT shaft]
 *   dbg_jog_speed  — jog speed magnitude  [rad/s, output shaft, >0]
 *   dbg_jog_step   — step jog magnitude   [rad,   output shaft, >0]
 *   robot.state    — 0=IDLE 1=MOVE 2=JOG_VEL 3=JOG_STEP 4..7=HOMING 8=ESTOP
 *   robot.theta    — current position [rad, OUTPUT shaft]
 *   robot.omega    — current velocity [rad/s, OUTPUT shaft]
 *
 * Motor Commands (dbg_cmd):
 *   1  Robot_Move       smooth constrained move to dbg_target
 *   2  Robot_Stop       stop and hold current position
 *   3  Robot_EStop      cut motor immediately
 *   4  Robot_SetHome    zero here with extra offset = dbg_home_offset [rad]
 *   5  Robot_Home       homing sequence → returns to 0
 *   6  Robot_JogVel CCW +dbg_jog_speed
 *   7  Robot_JogVel CW  -dbg_jog_speed
 *   8  Robot_JogStep CCW +dbg_jog_step
 *   9  Robot_JogStep CW  -dbg_jog_step
 *
 * Live-Expression Debug Commands (Gripper — also via dbg_cmd):
 *   10  Robot_Gripper_MoveUp    pulse elevator-up   output 100 ms
 *   11  Robot_Gripper_MoveDown  pulse elevator-down output 100 ms
 *   12  Robot_Gripper_Open      pulse claw-open     output 100 ms
 *   13  Robot_Gripper_Close     pulse claw-close    output 100 ms
 *
 * Gripper State Reads (read-only live expressions):
 *   Robot_Gripper_GetUpState(&robot)    — GRP_STATE_HIGH when top LS active
 *   Robot_Gripper_GetDownState(&robot)  — GRP_STATE_HIGH when bottom LS active
 *   Robot_Gripper_GetClawState(&robot)  — GRP_STATE_HIGH when claw LS active
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "Robot.h"
#include "JoystickInterface.h"
#include "BaseSystemInterface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* All robot/gripper parameters are in RobotConfig.h — nothing to define here */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

Robot_t             robot;
JoystickInterface_t joystick;
BaseSystemInterface_t BaseSystem;

uint8_t Emergency_State = 0;
uint8_t Mode_State      = 0;

/* ── Motor debug live-expressions ───────────────────────────────────────── */
volatile uint8_t dbg_cmd         = 0;
volatile float   dbg_target      = 0.0f;
volatile float   dbg_jog_speed   = 1.0f;
volatile float   dbg_jog_step    = 0.1f;
volatile float   dbg_home_offset = 0.0f;  /* extra home offset [rad] for Robot_SetHome */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration -------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */

  /* ── BaseSystemInterface (Modbus submodule embedded) ─────────────────── */
  BaseSystemInterface_Init(&BaseSystem, &huart2, &htim16, 21);

  /* ── Robot (motor + encoder + Kalman + PID + trajectory + gripper) ───── */
  Robot_Init(&robot);

  /* ── Start control timer & joystick ─────────────────────────────────── */
  HAL_TIM_Base_Start_IT(&htim3);
  JoystickInterface_Init(&joystick, &huart3, GPIOB, GPIO_PIN_14);

  /* USER CODE END 2 */

  /* Infinite loop -----------------------------------------------------------*/
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    BaseSystemInterface_Update(&BaseSystem);
    BaseSystem_Dispatch(&BaseSystem, &robot);

    Emergency_State = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
    Mode_State      = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);

    /* ── Motor commands (via dbg_cmd live-expression) ─────────────────── */
    uint8_t cmd = dbg_cmd;
    if (cmd != 0)
    {
        dbg_cmd = 0;
        switch (cmd)
        {
        case 1:  Robot_Move    (&robot, dbg_target);      break;  /* move to angle    */
        case 2:  Robot_Stop    (&robot);                  break;  /* stop & hold      */
        case 3:  Robot_EStop   (&robot);                  break;  /* emergency stop   */
        case 4:  Robot_SetHome (&robot, dbg_home_offset);    break;  /* zero here + offset  */
        case 5:  Robot_Home    (&robot);                  break;  /* full homing      */
        case 6:  Robot_JogVel  (&robot,  dbg_jog_speed);  break;  /* jog CCW          */
        case 7:  Robot_JogVel  (&robot, -dbg_jog_speed);  break;  /* jog CW           */
        case 8:  Robot_JogStep (&robot,  dbg_jog_step);   break;  /* step CCW         */
        case 9:  Robot_JogStep (&robot, -dbg_jog_step);   break;  /* step CW           */
        case 10: Robot_Gripper_MoveUp  (&robot);           break;  /* elevator up       */
        case 11: Robot_Gripper_MoveDown(&robot);           break;  /* elevator down     */
        case 12: Robot_Gripper_Open    (&robot);           break;  /* open claw         */
        case 13: Robot_Gripper_Close   (&robot);           break;  /* close claw        */
        default: break;
        }
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

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN            = 85;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    Error_Handler();
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3)
    {
        Robot_Update(&robot, htim);
    }
    else if (htim == &htim16)
    {
        BaseSystem.modbus.Flag_T35TimeOut = 1;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    Robot_EXTI_Callback(&robot, GPIO_Pin);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    JoystickInterface_RxCpltCallback(&joystick, huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        if (HAL_UART_GetError(huart) == HAL_UART_ERROR_RTO)
        {
            BaseSystem.modbus.Flag_T15TimeOut = 1;
            __HAL_TIM_SET_COUNTER(&htim16, 0);
            __HAL_TIM_ENABLE(&htim16);
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  Error handler.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
