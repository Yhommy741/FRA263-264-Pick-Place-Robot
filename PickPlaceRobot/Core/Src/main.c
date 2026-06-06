/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @brief   Robot library — RobotConfig.h driven, Gripper integrated
 *
 * System State Machine:
 *
 * ┌─────────────────────────────────────────────────────────────┐
 * │                     SYS_STATE_ESTOP                         │
 * │  Motor cut, heartbeat only, no commands accepted            │
 * └───────────────┬─────────────────────────────────────────────┘
 * E-Stop    │ released → SYS_STATE_RESET
 * pressed   │
 * ┌───────────────▼─────────────────────────────────────────────┐
 * │                     SYS_STATE_RESET                         │
 * │  Re-init all subsystems, then transition to AUTO or MANUAL  │
 * └───────────────┬─────────────────────────────────────────────┘
 * │ immediately
 * ┌──────────▼──────────┐
 * │                     │  Mode_State pin
 * ▼                     ▼
 * SYS_STATE_AUTO       SYS_STATE_MANUAL
 * Modbus cmds only     Joystick cmds only
 * (heartbeat + data    (heartbeat + data
 * to PC always)        to PC always)
 * │                     │
 * └──────┬──────────────┘
 * │ Mode_State pin changes → switch between AUTO/MANUAL
 * │ E-Stop pressed    → SYS_STATE_ESTOP
 * │ REG_SOFT_STOP=1   → SYS_STATE_SOFT_ESTOP
 *
 * ┌─────────────────────────────────────────────────────────────┐
 * │                  SYS_STATE_SOFT_ESTOP                       │
 * │  Motor cut, heartbeat only, NO exit — hardware reset only   │
 * └─────────────────────────────────────────────────────────────┘
 *
 * Emergency_State = 1  →  button pressed  (active-low PC13, inverted)
 * Mode_State      = 1  →  manual mode     (active-low PB4,  inverted)
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "Robot.h"
#include "RobotConfig.h"
#include "BaseSystemInterface.h"
#include "JoystickInterface.h"
#include "TaskManager.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Convert radians → degrees for Modbus write-back.
 * BaseSystemInterface_Update() multiplies by 10 before packing into the
 * register, so the PC receives (degrees × 10) and divides by 10 to display
 * the correct value — matching the BaseSystem protocol spec (0x28–0x30).    */
#define RAD_TO_DEG(r)   ((r) * 57.295779513f)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Robot_t               robot;
JoystickInterface_t   joystick;
BaseSystemInterface_t BaseSystem;
TaskManager_t         taskMgr;

uint8_t Emergency_State = 0;   /* 0 = normal, 1 = emergency active  */
uint8_t Mode_State      = 0;   /* 0 = Auto,   1 = Manual            */

/* ── Debug: Live Expression in STM32CubeIDE — watch these to diagnose ─────── */
uint8_t dbg_gripper_mode   = 0;   /* actual GripperMode_t of robot.gripper   */
uint8_t dbg_up_port_valid  = 0;   /* 1 = up_port_in pointer is non-NULL      */
uint8_t dbg_pin_up_raw     = 0;   /* raw HAL_GPIO_ReadPin result for Up pin  */
uint8_t dbg_pin_down_raw   = 0;   /* raw HAL_GPIO_ReadPin result for Down    */
uint8_t dbg_pin_claw_raw   = 0;   /* raw HAL_GPIO_ReadPin result for Claw    */
uint16_t dbg_sensor_bits   = 0;   /* final sensorBits value sent to Modbus   */

/* ── System state machine ────────────────────────────────────────────────── */
typedef enum {
    SYS_STATE_AUTO,          /* Normal operation — Modbus commands accepted        */
    SYS_STATE_MANUAL,        /* Manual operation — Joystick commands accepted       */
    SYS_STATE_ESTOP,         /* Emergency stop   — motor cut, heartbeat only        */
    SYS_STATE_RESET,         /* One-shot reset   — re-init all, then go to AUTO/MANUAL */
    SYS_STATE_SOFT_ESTOP,    /* Software stop    — motor cut via Modbus REG_SOFT_STOP  *
                              * NO exit path — hardware reset (power cycle) required  */
} SysState_t;

static SysState_t sysState = SYS_STATE_AUTO;

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_USART3_UART_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
    BaseSystemInterface_Init(&BaseSystem, &huart2, &htim16, 21);
    Robot_Init(&robot);
    Task_Init(&taskMgr);

    HAL_TIM_Base_Start_IT(&htim3);
    JoystickInterface_Init(&joystick, &huart3, GPIOB, GPIO_PIN_14);

    sysState = SYS_STATE_MANUAL;   /* Mode_State=0 at startup → MANUAL */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        /* ── Read physical pins ────────────────────────────────────────────── *
         * Active-low: pressed/selected = GPIO_PIN_RESET → invert to 1.        */
        Emergency_State = (HAL_GPIO_ReadPin(ESTOP_PORT,   ESTOP_PIN)   == GPIO_PIN_RESET) ? 1u : 0u;
        Mode_State      = (HAL_GPIO_ReadPin(MODE_SW_PORT, MODE_SW_PIN) == GPIO_PIN_RESET) ? 1u : 0u;

        /* ── Global E-Stop override — highest priority ─────────────────────── *
         * Transition to ESTOP from any state the moment button is pressed.     */
        if (Emergency_State == 1 && sysState != SYS_STATE_ESTOP)
            sysState = SYS_STATE_ESTOP;

        /* ── State machine ─────────────────────────────────────────────────── */
        switch (sysState)
        {
            /* ─────────────────────────────────────────────────────────────── *
             * AUTO — full BaseSystem: Modbus commands + heartbeat + data      *
             * Mode_State = 1                                                  *
             * ─────────────────────────────────────────────────────────────── */
            case SYS_STATE_AUTO:
                /* ── Debug: trace sensor pin state directly ─────────────── */
                dbg_gripper_mode  = (uint8_t)robot.gripper.mode;
                dbg_up_port_valid = (robot.gripper.up_port_in != NULL) ? 1u : 0u;
                dbg_pin_up_raw    = (uint8_t)HAL_GPIO_ReadPin(GRP_UP_PORT_IN,   GRP_UP_PIN_IN);
                dbg_pin_down_raw  = (uint8_t)HAL_GPIO_ReadPin(GRP_DOWN_PORT_IN, GRP_DOWN_PIN_IN);
                dbg_pin_claw_raw  = (uint8_t)HAL_GPIO_ReadPin(GRP_CLAW_PORT_IN, GRP_CLAW_PIN_IN);

                /* ── Write-back: push robot state into Modbus registers ──── *
                 * FIX: convert rad → deg before assigning.                   *
                 * BaseSystemInterface_Update() multiplies by 10 before        *
                 * packing into the register frame, so the PC receives         *
                 * (degrees × 10) and divides by 10 to get the display value. */
                BaseSystem.data.realPosition     = RAD_TO_DEG(Robot_GetPosition    (&robot));
                BaseSystem.data.realVelocity     = RAD_TO_DEG(Robot_GetVelocity    (&robot));
                BaseSystem.data.realAcceleration = RAD_TO_DEG(Robot_GetAcceleration(&robot));
                BaseSystem.data.currentTaskBits  = (uint16_t)Robot_GetState(&robot);
                BaseSystem.data.sensorBits       =
                    ((uint16_t)Robot_Gripper_GetUpState  (&robot) << 0u) |
                    ((uint16_t)Robot_Gripper_GetDownState(&robot) << 1u) |
                    ((uint16_t)Robot_Gripper_GetClawState(&robot) << 2u);
                dbg_sensor_bits = BaseSystem.data.sensorBits;
                BaseSystem.data.emergencyActive = 0u;

                BaseSystemInterface_Update(&BaseSystem);
                BaseSystem_Interface_Decode(&BaseSystem);
                JoystickInterface_Update(&joystick);   /* parse but don't post */

                Robot_CANBus_Update(&robot);           /* CAN RX + heartbeat TX */
                Task_PostFromModbus(&taskMgr, &BaseSystem);
                /* Joystick intentionally NOT posted in AUTO mode */

                Task_Run(&taskMgr, &robot);

                /* Software hard-stop: motor already cut by Robot_EStop() in
                 * TaskManager. Enter SOFT_ESTOP — no exit without hardware reset. */
                if (taskMgr.sysHardStopRequested)
                {
                    sysState = SYS_STATE_SOFT_ESTOP;
                    break;
                }

                /* (sysResetRequested unused — kept in case legacy path needed) */
                if (taskMgr.sysResetRequested)
                {
                    taskMgr.sysResetRequested = 0;
                    sysState = SYS_STATE_RESET;
                    break;
                }

                /* Transition to MANUAL when Mode_State = 0 */
                if (Mode_State == 0)
                    sysState = SYS_STATE_MANUAL;
                break;

            /* ─────────────────────────────────────────────────────────────── *
             * MANUAL — joystick commands only, heartbeat + data to PC        *
             * Mode_State = 0                                                  *
             * ─────────────────────────────────────────────────────────────── */
            case SYS_STATE_MANUAL:
                /* ── Write-back: push robot state into Modbus registers ──── *
                 * FIX: convert rad → deg before assigning.                   */
                BaseSystem.data.realPosition     = RAD_TO_DEG(Robot_GetPosition    (&robot));
                BaseSystem.data.realVelocity     = RAD_TO_DEG(Robot_GetVelocity    (&robot));
                BaseSystem.data.realAcceleration = RAD_TO_DEG(Robot_GetAcceleration(&robot));
                BaseSystem.data.currentTaskBits  = (uint16_t)Robot_GetState(&robot);
                BaseSystem.data.sensorBits       =
                    ((uint16_t)Robot_Gripper_GetUpState  (&robot) << 0u) |
                    ((uint16_t)Robot_Gripper_GetDownState(&robot) << 1u) |
                    ((uint16_t)Robot_Gripper_GetClawState(&robot) << 2u);
                BaseSystem.data.emergencyActive = 0u;

                BaseSystemInterface_Update(&BaseSystem);
                BaseSystem_Interface_Decode(&BaseSystem); /* decode but don't post */
                JoystickInterface_Update(&joystick);

                Robot_CANBus_Update(&robot);           /* CAN RX + heartbeat TX */
                /* Modbus commands intentionally NOT posted in MANUAL mode */
                Task_PostFromJoystick(&taskMgr, &joystick);

                Task_Run(&taskMgr, &robot);

                /* Software hard-stop: motor already cut by Robot_EStop() in
                 * TaskManager. Enter SOFT_ESTOP — no exit without hardware reset. */
                if (taskMgr.sysHardStopRequested)
                {
                    sysState = SYS_STATE_SOFT_ESTOP;
                    break;
                }

                /* (sysResetRequested unused — kept in case legacy path needed) */
                if (taskMgr.sysResetRequested)
                {
                    taskMgr.sysResetRequested = 0;
                    sysState = SYS_STATE_RESET;
                    break;
                }

                /* Transition to AUTO when Mode_State = 1 */
                if (Mode_State == 1)
                    sysState = SYS_STATE_AUTO;
                break;

            /* ─────────────────────────────────────────────────────────────── *
             * ESTOP — motor cut immediately every iteration                  *
             * heartbeat + data kept alive so PC knows we're alive            *
             * no motion commands accepted from any source                     *
             * ─────────────────────────────────────────────────────────────── */
            case SYS_STATE_ESTOP:
                Robot_EStop(&robot);                   /* cut motor every tick */

                /* ── Write-back: report emergency state to PC ────────────── *
                 * FIX: convert rad → deg for position.                        *
                 * Velocity is forced 0 (motor cut); acceleration stays 0.    */
                BaseSystem.data.realPosition     = RAD_TO_DEG(Robot_GetPosition(&robot));
                BaseSystem.data.realVelocity     = 0.0f;
                BaseSystem.data.realAcceleration = 0.0f;
                BaseSystem.data.currentTaskBits  = (uint16_t)Robot_GetState(&robot);
                BaseSystem.data.sensorBits       = 0u;
                BaseSystem.data.emergencyActive  = 1u;

                BaseSystemInterface_Update(&BaseSystem); /* heartbeat + data   */

                /* On button release: full system re-init (encoder, Kalman,
                 * home position all cleared) then go to RESET for queue/BSI  */
                if (Emergency_State == 0)
                {
                    Robot_Init(&robot);
                    HAL_TIM_Base_Start_IT(&htim3);
                    sysState = SYS_STATE_RESET;
                }
                break;

            /* ─────────────────────────────────────────────────────────────── *
             * RESET — restore robot to default idle state.                   *
             * ─────────────────────────────────────────────────────────────── */
            case SYS_STATE_RESET:
                /* 1. Halt physical motor outputs and clamp soft movement profiles */
                Robot_SoftReset(&robot);

                /* 2. Wipe the high-level task queues, automation sequences, and tests */
                Task_Init(&taskMgr);

                /* 3. Clear the decoded incoming Modbus data structures entirely.
                 * This purges any stale command from the network buffer,
                 * ensuring the sequence doesn't immediately re-execute. */
                memset(&BaseSystem.pending, 0, sizeof(BSI_PendingCmd_t));

                /* 4. Safely fall back into the active operating mode */
                sysState = (Mode_State == 1) ? SYS_STATE_AUTO : SYS_STATE_MANUAL;
                break;

            /* ─────────────────────────────────────────────────────────────── *
             * SOFT_ESTOP — triggered by Modbus REG_SOFT_STOP command.        *
             *                                                                  *
             * Motor is ALREADY cut (Robot_EStop called in TaskManager).       *
             * This state holds indefinitely — heartbeat + data kept alive so  *
             * the PC knows the robot is locked.                                *
             *                                                                  *
             * EXIT: Hardware reset (power cycle / reset button) ONLY.         *
             *       There is intentionally no software path out of this state. *
             * ─────────────────────────────────────────────────────────────── */
            case SYS_STATE_SOFT_ESTOP:
                Robot_EStop(&robot);                   /* enforce cut every tick */

                /* ── Write-back: report emergency state to PC ────────────── *
                 * FIX: convert rad → deg for position.                        *
                 * Velocity/acceleration forced 0 (motor cut).                */
                BaseSystem.data.realPosition     = RAD_TO_DEG(Robot_GetPosition(&robot));
                BaseSystem.data.realVelocity     = 0.0f;
                BaseSystem.data.realAcceleration = 0.0f;
                BaseSystem.data.currentTaskBits  = (uint16_t)Robot_GetState(&robot);
                BaseSystem.data.sensorBits       = 0u;
                BaseSystem.data.emergencyActive  = 1u;

                BaseSystemInterface_Update(&BaseSystem); /* heartbeat + data     */
                /* No exit condition — hardware reset required */
                break;
        }

  /* USER CODE END 3 */
  } /* while (1) */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3)
        Robot_Update(&robot, htim);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    Robot_EXTI_Callback(&robot, GPIO_Pin);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    JoystickInterface_RxCpltCallback(&joystick, huart);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
