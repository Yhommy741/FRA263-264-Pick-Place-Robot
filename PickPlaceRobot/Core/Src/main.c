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

#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "Robot.h"
#include "BaseSystemInterface.h"
#include "JoystickInterface.h"
#include "TaskManager.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
Robot_t               robot;
JoystickInterface_t   joystick;
BaseSystemInterface_t BaseSystem;
TaskManager_t         taskMgr;

uint8_t Emergency_State = 0;   /* 0 = normal, 1 = emergency active  */
uint8_t Mode_State      = 0;   /* 0 = Auto,   1 = Manual            */

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

void SystemClock_Config(void);

/* ── Private helpers ─────────────────────────────────────────────────────── */

/* Write current robot state into BaseSystem.data so BSI_Update packs it
 * into the Modbus register frame before sending to PC.                     */
static void update_writeback(uint8_t emergency_active)
{
    BaseSystemInterface_Data_t *d = &BaseSystem.data;

    d->realPosition     = Robot_GetPosition(&robot) * 57.295779513f; /* rad→deg */
    d->realVelocity     = Robot_GetVelocity(&robot);
    d->realAcceleration = Robot_GetAcceleration(&robot);
    d->emergencyActive  = emergency_active;

    Robot_State_t rs = Robot_GetState(&robot);
    uint16_t tb = 0;
    if (rs == ROBOT_HOMING_FAST_STATE   ||
        rs == ROBOT_HOMING_BACKOFF_STATE ||
        rs == ROBOT_HOMING_SLOW_STATE    ||
        rs == ROBOT_HOMING_OFFSET_STATE)   tb = 0x0001;
    else if (rs == ROBOT_MOVE    ||
             rs == ROBOT_JOG_VEL ||
             rs == ROBOT_JOG_STEP)         tb = 0x0008;
    else if (rs == ROBOT_PERF_TEST)        tb = 0x0010;
    d->currentTaskBits = tb;

    uint16_t sb = 0;
    sb |= (Robot_Gripper_GetUpState  (&robot) == GRP_STATE_HIGH) ? (1u<<0) : 0u;
    sb |= (Robot_Gripper_GetDownState(&robot) == GRP_STATE_HIGH) ? (1u<<1) : 0u;
    sb |= (Robot_Gripper_GetClawState(&robot) == GRP_STATE_HIGH) ? (1u<<2) : 0u;
    d->sensorBits = sb;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * main
 * ═══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();
    MX_TIM16_Init();
    MX_USART3_UART_Init();

    /* USER CODE BEGIN 2 */
    BaseSystemInterface_Init(&BaseSystem, &huart2, &htim16, 21);
    Robot_Init(&robot);
    Task_Init(&taskMgr);

    HAL_TIM_Base_Start_IT(&htim3);
    JoystickInterface_Init(&joystick, &huart3, GPIOB, GPIO_PIN_14);

    sysState = SYS_STATE_MANUAL;   /* Mode_State=0 at startup → MANUAL */
    /* USER CODE END 2 */

    while (1)
    {
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
                update_writeback(0);
                BaseSystemInterface_Update(&BaseSystem);
                BaseSystem_Interface_Decode(&BaseSystem);
                JoystickInterface_Update(&joystick);   /* parse but don't post */

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
                update_writeback(0);
                BaseSystemInterface_Update(&BaseSystem);
                BaseSystem_Interface_Decode(&BaseSystem); /* decode but don't post */
                JoystickInterface_Update(&joystick);

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

                update_writeback(1);                   /* emergencyActive = 1  */
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

                /* 3. FIX: Clear the decoded incoming Modbus data structures entirely.
                 * This purges the continuous "Run" task command from the network buffer,
                 * ensuring the sequence doesn't immediately execute again. */
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
                update_writeback(1);                   /* emergencyActive = 1    */
                BaseSystemInterface_Update(&BaseSystem); /* heartbeat + data     */
                /* No exit condition — hardware reset required */
                break;
        }

        /* USER CODE END 3 */
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * System Clock Configuration
 * ═══════════════════════════════════════════════════════════════════════════ */
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) { Error_Handler(); }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Error Handler Definition (Global Link Scope Fix)
 * ═══════════════════════════════════════════════════════════════════════════ */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        /* Trap peripheral failures safely */
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
