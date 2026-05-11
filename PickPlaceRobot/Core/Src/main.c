/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @brief   Robot library test — SerialFrame telemetry owned by App layer
 *
 *  All commands and getters are in OUTPUT shaft space.
 *  With N=5: Robot_Move(2π) → output shaft moves 2π rad, motor moves 10π rad.
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "Robot.h"
#include "SerialFrame.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */

/* ── Motor parameters ────────────────────────────────────────────────────── */
#define MOTOR_RM        2.2940f
#define MOTOR_LM        0.0020f
#define MOTOR_KE        0.73840f
#define MOTOR_KT        0.73840f
#define MOTOR_J         0.0067475f
#define MOTOR_B         0.0061979f

/* ── Encoder ─────────────────────────────────────────────────────────────── */
#define ENC_PPR         5120
#define ENC_X           4
#define ENC_OVERFLOW    61440

/* ── Gear ratio ──────────────────────────────────────────────────────────── */
#define GEAR_RATIO_N    5.0f    /* output_rad = motor_rad / 5                */

/* ── Control (OUTPUT shaft space) ───────────────────────────────────────── */
#define CTRL_TS_S       0.001f
#define CTRL_V_MAX      24.0f
#define CTRL_OMEGA_MAX  5.6f    /* rad/s output shaft (28 motor / 5)         */
#define CTRL_ALPHA_MAX  5.0f    /* rad/s² output shaft (21 motor / 5)        */

/* ── Kalman noise ─────────────────────────────────────────────────────────── */
#define KF_VAR_TAU_D    7.84e-8f
#define KF_VAR_THETA    7.84e-9f

/* ── PID gains (tuned for OUTPUT shaft) ─────────────────────────────────── */
#define KP_VEL   20.0f
#define KI_VEL   175.0f
#define KD_VEL   0.0f

#define KP_POS   20.0f
#define KI_POS   150.0f
#define KD_POS   0.0f

/* ── Telemetry ───────────────────────────────────────────────────────────── */
#define TELEM_TX_DIVIDER    10

/* USER CODE END PD */

/* USER CODE BEGIN PV */

Robot_t robot;

static SerialFrame_t frame;
static uint8_t       tx_tick   = 0;
static float         telem_pos = 0.0f;
static float         telem_vel = 0.0f;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Live-Expression Debug Commands
 *
 *  Add to Expressions window:
 *    dbg_cmd          — write command ID
 *    dbg_target       — target angle [rad, OUTPUT shaft]
 *    dbg_jog_speed    — velocity jog speed magnitude [rad/s, output shaft, >0]
 *    dbg_jog_step     — step jog size magnitude      [rad,   output shaft, >0]
 *    robot.state      — 0=IDLE 1=MOVE 2=JOG_VEL 3=JOG_STEP 4..6=HOMING 7=ESTOP
 *    robot.theta      — current position [rad, OUTPUT shaft]
 *    robot.omega      — current velocity [rad/s, OUTPUT shaft]
 *
 *  Commands:
 *    1  Robot_Move(&robot, dbg_target)          smooth constrained move
 *    2  Robot_Stop(&robot)                      stop and hold
 *    3  Robot_EStop(&robot)                     cut motor immediately
 *    4  Robot_SetHome(&robot)                   zero at current position
 *    5  Robot_Home(&robot)                      homing → always returns to 0
 *    6  Robot_JogVel  CCW  +dbg_jog_speed       continuous jog CCW
 *    7  Robot_JogVel  CW   -dbg_jog_speed       continuous jog CW
 *    8  Robot_JogStep CCW  +dbg_jog_step        single step CCW
 *    9  Robot_JogStep CW   -dbg_jog_step        single step CW
 * ═══════════════════════════════════════════════════════════════════════════ */
volatile uint8_t dbg_cmd       = 0;
volatile float   dbg_target    = 0.0f;
volatile float   dbg_jog_speed = 1.0f;   /* magnitude [rad/s] — direction set by cmd */
volatile float   dbg_jog_step  = 0.1f;   /* magnitude [rad]   — direction set by cmd */

/* USER CODE END PV */

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_LPUART1_UART_Init();
    MX_TIM3_Init();

    /* USER CODE BEGIN 2 */

    Robot_Config_t cfg = {
        .htim_encoder = &htim1,
        .htim_pwm     = &htim2,
        .htim_ctrl    = &htim3,

        .ch_dir = TIM_CHANNEL_1,
        .ch_pwm = TIM_CHANNEL_2,

        .enc_ppr      = ENC_PPR,
        .enc_x        = ENC_X,
        .enc_overflow = ENC_OVERFLOW,

        .Rm = MOTOR_RM, .Lm = MOTOR_LM,
        .Ke = MOTOR_KE, .Kt = MOTOR_KT,
        .J  = MOTOR_J,  .b  = MOTOR_B,

        .N         = GEAR_RATIO_N,
        .Ts        = CTRL_TS_S,
        .V_max     = CTRL_V_MAX,
        .omega_max = CTRL_OMEGA_MAX,
        .alpha_max = CTRL_ALPHA_MAX,

        .kf_var_tau_d = KF_VAR_TAU_D,
        .kf_var_theta = KF_VAR_THETA,

        .Kp_vel = KP_VEL, .Ki_vel = KI_VEL, .Kd_vel = KD_VEL,
        .Kp_pos = KP_POS, .Ki_pos = KI_POS, .Kd_pos = KD_POS,

        .ls_port = GPIOC,
        .ls_pin  = GPIO_PIN_2,
    };

    Robot_Init(&robot, &cfg);

    SerialFrame_Init(&frame, &hlpuart1, 0x44, 'Y');
    SerialFrame_Add_TX(&frame, "OutputPosition", &telem_pos, SF_FLOAT);
    SerialFrame_Add_TX(&frame, "OutputVelocity", &telem_vel, SF_FLOAT);
    SerialFrame_Receive(&frame, &hlpuart1);

    HAL_TIM_Base_Start_IT(&htim3);

    /* USER CODE END 2 */

    while (1)
    {
        uint8_t cmd = dbg_cmd;
        if (cmd != 0)
        {
            dbg_cmd = 0;
            switch (cmd)
            {
                case 1: Robot_Move    (&robot,  dbg_target);          break;
                case 2: Robot_Stop    (&robot);                        break;
                case 3: Robot_EStop   (&robot);                        break;
                case 4: Robot_SetHome (&robot);                        break;
                case 5: Robot_Home    (&robot);                        break;
                case 6: Robot_JogVel  (&robot,  dbg_jog_speed);       break; /* CCW */
                case 7: Robot_JogVel  (&robot, -dbg_jog_speed);       break; /* CW  */
                case 8: Robot_JogStep (&robot,  dbg_jog_step);        break; /* CCW */
                case 9: Robot_JogStep (&robot, -dbg_jog_step);        break; /* CW  */
                default: break;
            }
        }
    }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != &htim3) return;

    Robot_Update(&robot, htim);

    if (++tx_tick >= TELEM_TX_DIVIDER)
    {
        tx_tick   = 0;
        telem_pos = Robot_GetPosition(&robot);
        telem_vel = Robot_GetVelocity(&robot);
        SerialFrame_Transmit(&frame);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    SerialFrame_Receive(&frame, huart);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    Robot_EXTI_Callback(&robot, GPIO_Pin);
}

/* USER CODE END 4 */

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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
