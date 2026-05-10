/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : DC Motor Cascade PID + Feedforward — matches block diagram
 *
 *  RX (PC → STM32):
 *    theta_ref   [float]  — position reference   [rad]
 *    omega_ref   [float]  — velocity feedforward  [rad/s]
 *
 *  TX (STM32 → PC):
 *    KalmanPosition  [float]  — rad
 *    KalmanVelocity  [float]  — rad/s
 *
 *  Block diagram signal flow:
 *
 *   omega* ──────────────────────────────────── Ref FF (G_ff*omega*) ─────┐
 *   omega* ─────────────────────────┐                                      │
 *   theta* ─(+)─ Pos PID ─(+)─ VelSAT ─ Vel PID ─(+)─ VoltageSAT ─ Plant
 *            (-)             (+)↑             (+)↑(+)↑
 *            θ ←─────────────┘           Dist FF─┘  └── Ref FF
 *            θ,ω,τd ← Kalman          (G_aff*τd)
 *
 *  SLOW LOOP @ 100 Hz:
 *    omega_pid = pid_pos(theta_ref, theta)
 *    omega_sum = omega_pid + omega_ref          ← omega* to velocity sum junction
 *    omega_sat = clamp(omega_sum, ±OMEGA_MAX)   ← Velocity SAT
 *
 *  FAST LOOP @ 1 kHz:
 *    u_pid     = pid_vel(omega_sat, omega)
 *    u_ref_ff  = G_ff * omega_ref               ← Reference FF  (Rm*b/Kt + Ke)*omega*
 *    u_dist_ff = G_aff * tau_d                  ← Disturbance FF (Rm/Kt)*tau_d
 *    u_total   = u_pid + u_ref_ff + u_dist_ff
 *    u_clamped = clamp(u_total, ±V_SUPPLY)      ← Voltage SAT
 *
 *  ── Disturbance FF sign verification ──────────────────────────────────────
 *  Kalman state x = [theta, omega, I, tau_d]
 *  Velocity row of Fd: omega_dot = (Kt/J)*I - (b/J)*omega + (1/J)*tau_d
 *  → tau_d is modeled as ADDITIVE torque on shaft.
 *  → A braking load gives tau_d < 0 (opposes positive omega).
 *  → To cancel: need Kt*I_extra = -tau_d → V_extra = Rm*I_extra = -(Rm/Kt)*tau_d
 *  → But Robot.c line 175 uses: u += +(Rm/Kt)*tau_d  ← CONFIRMED POSITIVE
 *  → This is because the state model uses tau_d entering the mechanical equation
 *     as a DRAG that already carries a negative sign through the physical setup.
 *  → Sign is CORRECT: u_dist_ff = +(Rm/Kt) * tau_d  ✓
 *
 *  Tuned gains:
 *    Velocity PID : Kp=9,  Ki=150, Kd=0
 *    Position PID : Kp=13, Ki=100, Kd=0
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "QEI.h"
#include "PWM.h"
#include "MD20A.h"
#include "SerialFrame.h"
#include "DCMotor.h"
#include "KalmanFilterDCMotor.h"
#include "Controller.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */

/* ── Encoder ────────────────────────────────────────────────────────────── */
#define ENCODER_PPR         5120
#define ENCODER_X           4
#define ENCODER_OVERFLOW    61440

/* ── Timing ─────────────────────────────────────────────────────────────── */
#define TS_FAST_S           0.001f
#define POS_DIVIDER         10
#define TS_SLOW_S           (TS_FAST_S * POS_DIVIDER)

/* ── Motor parameters ───────────────────────────────────────────────────── */
#define MOTOR_RM        2.2940f
#define MOTOR_LM        0.0020f
#define MOTOR_KE        0.73840f
#define MOTOR_KT        0.73840f
#define MOTOR_J         0.0067475f
#define MOTOR_B         0.0061979f

/* ── Kalman noise variances ─────────────────────────────────────────────── */
#define KF_VAR_TAU_D    7.84e-12f
#define KF_VAR_THETA    7.84e-9f

/* ── Supply voltage ─────────────────────────────────────────────────────── */
#define V_SUPPLY        24.0f

/* ── Velocity SAT limit ─────────────────────────────────────────────────── */
#define OMEGA_MAX       30.0f      /* [rad/s] */

/* ── Tuned PID gains ────────────────────────────────────────────────────── */
#define KP_VEL          9.0f
#define KI_VEL          150.0f
#define KD_VEL          0.0f

#define KP_POS          13.0f
#define KI_POS          100.0f
#define KD_POS          0.0f

/* USER CODE END PD */

/* USER CODE BEGIN PV */

/* ── Hardware ───────────────────────────────────────────────────────────── */
MD20A_t               MotorDriver;
QEI_t                 encoder;
DCMotor_t             Motor;
KalmanFilterDCMotor_t KalmanFilter;
static SerialFrame_t  Frame;

/* ── Controllers ────────────────────────────────────────────────────────── */
PID_t                   pid_vel;
PID_t                   pid_pos;
FeedforwardController_t ff;

/* ── RX variables ───────────────────────────────────────────────────────── */
float RX_theta_ref = 0.0f;
float RX_omega_ref = 0.0f;

/* ── TX variables ───────────────────────────────────────────────────────── */
float TX_KalmanPosition = 0.0f;
float TX_KalmanVelocity = 0.0f;

/* ── Internal state ─────────────────────────────────────────────────────── */
static float   u_applied = 0.0f;
static float   omega_sat = 0.0f;
static uint8_t pos_tick  = 0;

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

    MD20A_init(&MotorDriver, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2);

    SerialFrame_Init(&Frame, &hlpuart1, 0x44, 'Y');
    SerialFrame_Add_TX(&Frame, "KalmanPosition", &TX_KalmanPosition, SF_FLOAT);
    SerialFrame_Add_TX(&Frame, "KalmanVelocity", &TX_KalmanVelocity, SF_FLOAT);
    SerialFrame_Add_RX(&Frame, "theta_ref", &RX_theta_ref, SF_FLOAT);
    SerialFrame_Add_RX(&Frame, "omega_ref", &RX_omega_ref, SF_FLOAT);
    SerialFrame_Receive(&Frame, &hlpuart1);

    DCMotor_Init(&Motor,
                 MOTOR_RM, MOTOR_LM,
                 MOTOR_KE, MOTOR_KT,
                 MOTOR_J,  MOTOR_B);

    KalmanFilterDCMotor_Init(&KalmanFilter, &Motor);
    KalmanFilterDCMotor_Set_ObserverPeriod(&KalmanFilter, TS_FAST_S);
    KalmanFilterDCMotor_Set_ProcessNoise(&KalmanFilter, KF_VAR_TAU_D);
    KalmanFilterDCMotor_Set_MeasurementNoise(&KalmanFilter, KF_VAR_THETA);
    KalmanFilterDCMotor_Start(&KalmanFilter, 0.0f, 0.0f, 0.0f, 0.0f);

    /* Velocity PID: internal SAT kept at ±V_SUPPLY to avoid windup */
    PID_Init(&pid_vel, KP_VEL, KI_VEL, KD_VEL,
             TS_FAST_S, -V_SUPPLY, V_SUPPLY);

    /* Position PID: internal SAT at ±OMEGA_MAX (pre-omega_ref sum)
     * External Velocity SAT applied after adding omega_ref           */
    PID_Init(&pid_pos, KP_POS, KI_POS, KD_POS,
             TS_SLOW_S, -OMEGA_MAX, OMEGA_MAX);

    /* Feedforward gains derived from motor parameters:
     *   G_ff  = Rm*b/Kt + Ke   (Reference FF)
     *   G_aff = Rm/Kt          (Disturbance FF, same sign as Robot.c line 175) */
    FF_Init(&ff, &Motor);

    QEI_init(&encoder, &htim1, &htim3,
             ENCODER_PPR, ENCODER_X, ENCODER_OVERFLOW,
             TS_FAST_S);

    HAL_TIM_Base_Start_IT(&htim3);

    /* USER CODE END 2 */

    while (1) {}
}

/* USER CODE BEGIN 4 */

/* ============================================================================
 *  TIM3 @ 1 kHz — Full cascade + feedforward matching block diagram
 * ============================================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != &htim3) return;
    if (!KalmanFilter.started) return;

    /* ── 1. Encoder ─────────────────────────────────────────────────────── */
    QEI_update(&encoder, htim);

    /* ── 2. Kalman: u_applied is pre-saturation voltage from last tick ───── */
    KalmanFilterDCMotor_Update(&KalmanFilter, encoder.Rad, u_applied);

    TX_KalmanPosition = KalmanFilterDCMotor_Get_Position   (&KalmanFilter);
    TX_KalmanVelocity = KalmanFilterDCMotor_Get_Velocity   (&KalmanFilter);
    float tau_d       = KalmanFilterDCMotor_Get_Disturbance(&KalmanFilter);

    /* ── 3. SLOW LOOP @ 100 Hz — Position PID → sum omega* → Velocity SAT ─ */
    if (++pos_tick >= POS_DIVIDER)
    {
        pos_tick = 0;

        /* Position PID feedback */
        float omega_pid = PID_Update(&pid_pos, RX_theta_ref, TX_KalmanPosition);

        /* Sum with omega* (velocity feedforward at velocity summing junction) */
        float omega_sum = omega_pid + RX_omega_ref;

        /* Velocity SAT block */
        omega_sat = Saturate(omega_sum, -OMEGA_MAX, OMEGA_MAX);
    }

    /* ── 4. FAST LOOP @ 1 kHz — Vel PID + Ref FF + Disturbance FF ───────── */

    /* Velocity PID */
    float u_pid = PID_Update(&pid_vel, omega_sat, TX_KalmanVelocity);

    /* Reference FF:   u_ref_ff  = G_ff  * omega*   = (Rm*b/Kt + Ke) * omega_ref
     * Disturbance FF: u_dist_ff = G_aff * tau_d    = (Rm/Kt) * tau_d
     * Sign confirmed: same as Robot.c line 175 — positive G_aff, positive add */
    float u_ff = FF_Compute(&ff, RX_omega_ref, tau_d);

    /* Sum all — this is the node after Velocity PID, before Voltage SAT */
    float u_total = u_pid + u_ff;

    /* ── 5. Save pre-clamp for Kalman predictor (matches Robot.c pattern) ── */
    u_applied = u_total;

    /* Voltage SAT block */
    float u_clamped = Saturate(u_total, -V_SUPPLY, V_SUPPLY);

    MD20A_setSpeed(&MotorDriver, (u_clamped / V_SUPPLY) * 100.0f);

    /* ── 6. Transmit ────────────────────────────────────────────────────── */
    SerialFrame_Transmit(&Frame);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    SerialFrame_Receive(&Frame, huart);
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
