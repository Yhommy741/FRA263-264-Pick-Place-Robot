/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @brief   Robot library test — Direct Pin Live Expression Control
 *
 * All commands and getters are in OUTPUT shaft space.
 * With N=5: Robot_Move(2π) → output shaft moves 2π rad, motor moves 10π rad.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Robot.h"
#include "JoystickInterface.h"
#include "ModbusRTU.h"
#include "BaseSystem.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ── Motor parameters ────────────────────────────────────────────────────── */
#define MOTOR_RM        2.2940f
#define MOTOR_LM        0.0020f
#define MOTOR_KE        0.73840f
#define MOTOR_KT        0.73840f
#define MOTOR_J         0.0067475f
#define MOTOR_B         0.0061979f

/* ── Encoder ─────────────────────────────────────────────────────────────── */
#define ENC_PPR         2048
#define ENC_X           4
#define ENC_OVERFLOW    65535

/* ── Gear ratio ──────────────────────────────────────────────────────────── */
#define GEAR_RATIO_N    5.0f    /* output_rad = motor_rad / 5                */

/* ── Control (OUTPUT shaft space) ───────────────────────────────────────── */
#define CTRL_TS_S       0.001f
#define CTRL_V_MAX      24.0f
#define CTRL_OMEGA_MAX  5.6f    /* rad/s output shaft (28 motor / 5)         */
#define CTRL_ALPHA_MAX  5.0f    /* rad/s² output shaft (21 motor / 5)        */

/* ── Kalman noise ─────────────────────────────────────────────────────────── */
#define KF_VAR_TAU_D    4.98e-3f
#define KF_VAR_THETA    4.98e-9f

/* ── PID gains (tuned for OUTPUT shaft) ─────────────────────────────────── */
#define KP_VEL   20.0f
#define KI_VEL   175.0f
#define KD_VEL   0.0f

#define KP_POS   25.0f
#define KI_POS   0.0f
#define KD_POS   0.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

typedef struct {
	/* Direct Actuator Controls (Write 1 to activate pin, 0 to clear pin) */
	uint8_t out_up; /* PC6 — Move Elevator Up    */
	uint8_t out_down; /* PC1 — Move Elevator Down  */
	uint8_t out_close; /* PC3 — Close Claw / Grip   */
	uint8_t out_open; /* PC0 — Open Claw / Ungrip  */

	/* Input Sensor Feedback (Read Only) */
	uint8_t sensor_up; /* PB0 — Reed switch top limit    */
	uint8_t sensor_down; /* PB1 — Reed switch bottom limit */
	uint8_t sensor_claw; /* PB2 — Reed switch claw state   */
} Gripper_Direct_t;

volatile Gripper_Direct_t gripper = { 0 };

Robot_t robot;
JoystickInterface_t joystick;
ModbusHandleTypedef hmodbus;
u16u8_t registerFrame[200];

volatile uint8_t lim = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 * Live-Expression Debug Commands (Main Robot)
 *
 * Add to Expressions window:
 * dbg_cmd          — write command ID for main motor
 * dbg_target       — target angle [rad, OUTPUT shaft]
 * dbg_jog_speed    — velocity jog speed magnitude [rad/s, output shaft, >0]
 * dbg_jog_step     — step jog size magnitude      [rad,   output shaft, >0]
 * robot.state      — 0=IDLE 1=MOVE 2=JOG_VEL 3=JOG_STEP 4..6=HOMING 7=ESTOP
 * robot.theta      — current position [rad, OUTPUT shaft]
 * robot.omega      — current velocity [rad/s, OUTPUT shaft]
 *
 * Motor Commands (dbg_cmd):
 * 1  Robot_Move(&robot, dbg_target)          smooth constrained move
 * 2  Robot_Stop(&robot)                      stop and hold
 * 3  Robot_EStop(&robot)                     cut motor immediately
 * 4  Robot_SetHome(&robot)                   zero at current position
 * 5  Robot_Home(&robot)                      homing → always returns to 0
 * 6  Robot_JogVel  CCW  +dbg_jog_speed       continuous jog CCW
 * 7  Robot_JogVel  CW   -dbg_jog_speed       continuous jog CW
 * 8  Robot_JogStep CCW  +dbg_jog_step        single step CCW
 * 9  Robot_JogStep CW   -dbg_jog_step        single step CW
 * ═══════════════════════════════════════════════════════════════════════════ */
volatile uint8_t dbg_cmd = 0;
volatile float dbg_target = 0.0f;
volatile float dbg_jog_speed = 1.0f;
volatile float dbg_jog_step = 0.1f;

volatile uint8_t joy_cmd = 0;
volatile float joy_data = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Gripper_Update_DirectPins(void);
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
  /* USER CODE BEGIN 2 */

	Robot_Config_t cfg = { .htim_encoder = &htim1, .htim_pwm = &htim2,
			.htim_ctrl = &htim3,

			.ch_dir = TIM_CHANNEL_1, .ch_pwm = TIM_CHANNEL_2,

			.enc_ppr = ENC_PPR, .enc_x = ENC_X, .enc_overflow = ENC_OVERFLOW,

			.Rm = MOTOR_RM, .Lm = MOTOR_LM, .Ke = MOTOR_KE, .Kt = MOTOR_KT, .J =
			MOTOR_J, .b = MOTOR_B,

			.N = GEAR_RATIO_N, .Ts = CTRL_TS_S, .V_max = CTRL_V_MAX,
			.omega_max = CTRL_OMEGA_MAX, .alpha_max = CTRL_ALPHA_MAX,

			.kf_var_tau_d = KF_VAR_TAU_D, .kf_var_theta = KF_VAR_THETA,

			.Kp_vel = KP_VEL, .Ki_vel = KI_VEL, .Kd_vel = KD_VEL, .Kp_pos =
			KP_POS, .Ki_pos = KI_POS, .Kd_pos = KD_POS,

			.ls_port = GPIOC, .ls_pin = GPIO_PIN_2, };
	hmodbus.huart = &huart2;
	hmodbus.htim = &htim16;
	hmodbus.slaveAddress = 21;
	hmodbus.RegisterSize = 200;

	Modbus_init(&hmodbus, registerFrame);
	BaseSystem_Init(&hmodbus, registerFrame);

	Robot_Init(&robot, &cfg);

	HAL_TIM_Base_Start_IT(&htim3);
	JoystickInterface_Init(&joystick, &huart3, GPIOB, GPIO_PIN_14);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Modbus_Protocol_Worker();
		BaseSystem_Update();

		uint8_t cmd = dbg_cmd;

		/* ── Core Robot Actuator Motor Actions ───────────────────────────── */
		if (cmd != 0) {
			dbg_cmd = 0;
			switch (cmd) {
			case 1:
				Robot_Move(&robot, dbg_target);
				break;
			case 2:
				Robot_Stop(&robot);
				break;
			case 3:
				Robot_EStop(&robot);
				break;
			case 4:
				Robot_SetHome(&robot);
				break;
			case 5:
				Robot_Home(&robot);
				break;
			case 6:
				Robot_JogVel(&robot, dbg_jog_speed);
				break; /* CCW */
			case 7:
				Robot_JogVel(&robot, -dbg_jog_speed);
				break; /* CW  */
			case 8:
				Robot_JogStep(&robot, dbg_jog_step);
				break; /* CCW */
			case 9:
				Robot_JogStep(&robot, -dbg_jog_step);
				break; /* CW  */
			default:
				break;
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim != &htim3)
		return;

	Robot_Update(&robot, htim);
	Gripper_Update_DirectPins();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	Robot_EXTI_Callback(&robot, GPIO_Pin);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	JoystickInterface_RxCpltCallback(&joystick, huart);
}
/**
 * @brief Directly updates GPIO outputs based on the debug structure fields
 * and mirrors the hardware limit inputs.
 */
void Gripper_Update_DirectPins(void) {
	/* 1. Mirror Input Pins to Live Expressions (Read Only) */
	gripper.sensor_up = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	gripper.sensor_down = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	gripper.sensor_claw = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);

	/* 2. Direct Write to Hardware Output Pins from Live Expressions */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,
			gripper.out_up ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,
			gripper.out_down ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,
			gripper.out_close ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,
			gripper.out_open ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
	while (1) {
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
