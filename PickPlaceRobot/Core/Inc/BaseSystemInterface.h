/*
 * BaseSystemInterface.h
 *
 * Created on: May 19, 2026
 * Author: Assistant
 *
 * ModbusRTU is embedded as a submodule inside BaseSystemInterface_t.
 * The global instance (BaseSystemInterface_t BaseSystem) is declared
 * in main.c — NOT here. No extern in this header.
 */

#ifndef INC_BASESYSTEMINTERFACE_H_
#define INC_BASESYSTEMINTERFACE_H_

#include "ModbusRTU.h"
#include "Robot.h"

/* ── Register Address Definitions ─────────────────────────────────────────── */
#define REG_HEARTBEAT          0x00
#define REG_OP_MODE            0x01
#define REG_MANUAL_GRIPPER     0x02
#define REG_GRIPPER_SEQ        0x03
#define REG_GRIPPER_AUTO_EN    0x04
#define REG_JOG_DEG            0x05
#define REG_TEST_TYPE          0x06
#define REG_PERF_VEL           0x07
#define REG_PERF_ACCEL         0x08
#define REG_PREC_INIT          0x09
#define REG_PREC_FINAL         0x10
#define REG_PREC_REPEATS       0x11
#define REG_SEQ_START          0x12
#define REG_SEQ_END            0x21
#define REG_SEQ_PAIRS          0x22
#define REG_P2P_UNIT           0x23
#define REG_P2P_TARGET         0x24
#define REG_SOFT_STOP          0x25
#define REG_SENSORS            0x26
#define REG_ROBOT_TASK         0x27
#define REG_POSITION           0x28
#define REG_VELOCITY           0x29
#define REG_ACCELERATION       0x30
#define REG_EMERGENCY          0x31

/* ── Magic Constants ───────────────────────────────────────────────────────── */
#define HEARTBEAT_ROBOT_YA     22881
#define HEARTBEAT_PC_HI        18537

/* ── Register Frame Size ───────────────────────────────────────────────────── */
#define BASE_SYSTEM_REG_COUNT  200

/* ── Operating Mode IDs (one-hot bit masks, per README 3.2) ───────────────── */
#define OP_MODE_IDLE           0x0000   /* no bit set                          */
#define OP_MODE_HOMING         0x0001   /* bit 0 — Go home                     */
#define OP_MODE_JOG            0x0002   /* bit 1 — Manual / Jog                */
#define OP_MODE_AUTO           0x0004   /* bit 2 — Auto (sequence)             */
#define OP_MODE_SET_HOME       0x0008   /* bit 3 — Set home                    */
#define OP_MODE_TEST           0x0010   /* bit 4 — Test                        */

/* ── Gripper FSM State IDs ─────────────────────────────────────────────────── */
/* Used by seqGripState in BaseSystemInterface_t.                               */
/* Shared between Section D (auto-sequence) and Section G (standalone Pick/Place). */
#define GRP_FSM_IDLE      0   /* waiting for robot to reach position              */
#define GRP_FSM_DOWN      1   /* MoveDown issued, waiting GRP_WAIT_TIME ms         */
#define GRP_FSM_PENDULUM  2   /* Place only: waiting GRP_WAIT_PENDULUM_TIME ms
                                  for pendulum to stabilise before Open             */
#define GRP_FSM_ACTION    3   /* Open/Close issued, waiting GRP_WAIT_TIME ms       */
#define GRP_FSM_UP        4   /* MoveUp issued, waiting GRP_WAIT_TIME ms           */
#define GRP_FSM_DONE      5   /* sequence complete — advance step or finish         */

/* ── Manual Gripper Command IDs (per README 3.3) ──────────────────────────── */
#define GRP_CMD_NONE           0x00   /* no command                            */
#define GRP_CMD_UP             0x00   /* Up = 0 (no bits set in group)         */
#define GRP_CMD_DOWN           0x01   /* bit 0                                 */
#define GRP_CMD_OPEN           0x02   /* bit 1                                 */
#define GRP_CMD_CLOSE          0x04   /* bit 2                                 */

/* ── P2P Unit IDs (per README 3.10) ───────────────────────────────────────── */
#define P2P_UNIT_DEG           0x00   /* bit 0 = 0 → degree  */
#define P2P_UNIT_INDEX         0x01   /* bit 0 = 1 → index   */

/* ─────────────────────────────────────────────────────────────────────────────
 * BSI_PendingCmd_t
 *
 * Debug-visible snapshot of every command decoded from BaseSystem.data.
 * BaseSystem_Dispatch() writes here on every call.
 * Actual Robot_*() / Gripper_*() calls are NOT made yet —
 * inspect these fields in the live-expression window first.
 * ───────────────────────────────────────────────────────────────────────────── */
typedef struct {

    /* ── Operating mode ──────────────────────────────────────────────────── */
    uint16_t opMode;            /* raw OP_MODE_* value from register           */

    /* ── Motion ──────────────────────────────────────────────────────────── */
    uint8_t  cmd_Home;          /* 1 = home requested (OP_MODE_HOMING)         */
    uint8_t  cmd_Stop;          /* 1 = soft-stop requested                     */
    uint8_t  cmd_EStop;         /* 1 = emergency-stop requested                */

    /* P2P */
    float    cmd_P2P_target_rad;/* converted move target [rad]                 */

    /* Jog */
    float    cmd_Jog_step_rad;  /* jog step converted to [rad]                 */

    /* Performance mode */
    float    cmd_Perf_vel_rad_s;     /* [rad/s]  */
    float    cmd_Perf_accel_rad_s2;  /* [rad/s²] */

    /* Precision test */
    float    cmd_Prec_init_rad;      /* init position  [rad] */
    float    cmd_Prec_final_rad;     /* final position [rad] */
    int16_t  cmd_Prec_repeats;       /* repeat count         */

    /* Sequence */
    float    cmd_Seq_slots_rad[16];  /* each slot converted to [rad] */
    uint16_t cmd_Seq_pairs;          /* number of valid pairs        */

    /* ── Gripper ─────────────────────────────────────────────────────────── */
    uint8_t  cmd_Gripper_manual;     /* GRP_CMD_* value                        */
    uint16_t cmd_Gripper_seq;        /* gripper sequence ID from register       */
    uint8_t  cmd_Gripper_auto_en;    /* 1 = gripper auto-mode enabled           */

    /* ── Diagnostics ────────────────────────────────────────────────────────── */
    uint32_t rx_frame_count;    /* increments every time Modbus writes any register   */
    uint32_t cmd_dispatch_count;/* increments every time a non-idle command is decoded */

    /* ── Write-back (Robot → PC) ─────────────────────────────────────────── */
    float    wb_position_rad;        /* robot.theta  [rad]                     */
    float    wb_velocity_rad_s;      /* robot.omega  [rad/s]                   */
    uint16_t wb_taskBits;            /* Robot_GetState() mapped to bits        */
    uint16_t wb_sensorBits;          /* gripper reed-switch states as bitmask  */
    uint8_t  wb_emergencyActive;     /* 1 when robot is in ESTOP state         */

} BSI_PendingCmd_t;

/* ── Application-level data (PC ↔ Robot register mirror) ──────────────────── */
typedef struct {
    /* Commands received from PC */
    uint16_t operatingMode;
    uint16_t manualGripper;
    uint16_t gripperSequence;
    uint8_t  gripperAutoEnable;
    int16_t  jogDegrees;
    uint16_t testType;
    int16_t  perfVelocity;
    int16_t  perfAcceleration;
    int16_t  precInitPosition;
    int16_t  precFinalPosition;
    int16_t  precRepeatCount;
    int16_t  sequenceSlots[16];
    uint16_t sequencePairs;
    uint16_t p2pUnit;
    int16_t  p2pTarget;
    uint8_t  softStopRequest;

    /* States reported back to PC */
    uint16_t sensorBits;
    uint16_t currentTaskBits;
    float    realPosition;
    float    realVelocity;
    float    realAcceleration;
    uint8_t  emergencyActive;
} BaseSystemInterface_Data_t;

/* ── Top-level handle ──────────────────────────────────────────────────────── */
typedef struct {
    ModbusHandleTypedef        modbus;
    u16u8_t                    registerFrame[BASE_SYSTEM_REG_COUNT];
    BaseSystemInterface_Data_t data;
    BSI_PendingCmd_t           pending;   /* live-expression debug target */

    /* ── Dispatch state (persists across calls, resets with handle) ──────── */
    uint16_t latchedOpMode;        /* last non-idle opMode seen by Update()   */
    /* ── Sequence state machine ─────────────────────────────────────────── */
    uint8_t  latchedSeqValid;      /* 1 = new sequence command pending        */
    uint16_t latchedSeqPairs;      /* number of pairs in latched sequence     */
    int16_t  latchedSeqSlots[16];  /* latched slot values (signed index)      */
    float    seqSlotsRad[16];      /* converted slot values [rad] for runner  */

    /* ── Set Home state machine ─────────────────────────────────────────── */
    uint8_t  setHomeRunning;       /* 1 = set-home sequence in progress       */
    float    setHomeSavedRad;      /* position to return to after homing      */

    /* ── Precision test state machine ───────────────────────────────────── */
    uint8_t  latchedPrecValid;     /* 1 = new precision test pending          */
    int16_t  latchedPrecInit;      /* raw init position from register         */
    int16_t  latchedPrecFinal;     /* raw final position from register        */
    uint16_t latchedPrecRepeats;   /* repeat count (magnitude)                */
    uint8_t  latchedPrecUseIndex;  /* 1 = index unit, 0 = degree unit         */
    uint8_t  precRunning;          /* 1 = precision test executing            */
    uint8_t  precGoingToFinal;     /* 1 = moving to final, 0 = back to init   */
    uint16_t precCurrentRep;       /* completed repeat count                  */
    uint16_t precTotalReps;        /* total repeats requested                 */
    float    precInitRad;          /* init position [rad]                     */
    float    precFinalRad;         /* final position [rad]                    */
    uint8_t  latchedGripperAuto;   /* gripper auto enable captured with seq   */
    uint8_t  seqRunning;           /* 1 = sequence is actively executing      */
    uint16_t seqStep;              /* current step index (0 to steps-1)       */
    uint16_t seqTotalSteps;        /* total steps = pairs × 2                 */

    /* ── Standalone Pick/Place FSM (completely separate from seqRunning) ── */
    uint8_t  grpState;             /* GRP_FSM_* state for standalone Pick/Place */
    uint32_t grpTimer;             /* HAL_GetTick() timestamp for grp FSM     */
    uint8_t  grpCmd;               /* 1=Pick, 2=Place                         */

    /* ── Sequence gripper state machine ─────────────────────────────────── */
    uint8_t  seqGripState;         /* 0=idle 1=down 2=action 3=up             */
    uint32_t seqGripTimer;         /* HAL_GetTick() timestamp for delay       */
    int16_t  latchedJogDeg;        /* last non-zero jog degrees seen by Update */
    int16_t  latchedP2PTarget;     /* latched P2P target value                 */
    uint16_t latchedP2PUnit;       /* latched P2P unit (deg/index)             */
    uint8_t  latchedP2PValid;      /* 1 = new P2P command pending dispatch     */
    int16_t  prevP2PTarget;        /* shadow — detect register change          */
    uint16_t prevP2PUnit;          /* shadow — detect register change          */

    uint16_t prevGripperCmd;       /* edge detection — last gripper command   */
    uint16_t prevGripperReg;       /* shadow — detect REG_MANUAL_GRIPPER write */
    uint8_t  latchedGripperCmd;    /* latched gripper action (Up/Down/Open/Close) */
    uint8_t  latchedGripperSeq;    /* latched gripper sequence (Pick=1/Place=2)   */
    uint8_t  latchedGripperValid;  /* 1 = new gripper command pending             */
    uint16_t prevGripperSeqReg;    /* shadow — detect REG_GRIPPER_SEQ write       */
    uint8_t  latchedGripperSeqCmd; /* latched sequence type (Pick=1 / Place=2)    */
    uint8_t  latchedGripperSeqValid; /* 1 = new Pick/Place sequence pending       */

    /* ── Dispatch diagnostics ───────────────────────────────────────────── */
    uint8_t  dbg_modeChanged;      /* 1 on the loop where mode changed        */
    uint16_t dbg_lastOpMode;       /* last opMode that was actually dispatched */
    uint32_t dbg_home_call_count;  /* total Robot_Home() calls made           */
} BaseSystemInterface_t;

/* ── Public API ────────────────────────────────────────────────────────────── */

/**
 * @brief  Init BaseSystemInterface. Configures embedded Modbus submodule
 *         and seeds safe default register values.
 */
void BaseSystemInterface_Init(BaseSystemInterface_t *hbs,
                               UART_HandleTypeDef    *huart,
                               TIM_HandleTypeDef     *htim,
                               uint8_t                slaveAddress);

/**
 * @brief  Run Modbus worker + sync register frame ↔ data struct.
 *         Call every main-loop iteration.
 */
void BaseSystemInterface_Update(BaseSystemInterface_t *hbs);

/**
 * @brief  Decode all commands from hbs->data into hbs->pending and
 *         copy robot state back into hbs->data for register write-back.
 *
 *         Robot commands are NOT issued yet — inspect hbs->pending
 *         (alias: BaseSystem.pending) in the live-expression window.
 *
 * @param  hbs    BaseSystemInterface handle.
 * @param  robot  Robot handle (read-only for write-back; no motion called).
 */
void BaseSystem_Dispatch(BaseSystemInterface_t *hbs, Robot_t *robot);

#endif /* INC_BASESYSTEMINTERFACE_H_ */
