/*
 * BaseSystemInterface.h
 *
 * Created on: May 19, 2026
 * Author: FRA263/264 Group 5
 *
 * ModbusRTU is embedded as a submodule inside BaseSystemInterface_t.
 * The global instance (BaseSystemInterface_t BaseSystem) is declared
 * in main.c — NOT here. No extern in this header.
 *
 * CHANGED (May 2026):
 *   • BaseSystem_Dispatch() renamed to BaseSystem_Interface_Decode().
 *   • BaseSystem_Interface_Decode() no longer calls any Robot_*() functions.
 *     It only decodes Modbus registers into BSI_PendingCmd_t (hbs->pending).
 *   • All Robot dispatch, FSM state, and gripper sequencing have been moved
 *     to TaskManager (TaskManager.h / TaskManager.c).
 *   • Robot.h is no longer included here — BSI is now Robot-agnostic.
 */

#ifndef INC_BASESYSTEMINTERFACE_H_
#define INC_BASESYSTEMINTERFACE_H_

#include "ModbusRTU.h"

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
/* Used by TaskManager's gripper FSMs.  Kept here as shared constants.         */
#define GRP_FSM_IDLE      0
#define GRP_FSM_DOWN      1
#define GRP_FSM_PENDULUM  2
#define GRP_FSM_ACTION    3
#define GRP_FSM_UP        4
#define GRP_FSM_DONE      5

/* ── Manual Gripper Command IDs (per README 3.3) ──────────────────────────── */
#define GRP_CMD_NONE           0x00
#define GRP_CMD_UP             0x00
#define GRP_CMD_DOWN           0x01
#define GRP_CMD_OPEN           0x02
#define GRP_CMD_CLOSE          0x04

/* ── P2P Unit IDs (per README 3.10) ───────────────────────────────────────── */
#define P2P_UNIT_DEG           0x00   /* bit 0 = 0 → degree  */
#define P2P_UNIT_INDEX         0x01   /* bit 0 = 1 → index   */

/* ─────────────────────────────────────────────────────────────────────────────
 * BSI_PendingCmd_t
 *
 * Written by BaseSystem_Interface_Decode() every call.
 * TaskManager reads these fields and dispatches Robot_*() calls.
 * ─────────────────────────────────────────────────────────────────────────── */
typedef struct {

    /* ── Operating mode ──────────────────────────────────────────────────── */
    uint16_t opMode;             /* raw OP_MODE_* value from register          */

    /* ── Motion ──────────────────────────────────────────────────────────── */
    uint8_t  cmd_Home;           /* 1 = home requested (OP_MODE_HOMING)        */
    uint8_t  cmd_Stop;           /* 1 = soft-stop requested                    */
    uint8_t  cmd_EStop;          /* 1 = emergency-stop requested               */

    /* P2P */
    float    cmd_P2P_target_rad; /* converted move target [rad]                */
    uint8_t  cmd_P2P_valid;      /* 1 = a P2P command is pending (target can
                                    be 0.0f, so value alone is not sufficient) */

    /* Jog */
    float    cmd_Jog_step_rad;   /* jog step converted to [rad]                */

    /* Performance mode */
    float    cmd_Perf_vel_rad_s;      /* [rad/s]  — clamped to RBT_MAX_SPEED  */
    float    cmd_Perf_accel_rad_s2;   /* [rad/s²] — clamped to RBT_MAX_ACCEL  */
    float    cmd_Perf_init_rad;       /* start position [rad] from PREC_INIT  */
    float    cmd_Perf_final_rad;      /* end   position [rad] from PREC_FINAL */
    uint8_t  cmd_Perf_valid;          /* 1 = performance test command pending  */

    /* Precision test */
    float    cmd_Prec_init_rad;       /* init position  [rad] */
    float    cmd_Prec_final_rad;      /* final position [rad] */
    int16_t  cmd_Prec_repeats;        /* repeat count         */

    /* Sequence */
    float    cmd_Seq_slots_rad[16];   /* each slot converted to [rad] */
    uint16_t cmd_Seq_pairs;           /* number of valid pairs        */

    /* ── Gripper ─────────────────────────────────────────────────────────── */
    uint8_t  cmd_Gripper_manual;       /* GRP_CMD_* value                      */
    uint8_t  cmd_Gripper_manual_valid; /* 1 = a gripper cmd is pending (needed
                                          because GRP_CMD_UP == 0x00)          */
    uint16_t cmd_Gripper_seq;          /* gripper sequence ID from register     */
    uint8_t  cmd_Gripper_auto_en;      /* 1 = gripper auto-mode enabled         */

    /* ── Diagnostics ─────────────────────────────────────────────────────── */
    uint32_t rx_frame_count;     /* increments each time Modbus writes any reg */
    uint32_t cmd_decode_count;   /* increments each time a non-idle cmd decoded */

    /* ── Write-back (Robot → PC, filled by TaskManager before Task_Run) ─── */
    float    wb_position_rad;         /* robot.theta  [rad]                    */
    float    wb_velocity_rad_s;       /* robot.omega  [rad/s]                  */
    uint16_t wb_taskBits;             /* Robot_GetState() mapped to bits       */
    uint16_t wb_sensorBits;           /* gripper reed-switch states as bitmask */
    uint8_t  wb_emergencyActive;      /* 1 when robot is in ESTOP state        */

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

    /* ── Latch / edge-detect state (used by BaseSystem_Interface_Decode) ── */
    uint16_t latchedOpMode;
    uint8_t  latchedSeqValid;
    uint16_t latchedSeqPairs;
    int16_t  latchedSeqSlots[16];
    uint8_t  latchedPrecValid;
    int16_t  latchedPrecInit;
    int16_t  latchedPrecFinal;
    uint16_t latchedPrecRepeats;
    uint8_t  latchedPrecUseIndex;
    uint8_t  latchedGripperAuto;
    int16_t  latchedJogDeg;
    int16_t  latchedP2PTarget;
    uint16_t latchedP2PUnit;
    uint8_t  latchedP2PValid;
    int16_t  prevP2PTarget;
    uint16_t prevP2PUnit;
    uint16_t prevGripperCmd;
    uint16_t prevGripperReg;
    uint8_t  latchedGripperCmd;
    uint8_t  latchedGripperSeq;
    uint8_t  latchedGripperValid;
    uint16_t prevGripperSeqReg;
    uint8_t  latchedGripperSeqCmd;
    uint8_t  latchedGripperSeqValid;

    /* ── Decode diagnostics ─────────────────────────────────────────────── */
    uint8_t  dbg_modeChanged;
    uint16_t dbg_lastOpMode;
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
 * @brief  Run Modbus protocol worker + sync register frame → data struct.
 *         Call every main-loop iteration.
 */
void BaseSystemInterface_Update(BaseSystemInterface_t *hbs);

/**
 * @brief  Decode all Modbus registers into hbs->pending.
 *         No Robot_*() calls are made — TaskManager reads pending and
 *         dispatches to the Robot API.
 *
 *         Also writes state back into hbs->data so register write-back
 *         in BaseSystemInterface_Update picks up current robot values.
 *         The caller (TaskManager / main) must update hbs->data fields
 *         (realPosition, realVelocity, sensorBits, etc.) before calling
 *         BaseSystemInterface_Update() each loop.
 *
 * @param  hbs  BaseSystemInterface handle.
 */
void BaseSystem_Interface_Decode(BaseSystemInterface_t *hbs);

#endif /* INC_BASESYSTEMINTERFACE_H_ */
