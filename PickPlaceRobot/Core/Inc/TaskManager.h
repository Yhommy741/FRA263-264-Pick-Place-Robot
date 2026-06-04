/*
 * TaskManager.h
 *
 * Created on: May 2026
 * Author: FRA263/264 Group 5
 *
 * Arbitrates service calls from BaseSystemInterface (Modbus) and
 * JoystickInterface, enforces a robot-state guard, then dispatches
 * the winning command to the Robot API.
 *
 * Also owns ALL gripper FSM state that previously lived inside
 * BaseSystemInterface_t, keeping BSI as a pure Modbus decoder.
 *
 * Typical main-loop usage:
 *
 *   BaseSystemInterface_Update(&BaseSystem);          // Modbus comms
 *   JoystickInterface_Update(&joystick);              // UART parse
 *
 *   BaseSystem_Interface_Decode(&BaseSystem, &robot); // register → pending (no Robot calls)
 *
 *   Task_PostFromModbus  (&taskMgr, &BaseSystem);     // feed event queue
 *   Task_PostFromJoystick(&taskMgr, &joystick);       // feed event queue
 *
 *   Task_Run(&taskMgr, &robot);                       // arbitrate + dispatch
 */

#ifndef INC_TASKMANAGER_H_
#define INC_TASKMANAGER_H_

#include "main.h"
#include "BaseSystemInterface.h"
#include "JoystickInterface.h"
#include "Robot.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Event IDs
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef enum {
    TASK_EVT_NONE        = 0,
    TASK_EVT_ESTOP,          /* Emergency stop — highest priority, always wins   */
    TASK_EVT_STOP,           /* Soft stop                                         */
    TASK_EVT_HOME,           /* Full homing sequence                              */
    TASK_EVT_SET_HOME,       /* Declare current position as home                  */
    TASK_EVT_MOVE,           /* Point-to-point move (arg_f = target_rad)          */
    TASK_EVT_JOG_STEP,       /* Incremental step   (arg_f = step_rad)             */
    TASK_EVT_JOG_VEL,        /* Continuous velocity (arg_f = speed_rad_s)         */
    TASK_EVT_GRIPPER_MANUAL, /* Single gripper action (arg_u8 = GRP_CMD_*)        */
    TASK_EVT_GRIPPER_SEQ,    /* Standalone Pick / Place (arg_u8 = 1=Pick,2=Place) */
    TASK_EVT_SEQUENCE,       /* Multi-position auto sequence                      */
    TASK_EVT_PREC_TEST,      /* Precision back-and-forth test                     */
    TASK_EVT_PERF_TEST,      /* Performance test — 180° move at custom vel/accel  */
} TaskEvent_ID_t;

/* ── Event source ─────────────────────────────────────────────────────────── */
#define TASK_SRC_MODBUS    0x01
#define TASK_SRC_JOYSTICK  0x02
#define TASK_SRC_GPIO      0x03   /* physical E-stop / mode button */

/* ── Event queue depth ────────────────────────────────────────────────────── */
#define TASK_QUEUE_SIZE    8

/* ── Joystick command byte definitions ───────────────────────────────────── *
 *  These must match whatever the joystick firmware transmits as cmd byte [0]. *
 * ─────────────────────────────────────────────────────────────────────────── */
/* JOY_CMD_* — matches StateMachine.cpp protocol spec */
#define JOY_CMD_NONE         0x00
#define JOY_CMD_MOVE         0x01
#define JOY_CMD_STOP         0x02
#define JOY_CMD_SET_HOME     0x04
#define JOY_CMD_HOME         0x05
#define JOY_CMD_JOG_VEL_CCW  0x06
#define JOY_CMD_JOG_VEL_CW   0x07
#define JOY_CMD_JOG_STEP_CCW 0x08
#define JOY_CMD_JOG_STEP_CW  0x09
#define JOY_CMD_GRP_UP       0x0A
#define JOY_CMD_GRP_DOWN     0x0B
#define JOY_CMD_GRP_CLOSE    0x0C
#define JOY_CMD_GRP_OPEN     0x0D

/* ═══════════════════════════════════════════════════════════════════════════
 *  TaskEvent_t  — one entry in the FIFO queue
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    TaskEvent_ID_t id;
    uint8_t        source;      /* TASK_SRC_* */
    float          arg_f;       /* float argument (rad, rad/s)   */
    uint8_t        arg_u8;      /* small integer argument        */
} TaskEvent_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Gripper FSM state (moved here from BaseSystemInterface_t)
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    /* ── Multi-position sequence gripper FSM ───────────────────────────── */
    uint8_t  seqGripState;       /* GRP_FSM_* for sequence steps             */
    uint32_t seqGripTimer;

    /* ── Standalone Pick/Place FSM ─────────────────────────────────────── */
    uint8_t  grpState;           /* GRP_FSM_* for standalone Pick/Place      */
    uint32_t grpTimer;
    uint8_t  grpCmd;             /* 1 = Pick, 2 = Place                      */
} TM_GripperFSM_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  TaskManager_t  — top-level handle
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    /* ── Event FIFO ────────────────────────────────────────────────────── */
    TaskEvent_t queue[TASK_QUEUE_SIZE];
    uint8_t     q_head;          /* index of next event to consume           */
    uint8_t     q_tail;          /* index where next event will be written   */
    uint8_t     q_count;         /* number of events currently in the queue  */

    /* ── Active task state ─────────────────────────────────────────────── */
    TaskEvent_ID_t activeTask;   /* command currently being executed         */
    uint8_t        activeSource; /* source of the active command             */

    /* ── Sequence runner state ─────────────────────────────────────────── */
    uint8_t  seqRunning;
    uint16_t seqStep;
    uint16_t seqTotalSteps;
    float    seqSlotsRad[16];
    uint8_t  gripperAutoEn;

    /* ── Precision test runner state ───────────────────────────────────── */
    uint8_t  precRunning;
    uint8_t  precGoingToFinal;
    uint16_t precCurrentRep;
    uint16_t precTotalReps;
    float    precInitRad;
    float    precFinalRad;

    /* ── Performance test runner state ─────────────────────────────────── */
    uint8_t  perfRunning;
    uint8_t  perfGoingToFinal;       /* 1 = moving to final, 0 = returning    */
    uint8_t  perfRestartPending;     /* 1 = stop robot then restart with new params */
    float    perfVelRad;             /* velocity constraint  [rad/s]          */
    float    perfAccelRad;           /* accel constraint     [rad/s²]         */
    float    perfInitRad;            /* start position [rad]                  */
    float    perfFinalRad;           /* end   position [rad]                  */

    /* ── Gripper FSMs ──────────────────────────────────────────────────── */
    TM_GripperFSM_t grp;

    /* ── Diagnostics ───────────────────────────────────────────────────── */
    uint32_t dbg_eventsPosted;   /* total events posted since init           */
    uint32_t dbg_eventsDropped;  /* total events dropped (queue full)        */
    uint32_t dbg_eventsRun;      /* total events dispatched to Robot         */
    uint32_t dbg_estopCount;

    /* ── System flags for main.c ───────────────────────────────────────── */

    /* sysResetRequested: (unused — kept for ABI compatibility)             */
    uint8_t sysResetRequested;

    /* sysHardStopRequested: set by TASK_EVT_STOP dispatch.                 *
     * Signals main.c to enter SYS_STATE_SOFT_ESTOP — motor is already cut  *
     * by Robot_EStop(). This state has NO software exit path; only an MCU  *
     * power cycle or hardware reset button will clear it.                   *
     * main.c checks and latches this flag; it is never cleared in software. */
    uint8_t sysHardStopRequested;
} TaskManager_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise the TaskManager. Call once after Robot_Init().
 */
void Task_Init(TaskManager_t *tm);

/**
 * @brief  Read BaseSystem.pending (filled by BaseSystem_Interface_Decode)
 *         and push any new commands onto the event queue.
 *         Call every main-loop iteration, after BaseSystem_Interface_Decode().
 */
void Task_PostFromModbus(TaskManager_t *tm, BaseSystemInterface_t *hbs);

/**
 * @brief  Read a freshly-parsed joystick frame and push the command onto
 *         the event queue.
 *         Call every main-loop iteration, after JoystickInterface_Update()
 *         returns non-zero (or unconditionally — it is idempotent when no
 *         new frame arrived).
 */
void Task_PostFromJoystick(TaskManager_t *tm, JoystickInterface_t *joy);

/**
 * @brief  Arbitrate the event queue and dispatch the winning command to
 *         the Robot API.  Also ticks the running FSMs (sequence, precision
 *         test, gripper).
 *         Call every main-loop iteration, after both Post functions.
 */
void Task_Run(TaskManager_t *tm, Robot_t *robot);

/**
 * @brief  Push a raw event directly (used internally and for GPIO/button events).
 *         Returns 1 if the event was accepted, 0 if the queue was full.
 */
uint8_t Task_PostEvent(TaskManager_t *tm, TaskEvent_t evt);

#endif /* INC_TASKMANAGER_H_ */
