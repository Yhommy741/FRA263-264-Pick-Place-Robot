/*
 * TaskManager.c
 *
 * Created on: May 2026
 * Author: FRA263/264 Group 5
 *
 * Arbitration rules (highest priority first):
 *   1. ESTOP   — any source, always executed immediately, clears the queue
 *   2. STOP    — any source, clears moving tasks
 *   3. Joystick motion (JOG_STEP, JOG_VEL, HOME) — operator override
 *   4. Modbus motion (HOME, MOVE, SEQUENCE, PREC_TEST, JOG_STEP)
 *   5. Gripper commands — either source, no motion dependency
 *
 * State guard: commands that conflict with the current robot state are
 * silently dropped (e.g. MOVE while homing, HOME while already homing).
 */

#include "TaskManager.h"
#include "RobotConfig.h"
#include <string.h>
#include <math.h>

#define TWO_PI  6.28318530718f

/* ── Local helpers ────────────────────────────────────────────────────────── */
static inline uint8_t _queue_empty(const TaskManager_t *tm)
{ return (tm->q_count == 0); }

static inline uint8_t _queue_full(const TaskManager_t *tm)
{ return (tm->q_count >= TASK_QUEUE_SIZE); }

static uint8_t _queue_has_estop(const TaskManager_t *tm)
{
    for (uint8_t i = 0; i < tm->q_count; i++)
    {
        uint8_t idx = (tm->q_head + i) % TASK_QUEUE_SIZE;
        if (tm->queue[idx].id == TASK_EVT_ESTOP) return 1;
    }
    return 0;
}

static uint8_t _queue_has_stop(const TaskManager_t *tm)
{
    for (uint8_t i = 0; i < tm->q_count; i++)
    {
        uint8_t idx = (tm->q_head + i) % TASK_QUEUE_SIZE;
        if (tm->queue[idx].id == TASK_EVT_STOP) return 1;
    }
    return 0;
}

static TaskEvent_t _queue_pop(TaskManager_t *tm)
{
    TaskEvent_t evt = tm->queue[tm->q_head];
    tm->q_head = (tm->q_head + 1) % TASK_QUEUE_SIZE;
    tm->q_count--;
    return evt;
}

static void _queue_flush(TaskManager_t *tm)
{
    tm->q_head = 0; tm->q_tail = 0; tm->q_count = 0;
}

/* ── _seq_move_directed ───────────────────────────────────────────────────── *
 * Moves the robot to abs(signed_slot_rad) with the direction encoded by its   *
 * sign (positive = CW, negative = CCW).                                       *
 *                                                                               *
 * robot->theta is an unbounded accumulator (encoder-derived), NOT normalized  *
 * to 0–2π.  The abs_target from the BaseSystem IS in 0–2π.  We must find the  *
 * correct absolute target by:                                                  *
 *   1. Bring abs_target into the same revolution as theta using floor division.*
 *   2. Check if that candidate gives the correct direction (sign of delta).    *
 *   3. If not, add or subtract one full revolution to flip direction.          *
 * ─────────────────────────────────────────────────────────────────────────── */
static void _seq_move_directed(Robot_t *robot, float signed_slot_rad,
                                float omega_max, float alpha_max)
{
    float abs_target = fabsf(signed_slot_rad);
    float theta      = robot->theta;

    /* Step 1: align abs_target to the same revolution as theta.
     * floor(theta / 2π) gives the revolution count; multiply back to get
     * the base offset of the current revolution.                           */
    float rev_base = floorf(theta / TWO_PI) * TWO_PI;
    float target   = rev_base + abs_target;

    /* Step 2: enforce direction by adjusting by ±2π if needed.
     *   CW  (positive slot): delta must be > 0  → if target <= theta, +2π
     *   CCW (negative slot): delta must be < 0  → if target >= theta, -2π */
    if (signed_slot_rad >= 0.0f)
    {
        /* CW */
        if (target <= theta) target += TWO_PI;
    }
    else
    {
        /* CCW */
        if (target >= theta) target -= TWO_PI;
    }

    Robot_MoveConstrained(robot, target, omega_max, alpha_max);
}

/* ── State guard ──────────────────────────────────────────────────────────── */
static uint8_t _guard_ok(TaskEvent_ID_t id, const Robot_t *robot)
{
    Robot_State_t rs = Robot_GetState(robot);

    switch (id)
    {
        case TASK_EVT_ESTOP:
        case TASK_EVT_STOP:
            return 1;

        case TASK_EVT_HOME:
            if (rs == ROBOT_ESTOP) return 0;
            if (rs == ROBOT_HOMING_FAST_STATE  ||
                rs == ROBOT_HOMING_BACKOFF_STATE ||
                rs == ROBOT_HOMING_SLOW_STATE   ||
                rs == ROBOT_HOMING_OFFSET_STATE) return 0;
            return 1;

        case TASK_EVT_MOVE:
        case TASK_EVT_JOG_STEP:
        case TASK_EVT_JOG_VEL:
            if (rs == ROBOT_ESTOP) return 0;
            if (rs == ROBOT_HOMING_FAST_STATE  ||
                rs == ROBOT_HOMING_BACKOFF_STATE ||
                rs == ROBOT_HOMING_SLOW_STATE   ||
                rs == ROBOT_HOMING_OFFSET_STATE) return 0;
            return 1;

        case TASK_EVT_SEQUENCE:
            if (rs == ROBOT_ESTOP) return 0;
            if (rs == ROBOT_HOMING_FAST_STATE  ||
                rs == ROBOT_HOMING_BACKOFF_STATE ||
                rs == ROBOT_HOMING_SLOW_STATE   ||
                rs == ROBOT_HOMING_OFFSET_STATE) return 0;
            return 1;

        case TASK_EVT_PREC_TEST:
            if (rs == ROBOT_ESTOP) return 0;
            if (rs == ROBOT_HOMING_FAST_STATE  ||
                rs == ROBOT_HOMING_BACKOFF_STATE ||
                rs == ROBOT_HOMING_SLOW_STATE   ||
                rs == ROBOT_HOMING_OFFSET_STATE) return 0;
            return 1;

        case TASK_EVT_PERF_TEST:
            if (rs == ROBOT_ESTOP) return 0;
            if (rs == ROBOT_HOMING_FAST_STATE  ||
                rs == ROBOT_HOMING_BACKOFF_STATE ||
                rs == ROBOT_HOMING_SLOW_STATE   ||
                rs == ROBOT_HOMING_OFFSET_STATE) return 0;
            return 1;

        case TASK_EVT_SET_HOME:
            if (rs == ROBOT_ESTOP)  return 0;
            if (rs == ROBOT_MOVE || rs == ROBOT_JOG_VEL || rs == ROBOT_JOG_STEP) return 0;
            if (rs == ROBOT_HOMING_FAST_STATE  ||
                rs == ROBOT_HOMING_BACKOFF_STATE ||
                rs == ROBOT_HOMING_SLOW_STATE   ||
                rs == ROBOT_HOMING_OFFSET_STATE) return 0;
            return 1;

        case TASK_EVT_GRIPPER_MANUAL:
        case TASK_EVT_GRIPPER_SEQ:
            if (rs == ROBOT_ESTOP) return 0;
            return 1;

        default:
            return 1;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Task_Init
 * ═══════════════════════════════════════════════════════════════════════════ */
void Task_Init(TaskManager_t *tm)
{
    if (tm == NULL) return;
    memset(tm, 0, sizeof(TaskManager_t));
    tm->seqRunning          = 0;
    tm->seqStep             = 0;
    tm->seqPendingAfterHome = 0;
    tm->precRunning         = 0;
    tm->perfRunning         = 0;
    tm->activeTask    = TASK_EVT_NONE;
    tm->sysResetRequested = 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Task_PostEvent  (public helper)
 * ═══════════════════════════════════════════════════════════════════════════ */
uint8_t Task_PostEvent(TaskManager_t *tm, TaskEvent_t evt)
{
    if (_queue_full(tm))
    {
        tm->dbg_eventsDropped++;
        return 0;
    }
    tm->queue[tm->q_tail] = evt;
    tm->q_tail = (tm->q_tail + 1) % TASK_QUEUE_SIZE;
    tm->q_count++;
    tm->dbg_eventsPosted++;
    return 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Task_PostFromModbus
 * ═══════════════════════════════════════════════════════════════════════════ */
void Task_PostFromModbus(TaskManager_t *tm, BaseSystemInterface_t *hbs)
{
    BSI_PendingCmd_t *pnd = &hbs->pending;

    /* ── E-Stop ─────────────────────────────────────────────────────────── */
    if (pnd->cmd_EStop)
    {
        TaskEvent_t e = { TASK_EVT_ESTOP, TASK_SRC_MODBUS, 0.0f, 0 };
        Task_PostEvent(tm, e);
    }

    /* ── Soft stop ──────────────────────────────────────────────────────── */
    if (pnd->cmd_Stop)
    {
        TaskEvent_t e = { TASK_EVT_STOP, TASK_SRC_MODBUS, 0.0f, 0 };
        Task_PostEvent(tm, e);
    }

    if (pnd->cmd_EStop) goto clear_pending;

    /* ── Home ───────────────────────────────────────────────────────────── */
    if (pnd->cmd_Home)
    {
        TaskEvent_t e = { TASK_EVT_HOME, TASK_SRC_MODBUS, 0.0f, 0 };
        Task_PostEvent(tm, e);
    }

    /* ── Set Home ───────────────────────────────────────────────────────── */
    if (pnd->opMode == OP_MODE_SET_HOME)
    {
        TaskEvent_t e = { TASK_EVT_SET_HOME, TASK_SRC_MODBUS, 0.0f, 0 };
        Task_PostEvent(tm, e);
    }

    /* ── P2P Move ───────────────────────────────────────────────────────── */
    if (pnd->cmd_P2P_valid)
    {
        TaskEvent_t e = { TASK_EVT_MOVE, TASK_SRC_MODBUS, pnd->cmd_P2P_target_rad, 0 };
        Task_PostEvent(tm, e);
    }

    /* ── Jog step ───────────────────────────────────────────────────────── */
    if (pnd->cmd_Jog_step_rad != 0.0f)
    {
        TaskEvent_t e = { TASK_EVT_JOG_STEP, TASK_SRC_MODBUS, pnd->cmd_Jog_step_rad, 0 };
        Task_PostEvent(tm, e);
    }

    /* ── Multi-position sequence ────────────────────────────────────────── */
    if (pnd->cmd_Seq_pairs > 0 && !tm->seqRunning)
    {
        TaskEvent_t e;
        e.id     = TASK_EVT_SEQUENCE;
        e.source = TASK_SRC_MODBUS;
        e.arg_f  = 0.0f;
        e.arg_u8 = pnd->cmd_Gripper_auto_en;
        Task_PostEvent(tm, e);

        tm->seqTotalSteps = pnd->cmd_Seq_pairs * 2;
        if (tm->seqTotalSteps > 16) tm->seqTotalSteps = 16;
        for (int i = 0; i < 16; i++)
            tm->seqSlotsRad[i] = pnd->cmd_Seq_slots_rad[i];
        tm->gripperAutoEn = pnd->cmd_Gripper_auto_en;
    }

    /* ── Precision test ─────────────────────────────────────────────────── */
    if (pnd->cmd_Prec_repeats > 0)
    {
        TaskEvent_t e = { TASK_EVT_PREC_TEST, TASK_SRC_MODBUS, 0.0f, 0 };
        Task_PostEvent(tm, e);
        tm->precInitRad   = pnd->cmd_Prec_init_rad;
        tm->precFinalRad  = pnd->cmd_Prec_final_rad;
        tm->precTotalReps = (uint16_t)pnd->cmd_Prec_repeats;
    }

    /* ── Performance test ───────────────────────────────────────────────── */
    if (pnd->cmd_Perf_valid)
    {
        tm->perfVelRad   = pnd->cmd_Perf_vel_rad_s;
        tm->perfAccelRad = pnd->cmd_Perf_accel_rad_s2;
        tm->perfInitRad  = pnd->cmd_Perf_init_rad;
        tm->perfFinalRad = pnd->cmd_Perf_final_rad;

        if (tm->perfRunning)
        {
            tm->perfRunning      = 0;
            tm->perfGoingToFinal = 0;
            tm->perfRestartPending = 1;
        }

        TaskEvent_t e = { TASK_EVT_PERF_TEST, TASK_SRC_MODBUS, 0.0f, 0 };
        Task_PostEvent(tm, e);
    }

    /* ── Manual gripper ─────────────────────────────────────────────────── */
    if (pnd->cmd_Gripper_manual_valid)
    {
        TaskEvent_t e = { TASK_EVT_GRIPPER_MANUAL, TASK_SRC_MODBUS, 0.0f, pnd->cmd_Gripper_manual };
        Task_PostEvent(tm, e);
    }

    /* ── Gripper Pick / Place sequence ──────────────────────────────────── */
    if (pnd->cmd_Gripper_seq != 0)
    {
        TaskEvent_t e = { TASK_EVT_GRIPPER_SEQ, TASK_SRC_MODBUS, 0.0f, (uint8_t)(pnd->cmd_Gripper_seq & 0xFF) };
        Task_PostEvent(tm, e);
    }

    clear_pending:
    pnd->cmd_P2P_target_rad       = 0.0f;
    pnd->cmd_P2P_valid            = 0;
    pnd->cmd_Jog_step_rad         = 0.0f;
    pnd->cmd_Home                 = 0;
    pnd->cmd_Stop                 = 0;
    pnd->cmd_EStop                = 0;
    pnd->cmd_Seq_pairs            = 0;
    pnd->cmd_Prec_repeats         = 0;
    pnd->cmd_Perf_valid           = 0;
    pnd->cmd_Gripper_manual       = 0;
    pnd->cmd_Gripper_manual_valid = 0;
    pnd->cmd_Gripper_seq          = 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Task_PostFromJoystick
 * ═══════════════════════════════════════════════════════════════════════════ */
void Task_PostFromJoystick(TaskManager_t *tm, JoystickInterface_t *joy)
{
    uint8_t cmd  = JoystickInterface_Get_Command(joy);
    float   data = JoystickInterface_Get_Data(joy);

    if (cmd == JOY_CMD_NONE) return;

    TaskEvent_t e;
    e.source = TASK_SRC_JOYSTICK;
    e.arg_u8 = 0;

    switch (cmd)
    {
        case JOY_CMD_MOVE:
            e.id    = TASK_EVT_MOVE;
            e.arg_f = data;
            break;
        case JOY_CMD_STOP:
            e.id    = TASK_EVT_STOP;
            e.arg_f = 0.0f;
            break;
        case JOY_CMD_SET_HOME:
            e.id    = TASK_EVT_SET_HOME;
            e.arg_f = data;
            break;
        case JOY_CMD_HOME:
            e.id    = TASK_EVT_HOME;
            e.arg_f = 0.0f;
            break;
        case JOY_CMD_JOG_VEL_CCW:
            e.id    = TASK_EVT_JOG_VEL;
            e.arg_f = fabsf(data);
            break;
        case JOY_CMD_JOG_VEL_CW:
            e.id    = TASK_EVT_JOG_VEL;
            e.arg_f = -fabsf(data);
            break;
        case JOY_CMD_JOG_STEP_CCW:
            e.id    = TASK_EVT_JOG_STEP;
            e.arg_f = fabsf(data);
            break;
        case JOY_CMD_JOG_STEP_CW:
            e.id    = TASK_EVT_JOG_STEP;
            e.arg_f = -fabsf(data);
            break;
        case JOY_CMD_GRP_UP:
            e.id    = TASK_EVT_GRIPPER_MANUAL;
            e.arg_u8 = GRP_CMD_UP;
            e.arg_f  = 0.0f;
            break;
        case JOY_CMD_GRP_DOWN:
            e.id    = TASK_EVT_GRIPPER_MANUAL;
            e.arg_u8 = GRP_CMD_DOWN;
            e.arg_f  = 0.0f;
            break;
        case JOY_CMD_GRP_CLOSE:
            e.id    = TASK_EVT_GRIPPER_MANUAL;
            e.arg_u8 = GRP_CMD_CLOSE;
            e.arg_f  = 0.0f;
            break;
        case JOY_CMD_GRP_OPEN:
            e.id    = TASK_EVT_GRIPPER_MANUAL;
            e.arg_u8 = GRP_CMD_OPEN;
            e.arg_f  = 0.0f;
            break;
        default:
            return;
    }

    Task_PostEvent(tm, e);
    joy->parsed_cmd  = JOY_CMD_NONE;
    joy->parsed_data = 0.0f;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  _gripper_seq_tick  —  Standalone Pick/Place FSM (manual command)
 *
 *  No pendulum wait — MoveDown fires immediately for Place so manual
 *  commands respond instantly.  The sequence FSM (_seq_grip_tick) retains
 *  the pendulum delay for automated runs.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void _gripper_seq_tick(TM_GripperFSM_t *grp, Robot_t *robot)
{
    switch (grp->grpState)
    {
        case GRP_FSM_IDLE:
            break;

        case GRP_FSM_PENDULUM:
            /* Manual Place: skip pendulum wait — move down immediately. */
            Robot_Gripper_MoveDown(robot);
            grp->grpTimer = HAL_GetTick();
            grp->grpState = GRP_FSM_DOWN;
            break;

        case GRP_FSM_DOWN:
            if (HAL_GetTick() - grp->grpTimer >= GRP_WAIT_TIME)
            {
                if (grp->grpCmd == 1) Robot_Gripper_Close(robot);
                else                  Robot_Gripper_Open(robot);
                grp->grpTimer = HAL_GetTick();
                grp->grpState = GRP_FSM_ACTION;
            }
            break;

        case GRP_FSM_ACTION:
            if (HAL_GetTick() - grp->grpTimer >= GRP_WAIT_TIME)
            {
                Robot_Gripper_MoveUp(robot);
                grp->grpTimer = HAL_GetTick();
                grp->grpState = GRP_FSM_UP;
            }
            break;

        case GRP_FSM_UP:
            if (HAL_GetTick() - grp->grpTimer >= GRP_WAIT_TIME)
                grp->grpState = GRP_FSM_IDLE;
            break;

        default:
            grp->grpState = GRP_FSM_IDLE;
            break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  _seq_grip_tick  —  Gripper FSM embedded in multi-position sequence
 * ═══════════════════════════════════════════════════════════════════════════ */
static void _seq_grip_tick(TaskManager_t *tm, Robot_t *robot)
{
    TM_GripperFSM_t *grp = &tm->grp;

    switch (grp->seqGripState)
    {
        case GRP_FSM_IDLE:
            if (Robot_IsIdle(robot))
            {
                if (tm->seqStep % 2 == 0)
                {
                    Robot_Gripper_MoveDown(robot);
                    grp->seqGripTimer = HAL_GetTick();
                    grp->seqGripState = GRP_FSM_DOWN;
                }
                else
                {
                    grp->seqGripTimer = HAL_GetTick();
                    grp->seqGripState = GRP_FSM_PENDULUM;
                }
            }
            break;

        case GRP_FSM_PENDULUM:
            if (HAL_GetTick() - grp->seqGripTimer >= GRP_WAIT_PENDULUM_TIME)
            {
                Robot_Gripper_MoveDown(robot);
                grp->seqGripTimer = HAL_GetTick();
                grp->seqGripState = GRP_FSM_DOWN;
            }
            break;

        case GRP_FSM_DOWN:
            if (HAL_GetTick() - grp->seqGripTimer >= GRP_WAIT_TIME)
            {
                if (tm->seqStep % 2 == 0) Robot_Gripper_Close(robot);
                else                      Robot_Gripper_Open(robot);
                grp->seqGripTimer = HAL_GetTick();
                grp->seqGripState = GRP_FSM_ACTION;
            }
            break;

        case GRP_FSM_ACTION:
            if (HAL_GetTick() - grp->seqGripTimer >= GRP_WAIT_TIME)
            {
                Robot_Gripper_MoveUp(robot);
                grp->seqGripTimer = HAL_GetTick();
                grp->seqGripState = GRP_FSM_UP;
            }
            break;

        case GRP_FSM_UP:
            if (HAL_GetTick() - grp->seqGripTimer >= GRP_WAIT_TIME)
            {
                grp->seqGripState = GRP_FSM_IDLE;
                tm->seqStep++;
                if (tm->seqStep >= tm->seqTotalSteps)
                    tm->seqRunning = 0;
                else
                    _seq_move_directed(robot, tm->seqSlotsRad[tm->seqStep],
                                       RBT_MAX_SPEED, RBT_MAX_ACCEL);
            }
            break;

        default:
            grp->seqGripState = GRP_FSM_IDLE;
            break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Task_Run
 * ═══════════════════════════════════════════════════════════════════════════ */
void Task_Run(TaskManager_t *tm, Robot_t *robot)
{
    /* ── Step 1: E-Stop scan ─────────────────────────────────────────────── */
    if (Robot_GetState(robot) != ROBOT_ESTOP && _queue_has_estop(tm))
    {
        _queue_flush(tm);
        tm->seqRunning          = 0;
        tm->seqPendingAfterHome = 0;
        tm->precRunning         = 0;
        tm->grp.grpState     = GRP_FSM_IDLE;
        tm->grp.seqGripState = GRP_FSM_IDLE;
        Robot_EStop(robot);
        tm->activeTask = TASK_EVT_ESTOP;
        tm->dbg_estopCount++;
        tm->dbg_eventsRun++;
        return;
    }

    /* ── Step 1b: Soft-Stop scan ─────────────────────────────────────────── */
    if (_queue_has_stop(tm))
    {
        _queue_flush(tm);
        tm->seqRunning           = 0;
        tm->seqStep              = 0;
        tm->seqPendingAfterHome  = 0;
        tm->precRunning          = 0;
        tm->precGoingToFinal     = 0;
        tm->perfRunning          = 0;
        tm->perfGoingToFinal     = 0;
        tm->perfRestartPending   = 0;
        tm->grp.grpState         = GRP_FSM_IDLE;
        tm->grp.seqGripState     = GRP_FSM_IDLE;
        tm->activeTask           = TASK_EVT_NONE;
        Robot_Stop(robot);
        tm->dbg_eventsRun++;
        return;
    }

    /* ── Step 2: Tick running FSMs ───────────────────────────────────────── */

    /* Precision test runner */
    if (tm->precRunning)
    {
        if (Robot_IsIdle(robot))
        {
            if (tm->precGoingToFinal)
            {
                tm->precGoingToFinal = 0;
                Robot_MoveConstrained(robot, tm->precInitRad,
                                      RBT_MAX_SPEED, RBT_MAX_ACCEL);
            }
            else
            {
                tm->precCurrentRep++;
                if (tm->precCurrentRep >= tm->precTotalReps)
                {
                    tm->precRunning = 0;
                }
                else
                {
                    tm->precGoingToFinal = 1;
                    Robot_MoveConstrained(robot, tm->precFinalRad,
                                          RBT_MAX_SPEED, RBT_MAX_ACCEL);
                }
            }
        }
        _gripper_seq_tick(&tm->grp, robot);
        return;
    }

    /* Performance test restart */
    if (tm->perfRestartPending)
    {
        tm->perfRestartPending = 0;
        Robot_Stop(robot);
        return;
    }

    /* Performance test runner */
    if (tm->perfRunning)
    {
        if (Robot_IsIdle(robot))
        {
            if (tm->perfGoingToFinal)
            {
                tm->perfGoingToFinal = 0;
                Robot_PerfTest_Start(robot, 0.0f,
                                     tm->perfVelRad, tm->perfAccelRad);
            }
            else
            {
                tm->perfRunning = 0;
            }
        }
        return;
    }

    /* Sequence auto-start after homing ─────────────────────────────────────── *
     * If a START was received during homing, seqPendingAfterHome is set.       *
     * Once the robot is idle (homing done), kick off the sequence immediately  *
     * using the latest seqSlotsRad[] and gripperAutoEn without another START.  */
    if (tm->seqPendingAfterHome && !tm->seqRunning && Robot_IsIdle(robot))
    {
        tm->seqPendingAfterHome = 0;
        tm->seqStep             = 0;
        tm->grp.seqGripState    = GRP_FSM_IDLE;
        tm->seqRunning          = 1;
        /* First slot: use Robot_Move (shortest path / P2P) so the robot goes
         * directly to the starting position without forcing a direction.       */
        Robot_Move(robot, tm->seqSlotsRad[0]);
        return;
    }

    /* Sequence runner */
    if (tm->seqRunning)
    {
        if (!tm->gripperAutoEn)
        {
            if (Robot_IsIdle(robot))
            {
                tm->seqStep++;
                if (tm->seqStep >= tm->seqTotalSteps)
                    tm->seqRunning = 0;
                else
                    _seq_move_directed(robot, tm->seqSlotsRad[tm->seqStep],
                                       RBT_MAX_SPEED, RBT_MAX_ACCEL);
            }
        }
        else
        {
            _seq_grip_tick(tm, robot);
        }
        return;
    }

    /* Standalone Pick/Place FSM */
    _gripper_seq_tick(&tm->grp, robot);

    /* ── Step 3: Pop and dispatch the next queued event ─────────────────── */
    if (_queue_empty(tm)) return;

    /* ── Arbitration: Joystick motion preempts queued Modbus motion ──────── */
    uint8_t best_idx  = tm->q_head;
    uint8_t found_joy = 0;
    for (uint8_t i = 0; i < tm->q_count; i++)
    {
        uint8_t idx = (tm->q_head + i) % TASK_QUEUE_SIZE;
        TaskEvent_t *e = &tm->queue[idx];
        if (e->source == TASK_SRC_JOYSTICK &&
           (e->id == TASK_EVT_JOG_STEP || e->id == TASK_EVT_JOG_VEL ||
            e->id == TASK_EVT_HOME     || e->id == TASK_EVT_STOP))
        {
            best_idx  = idx;
            found_joy = 1;
            break;
        }
    }

    TaskEvent_t evt;
    if (found_joy)
    {
        TaskEvent_t tmp         = tm->queue[tm->q_head];
        tm->queue[tm->q_head]   = tm->queue[best_idx];
        tm->queue[best_idx]     = tmp;
    }
    evt = _queue_pop(tm);

    if (!_guard_ok(evt.id, robot))
    {
        /* When a sequence is rejected during homing, set a pending flag instead
         * of re-queuing the event.  The flag is checked after homing completes
         * and the sequence auto-starts using the latest seqSlotsRad[] and
         * gripperAutoEn — which are always kept up to date by Task_PostFromModbus
         * on every START press, so a second START simply refreshes the data.    */
        if (evt.id == TASK_EVT_SEQUENCE)
            tm->seqPendingAfterHome = 1;
        return;
    }

    tm->activeTask   = evt.id;
    tm->activeSource = evt.source;
    tm->dbg_eventsRun++;

    switch (evt.id)
    {
        case TASK_EVT_STOP:
            tm->seqRunning           = 0;
            tm->seqStep              = 0;
            tm->seqPendingAfterHome  = 0;
            tm->precRunning          = 0;
            tm->precGoingToFinal     = 0;
            tm->perfRunning          = 0;
            tm->perfGoingToFinal     = 0;
            tm->perfRestartPending   = 0;
            tm->grp.grpState         = GRP_FSM_IDLE;
            tm->grp.seqGripState     = GRP_FSM_IDLE;
            tm->activeTask           = TASK_EVT_NONE;
            _queue_flush(tm);
            Robot_Stop(robot);
            break;

        case TASK_EVT_HOME:
            Robot_Home(robot);
            break;

        case TASK_EVT_SET_HOME:
            __disable_irq();
            robot->home_offset  = (float)robot->encoder.Rad;
            robot->theta        = 0.0f;
            robot->theta_target = 0.0f;
            __enable_irq();
            break;

        case TASK_EVT_MOVE:
            Robot_MoveConstrained(robot, evt.arg_f,
                                  RBT_MAX_SPEED, RBT_MAX_ACCEL);
            break;

        case TASK_EVT_JOG_STEP:
            Robot_JogStep(robot, evt.arg_f);
            break;

        case TASK_EVT_JOG_VEL:
            Robot_JogVel(robot, evt.arg_f);
            break;

        case TASK_EVT_SEQUENCE:
            if (tm->seqRunning) break;
            tm->seqStep          = 0;
            tm->grp.seqGripState = GRP_FSM_IDLE;
            tm->seqRunning       = 1;
            tm->gripperAutoEn    = evt.arg_u8;
            /* First slot: Robot_Move (shortest path / P2P) to get to the
             * starting position directly. Subsequent slots use
             * _seq_move_directed to enforce the BaseSystem-commanded direction. */
            Robot_Move(robot, tm->seqSlotsRad[0]);
            break;

        case TASK_EVT_PREC_TEST:
            if (tm->precRunning)         break;
            if (tm->precTotalReps == 0)  break;
            tm->precCurrentRep   = 0;
            tm->precGoingToFinal = 1;
            tm->precRunning      = 1;
            Robot_MoveConstrained(robot, tm->precFinalRad,
                                  RBT_MAX_SPEED, RBT_MAX_ACCEL);
            break;

        case TASK_EVT_PERF_TEST:
            if (tm->perfRunning) break;
            tm->perfGoingToFinal = 1;
            tm->perfRunning      = 1;
            Robot_PerfTest_Start(robot, 3.14159265359f,
                                 tm->perfVelRad, tm->perfAccelRad);
            break;

        case TASK_EVT_GRIPPER_MANUAL:
            switch (evt.arg_u8)
            {
                case GRP_CMD_UP:    Robot_Gripper_MoveUp  (robot); break;
                case GRP_CMD_DOWN:  Robot_Gripper_MoveDown(robot); break;
                case GRP_CMD_OPEN:  Robot_Gripper_Open    (robot); break;
                case GRP_CMD_CLOSE: Robot_Gripper_Close   (robot); break;
                default: break;
            }
            break;

        case TASK_EVT_GRIPPER_SEQ:
            if (tm->grp.grpState == GRP_FSM_IDLE)
            {
                tm->grp.grpCmd   = evt.arg_u8;
                tm->grp.grpTimer = HAL_GetTick();
                if (tm->grp.grpCmd == 1)
                {
                    Robot_Gripper_MoveDown(robot);
                    tm->grp.grpState = GRP_FSM_DOWN;
                }
                else
                {
                    tm->grp.grpState = GRP_FSM_PENDULUM;
                }
            }
            break;

        default:
            break;
    }
}
