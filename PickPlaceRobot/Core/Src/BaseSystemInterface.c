/*
 * BaseSystemInterface.c
 *
 * Created on: May 19, 2026
 * Author: Assistant
 *
 * Register map aligned to README (FRA263/FRA264 Base System).
 * ModbusRTU is a submodule of BaseSystemInterface_t.
 * Global instance (BaseSystemInterface_t BaseSystem) lives in main.c.
 */

#include "BaseSystemInterface.h"

/* ── Unit conversion helpers ───────────────────────────────────────────────── */
#define DEG_TO_RAD(d)     ((d) * 0.017453293f)  /* π/180        */
#define RAD_TO_DEG(r)     ((r) * 57.295779513f) /* 180/π        */
#define TENTH_TO_FLOAT(x) ((x) * 0.1f)         /* int16 × 0.1  */

/* ═══════════════════════════════════════════════════════════════════════════
 *  BaseSystemInterface_Init
 * ═══════════════════════════════════════════════════════════════════════════ */
void BaseSystemInterface_Init(BaseSystemInterface_t *hbs,
                               UART_HandleTypeDef    *huart,
                               TIM_HandleTypeDef     *htim,
                               uint8_t                slaveAddress)
{
    memset(hbs, 0, sizeof(BaseSystemInterface_t));

    hbs->modbus.huart        = huart;
    hbs->modbus.htim         = htim;
    hbs->modbus.slaveAddress = slaveAddress;
    hbs->modbus.RegisterSize = BASE_SYSTEM_REG_COUNT;

    Modbus_init(&hbs->modbus, hbs->registerFrame);

    hbs->registerFrame[REG_HEARTBEAT].U16 = HEARTBEAT_ROBOT_YA;
    hbs->registerFrame[REG_OP_MODE].U16   = 0x0000;
    hbs->registerFrame[REG_SOFT_STOP].U16 = 0x0000;
    hbs->registerFrame[REG_EMERGENCY].U16 = 0x0000;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  BaseSystemInterface_Update
 * ═══════════════════════════════════════════════════════════════════════════ */
void BaseSystemInterface_Update(BaseSystemInterface_t *hbs)
{
    Modbus_Protocol_Worker();

    u16u8_t                    *rf   = hbs->registerFrame;
    BaseSystemInterface_Data_t *data = &hbs->data;

    /* ── 1. Heartbeat ────────────────────────────────────────────────────── */
    if (rf[REG_HEARTBEAT].U16 == HEARTBEAT_PC_HI)
        rf[REG_HEARTBEAT].U16 = HEARTBEAT_ROBOT_YA;

    /* ── 2. Unpack PC → Robot ────────────────────────────────────────────── */
    data->manualGripper     = rf[REG_MANUAL_GRIPPER].U16;
    data->gripperSequence   = rf[REG_GRIPPER_SEQ].U16;
    data->gripperAutoEnable = (uint8_t)(rf[REG_GRIPPER_AUTO_EN].U16 & 0x01);
    data->jogDegrees        = (int16_t) rf[REG_JOG_DEG].U16;
    data->testType          = rf[REG_TEST_TYPE].U16 & 0x01;
    data->perfVelocity      = (int16_t) rf[REG_PERF_VEL].U16;
    data->perfAcceleration  = (int16_t) rf[REG_PERF_ACCEL].U16;
    data->precInitPosition  = (int16_t) rf[REG_PREC_INIT].U16;
    data->precFinalPosition = (int16_t) rf[REG_PREC_FINAL].U16;
    data->precRepeatCount   = (int16_t) rf[REG_PREC_REPEATS].U16;

    for (int i = 0; i < 16; i++)
        data->sequenceSlots[i] = (int16_t)rf[REG_SEQ_START + i].U16;

    data->sequencePairs   = rf[REG_SEQ_PAIRS].U16;
    data->p2pUnit         = rf[REG_P2P_UNIT].U16 & 0x01;
    data->p2pTarget       = (int16_t)rf[REG_P2P_TARGET].U16;
    data->softStopRequest = (uint8_t)(rf[REG_SOFT_STOP].U16 & 0x01);

    /* One-shot latch for P2P — shadow comparison detects any register change */
    {
        uint16_t cur_unit   = rf[REG_P2P_UNIT].U16 & 0x01;
        int16_t  cur_target = (int16_t)rf[REG_P2P_TARGET].U16;
        if (!hbs->latchedP2PValid &&
            (cur_unit != hbs->prevP2PUnit || cur_target != hbs->prevP2PTarget))
        {
            hbs->latchedP2PTarget = cur_target;
            hbs->latchedP2PUnit   = cur_unit;
            hbs->latchedP2PValid  = 1;
        }
        hbs->prevP2PUnit   = cur_unit;
        hbs->prevP2PTarget = cur_target;
    }

    /* One-shot latch for precision test — triggered by REG_PREC_REPEATS (0x11).
     * Sign of repeat count encodes unit: negative = index, positive = degree. */
    if (rf[REG_PREC_REPEATS].U16 != 0 && !hbs->latchedPrecValid)
    {
        int16_t rep = (int16_t)rf[REG_PREC_REPEATS].U16;
        hbs->latchedPrecRepeats  = (rep < 0) ? (uint16_t)(-rep) : (uint16_t)rep;
        hbs->latchedPrecUseIndex = (rep < 0) ? 1 : 0;
        hbs->latchedPrecInit     = (int16_t)rf[REG_PREC_INIT].U16;
        hbs->latchedPrecFinal    = (int16_t)rf[REG_PREC_FINAL].U16;
        rf[REG_PREC_REPEATS].U16 = 0;
        data->precRepeatCount    = 0;
        hbs->latchedPrecValid    = 1;
    }

    /* One-shot latch for sequence — triggered by REG_SEQ_PAIRS (0x22) */
    if (rf[REG_SEQ_PAIRS].U16 != 0 && !hbs->latchedSeqValid)
    {
        hbs->latchedSeqPairs    = rf[REG_SEQ_PAIRS].U16;
        hbs->latchedGripperAuto = (uint8_t)(rf[REG_GRIPPER_AUTO_EN].U16 & 0x01);
        for (int i = 0; i < 16; i++)
            hbs->latchedSeqSlots[i] = (int16_t)rf[REG_SEQ_START + i].U16;
        rf[REG_SEQ_PAIRS].U16 = 0;
        data->sequencePairs   = 0;
        hbs->latchedSeqValid  = 1;
    }

    /* One-shot latch for jog */
    if (rf[REG_JOG_DEG].U16 != 0)
    {
        hbs->latchedJogDeg  = (int16_t)rf[REG_JOG_DEG].U16;
        rf[REG_JOG_DEG].U16 = 0;
        data->jogDegrees    = 0;
    }

    /* One-shot latch for opMode */
    if (rf[REG_OP_MODE].U16 != OP_MODE_IDLE)
    {
        hbs->latchedOpMode  = rf[REG_OP_MODE].U16;
        rf[REG_OP_MODE].U16 = OP_MODE_IDLE;
    }
    data->operatingMode = rf[REG_OP_MODE].U16;

    /* Count every completed unpack cycle */
    hbs->pending.rx_frame_count++;

    /* ── 3. Pack Robot → PC ──────────────────────────────────────────────── */
    rf[REG_SENSORS].U16      = data->sensorBits;
    rf[REG_ROBOT_TASK].U16   = data->currentTaskBits;
    rf[REG_POSITION].U16     = (uint16_t)(int16_t)(data->realPosition     * 10.0f);
    rf[REG_VELOCITY].U16     = (uint16_t)(int16_t)(data->realVelocity     * 10.0f);
    rf[REG_ACCELERATION].U16 = (uint16_t)(int16_t)(data->realAcceleration * 10.0f);
    rf[REG_EMERGENCY].U16    = (uint16_t)(data->emergencyActive & 0x01);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  BaseSystem_Dispatch
 * ═══════════════════════════════════════════════════════════════════════════ */
void BaseSystem_Dispatch(BaseSystemInterface_t *hbs, Robot_t *robot)
{
    BaseSystemInterface_Data_t *data = &hbs->data;
    BSI_PendingCmd_t           *pnd  = &hbs->pending;

    /* Preserve counters across memset */
    uint32_t _rx   = pnd->rx_frame_count;
    uint32_t _disp = pnd->cmd_dispatch_count;
    memset(pnd, 0, sizeof(BSI_PendingCmd_t));
    pnd->rx_frame_count     = _rx;
    pnd->cmd_dispatch_count = _disp;

    /* ─────────────────────────────────────────────────────────────────────
     * SECTION A: Safety — E-Stop and Soft-Stop (highest priority)
     * ───────────────────────────────────────────────────────────────────── */
    pnd->cmd_EStop = (uint8_t)(data->emergencyActive & 0x01);
    if (pnd->cmd_EStop)
    {
        hbs->seqRunning  = 0;
        hbs->precRunning = 0;
        Robot_EStop(robot);
        return;
    }

    pnd->cmd_Stop = data->softStopRequest;
    if (pnd->cmd_Stop)
    {
        hbs->seqRunning  = 0;
        hbs->precRunning = 0;
        Robot_Stop(robot);
    }

    /* ─────────────────────────────────────────────────────────────────────
     * SECTION B: Operating mode — one-shot latch
     * ───────────────────────────────────────────────────────────────────── */
    uint8_t modeChanged = (hbs->latchedOpMode != OP_MODE_IDLE);
    if (modeChanged)
    {
        pnd->opMode         = hbs->latchedOpMode;
        hbs->dbg_lastOpMode = hbs->latchedOpMode;
        hbs->latchedOpMode  = OP_MODE_IDLE;
        pnd->cmd_dispatch_count++;
    }
    hbs->dbg_modeChanged = modeChanged;

    switch (pnd->opMode)
    {
        case OP_MODE_IDLE:
            break;

        /* ── HOMING  (0x01) ────────────────────────────────────────────── */
        case OP_MODE_HOMING:
            pnd->cmd_Home = 1;
            hbs->dbg_home_call_count++;
            Robot_Home(robot);
            break;

        /* ── JOG  (0x02) ───────────────────────────────────────────────── */
        case OP_MODE_JOG:
            pnd->cmd_Jog_step_rad = DEG_TO_RAD((float)data->jogDegrees);
            if (pnd->cmd_Jog_step_rad != 0.0f)
                Robot_JogStep(robot, pnd->cmd_Jog_step_rad);
            break;

        /* ── AUTO / SEQUENCE  (0x04) ───────────────────────────────────── */
        case OP_MODE_AUTO:
            break;

        /* ── SET HOME  (0x08) — instantly zero current position, no motion */
        case OP_MODE_SET_HOME:
            robot->home_offset  = (float)robot->encoder.Rad;
            robot->theta        = 0.0f;
            robot->theta_target = 0.0f;
            break;

        /* ── TEST  (0x10) ──────────────────────────────────────────────── */
        case OP_MODE_TEST:
            break;

        default:
            break;
    }

    /* ─────────────────────────────────────────────────────────────────────
     * SECTION C: Precision test state machine (non-blocking)
     * ───────────────────────────────────────────────────────────────────── */
    if (hbs->latchedPrecValid)
    {
        hbs->latchedPrecValid = 0;
        pnd->cmd_dispatch_count++;

        if (hbs->latchedPrecUseIndex)
        {
            hbs->precInitRad  = DEG_TO_RAD((float)hbs->latchedPrecInit  * 5.0f);
            hbs->precFinalRad = DEG_TO_RAD((float)hbs->latchedPrecFinal * 5.0f);
        }
        else
        {
            hbs->precInitRad  = DEG_TO_RAD((float)hbs->latchedPrecInit);
            hbs->precFinalRad = DEG_TO_RAD((float)hbs->latchedPrecFinal);
        }
        hbs->precTotalReps    = hbs->latchedPrecRepeats;
        hbs->precCurrentRep   = 0;
        hbs->precGoingToFinal = 1;
        hbs->precRunning      = 1;
        Robot_Move(robot, hbs->precFinalRad);
    }
    else if (hbs->precRunning)
    {
        if (Robot_IsIdle(robot))
        {
            if (hbs->precGoingToFinal)
            {
                hbs->precGoingToFinal = 0;
                Robot_Move(robot, hbs->precInitRad);
            }
            else
            {
                hbs->precCurrentRep++;
                if (hbs->precCurrentRep >= hbs->precTotalReps)
                    hbs->precRunning = 0;
                else
                {
                    hbs->precGoingToFinal = 1;
                    Robot_Move(robot, hbs->precFinalRad);
                }
            }
        }
    }

    /* ─────────────────────────────────────────────────────────────────────
     * SECTION D: Sequence state machine (non-blocking)
     * ───────────────────────────────────────────────────────────────────── */
    if (hbs->latchedSeqValid)
    {
        hbs->latchedSeqValid = 0;
        pnd->cmd_Seq_pairs   = hbs->latchedSeqPairs;
        pnd->cmd_dispatch_count++;

        for (int i = 0; i < 16; i++)
        {
            hbs->seqSlotsRad[i]       = DEG_TO_RAD((float)hbs->latchedSeqSlots[i] * 5.0f);
            pnd->cmd_Seq_slots_rad[i] = hbs->seqSlotsRad[i];
        }

        hbs->seqTotalSteps = hbs->latchedSeqPairs * 2;
        if (hbs->seqTotalSteps > 16) hbs->seqTotalSteps = 16;
        hbs->seqStep    = 0;
        hbs->seqRunning = 1;
        Robot_Move(robot, hbs->seqSlotsRad[0]);
    }
    else if (hbs->seqRunning)
    {
        if (Robot_IsIdle(robot))
        {
            hbs->seqStep++;
            if (hbs->seqStep >= hbs->seqTotalSteps)
                hbs->seqRunning = 0;
            else
                Robot_Move(robot, hbs->seqSlotsRad[hbs->seqStep]);
        }
    }

    /* ─────────────────────────────────────────────────────────────────────
     * SECTION E: P2P move (non-blocking, target can be 0)
     * ───────────────────────────────────────────────────────────────────── */
    if (hbs->latchedP2PValid)
    {
        float target_rad;
        if (hbs->latchedP2PUnit == P2P_UNIT_DEG)
            target_rad = DEG_TO_RAD((float)hbs->latchedP2PTarget);
        else
            target_rad = DEG_TO_RAD((float)hbs->latchedP2PTarget * 5.0f);

        pnd->cmd_P2P_target_rad = target_rad;
        hbs->latchedP2PValid    = 0;
        pnd->cmd_dispatch_count++;
        Robot_Move(robot, target_rad);
    }

    /* ─────────────────────────────────────────────────────────────────────
     * SECTION F: Jog (independent of opMode)
     * ───────────────────────────────────────────────────────────────────── */
    if (!modeChanged && hbs->latchedJogDeg != 0)
    {
        pnd->cmd_Jog_step_rad = DEG_TO_RAD((float)hbs->latchedJogDeg);
        hbs->latchedJogDeg    = 0;
        pnd->cmd_dispatch_count++;
        Robot_JogStep(robot, pnd->cmd_Jog_step_rad);
    }

    /* ─────────────────────────────────────────────────────────────────────
     * SECTION G: Manual Gripper
     * Values per README 3.3: Up=0x00, Down=0x01, Open=0x02, Close=0x04
     * ───────────────────────────────────────────────────────────────────── */
    pnd->cmd_Gripper_manual  = (uint8_t)(data->manualGripper & 0xFF);
    pnd->cmd_Gripper_seq     = data->gripperSequence;
    pnd->cmd_Gripper_auto_en = data->gripperAutoEnable;

    if (pnd->cmd_Gripper_manual != hbs->prevGripperCmd)
    {
        switch (pnd->cmd_Gripper_manual)
        {
            case 0x01: Robot_Gripper_MoveDown(robot); break;
            case 0x02: Robot_Gripper_Open    (robot); break;
            case 0x04: Robot_Gripper_Close   (robot); break;
            case 0x00:
                if (hbs->prevGripperCmd != 0x00)
                    Robot_Gripper_MoveUp(robot);
                break;
            default: break;
        }
    }
    hbs->prevGripperCmd = pnd->cmd_Gripper_manual;

    /* ─────────────────────────────────────────────────────────────────────
     * SECTION H: Write-back — Robot state → data → registers
     * ───────────────────────────────────────────────────────────────────── */
    pnd->wb_position_rad   = Robot_GetPosition(robot);
    pnd->wb_velocity_rad_s = Robot_GetVelocity(robot);

    data->realPosition     = RAD_TO_DEG(pnd->wb_position_rad);
    data->realVelocity     = pnd->wb_velocity_rad_s;
    data->realAcceleration = 0.0f;

    /* Task bits — README 4.3: bit0=Homing, bit1=Pick, bit2=Place, bit3=Point */
    {
        uint16_t tb = 0;
        Robot_State_t rs = Robot_GetState(robot);
        if (rs == ROBOT_HOMING_FAST_STATE   ||
            rs == ROBOT_HOMING_BACKOFF_STATE ||
            rs == ROBOT_HOMING_SLOW_STATE    ||
            rs == ROBOT_HOMING_OFFSET_STATE)   tb = 0x0001;
        else if (rs == ROBOT_MOVE   ||
                 rs == ROBOT_JOG_VEL ||
                 rs == ROBOT_JOG_STEP)         tb = 0x0008;
        pnd->wb_taskBits      = tb;
        data->currentTaskBits = tb;
    }

    pnd->wb_emergencyActive = (Robot_GetState(robot) == ROBOT_ESTOP) ? 1U : 0U;
    data->emergencyActive   = pnd->wb_emergencyActive;

    /* Gripper sensor bitmask: bit0=Up, bit1=Down, bit2=Claw */
    pnd->wb_sensorBits  = 0;
    pnd->wb_sensorBits |= (Robot_Gripper_GetUpState  (robot) == GRP_STATE_HIGH) ? (1u<<0) : 0u;
    pnd->wb_sensorBits |= (Robot_Gripper_GetDownState(robot) == GRP_STATE_HIGH) ? (1u<<1) : 0u;
    pnd->wb_sensorBits |= (Robot_Gripper_GetClawState(robot) == GRP_STATE_HIGH) ? (1u<<2) : 0u;
    data->sensorBits = pnd->wb_sensorBits;
}
