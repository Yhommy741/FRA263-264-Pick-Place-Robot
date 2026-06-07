/*
 * BaseSystemInterface.c
 *
 * Created on: May 19, 2026
 * Author: FRA263/264 Group 5
 *
 * CHANGED (May 2026):
 *   • BaseSystem_Dispatch() renamed to BaseSystem_Interface_Decode().
 *   • All Robot_*() calls, FSM state machines, and gripper sequencing
 *     removed — this file is now a pure Modbus register decoder.
 *   • Robot.h / RobotConfig.h no longer included.
 *   • Conversion macros (DEG_TO_RAD) kept locally for register decoding.
 *
 * FIX (June 2026):
 *   • Section D: cmd_Gripper_auto_en now reads hbs->latchedGripperAuto
 *     (captured atomically with the sequence slots at latch time) instead
 *     of re-reading the live REG_GRIPPER_AUTO_EN register.
 *   • Section D: Slot sign preserved as signed rad so TaskManager can
 *     enforce CW/CCW direction (positive = CW, negative = CCW).
 *   • Sequence latch: removed the prevGripperAutoReg tracking branch that
 *     ran between runs and overwrote the sentinel 0xFF with the live
 *     register value.  prevGripperAutoReg is now ALWAYS 0xFF when not
 *     pending — it is only written to a real value during the pending
 *     window (to detect the checkbox edge), then immediately reset to 0xFF
 *     on commit.  This fixes the bug where the second START was ignored
 *     whenever the gripper checkbox value matched the previous run's value
 *     (e.g. disable→disable or enable→enable).
 */

#include "BaseSystemInterface.h"
#include "RobotConfig.h"
#include <string.h>

#define DEG_TO_RAD(d)     ((d) * 0.017453293f)
#define RAD_TO_DEG(r)     ((r) * 57.295779513f)

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

    hbs->prevP2PTarget      = INT16_MIN;
    hbs->prevGripperAutoReg = 0xFF;
    hbs->registerFrame[REG_GRIPPER_AUTO_EN].U16 = 0xFF;  /* sentinel until first PC write */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  BaseSystemInterface_Update
 * ═══════════════════════════════════════════════════════════════════════════ */
void BaseSystemInterface_Update(BaseSystemInterface_t *hbs)
{
    Modbus_Protocol_Worker();
    u16u8_t                    *rf   = hbs->registerFrame;
    BaseSystemInterface_Data_t *data = &hbs->data;

    /* Heartbeat */
    if (rf[REG_HEARTBEAT].U16 == HEARTBEAT_PC_HI)
        rf[REG_HEARTBEAT].U16 = HEARTBEAT_ROBOT_YA;

    /* ── Sync registers → data struct ───────────────────────────────────── */
    data->manualGripper     = rf[REG_MANUAL_GRIPPER].U16;
    data->gripperSequence   = rf[REG_GRIPPER_SEQ].U16;
    data->gripperAutoEnable = (uint8_t)(rf[REG_GRIPPER_AUTO_EN].U16 & 0x01);
    data->jogDegrees        = (int16_t)rf[REG_JOG_DEG].U16;
    data->testType          = rf[REG_TEST_TYPE].U16 & 0x01;
    data->perfVelocity      = (int16_t)rf[REG_PERF_VEL].U16;
    data->perfAcceleration  = (int16_t)rf[REG_PERF_ACCEL].U16;
    data->precInitPosition  = (int16_t)rf[REG_PREC_INIT].U16;
    data->precFinalPosition = (int16_t)rf[REG_PREC_FINAL].U16;
    data->precRepeatCount   = (int16_t)rf[REG_PREC_REPEATS].U16;
    for (int i = 0; i < 16; i++)
        data->sequenceSlots[i] = (int16_t)rf[REG_SEQ_START + i].U16;
    data->sequencePairs   = rf[REG_SEQ_PAIRS].U16;
    data->p2pUnit         = rf[REG_P2P_UNIT].U16 & 0x01;
    data->p2pTarget       = (int16_t)rf[REG_P2P_TARGET].U16;
    data->softStopRequest = (uint8_t)(rf[REG_SOFT_STOP].U16 & 0x01);
    if (data->softStopRequest)
        rf[REG_SOFT_STOP].U16 = 0;

    /* ── Latch: P2P ──────────────────────────────────────────────────────── */
    {
        uint16_t cu = rf[REG_P2P_UNIT].U16 & 0x01;
        int16_t  ct = (int16_t)rf[REG_P2P_TARGET].U16;
        if (!hbs->latchedP2PValid && (cu != hbs->prevP2PUnit || ct != hbs->prevP2PTarget))
        { hbs->latchedP2PTarget = ct; hbs->latchedP2PUnit = cu; hbs->latchedP2PValid = 1; }
        hbs->prevP2PUnit   = cu;
        hbs->prevP2PTarget = ct;
    }

    /* ── Latch: Precision test ───────────────────────────────────────────── */
    if (rf[REG_PREC_REPEATS].U16 != 0 && !hbs->latchedPrecValid)
    {
        int16_t rep = (int16_t)rf[REG_PREC_REPEATS].U16;
        hbs->latchedPrecRepeats  = (rep < 0) ? (uint16_t)(-rep) : (uint16_t)rep;
        hbs->latchedPrecUseIndex = (rep < 0) ? 1 : 0;
        hbs->latchedPrecInit     = (int16_t)rf[REG_PREC_INIT].U16;
        hbs->latchedPrecFinal    = (int16_t)rf[REG_PREC_FINAL].U16;
        rf[REG_PREC_REPEATS].U16 = 0; data->precRepeatCount = 0;
        hbs->latchedPrecValid = 1;
    }

    /* ── Latch: Sequence ─────────────────────────────────────────────────── *
     * Two-frame protocol: REG_SEQ_PAIRS arrives first, REG_GRIPPER_AUTO_EN  *
     * arrives in the next Modbus frame (one frame per Update() call).        *
     *                                                                         *
     * prevGripperAutoReg is ALWAYS 0xFF when not pending.  It is never       *
     * tracked between runs — only used as an edge detector inside the        *
     * pending window.  This guarantees that ANY value the PC writes (0 or 1) *
     * is always different from 0xFF and always fires the latch, regardless   *
     * of what value was used in the previous run.                             */
    /* ── Latch: Sequence ─────────────────────────────────────────────────── *
     * The PC always writes registers in this order:                           *
     *   1. REG_SEQ_START..END  (slot positions)                               *
     *   2. REG_SEQ_PAIRS       (pair count — triggers staging here)           *
     *   3. REG_GRIPPER_AUTO_EN (checkbox — THIS is the final trigger)         *
     *                                                                          *
     * Strategy: stage slots+pairs when REG_SEQ_PAIRS arrives, but do NOT     *
     * fire latchedSeqValid yet.  Fire it only when REG_GRIPPER_AUTO_EN        *
     * is written (raw register changes from its cleared value of 0xFF).       *
     * This guarantees latchedGripperAuto always holds the current run's       *
     * checkbox value — never the previous run's stale value.                  *
     *                                                                          *
     * prevGripperAutoReg is initialised to 0xFF (sentinel) and reset to 0xFF  *
     * after every commit, so ANY value the PC writes (0 or 1) is always       *
     * detected as a change.                                                    */

    /* Step 1: Stage slots and pairs when REG_SEQ_PAIRS arrives. */
    if (rf[REG_SEQ_PAIRS].U16 != 0 && !hbs->latchedSeqValid && !hbs->seqPending)
    {
        hbs->latchedSeqPairs = rf[REG_SEQ_PAIRS].U16;
        for (int i = 0; i < 16; i++)
            hbs->latchedSeqSlots[i] = (int16_t)rf[REG_SEQ_START + i].U16;
        rf[REG_SEQ_PAIRS].U16 = 0; data->sequencePairs = 0;
        hbs->seqPending         = 1;
        hbs->prevGripperAutoReg = 0xFF;  /* arm sentinel — do NOT read checkbox yet */
        /* Stop here. REG_GRIPPER_AUTO_EN still holds the previous run's value
         * in this same Update() call. Wait for it to arrive in the next frame. */
    }
    /* Step 2: Fire latch when REG_GRIPPER_AUTO_EN is written after staging.
     * Use else-if so this never runs in the same call as Step 1.             */
    else if (hbs->seqPending && !hbs->latchedSeqValid)
    {
        uint16_t curAuto = rf[REG_GRIPPER_AUTO_EN].U16;
        if (curAuto != hbs->prevGripperAutoReg)
        {
            hbs->latchedGripperAuto = (uint8_t)(curAuto & 0x01);
            hbs->latchedSeqValid    = 1;
            hbs->seqPending         = 0;
            hbs->prevGripperAutoReg = 0xFF;  /* re-arm sentinel for next run */
            /* Clear the register so the next run's write is always a fresh
             * edge (PC always re-sends it, so clearing here is safe).        */
            rf[REG_GRIPPER_AUTO_EN].U16 = 0xFF;
        }
    }

    /* ── Latch: Gripper Pick/Place sequence ──────────────────────────────── */
    if (rf[REG_GRIPPER_SEQ].U16 != 0 && rf[REG_GRIPPER_SEQ].U16 != hbs->prevGripperSeqReg)
    {
        hbs->latchedGripperSeqCmd   = (uint8_t)(rf[REG_GRIPPER_SEQ].U16 & 0xFF);
        hbs->latchedGripperSeqValid = 1;
        hbs->prevGripperSeqReg      = rf[REG_GRIPPER_SEQ].U16;
    }

    /* ── Latch: Manual gripper ───────────────────────────────────────────── */
    if (rf[REG_MANUAL_GRIPPER].U16 != hbs->prevGripperReg)
    {
        hbs->latchedGripperCmd   = (uint8_t)(rf[REG_MANUAL_GRIPPER].U16 & 0xFF);
        hbs->latchedGripperValid = 1;
        hbs->prevGripperReg      = rf[REG_MANUAL_GRIPPER].U16;
    }

    /* ── Latch: Jog degrees ──────────────────────────────────────────────── */
    if (rf[REG_JOG_DEG].U16 != 0)
    { hbs->latchedJogDeg=(int16_t)rf[REG_JOG_DEG].U16; rf[REG_JOG_DEG].U16=0; data->jogDegrees=0; }

    /* ── Latch: OpMode ───────────────────────────────────────────────────── */
    if (rf[REG_OP_MODE].U16 != OP_MODE_IDLE)
    { hbs->latchedOpMode=rf[REG_OP_MODE].U16; rf[REG_OP_MODE].U16=OP_MODE_IDLE; }
    data->operatingMode = rf[REG_OP_MODE].U16;

    hbs->pending.rx_frame_count++;

    /* ── Pack Robot → PC ─────────────────────────────────────────────────── */
    rf[REG_SENSORS].U16      = data->sensorBits;
    rf[REG_ROBOT_TASK].U16   = data->currentTaskBits;
    rf[REG_POSITION].U16     = (uint16_t)(int16_t)(data->realPosition     * 10.0f);
    rf[REG_VELOCITY].U16     = (uint16_t)(int16_t)(data->realVelocity     * 10.0f);
    rf[REG_ACCELERATION].U16 = (uint16_t)(int16_t)(data->realAcceleration * 10.0f);
    rf[REG_EMERGENCY].U16    = (uint16_t)(data->emergencyActive & 0x01);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  BaseSystem_Interface_Decode
 * ═══════════════════════════════════════════════════════════════════════════ */
void BaseSystem_Interface_Decode(BaseSystemInterface_t *hbs)
{
    BaseSystemInterface_Data_t *data = &hbs->data;
    BSI_PendingCmd_t           *pnd  = &hbs->pending;

    uint32_t _rx   = pnd->rx_frame_count;
    uint32_t _dec  = pnd->cmd_decode_count;
    memset(pnd, 0, sizeof(BSI_PendingCmd_t));
    pnd->rx_frame_count   = _rx;
    pnd->cmd_decode_count = _dec;

    /* ── SECTION A: Safety ───────────────────────────────────────────────── */
    pnd->cmd_EStop = (uint8_t)(data->emergencyActive & 0x01);
    pnd->cmd_Stop  = data->softStopRequest;

    /* ── SECTION B: Operating mode ───────────────────────────────────────── */
    uint8_t modeChanged = (hbs->latchedOpMode != OP_MODE_IDLE);
    if (modeChanged)
    {
        pnd->opMode = hbs->latchedOpMode;
        hbs->dbg_lastOpMode = hbs->latchedOpMode;
        hbs->latchedOpMode  = OP_MODE_IDLE;
        pnd->cmd_decode_count++;
    }
    hbs->dbg_modeChanged = modeChanged;

    switch (pnd->opMode)
    {
        case OP_MODE_HOMING:
            pnd->cmd_Home = 1;
            break;
        case OP_MODE_JOG:
            if (hbs->latchedJogDeg != 0)
            {
                pnd->cmd_Jog_step_rad = DEG_TO_RAD((float)hbs->latchedJogDeg);
                hbs->latchedJogDeg    = 0;
                pnd->cmd_decode_count++;
            }
            break;
        case OP_MODE_SET_HOME:
            break;
        default:
            break;
    }

    /* ── SECTION B2: Performance test ────────────────────────────────────── */
    if (data->testType == 1 && data->perfVelocity != 0 && data->perfAcceleration != 0)
    {
        pnd->cmd_Perf_vel_rad_s    = (float)data->perfVelocity;
        pnd->cmd_Perf_accel_rad_s2 = (float)data->perfAcceleration;
        pnd->cmd_Perf_init_rad  = hbs->latchedPrecUseIndex
                                 ? DEG_TO_RAD((float)hbs->latchedPrecInit  * 5.0f)
                                 : DEG_TO_RAD((float)hbs->latchedPrecInit);
        pnd->cmd_Perf_final_rad = hbs->latchedPrecUseIndex
                                 ? DEG_TO_RAD((float)hbs->latchedPrecFinal * 5.0f)
                                 : DEG_TO_RAD((float)hbs->latchedPrecFinal);
        pnd->cmd_Perf_valid = 1;
        hbs->registerFrame[REG_TEST_TYPE].U16  = 0;
        hbs->registerFrame[REG_PERF_VEL].U16   = 0;
        hbs->registerFrame[REG_PERF_ACCEL].U16 = 0;
        data->testType         = 0;
        data->perfVelocity     = 0;
        data->perfAcceleration = 0;
        pnd->cmd_decode_count++;
    }

    /* ── SECTION C: Precision test ───────────────────────────────────────── */
    if (hbs->latchedPrecValid)
    {
        hbs->latchedPrecValid = 0;
        pnd->cmd_decode_count++;
        pnd->cmd_Prec_init_rad  = hbs->latchedPrecUseIndex
                                 ? DEG_TO_RAD((float)hbs->latchedPrecInit  * 5.0f)
                                 : DEG_TO_RAD((float)hbs->latchedPrecInit);
        pnd->cmd_Prec_final_rad = hbs->latchedPrecUseIndex
                                 ? DEG_TO_RAD((float)hbs->latchedPrecFinal * 5.0f)
                                 : DEG_TO_RAD((float)hbs->latchedPrecFinal);
        pnd->cmd_Prec_repeats   = (int16_t)hbs->latchedPrecRepeats;
    }

    /* ── SECTION D: Multi-position sequence ──────────────────────────────── */
    if (hbs->latchedSeqValid)
    {
        hbs->latchedSeqValid = 0;
        pnd->cmd_Seq_pairs   = hbs->latchedSeqPairs;
        pnd->cmd_decode_count++;
        pnd->cmd_Gripper_auto_en = hbs->latchedGripperAuto;
        for (int i = 0; i < 16; i++)
        {
            /* positive index → CW  to  index*5°  (stored as +rad)
             * negative index → CCW to  abs*5°    (stored as -rad)
             * TaskManager._seq_move_directed() uses sign to force direction. */
            int16_t s = hbs->latchedSeqSlots[i];
            pnd->cmd_Seq_slots_rad[i] = (s < 0)
                ? -DEG_TO_RAD((float)(-s) * 5.0f)
                :  DEG_TO_RAD((float)( s) * 5.0f);
        }
    }

    /* ── SECTION E: P2P move ─────────────────────────────────────────────── */
    if (hbs->latchedP2PValid)
    {
        pnd->cmd_P2P_target_rad = (hbs->latchedP2PUnit == P2P_UNIT_DEG)
                                 ? DEG_TO_RAD((float)hbs->latchedP2PTarget)
                                 : DEG_TO_RAD((float)hbs->latchedP2PTarget * 5.0f);
        pnd->cmd_P2P_valid   = 1;
        hbs->latchedP2PValid = 0;
        pnd->cmd_decode_count++;
    }

    /* ── SECTION F: Jog ─────────────────────────────────────────────────── */
    if (!modeChanged && hbs->latchedJogDeg != 0)
    {
        pnd->cmd_Jog_step_rad = DEG_TO_RAD((float)hbs->latchedJogDeg);
        hbs->latchedJogDeg    = 0;
        pnd->cmd_decode_count++;
    }

    /* ── SECTION G: Manual gripper ───────────────────────────────────────── */
    if (hbs->latchedGripperValid)
    {
        hbs->latchedGripperValid      = 0;
        pnd->cmd_Gripper_manual       = hbs->latchedGripperCmd;
        pnd->cmd_Gripper_manual_valid = 1;
    }

    if (hbs->latchedGripperSeqValid)
    {
        hbs->latchedGripperSeqValid = 0;
        pnd->cmd_Gripper_seq = hbs->latchedGripperSeqCmd;
    }
}
