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
 *     of re-reading the live REG_GRIPPER_AUTO_EN register.  Re-reading the
 *     live register created a race: if the PC changed REG_GRIPPER_AUTO_EN
 *     between the latch call and the decode call (across main-loop
 *     iterations), the sequence would run with the wrong enable state.
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

    /* Sentinel: INT16_MIN can never arrive from Modbus (Modbus is unsigned
     * wire; signed reinterpretation gives range -32768..+32767, but the PC
     * treats 0x8000 as -32768 which is not a valid target angle).
     * Setting prevP2PTarget to INT16_MIN guarantees the very first write —
     * including target = 0 degrees / index 0 — triggers the P2P latch.     */
    hbs->prevP2PTarget    = INT16_MIN;
    hbs->prevGripperAutoReg = 0xFF;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  BaseSystemInterface_Update
 *
 *  Runs the Modbus protocol worker and syncs all live registers into
 *  hbs->data.  Also packs Robot state back into the register frame so the
 *  PC sees current position, task bits, and sensor bits.
 *
 *  The write-back fields (realPosition, realVelocity, sensorBits, etc.)
 *  must be refreshed by the caller (main.c or TaskManager) before this
 *  function is called each loop.
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
    /* Clear immediately — soft-stop is a one-shot trigger, not a persistent
     * state.  Leaving it set causes TASK_EVT_STOP to be posted every loop. */
    if (data->softStopRequest)
    {
        rf[REG_SOFT_STOP].U16 = 0;
    }

    /* ── Latch: P2P ──────────────────────────────────────────────────────── *
     * Fires whenever either register changes and no command is pending.      *
     * prevP2PTarget is initialised to INT16_MIN in Init (not a valid angle)  *
     * so the very first write — including target = 0 — always triggers.      */
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
     * Modbus_Protocol_Worker() processes ONE frame per Update() call.        *
     * REG_SEQ_PAIRS and REG_GRIPPER_AUTO_EN always arrive in separate calls. *
     * Step 1: stage slots when REG_SEQ_PAIRS arrives, set seqPending.        *
     * Step 2: fire latch when REG_GRIPPER_AUTO_EN arrives (next Update call).*
     * prevGripperAutoReg is only updated when NOT pending so the sentinel    *
     * 0xFF survives until the checkbox write comes.                          */
    if (rf[REG_SEQ_PAIRS].U16 != 0 && !hbs->latchedSeqValid && !hbs->seqPending)
    {
        hbs->latchedSeqPairs = rf[REG_SEQ_PAIRS].U16;
        for (int i = 0; i < 16; i++)
            hbs->latchedSeqSlots[i] = (int16_t)rf[REG_SEQ_START + i].U16;
        rf[REG_SEQ_PAIRS].U16 = 0; data->sequencePairs = 0;
        hbs->seqPending = 1;
        /* prevGripperAutoReg already 0xFF from init/last-commit — do NOT touch */
    }

    if (hbs->seqPending && !hbs->latchedSeqValid)
    {
        uint16_t curAuto = rf[REG_GRIPPER_AUTO_EN].U16;
        if (curAuto != hbs->prevGripperAutoReg)  /* 0xFF != 0 or 1 → always fires */
        {
            hbs->latchedGripperAuto = (uint8_t)(curAuto & 0x01);
            hbs->latchedSeqValid    = 1;
            hbs->seqPending         = 0;
            hbs->prevGripperAutoReg = 0xFF;  /* reset sentinel for next run */
        }
        /* else: checkbox not yet received — stay pending, wait next Update */
    }
    else if (!hbs->seqPending)
    {
        /* Not pending — track live value so it stays current between runs */
        hbs->prevGripperAutoReg = rf[REG_GRIPPER_AUTO_EN].U16;
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

    /* ── Pack Robot → PC (write-back — fields written by caller/TaskManager) */
    rf[REG_SENSORS].U16      = data->sensorBits;
    rf[REG_ROBOT_TASK].U16   = data->currentTaskBits;
    rf[REG_POSITION].U16     = (uint16_t)(int16_t)(data->realPosition     * 10.0f);
    rf[REG_VELOCITY].U16     = (uint16_t)(int16_t)(data->realVelocity     * 10.0f);
    rf[REG_ACCELERATION].U16 = (uint16_t)(int16_t)(data->realAcceleration * 10.0f);
    rf[REG_EMERGENCY].U16    = (uint16_t)(data->emergencyActive & 0x01);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  BaseSystem_Interface_Decode
 *
 *  Pure decoder: reads hbs latch state (populated by BaseSystemInterface_Update)
 *  and writes every decoded command into hbs->pending.
 *
 *  NO Robot_*() calls are made here.
 *  TaskManager calls Task_PostFromModbus() after this function to feed
 *  hbs->pending into the event queue.
 * ═══════════════════════════════════════════════════════════════════════════ */
void BaseSystem_Interface_Decode(BaseSystemInterface_t *hbs)
{
    BaseSystemInterface_Data_t *data = &hbs->data;
    BSI_PendingCmd_t           *pnd  = &hbs->pending;

    /* Preserve diagnostic counters across the memset */
    uint32_t _rx   = pnd->rx_frame_count;
    uint32_t _dec  = pnd->cmd_decode_count;
    memset(pnd, 0, sizeof(BSI_PendingCmd_t));
    pnd->rx_frame_count   = _rx;
    pnd->cmd_decode_count = _dec;

    /* ── SECTION A: Safety ───────────────────────────────────────────────── */
    /* Read emergencyActive for informational use only.
     * Do NOT return early here — Stop commands must still be decoded and
     * posted so the user can clear an ESTOP latch via Modbus Soft Stop.     */
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
            /* latchedJogDeg was captured by Update(); data->jogDegrees is
             * already 0 at this point (zeroed when latched).  Read the latch. */
            if (hbs->latchedJogDeg != 0)
            {
                pnd->cmd_Jog_step_rad = DEG_TO_RAD((float)hbs->latchedJogDeg);
                hbs->latchedJogDeg    = 0;
                pnd->cmd_decode_count++;
            }
            break;
        case OP_MODE_SET_HOME:
            /* Signal SET_HOME via opMode — TaskManager checks for this value */
            break;
        default:
            break;
    }

    /* ── SECTION B2: Performance test ────────────────────────────────────── *
     * Triggered by testType=1.  Positions come from PREC_INIT / PREC_FINAL  *
     * (same registers as precision test, same unit: degree or index×5°).    *
     * Velocity and accel limits come from PERF_VEL / PERF_ACCEL.            */
    if (data->testType == 1 && data->perfVelocity != 0 && data->perfAcceleration != 0)
    {
        pnd->cmd_Perf_vel_rad_s    = (float)data->perfVelocity;
        pnd->cmd_Perf_accel_rad_s2 = (float)data->perfAcceleration;

        /* Decode positions — mirror of Section C logic */
        pnd->cmd_Perf_init_rad  = hbs->latchedPrecUseIndex
                                 ? DEG_TO_RAD((float)hbs->latchedPrecInit  * 5.0f)
                                 : DEG_TO_RAD((float)hbs->latchedPrecInit);
        pnd->cmd_Perf_final_rad = hbs->latchedPrecUseIndex
                                 ? DEG_TO_RAD((float)hbs->latchedPrecFinal * 5.0f)
                                 : DEG_TO_RAD((float)hbs->latchedPrecFinal);

        pnd->cmd_Perf_valid = 1;

        /* Clear ALL trigger registers so stale values cannot re-fire.
         * The PC must write fresh vel/accel values with every new trigger. */
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
        /* FIX: Use hbs->latchedGripperAuto — captured atomically with the
         * sequence slots when REG_SEQ_PAIRS was written in Update().
         * Do NOT re-read the live REG_GRIPPER_AUTO_EN register here: if the
         * PC changes that register between the latch call and this decode
         * call (across main-loop iterations), the sequence would run with
         * the wrong enable state — exactly the stale-data bug this fixes.  */
        pnd->cmd_Gripper_auto_en = hbs->latchedGripperAuto;
        for (int i = 0; i < 16; i++)
        {
            float idx = (float)(hbs->latchedSeqSlots[i] < 0
                                ? -hbs->latchedSeqSlots[i]
                                :  hbs->latchedSeqSlots[i]);
            pnd->cmd_Seq_slots_rad[i] = DEG_TO_RAD(idx * 5.0f);
        }
    }

    /* ── SECTION E: P2P move ─────────────────────────────────────────────── */
    if (hbs->latchedP2PValid)
    {
        pnd->cmd_P2P_target_rad = (hbs->latchedP2PUnit == P2P_UNIT_DEG)
                                 ? DEG_TO_RAD((float)hbs->latchedP2PTarget)
                                 : DEG_TO_RAD((float)hbs->latchedP2PTarget * 5.0f);
        pnd->cmd_P2P_valid  = 1;   /* explicit flag — target can legally be 0.0f */
        hbs->latchedP2PValid = 0;
        pnd->cmd_decode_count++;
    }

    /* ── SECTION F: Jog (latched from Update, independent of mode) ───────── */
    if (!modeChanged && hbs->latchedJogDeg != 0)
    {
        pnd->cmd_Jog_step_rad = DEG_TO_RAD((float)hbs->latchedJogDeg);
        hbs->latchedJogDeg = 0;
        pnd->cmd_decode_count++;
    }

    /* ── SECTION G: Manual gripper ───────────────────────────────────────── */
    /* NOTE: cmd_Gripper_auto_en is set in Section D from the atomically
     * latched value (hbs->latchedGripperAuto).  It must NOT be overwritten
     * here from the live register.                                           */

    if (hbs->latchedGripperValid)
    {
        hbs->latchedGripperValid      = 0;
        pnd->cmd_Gripper_manual       = hbs->latchedGripperCmd;
        pnd->cmd_Gripper_manual_valid = 1;   /* GRP_CMD_UP == 0, so need a separate flag */
    }

    if (hbs->latchedGripperSeqValid)
    {
        hbs->latchedGripperSeqValid = 0;
        pnd->cmd_Gripper_seq = hbs->latchedGripperSeqCmd;
    }
}
