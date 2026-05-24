/*
 * BaseSystem.c
 *
 * Created on: May 19, 2026
 * Author: Assistant
 */

#include "BaseSystem.h"

/* Internal References */
static ModbusHandleTypedef* pModbusHandle = NULL;
static u16u8_t* pRegisterFrame = NULL;

BaseSystem_DataTypedef BaseSystemData;

/**
 * @brief Initializes the connection mapping layer and links to ModbusRTU frame registers.
 */
void BaseSystem_Init(ModbusHandleTypedef* hmodbus, u16u8_t* regFrame) {
    pModbusHandle = hmodbus;
    pRegisterFrame = regFrame;

    // Zero out base-system structured memory space
    memset(&BaseSystemData, 0, sizeof(BaseSystem_DataTypedef));

    // Seed initial safe registers conditions inside Modbus Frame array map
    pRegisterFrame[REG_HEARTBEAT].U16       = HEARTBEAT_ROBOT_YA;
    pRegisterFrame[REG_OP_MODE].U16         = 0x0000; // System Booted up to safe state
    pRegisterFrame[REG_SOFT_STOP].U16       = 0x0000;
    pRegisterFrame[REG_EMERGENCY].U16       = 0x0000;
}

/**
 * @brief Processes the data mapping, handles the automated heartbeat exchange,
 * and syncs firmware state variables with Modbus registers.
 */
void BaseSystem_Update(void) {
    if (pModbusHandle == NULL || pRegisterFrame == NULL) {
        return;
    }

    //---------------------------------------------------------
    // 1. HEARTBEAT MANAGEMENT ("YA" -> "HI" handshake loop)
    //---------------------------------------------------------
    if (pRegisterFrame[REG_HEARTBEAT].U16 == HEARTBEAT_PC_HI) {
        // PC acknowledged. Reset register value immediately back to "YA"
        // to check if PC is continuously writing and still alive.
        pRegisterFrame[REG_HEARTBEAT].U16 = HEARTBEAT_ROBOT_YA;
    }

    //---------------------------------------------------------
    // 2. UNPACK WRITE REGISTERS (PC -> Robot Commands)
    //---------------------------------------------------------
    BaseSystemData.operatingMode     = pRegisterFrame[REG_OP_MODE].U16;
    BaseSystemData.manualGripper     = pRegisterFrame[REG_MANUAL_GRIPPER].U16;
    BaseSystemData.gripperSequence   = pRegisterFrame[REG_GRIPPER_SEQ].U16;
    BaseSystemData.gripperAutoEnable = (uint8_t)(pRegisterFrame[REG_GRIPPER_AUTO_EN].U16 & 0x01);
    BaseSystemData.jogDegrees        = (int16_t)pRegisterFrame[REG_JOG_DEG].U16;
    BaseSystemData.testType          = pRegisterFrame[REG_TEST_TYPE].U16 & 0x01;
    BaseSystemData.perfVelocity      = (int16_t)pRegisterFrame[REG_PERF_VEL].U16;
    BaseSystemData.perfAcceleration  = (int16_t)pRegisterFrame[REG_PERF_ACCEL].U16;
    BaseSystemData.precInitPosition  = (int16_t)pRegisterFrame[REG_PREC_INIT].U16;
    BaseSystemData.precFinalPosition = (int16_t)pRegisterFrame[REG_PREC_FINAL].U16;
    BaseSystemData.precRepeatCount   = (int16_t)pRegisterFrame[REG_PREC_REPEATS].U16;

    // Unpack Sequence Slots
    for (int i = 0; i < 16; i++) {
        BaseSystemData.sequenceSlots[i] = (int16_t)pRegisterFrame[REG_SEQ_START + i].U16;
    }

    BaseSystemData.sequencePairs     = pRegisterFrame[REG_SEQ_PAIRS].U16;
    BaseSystemData.p2pUnit           = pRegisterFrame[REG_P2P_UNIT].U16 & 0x01;
    BaseSystemData.p2pTarget         = (int16_t)pRegisterFrame[REG_P2P_TARGET].U16;
    BaseSystemData.softStopRequest   = (uint8_t)(pRegisterFrame[REG_SOFT_STOP].U16 & 0x01);

    //---------------------------------------------------------
    // 3. PACK READ REGISTERS (Robot States -> PC Dashboard)
    //---------------------------------------------------------

    // Bitmasks mapping for Gripper Status
    pRegisterFrame[REG_SENSORS].U16 = BaseSystemData.sensorBits;

    // Status tasks identifier bit placement logic
    pRegisterFrame[REG_ROBOT_TASK].U16 = BaseSystemData.currentTaskBits;

    // Raw Signed Values Conversion Scaled (Float Real units * 10 -> int16_t payload representation)
    pRegisterFrame[REG_POSITION].U16     = (int16_t)(BaseSystemData.realPosition * 10.0f);
    pRegisterFrame[REG_VELOCITY].U16     = (int16_t)(BaseSystemData.realVelocity * 10.0f);
    pRegisterFrame[REG_ACCELERATION].U16 = (int16_t)(BaseSystemData.realAcceleration * 10.0f);

    // Safety Latches State Assignment
    pRegisterFrame[REG_EMERGENCY].U16 = (uint16_t)(BaseSystemData.emergencyActive & 0x01);
}
