/*
 * BaseSystem.h
 *
 * Created on: May 19, 2026
 * Author: Assistant
 */

#ifndef INC_BASESYSTEM_H_
#define INC_BASESYSTEM_H_

#include "ModbusRTU.h"

/* Register Address Definitions */
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

/* Magic Constants */
#define HEARTBEAT_ROBOT_YA     22881
#define HEARTBEAT_PC_HI        18537

/* Structure to simplify application level access to Base System states */
typedef struct {
    // Commands received from PC
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
    int16_t  sequenceSlots[16]; // 0x12 to 0x21
    uint16_t sequencePairs;
    uint16_t p2pUnit;
    int16_t  p2pTarget;
    uint8_t  softStopRequest;

    // States to report back to PC
    uint16_t sensorBits;
    uint16_t currentTaskBits;
    float    realPosition;     // Internal engineering value (floating point)
    float    realVelocity;     // Internal engineering value
    float    realAcceleration; // Internal engineering value
    uint8_t  emergencyActive;
} BaseSystem_DataTypedef;

/* Global instance handle */
extern BaseSystem_DataTypedef BaseSystemData;

/* Public API Functions */
void BaseSystem_Init(ModbusHandleTypedef* hmodbus, u16u8_t* regFrame);
void BaseSystem_Update(void);

#endif /* INC_BASESYSTEM_H_ */
