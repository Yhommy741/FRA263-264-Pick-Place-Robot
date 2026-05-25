# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Bare-metal STM32G474RET firmware for a pick-and-place robotic arm. The system implements a cascaded PID controller with Kalman-filtered state estimation, quintic trajectory generation, and Modbus RTU supervisory communication.

**MCU:** STM32G474RET (ARM Cortex-M4 @ 170 MHz with FPU)  
**Toolchain:** GNU Tools for STM32 (arm-none-eabi-gcc 13.3.rel1)  
**IDE:** STM32CubeIDE (Eclipse-based)

## Build & Flash

Build from STM32CubeIDE, or manually:

```bash
make -C PickPlaceRobot/Debug
```

Output: `PickPlaceRobot/Debug/G05_PickPlaceRobot.elf`

Flash via STM32CubeProgrammer or the `.launch` file in STM32CubeIDE (ST-Link v2, SWD).

There is no automated test suite. Validation is done via STM32CubeIDE live expressions (watch variables) and serial monitoring through the `SerialFrame` interface.

## Debug Interface

Live control via STM32CubeIDE debugger — modify these variables at runtime:

| Variable | Type | Description |
|---|---|---|
| `dbg_cmd` | 1–9 | Motor commands: 1=move, 2=stop, 3=home, 4=jog vel, 5=jog step, 6=E-stop, 7=enable, 8=disable, 9=test |
| `dbg_target` | float (rad) | Target position for move command |
| `dbg_jog_speed` | float (rad/s) | Jog velocity |
| `dbg_jog_step` | float (rad) | Jog step increment |
| `robot.state` | enum 0–7 | Current FSM state |
| `robot.theta` | float (rad) | Measured output shaft angle |
| `robot.omega` | float (rad/s) | Measured output shaft velocity |

All positions/velocities are in **output shaft space** (motor shaft / gear ratio N=5).

## Architecture

### Real-Time Loop Structure

**TIM3 ISR @ 1 kHz (Ts = 1 ms):**
- `Robot_Update()` → Kalman filter predict/update → velocity PID → feedforward → MD20A output
- `Gripper_Update_DirectPins()`

**Position loop @ 100 Hz** (every 10th fast-loop tick):
- Trajectory evaluation → position PID → velocity setpoint

**Main loop (non-RT):**
- `Modbus_Protocol_Worker()` — blocking on T3.5 frame timeout
- `BaseSystem_Update()` — maps Modbus registers ↔ application commands

### Motor Control Pipeline

```
Trajectory ──→ Position PID ──→ ω_ref ──→ Velocity PID ──→ Feedforward ──→ MD20A ──→ Motor
     │                                          ↑                 ↑
     └── θ_ref, ω_ref                     Kalman ω̂          Kalman τ̂_d
                                     (from θ_enc only)
```

- **Trajectory:** Quintic (5th-order) polynomial, auto-computes duration from ω_max=5.6 rad/s, α_max=5.0 rad/s²
- **Kalman Filter:** 4-state discrete observer [θ, ω, I, τ_d] with position-only measurement
- **Velocity PID:** Kp=20, Ki=175, Kd=0 (velocity-form / incremental)
- **Position PID:** Kp=25, Ki=0, Kd=0
- **Feedforward:** `u_ff = G_ff·ω* + G_aff·τ̂_d`, where G_ff = Rm·b/Kt + Ke, G_aff = Rm/Kt

### Key Source Files

| File | Role |
|---|---|
| `Core/Src/main.c` | Entry point, HAL init, debug command handler, gripper GPIO |
| `Core/Src/Robot.c` | Central FSM orchestrating all motor operations (states: IDLE, MOVE, JOG_VEL, JOG_STEP, HOMING_*, ESTOP) |
| `Core/Src/KalmanFilterDCMotor.c` | 4-state Kalman filter using CMSIS-DSP matrix ops |
| `Core/Src/TrajectoryGen.c` | Quintic trajectory with constraint-based duration |
| `Core/Src/Controller.c` | Incremental PID + feedforward controller |
| `Core/Src/ModbusRTU.c` | Modbus RTU slave (FC3/6/16), UART2 @ 9600 baud, address 21 |
| `Core/Src/BaseSystem.c` | Modbus register map ↔ application command/state bridge |
| `Core/Src/JoystickInterface.c` | RS-485 joystick input (5-byte frame, PA1 = direction control) |
| `Core/Src/SerialFrame.c` | Binary serial framing for host-side monitoring (DMA-based) |
| `Core/Src/MD20A.c` | Motor driver: DIR + PWM, 2 kHz carrier, 0.5% deadband |
| `Core/Src/QEI.c` | Quadrature encoder: 2048 PPR × 4x = 8192 counts/rev |
| `Core/Src/DCMotor.c` | Motor parameters struct (Rm, Lm, Ke, Kt, J, b, N=5) |

### Modbus Register Map (BaseSystem)

| Address | Description |
|---|---|
| 0x00 | Heartbeat ("YA" ↔ "HI") |
| 0x01 | Operating mode (manual, automated, test) |
| 0x02–0x04 | Gripper control |
| 0x05–0x08 | Motion parameters (jog speed, velocity, acceleration) |
| 0x09–0x11 | Precision test parameters |
| 0x12–0x25 | Sequence slots and state requests |
| 0x26–0x31 | Sensor/state readback |

### Homing Sequence

1. Fast approach at −1.0 rad/s until reed switch limit triggered  
2. Backoff 0.5 rad CCW  
3. Slow creep at −0.4 rad/s until limit re-engaged  
4. Move to home position (θ = 0)

### Motor Parameters (Empirically Identified)

- Rm = 2.294 Ω, Lm = 0.002 H
- Ke = Kt = 0.7384 V·s/rad (back-EMF / torque constant)
- J = 0.006747 kg·m², b = 0.0062 N·m·s/rad
- Gear ratio N = 5 (all user-facing values in output shaft space)
