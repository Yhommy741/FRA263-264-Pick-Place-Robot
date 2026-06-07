# FRA263/264 — Circular Pattern Pick & Place Robot
### Embedded Firmware · Group 5 · Yhommy

> **Platform:** STM32G474RE (Nucleo-64) · **IDE:** STM32CubeIDE · **HAL:** STM32 HAL/LL  
> **Course:** FRA263 / FRA264 — Robotics Studio II | Industrial Robot

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Hardware Summary](#2-hardware-summary)
3. [Software Architecture](#3-software-architecture)
4. [Module Reference](#4-module-reference)
5. [Communication Interfaces](#5-communication-interfaces)
6. [System State Machine](#6-system-state-machine)
7. [Trajectory Profiles](#7-trajectory-profiles)
8. [Gripper Subsystem](#8-gripper-subsystem)
9. [Configuration Reference](#9-configuration-reference)
10. [Building & Flashing](#10-building--flashing)
11. [Debugging Tips](#11-debugging-tips)

---

## 1. Project Overview

This firmware controls a **single-axis revolute joint robot arm** as part of the FRA263/264 university course project. The arm is driven by a 24 V DC motor with a gear ratio of N = 5, and commanded from a PC-based dashboard ("BaseSystem") over Modbus RTU / RS-485, as well as from a physical joystick controller over a second RS-485 link. A pneumatic/solenoid gripper at the end-effector can be driven either by direct GPIO (IO mode) or via CAN bus (CAN mode).

The firmware is structured around a clean three-layer pipeline:

```
BaseSystemInterface  ──┐
(Modbus decoder)       ├──►  TaskManager  ──►  Robot API
JoystickInterface  ────┘    (event queue,      (motion + gripper)
                             FSM runner)
```

All hardware constants, gains, and compile-time options live in a single file: **`RobotConfig.h`**.

---

## 2. Hardware Summary

| Item | Detail |
|---|---|
| **MCU** | STM32G474RET6 (Nucleo-64), Cortex-M4 @ 170 MHz |
| **Motor** | DC brushed, 24 V, Rm = 2.294 Ω, Ke = Kt = 0.7384 V·s/rad |
| **Encoder** | Quadrature, 2048 PPR × 4 = 8192 CPR (TIM1) |
| **Gear Ratio** | N = 5 |
| **Driver** | MD20A — DIR + PWM (TIM2 CH1 / CH2) |
| **Workspace** | −90° to +450° (encoder-accumulated, not wrapped) |
| **Max Speed** | 5.6 rad/s (joint) |
| **Max Accel** | 5.0 rad/s² |
| **Max Jerk** | 12.0 rad/s³ |
| **Control Timer** | TIM3 @ 0.5 ms (2 kHz fast loop) |
| **Limit Switch** | PC2 (active-low) |
| **E-Stop Button** | PC13 (active-low) |
| **Mode Switch** | PB4 (active-low: LOW = Manual, HIGH = Auto) |
| **PC Link** | USART2, RS-485, Modbus RTU, 19200 baud, 8E1 |
| **Joystick Link** | USART3, RS-485, 9600 baud, 8N1, DMA RX |
| **CAN Bus** | FDCAN1 (gripper node ID = 0x10) |

---

## 3. Software Architecture

### 3.1 Three-Layer Pipeline

```
┌──────────────────────────────────────────────────────────────┐
│  Main Loop (main.c)                                          │
│                                                              │
│  BaseSystemInterface_Update()   ← Modbus RX/TX, reg sync    │
│  JoystickInterface_Update()     ← DMA UART parse            │
│                                                              │
│  BaseSystem_Interface_Decode()  ← reg → BSI_PendingCmd_t    │
│                                                              │
│  Task_PostFromModbus()   ─┐                                  │
│  Task_PostFromJoystick() ─┼─► event queue (depth = 8)       │
│                            │                                  │
│  Task_Run()  ──────────────┘                                 │
│    ├─ Priority pre-scan (ESTOP / STOP interrupt active FSM)  │
│    ├─ FSM tick (sequence / prec-test / perf-test / gripper)  │
│    └─ Queue dispatch → Robot API                             │
│                                                              │
│  Robot_Update()         ← called from TIM3 ISR @ 0.5 ms     │
│  Robot_CANBus_Update()  ← called from main loop (CAN mode)  │
└──────────────────────────────────────────────────────────────┘
```

### 3.2 Event Priority (TaskManager)

| Priority | Event | Source |
|---|---|---|
| 1 (highest) | `TASK_EVT_ESTOP` | Any — clears queue, cuts motor |
| 2 | `TASK_EVT_STOP` | Any — soft stop |
| 3 | Joystick motion (`JOG_STEP`, `JOG_VEL`, `HOME`) | Joystick override |
| 4 | Modbus motion (`HOME`, `MOVE`, `SEQUENCE`, `PREC_TEST`) | PC |
| 5 (lowest) | Gripper commands | Either source |

Commands that conflict with the current robot state are silently dropped (e.g. `MOVE` while homing).

### 3.3 Control Loop

The inner control loop runs inside the **TIM3 ISR at 0.5 ms (2 kHz)**. The outer position loop runs every 10th fast-loop tick (5 ms, 200 Hz).

```
Position PID (200 Hz) ──► velocity setpoint
Velocity PID + FF (2 kHz) ──► voltage command ──► MD20A PWM
                   ↑
        Kalman filter (4-state observer)
        State: [ θ  ω  I  τ_d ]
```

---

## 4. Module Reference

| File | Role |
|---|---|
| `RobotConfig.h` | **Single source of truth** — all hardware constants, PID gains, compile-time options |
| `main.c` | System state machine, main loop, peripheral init |
| `Robot.c/.h` | High-level robot API — init, move, jog, home, e-stop, update |
| `TaskManager.c/.h` | Command arbitration, event queue, FSM runner |
| `BaseSystemInterface.c/.h` | Modbus register decoder (Robot-agnostic) |
| `JoystickInterface.c/.h` | Joystick RS-485 UART frame parser |
| `Controller.c/.h` | Cascaded PID (velocity + position) with feed-forward |
| `KalmanFilterDCMotor.c/.h` | 4-state discrete Kalman filter (CMSIS-DSP) |
| `SCurve.c/.h` | 7-segment jerk-limited S-curve trajectory generator |
| `Trapezoid.c/.h` | 3-phase position-triggered trapezoidal trajectory |
| `Gripper.c/.h` | Dual-mode gripper driver (IO / CAN bus) |
| `CANBus.c/.h` | CAN protocol v1.0.1 driver (FDCAN) |
| `MD20A.c/.h` | Motor driver (DIR + PWM via TIM2) |
| `QEI.c/.h` | Quadrature encoder interface |
| `DCMotor.c/.h` | DC motor parameter struct |
| `PWM.c/.h` | General-purpose PWM helper |
| `SerialFrame.c/.h` | Binary serial framing over UART (DMA) |
| `ModbusRTU.c/.h` | Third-party Modbus RTU slave stack |

---

## 5. Communication Interfaces

### 5.1 Modbus RTU — PC BaseSystem (USART2)

| Parameter | Value |
|---|---|
| Baud | 19 200 |
| Frame | 8E1 |
| Bus | RS-485 |
| Slave Address | 21 |
| Function Codes | FC03 (read holding registers), FC06 (write single register) |

**Key registers (subset):**

| Address | Name | Direction | Description |
|---|---|---|---|
| `0x00` | `REG_HEARTBEAT` | R | Robot alive token (22881) |
| `0x01` | `REG_OP_MODE` | W | Operating mode bitmask |
| `0x02` | `REG_MANUAL_GRIPPER` | W | Manual gripper command |
| `0x04` | `REG_GRIPPER_AUTO_EN` | W | Enable gripper in auto sequence |
| `0x05` | `REG_JOG_DEG` | W | Jog step size (degrees × 10) |
| `0x22` | `REG_SEQ_PAIRS` | W | Number of sequence positions |
| `0x24` | `REG_P2P_TARGET` | W | Point-to-point target (degrees × 10) |
| `0x25` | `REG_SOFT_STOP` | W | Software emergency stop (write 1) |
| `0x26` | `REG_SENSORS` | R | Gripper sensor bits |
| `0x27` | `REG_ROBOT_TASK` | R | Current task bitmask |
| `0x28` | `REG_POSITION` | R | Joint angle (degrees × 10) |
| `0x29` | `REG_VELOCITY` | R | Joint velocity (rad/s) |
| `0x31` | `REG_EMERGENCY` | R | Emergency active flag |

> **One-shot commands:** The PC writes a value then immediately writes 0. The firmware uses a latch-and-consume pattern; registers are cleared on capture.

### 5.2 Joystick RS-485 (USART3)

| Parameter | Value |
|---|---|
| Baud | 9 600 |
| Frame | 8N1 |
| Bus | RS-485 |
| RX | DMA, circular, 6-byte frames |

**Frame layout:**

```
[ 0xAA | CMD | PARAM_HI | PARAM_LO | MODE | CHECKSUM ]
```

**Command bytes:**

| Byte | Command |
|---|---|
| `0x01` | Move to position |
| `0x02` | Soft stop |
| `0x04` | Set home |
| `0x05` | Go home |
| `0x06` | Jog CCW (velocity) |
| `0x07` | Jog CW (velocity) |
| `0x08` | Jog step CCW |
| `0x09` | Jog step CW |
| `0x0A`–`0x0D` | Gripper Up/Down/Open/Close |

### 5.3 CAN Bus — Gripper Node (FDCAN1)

| Parameter | Value |
|---|---|
| Protocol | Custom v1.0.1 |
| Node ID | `0x10` |
| ID Width | 11-bit |
| Heartbeat | Master → broadcast every ≤ 500 ms |

**ID encoding:** `[Bits 10:8] = Function Code` · `[Bits 7:0] = Node ID`

| FC | Direction | Meaning |
|---|---|---|
| `0x0` | Node → Master | EMCY |
| `0x1` | Node → Master | Real-Time Data |
| `0x2` | Master → Node | Command Request |
| `0x3` | Node → Master | Command Response |
| `0x6` | Broadcast | Master Heartbeat |
| `0x7` | Node → Master | Node Heartbeat |

---

## 6. System State Machine

```
                     ┌──────────────────────┐
                     │    SYS_STATE_ESTOP   │
                     │  Motor cut, no cmds  │
                     └──────────┬───────────┘
          E-Stop pressed        │ E-Stop released
          (any state)           ▼
                     ┌──────────────────────┐
                     │    SYS_STATE_RESET   │
                     │  Re-init all, then → │
                     └──────────┬───────────┘
                                │ immediately
              ┌─────────────────┴─────────────────┐
              ▼                                     ▼
   ┌─────────────────────┐             ┌─────────────────────┐
   │   SYS_STATE_AUTO    │◄───────────►│  SYS_STATE_MANUAL   │
   │  Modbus cmds only   │  Mode pin   │  Joystick cmds only  │
   └─────────┬───────────┘  toggles   └─────────────────────┘
             │ REG_SOFT_STOP = 1
             ▼
   ┌─────────────────────┐
   │ SYS_STATE_SOFT_ESTOP│
   │  No exit — hardware │
   │  reset required     │
   └─────────────────────┘
```

| State | Mode Pin | Commands accepted |
|---|---|---|
| `SYS_STATE_AUTO` | HIGH | Modbus (PC dashboard) |
| `SYS_STATE_MANUAL` | LOW | Joystick only |
| `SYS_STATE_ESTOP` | — | None (hardware E-stop) |
| `SYS_STATE_RESET` | — | None (one-shot re-init) |
| `SYS_STATE_SOFT_ESTOP` | — | None (software lock, hardware reset to exit) |

---

## 7. Trajectory Profiles

Selected at compile time via `TRAJ_MODE` in `RobotConfig.h`.

### 7.1 S-Curve (default) — `TRAJ_MODE_SCURVE`

7-segment jerk-limited profile. Guarantees smooth acceleration and deceleration with bounded jerk.

```
Phases:  +J │ const-A │ −J │ cruise │ −J │ const-D │ +J
```

Boundary conditions: `θ̇(0) = 0`, `θ̈(0) = 0`, `θ̇(T) = 0`, `θ̈(T) = 0`

Homing **always** uses S-curve regardless of `TRAJ_MODE`.

### 7.2 Trapezoid — `TRAJ_MODE_TRAPEZOID`

3-phase position-triggered trapezoidal profile. Phase transitions are triggered by distance travelled (not elapsed time), which prevents overshoot caused by motor lag at deceleration entry.

```
Trapezoidal (v²/a ≤ stroke):    Triangular (v²/a > stroke):
vel                               vel
 ^  _________                      ^    /\
 | /         \                     |   /  \
 |/           \                    |  /    \
 +--+-------+--+--> pos            +-+------+--> pos
 0  s_a   s-s_d s                  0  s/2   s
```

To switch profiles, edit `RobotConfig.h`:

```c
#define TRAJ_MODE    TRAJ_MODE_SCURVE      // or TRAJ_MODE_TRAPEZOID
```

---

## 8. Gripper Subsystem

The gripper supports two operating modes, selected at compile time via `GRP_MODE` in `RobotConfig.h`.

### 8.1 IO Mode — `GRP_MODE_IO`

Each command pulses a dedicated GPIO pin HIGH for `pulse_duration_ms`, then resets it LOW. Sensor state is read from GPIO input pins directly.

| Action | Output GPIO |
|---|---|
| Move Up | PC1 |
| Move Down | PC6 |
| Open | PC0 |
| Close | PC3 |

| Sensor | Input GPIO |
|---|---|
| Up | PB0 |
| Down | PB1 |
| Claw (open/close) | PB2 |

### 8.2 CAN Mode — `GRP_MODE_CANBUS`

Each command transmits a Write Relay frame (`ID = 0x210`, DLC = 3) with the appropriate relay bitmask. The all-off frame is sent after `pulse_duration_ms`. Sensor lines are **always read from GPIO** — CAN protocol v1.0.1 does not provide sensor readback.

| Relay Bit | Solenoid |
|---|---|
| Bit 0 | Up |
| Bit 1 | Down |
| Bit 2 | Close |
| Bit 3 | Open |

> **Note:** `Robot_CANBus_Update()` must be called every main-loop iteration in CAN mode. `Gripper_Update()` and `CANBus_Update()` are **not ISR-safe** and must not be called from TIM3.

To switch modes, edit `RobotConfig.h`:

```c
#define GRP_MODE    GRP_MODE_IO      // or GRP_MODE_CANBUS
```

---

## 9. Configuration Reference

All tunable parameters are in **`RobotConfig.h`**. No other file needs to be edited for a typical hardware bring-up.

### Motor Parameters
| Define | Value | Unit |
|---|---|---|
| `MOTOR_RM` | 2.2940 | Ω |
| `MOTOR_LM` | 0.0020 | H |
| `MOTOR_KE` | 0.73840 | V·s/rad |
| `MOTOR_KT` | 0.73840 | N·m/A |
| `MOTOR_J` | 0.0067475 | kg·m² |
| `MOTOR_B` | 0.0061979 | N·m·s/rad |
| `MOTOR_V_MAX` | 24.0 | V |

### Kinematics
| Define | Value | Unit |
|---|---|---|
| `SPEED_RATIO` | 5.0 | — |
| `RBT_WORKSPACE_MIN` | −90.0 | degrees |
| `RBT_WORKSPACE_MAX` | 450.0 | degrees |
| `RBT_MAX_SPEED` | 5.6 | rad/s |
| `RBT_MAX_ACCEL` | 5.0 | rad/s² |
| `RBT_MAX_JERK` | 12.0 | rad/s³ |

### PID Gains
| Define | Value |
|---|---|
| `KP_VEL` | 1.5 |
| `KI_VEL` | 40.0 |
| `KD_VEL` | 0.0005 |
| `KP_POS` | 4.0 |
| `KI_POS` | 0.0 |
| `KD_POS` | 0.0 |

### Control Loop Timing
| Define | Value | Description |
|---|---|---|
| `CTRL_PERIOD` | 0.0005 s | Fast loop (TIM3 ISR, 2 kHz) |
| `CTRL_LOOP_MULTI` | 10 | Position loop = 5 ms (200 Hz) |

### Homing Constants
| Define | Value | Description |
|---|---|---|
| `RBT_HOMING_FAST` | −0.5 rad/s | Fast approach (CW) |
| `RBT_HOMING_SLOW` | −0.3 rad/s | Slow creep (CW) |
| `RBT_HOMING_BACKOFF` | 0.3 rad | Back-off distance (CCW) |
| `RBT_DEFAULT_HOMING_OFFSET` | 0.064577 rad | Fine offset after limit switch |

---

## 10. Building & Flashing

### Prerequisites
- STM32CubeIDE (tested with CubeIDE 1.15+)
- ST-Link V2 (on-board Nucleo programmer)
- No external libraries required beyond the STM32 HAL and CMSIS-DSP (bundled)

### Build Steps
1. Open STM32CubeIDE and import the project folder.
2. Select **Debug** or **Release** configuration.
3. Press **Build** (Ctrl+B) — no CMake or Makefile steps needed.
4. Connect the Nucleo board via USB.
5. Press **Run** or **Debug** (F11) to flash and start.

### CubeMX Regeneration
The following files are **auto-generated by CubeMX** and must **not** be edited manually. Use CubeMX to change peripheral configuration, then regenerate:

```
dma.c / dma.h
fdcan.c / fdcan.h
gpio.c / gpio.h
tim.c / tim.h
usart.c / usart.h
main.h
stm32g4xx_it.c / stm32g4xx_it.h
stm32g4xx_hal_msp.c
stm32g4xx_hal_conf.h
```

User code is placed only inside `/* USER CODE BEGIN */` / `/* USER CODE END */` blocks.

---

## 11. Debugging Tips

The following Live Expression variables are declared in `main.c` and can be watched in STM32CubeIDE's **Live Expressions** panel during a debug session:

| Variable | What it shows |
|---|---|
| `dbg_gripper_mode` | Active `GripperMode_t` (0 = IO, 1 = CAN) |
| `dbg_up_port_valid` | `up_port_in` pointer is non-NULL (1 = wired) |
| `dbg_pin_up_raw` | Raw `HAL_GPIO_ReadPin` result for the Up sensor |
| `dbg_pin_down_raw` | Raw result for the Down sensor |
| `dbg_pin_claw_raw` | Raw result for the Claw sensor |
| `dbg_sensor_bits` | Final `sensorBits` value written to Modbus |
| `taskMgr.dbg_eventsPosted` | Total events queued since init |
| `taskMgr.dbg_eventsDropped` | Events dropped (queue full) |
| `taskMgr.dbg_eventsRun` | Events dispatched to Robot API |
| `taskMgr.dbg_estopCount` | E-Stop triggers since init |

### Common Pitfalls

- **Mode switch stuck in wrong state:** Check all four locations in `main.c` where `Mode_State` maps to `SYS_STATE_AUTO` / `SYS_STATE_MANUAL`. The mapping is `Mode_State == 1 → AUTO`, `Mode_State == 0 → MANUAL`.
- **Joystick bytes garbled:** Swapped A/B RS-485 lines. Fixed via `UART_ADVFEATURE_RXINV_ENABLE` on USART3.
- **CAN gripper not responding:** Node may not have reached Operational state yet. Check `can_node_init_done` in `robot.can_node_init_done`. Ensure `Robot_CANBus_Update()` is called every main-loop iteration.
- **Modbus command ignored on second press:** One-shot registers use a latch-and-consume pattern. Ensure `BaseSystem_Interface_Decode()` runs every loop iteration.
- **Trajectory overshoot (trapezoid):** Confirm `TRAJ_MODE_TRAPEZOID` uses position-triggered deceleration, not time-triggered. Check `Trapezoid.c`.

---

*FRA263/264 Group 5 · Yhommy · June 2026*
