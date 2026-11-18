# Hexapod_1_Low_Level

**The motor level of a six-legged robot: a cascaded current–speed–angle controller for two DC drives on an STM32F103C8T6, commanded over CAN.**

English | [Русский](README.ru.md)

```
 Hexapod_3_High_Level   ROS 2 / Gazebo: gaits, CPG, navigation
          │  ROS 2 topics
 Hexapod_2_Middle_Level STM32F446RE: FreeRTOS + micro-ROS over Ethernet, odometry
          │  CAN (2 buses)
 ► Hexapod_1_Low_Level  3 × STM32F103C8T6, 2 motors each = 6 leg drives          ◄ you are here
```

This is the firmware of one motor board. Three identical boards drive the six legs, two motors per board; the same code
runs on all of them and `number_bridge` in `Core/Src/main.c` (1, 2 or 3) picks the board's identity and its CAN
addresses. Every motor is closed in three nested loops: current (fast, inner), speed and angle (outer). The master sends
target angles, limits and controller coefficients over CAN and gets angle, speed and current of both motors back.

## Features

* **Cascaded control** per drive: a current loop on the inside, a speed loop around it and an angle loop outside, each
  with its own gains (`kp/ki` for current and speed, `kp/ki/kd` for angle), changeable at run time over CAN.
* **Selectable structure**: a 3-bit `system_type` in the settings frame picks, per motor, which loops are closed:
  0 angle → speed → current, 1 angle → speed, 2 angle → current, 3 angle only, 4 speed → current, 5 speed only,
  6 current only (`pid_*` in `motor_CONTROL.c`), which makes tuning one loop at a time possible.
* **Timing**: the PWM runs at 6 kHz (TIM3, 72 MHz core clock) and the current loop is discretised at the same
  1/6000 s step; speed and angle are estimated every 5 ms (200 Hz).
* **Current and voltage measurement** with the ADC triggered by the PWM timer and read through DMA, so sampling is
  synchronous with the switching and costs no CPU; a 3-sample median filter cleans the current-sensor channels.
* **Encoder interface** on hardware timers (TIM1, TIM2 in encoder mode) with overflow handling, converted to the output
  shaft angle through the gear ratio (26.83).
* **Absolute angle** at start-up from analog position-sensor channels read through the ADC, so the leg knows where it is
  before it moves.
* **CAN node**: hardware acceptance filters per board, receive in interrupts, transmission of angle, speed and current
  to the master; the protocol is described below.
* **STM32CubeMX project** (`F103C8T6_with_help.ioc`): clocks, timers, ADC, DMA and CAN are regenerated from it.

### CAN addresses

Each board listens to its own block of standard IDs (two filters with mask `0x07E1`, even and odd IDs):

| board (`number_bridge`) | IDs it accepts | content |
|---|---|---|
| 1 | `0x200`–`0x21F` | `0x200` target angles, `0x201` limits and other settings, `0x202`, `0x203` … controller coefficients |
| 2 | `0x300`–`0x31F` | same layout |
| 3 | `0x400`–`0x41F` | same layout |

The board answers with angle, speed and current of its two motors, two 32-bit values per frame. The master side of the
protocol is in [Hexapod_2_Middle_Level](https://github.com/<your-account>/Hexapod_2_Middle_Level).

## Requirements

* STM32CubeIDE (or STM32CubeMX + an ARM GCC toolchain);
* an ST-LINK programmer;
* the hardware: an STM32F103C8T6 board, two DC motors with encoders and current sensing on an H-bridge, a CAN
  transceiver.

## Installation and flashing

```bash
git clone https://github.com/<your-account>/Hexapod_1_Low_Level.git
```

1. In STM32CubeIDE: *File → Open Projects from File System…* and select the folder.
2. In `Core/Src/main.c` set `number_bridge = 1;` to `1`, `2` or `3` for the board you are about to flash.
3. *Project → Build*, then *Run → Debug* (or `Run`) with the ST-LINK attached.

Repeat with the next number for the other boards. To change pins, clocks or peripherals, edit
`F103C8T6_with_help.ioc` and generate the code; the control logic lives in the `USER CODE` sections and survives
regeneration.

## Repository layout

```
Core/Src/main.c            init and the main loop: ADC → current/voltage, encoder → speed/angle, PID step
Core/Src/motor_CONTROL.c   the seven regulator structures (pid_angle_speed_curr, pid_speed, pid_curr, ...)
Core/Src/motor_FILTERS.c   ADC → current and voltage with a median filter; encoder counts → speed and angle
Core/Src/angle_ABSOLUTE.c  absolute-angle initialisation from the analog sensor channels
Core/Src/can*.c            CAN setup, filters, receive and transmit
Core/Src/stm32f1xx_it.c    interrupts: encoder overflows, control-loop timers
Core/Src/tim.c, adc.c      generated peripheral setup (timers, ADC + DMA)
Core/Src/VARIABLES.c       shared state and constants
Core/Inc/main.h            pins, reference voltages, current-sensor scaling
F103C8T6_with_help.ioc     STM32CubeMX project
Drivers/                   ST HAL and CMSIS (their own licences)
```

## Part of the Hexapod stack

| level | repository | runs on |
|---|---|---|
| 1 — motors | **Hexapod_1_Low_Level** | STM32F103C8T6 × 3 |
| 2 — coordination | [Hexapod_2_Middle_Level](https://github.com/<your-account>/Hexapod_2_Middle_Level) | STM32F446RE |
| 3 — gaits and navigation | [Hexapod_3_High_Level](https://github.com/<your-account>/Hexapod_3_High_Level) | ROS 2 / Gazebo |

## Licence

The ST HAL and CMSIS code in `Drivers/` keeps its original licences.
