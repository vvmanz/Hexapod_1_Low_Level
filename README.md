# Hexapod · Low-Level Control

**STM32F103 firmware for closed-loop actuator control: current, velocity and position loops, ADC filtering, encoders and CAN communication.**

English | [Русский](README.ru.md)

This is the **low-level embedded layer** of a multi-level hexapod control project. The firmware runs on an STM32F103C8T6 and handles time-critical motor measurements and control close to the actuators. It provides several selectable combinations of cascaded control loops, rather than a single fixed PID mode.

## Features

- **Two motor channels per controller** with PWM output and direction control.
- **Cascaded feedback loops** for position/angle, speed and motor current; configurable combinations are selected in `pid_enable()`.
- **ADC + DMA measurements** with filtering and conversion into electrical quantities.
- **Timer/encoder interrupts** for speed and angular-position calculations.
- **CAN protocol routines** for exchanging commands and telemetry with the higher control level.
- **Absolute-angle calibration** and controller coefficients in dedicated modules.

## Control architecture

```text
High-level target / CAN message
              │
              ▼
       Position controller
              │
              ▼
        Speed controller     ── encoder feedback
              │
              ▼
       Current controller    ── ADC + DMA + filtering
              │
              ▼
       PWM / motor driver    ── two actuator channels
```

The firmware also supports shorter control chains, for example speed + current, position only or current only. The implemented modes are visible in [`Core/Src/main.c`](Core/Src/main.c) and [`Core/Src/motor_CONTROL.c`](Core/Src/motor_CONTROL.c).

## Hardware and tools

- **Target:** STM32F103C8T6, with the peripheral configuration stored in `F103C8T6_with_help.ioc`.
- **IDE:** STM32CubeIDE / STM32CubeMX-generated STM32 HAL project.
- **Interfaces used:** ADC, DMA, timers, GPIO, CAN and CRC.
- **For hardware tests:** compatible motor driver, current measurement circuit, position feedback, power supply and suitable debugger (for example ST-LINK).

Hardware wiring, board revision and motor limits must be checked against the `.ioc` pin assignments and your own electronics; a complete wiring diagram is **not included** in this archive.

## Build and flash

1. Open the repository as an existing STM32CubeIDE project (the root contains `.project` and `.cproject`).
2. Inspect the MCU/pin configuration in [`F103C8T6_with_help.ioc`](F103C8T6_with_help.ioc).
3. Review `number_bridge`, controller gains, current calibration and PWM limits before powering any actuators.
4. Build the `Debug` configuration with the STM32CubeIDE toolchain.
5. Flash and debug using a compatible STM32 programmer/debugger.

A `Debug/` folder with compiled artifacts is present in the uploaded snapshot, but **building from source** is recommended. There is no desktop executable or one-command software-only demo.

## Code map

| File | Role |
|---|---|
| `Core/Src/main.c` | Peripheral initialization, main loop, controller mode dispatch |
| `Core/Src/motor_CONTROL.c` | Cascaded controllers and PWM control |
| `Core/Src/motor_FILTERS.c` | Signal filtering and speed/voltage/current processing |
| `Core/Src/angle_ABSOLUTE.c` | Angle calculation and calibration |
| `Core/Src/can_TRANSMIT_RECEIVE.c` | CAN TX/RX handlers and data packing |
| `Core/Src/stm32f1xx_it.c` | Interrupt handlers, including timer/encoder processing |
| `Core/Inc/main.h` | Shared definitions and controller data |
| `Drivers/` | ST HAL and CMSIS source |

## Integration with the other hexapod layers

```text
Hexapod_3_High_Level   motion planning and gait commands
           │
Hexapod_2_Middle_Level command distribution, ROS 2, odometry, CAN
           │
Hexapod_1_Low_Level    motor feedback and actuator control  ← this repo
```

These are separate repositories representing an engineering architecture; they are **not delivered as a single preconfigured launchable system**.

## Status and notes

- Embedded research/development firmware for a physical prototype.
- CAN message functions are present, but parts of CAN initialization/transmission in `main.c` are commented out in the archived revision; check the relevant path before integration.
- Motor outputs are **physical and potentially hazardous**. Test with appropriate current limits, unloaded mechanics and a stop mechanism.
- No separate top-level project licence was included; the STM32 driver files retain their own licences.
