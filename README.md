# STM32 Modular Flight Controller Firmware

![C++](https://img.shields.io/badge/Standard-C++20-blue.svg)
![MCU](https://img.shields.io/badge/MCU-STM32F103-lightgrey.svg)
![Architecture](https://img.shields.io/badge/Architecture-HAL%20Decoupled-success.svg)
![Status](https://img.shields.io/badge/Phase-SITL%20Integration-orange.svg)

A bare-metal, modular flight controller firmware developed for the STM32F103 (ARM Cortex-M3) microcontroller. 

Designed with a strict adherence to safety-critical engineering principles, this project emphasizes deterministic state management, numerical stability, and a rigid separation between the Hardware Abstraction Layer (HAL) and flight kinematics logic.

## Development Phase: SITL & Hardware Abstraction

Development is currently focused on architectural validation through Software-In-The-Loop (SITL) methodologies. 

To ensure maximum software reliability prior to physical hardware deployment, the core control loop (`FlightTask`) has been completely decoupled from MCU-specific registers and interfaces. This architectural decision allows the flight logic to be compiled and executed within a desktop environment, enabling rigorous testing of PID algorithms and state transitions against a simulated physics engine.

## Core Capabilities & Safety Protocols

Reliability in UAV operations is achieved through strict software constraints and fault mitigation:

*   **Numerical Stability & Fault Handling:** The Motor Mixer incorporates strict validation against `NaN` and `Infinity` float values, preventing arithmetic exceptions (e.g., sensor division-by-zero or integrator windup) from propagating to the PWM actuation layer.
*   **Deterministic State Machine (FSM):** Arming sequences and ESC calibrations are controlled by a rigid finite-state machine. The system defaults to a `DISARMED` safe state upon detecting invalid RC inputs or logic violations.
*   **Real-Time Telemetry & Configuration:** A custom UART command parser (`CLI`) enables real-time tuning of PID coefficients. It utilizes secure float parsing (`atof()`) to circumvent embedded `newlib-nano` limitations without triggering memory faults.
*   **Digital Signal Processing (DSP):** Implements First-Order Infinite Impulse Response (IIR) Low-Pass Filters on raw MPU6050 gyroscope data to attenuate high-frequency mechanical resonance and electrical noise.

## Architectural Design: Inversion of Control

The firmware avoids monolithic design patterns by implementing Dependency Injection. The hardware drivers (I2C, UART, PWM Timers) are instantiated exclusively in the main execution wrapper and injected into the `FlightTask` class via abstract interfaces.

```mermaid
graph TD
    subgraph Hardware Abstraction Layer (HAL)
        I2C[I2C Bus Driver] --> MPU[MPU6050]
        UART[UART DMA Driver] --> RX[IBus Protocol Parser]
        CLI_In[UART RX Buffer] --> CLI[Command Interface]
        PWM[Hardware Timers] --> Motors[ESC Actuators]
    end

    subgraph Flight Control Loop (FlightTask)
        MPU --> Filter[IIR Low-Pass Filters]
        RX --> State[FSM / State Manager]
        CLI --> PID[PID Controllers]
        Filter --> PID
        State --> Mixer[Kinematic Motor Mixer]
        PID --> Mixer
        Mixer --> PWM
    end
```

## Repository Structure

```text
├── src/
│   ├── core/
│   │   ├── controllers/   # FlightTask, PIDController, SystemState, CLI
│   │   └── filters/       # LowPassFilter implementation
│   ├── devices/           # Sensor drivers (e.g., MPU6050)
│   └── hal/               # STM32 hardware interfaces (I2C, UART, PWM, SysTick)
└── main.cpp               # System initialization and Dependency Injection
```

## Technical Roadmap

Future development iterations will focus on advanced physical modeling and high-speed digital actuation:

- [ ] **SITL IPC Bridge:** Develop a UDP-based Inter-Process Communication (IPC) layer to stream virtual sensor telemetry to the `FlightTask` and relay PWM outputs to a host machine.
- [ ] **Rigid Body Physics Engine:** Implement a lightweight 3D physics simulator (C++ or Rust) to visualize control algorithms and flight kinematics in real-time.
- [ ] **Advanced Attitude Estimation:** Replace linear geometric calculations with a quaternion-based AHRS (Madgwick or Mahony filter) to eliminate gimbal lock and improve 3D spatial awareness.
- [ ] **DShot Implementation:** Migrate from analog PWM signaling to the DShot300 digital ESC protocol, utilizing STM32 DMA for deterministic, microsecond-level actuation.

---
*Developed by Daniil*