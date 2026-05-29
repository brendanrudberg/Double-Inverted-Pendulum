# Double Inverted Pendulum on a Cart

A double inverted pendulum stabilization and swing-up system built by Division One (Brendan Rudberg, Emily Stanton, Rajat Bidarkota, Mark Luo, Maxwell Braithwaite, Luis Yael Serrano Laguna) for ME14 at Caltech.

## Overview

This project implements nonlinear control of a double inverted pendulum on a belt-driven cart. The system uses a 42mm brushless DC motor with Field-Oriented Control (FOC) for smooth actuation, three capacitive encoders for state feedback, and a Teensy 4.0 running both the FOC motor control loop and the pendulum stabilization controller at 600 MHz.

## Hardware

### Mechanical
- **Rail:** 1m 2020 V-slot aluminum extrusion with V-wheel gantry plate
- **Cart actuation:** GT2 timing belt driven by 42mm BLDC motor (24V, 5000 RPM, 8-pole)
- **Pendulum arms:** 3/8"-16 aluminum threaded rod with sliding nuts for adjustable center of mass
  - Link 1 (lower): ~15 cm
  - Link 2 (upper): ~25 cm
- **Pivot shafts:** 1/4" tight-tolerance ground carbon steel, drilled hollow for wire routing
- **Bearings:** 1/4" bore flanged ball bearings (2x at joint 1, 1-2x at joint 2)
- **Slip ring:** 6-wire capsule slip ring at joint 1 for encoder 2 wire routing

### Electrical
- **Controller:** Teensy 4.0 (ARM Cortex-M7, 600 MHz, hardware FPU)
- **Motor driver:** KOOBOOK SimpleFOC Shield V2.0.4 compatible (discrete MOSFETs, INA240 current sensing)
- **Encoders:** 3x AMT10E2-V capacitive incremental encoders (up to 5120 PPR, 3.3V compatible)
  - Encoder 1: motor shaft (cart position + FOC commutation)
  - Encoder 2: joint 1 (link 1 angle)
  - Encoder 3: joint 2 (link 2 angle, routed through slip ring)
- **Power:** 24V 5A DC supply → motor driver, buck converter → 5V → Teensy VIN
- **Safety:** 2x limit switches at rail ends, 2x toggle switches (start/stop, equilibrium select)

## Software Architecture

### Control Loop Structure
```
┌─────────────────────────────────────────────────┐
│                  Teensy 4.0                      │
│                                                  │
│  ┌──────────────┐     ┌───────────────────────┐ │
│  │  FOC Motor   │     │  Pendulum Controller  │ │
│  │  Control     │◄────│  (Nonlinear)          │ │
│  │  (~30 kHz)   │     │  (~500-1000 Hz)       │ │
│  └──────┬───────┘     └───────────┬───────────┘ │
│         │                         │              │
│         ▼                         ▼              │
│  KOOBOOK Shield          3x AMT10E Encoders     │
│  (3-phase PWM)           (quadrature decode)     │
│         │                         │              │
│         ▼                         │              │
│    BLDC Motor ◄───────────────────┘              │
└─────────────────────────────────────────────────┘
```

### Modules

```
src/
├── main.cpp                 # Main loop, control mode switching
├── foc/
│   ├── foc_config.h         # Motor parameters, pin assignments
│   └── foc_setup.cpp        # SimpleFOC initialization and tuning
├── control/
│   ├── dynamics.h           # Double pendulum equations of motion
│   ├── dynamics.cpp         # M(y)ÿ = f(y, ẏ, u, w) implementation
│   ├── stabilizer.h         # Nonlinear stabilization controller
│   ├── stabilizer.cpp       # Upright equilibrium stabilization
│   ├── swing_up.h           # Energy-based swing-up controller
│   └── swing_up.cpp         # Chatterjee-style cart potential well
├── sensors/
│   ├── encoder.h            # AMT10E quadrature decoder wrapper
│   ├── encoder.cpp          # Position/velocity estimation
│   └── state_estimator.cpp  # Full state estimation from encoders
├── safety/
│   ├── limits.h             # Limit switch handling
│   └── limits.cpp           # Emergency stop, rail boundary protection
└── config/
    ├── pins.h               # Teensy pin assignments
    ├── params.h             # System physical parameters (masses, lengths, inertias)
    └── gains.h              # Controller gains for each equilibrium
```

## Equations of Motion

The system dynamics follow the standard Euler-Lagrange formulation for a double pendulum on a cart with distributed-mass arms:

```
M(y)ÿ = f(y, ẏ, u, w)
```

Where the mass matrix M(y) is 3x3 (cart + two pendulum angles), u is the cart acceleration (control input), and w represents disturbances. The full expanded form follows Kaheman et al. (2022) with parameters for mass (mᵢ), center of mass distance (aᵢ), arm length (lᵢ), and rotational inertia (Jᵢ).

See `src/control/dynamics.h` for the complete implementation.

## Control Modes

### Stabilization
Nonlinear controller stabilizes the double pendulum at selectable equilibria:
- **Up-Up** (both links inverted) — primary target
- **Down-Up** / **Up-Down** — stretch goals, requires asymmetric link lengths (satisfied by design)

Equilibrium selection via two toggle switches (2-bit binary encoding).

### Swing-Up
Energy-based swing-up using the Chatterjee et al. (2002) cart potential well method, extended to the double pendulum. The controller injects energy while respecting track length constraints through a logarithmic penalty function. A cruise mode maintains energy until the pendulum enters the capture region, where the stabilization controller takes over.

## Setup and Calibration

### 1. Flash SimpleFOC Configuration
Configure motor parameters in `src/foc/foc_config.h`:
```cpp
#define POLE_PAIRS 4
#define PHASE_RESISTANCE 3.0  // measure with multimeter
#define PWM_FREQUENCY 30000   // 30 kHz FOC loop
```

### 2. Encoder Calibration
With the belt disconnected, run the FOC calibration routine to learn the encoder-to-electrical-angle mapping:
```cpp
motor.initFOC();  // spins motor slowly, maps encoder positions
```

### 3. System Identification
Hang the pendulum freely, record free-swing data, and fit parameters:
- Link masses (m1, m2)
- Center of mass distances (a1, a2) — adjustable via sliding nuts
- Link lengths (l1, l2)
- Moments of inertia (J1, J2)
- Friction coefficients (d1, d2, d3)

Update values in `src/config/params.h`.

### 4. Homing
On startup, the controller:
1. Waits for toggle switch activation
2. Drives cart to each limit switch to find rail boundaries
3. Centers the cart
4. Zeros the cart encoder
5. Reads pendulum angles (assumes pendant resting position = known angle)
6. Enters idle mode, awaiting start command

## Pin Assignments

| Function | Teensy Pin | Notes |
|----------|-----------|-------|
| FOC PWM A | TBD | Must be PWM-capable |
| FOC PWM B | TBD | Must be PWM-capable |
| FOC PWM C | TBD | Must be PWM-capable |
| FOC Enable | TBD | Digital output |
| Current Sense A | TBD | Analog input |
| Current Sense B | TBD | Analog input |
| Motor Encoder A | TBD | Digital input (also wired to KOOBOOK) |
| Motor Encoder B | TBD | Digital input (also wired to KOOBOOK) |
| Joint 1 Encoder A | TBD | Digital input |
| Joint 1 Encoder B | TBD | Digital input |
| Joint 2 Encoder A | TBD | Digital input (through slip ring) |
| Joint 2 Encoder B | TBD | Digital input (through slip ring) |
| Limit Switch L | TBD | Digital input, internal pull-up |
| Limit Switch R | TBD | Digital input, internal pull-up |
| Toggle 1 (Start) | TBD | Digital input, internal pull-down |
| Toggle 2 (Equilib) | TBD | Digital input, internal pull-down |

## Dependencies

- [SimpleFOC](https://simplefoc.com/) — Field-Oriented Control library for BLDC motors
- [Encoder](https://www.pjrc.com/teensy/td_libs_Encoder.html) — Teensy quadrature encoder library (hardware-accelerated)
- [TeensyTimerTool](https://github.com/luni64/TeensyTimerTool) — Precise timer interrupts for control loop scheduling

## Key References

1. Kaheman, K., Fasel, U., Bramburger, J.J., Strom, B., Kutz, J.N., & Brunton, S.L. (2022). "The Experimental Multi-Arm Pendulum on a Cart: A Benchmark System for Chaos, Learning, and Control." *arXiv:2205.06231*. [GitHub](https://github.com/dynamicslab/MultiArm-Pendulum)

2. Chatterjee, D., Patra, A., & Joglekar, H.K. (2002). "Swing-up and stabilization of a cart-pendulum system under restricted cart track length." *Systems & Control Letters*, 47, 355-364.

3. Åström, K.J. & Furuta, K. (2000). "Swinging up a pendulum by energy control." *Automatica*, 36(2), 287-295.

## Safety

- Limit switches at both rail ends trigger immediate motor shutdown
- Software position/velocity limits enforced before hardware limits
- Emergency stop: toggle switch kills control loop, motor coasts to stop
- 24V power supply switch provides hard motor power cutoff
- Never stand to the side of the pendulum arms during operation
- Wear safety glasses during testing

## License

MIT
