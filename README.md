# Furuta Rotary Inverted Pendulum

A fully custom-built Furuta pendulum — designed, 3D printed, wired, and programmed from scratch. Balanced using a dual parallel PID control loop, with a MuJoCo digital twin used to pre-tune gains before real hardware deployment.

**sim-to-real accuracy. (with some mild scaling)** Built in ~4 weeks with no formal engineering training.
Received **1.2K+** views on YouTube.

📝 [Full engineering write-up on Medium](https://medium.com/@davesoh929/exploring-the-master-of-control-systems-furuta-2c28d4e2d2b4)

<p align="center">
  <img src="pend.gif" alt="Working Pendulum" width="600"/>
</p>

---

## What is a Furuta Pendulum?

A Furuta pendulum is a rotary inverted pendulum — one of the hardest benchmark systems in control theory. Unlike a linear cart-pole, the rotating arm introduces centripetal coupling between the two joints, making it significantly harder to control.

It's the same class of underactuated system found in:
- Humanoid robot balance (bipedal locomotion)
- Rocket thrust vector control
- Drone attitude stabilization

The system has **2 degrees of freedom but only 1 actuator** — making it fundamentally underactuated. Standard control techniques break down. It requires careful coupling of two control loops.

---

## System Overview

| Component | Detail |
|---|---|
| Microcontroller | ESP32 DevKit C |
| Motor | 500 RPM N20 DC gear motor |
| Motor Driver | DRV8833 H-Bridge |
| Arm Encoder | Quadrature encoder on N20 |
| Pendulum Encoder | AS5600 magnetic position sensor + diametric magnet |
| Power | 4x AA batteries in series (6V) |
| Frame | Custom 3D printed (Elegoo Neptune 3 Pro) |
| CAD | Onshape — 12 unique parts, 50+ revisions |
| Total Build Cost | ~$50 electronics + filament |

---

## Control Architecture

### Dual Parallel PID

The core challenge: the arm must stay centered (arm PID) while the pendulum must stay upright (pendulum PID). These two objectives directly oppose each other — correcting the arm moves the pendulum, and vice versa.

The solution is two independent PID loops whose outputs are summed with opposing signs:

```
totalOutput = -arm_output + pend_output
```

**Pendulum PID** — high gain, keeps the pendulum upright at 185°
**Arm PID** — lower gain, prevents the arm from spinning out of range

Key implementation details:
- **Modular angle error** — wraps error correctly across the 0°/360° boundary to prevent sign flips
- **Deadzone compensation** — motor requires minimum ~55 PWM to overcome static friction; offset applied to all non-zero outputs
- **Safety range** — motor output disabled if pendulum or arm exits safe bounds, preventing wire snap
- **Fixed loop rate** — `micros()` used instead of `delay()` to maintain stable dt for derivative calculation
- **Ziegler-Nichols tuning** — KP raised until oscillation, then KD added to dampen

### Custom PID Library

Rather than using an existing library, I wrote a standalone C++ header-only PID class (`Pid.h`) with:
- Configurable KP, KI, KD
- Angle output clamping
- Modular error wrapping (optional per instance)
- Stable dt calculation via `last_time` tracking

---

## MuJoCo Digital Twin

Before touching real hardware, I built a complete digital twin in MuJoCo — importing the actual STL files from Onshape and parameterizing mass, damping, and friction to match the physical system.

**Why this mattered:**
- Tuned PID gains in simulation first — safe and fast iteration
- Identified coupling behavior between arm and pendulum loops
- Achieved **sim-to-real transfer accuracy** on final gain set
- Took 20+ hours to get the simulation working correctly

The simulation uses the same PID class logic as the firmware, written in Python with the MuJoCo library.

---

## Known Limitations

- **Motor deadzone jitter** — the N20 motor has a high minimum actuation threshold (~55 PWM). This causes constant slight overcorrection visible as pendulum oscillation. A higher-precision motor (e.g., coreless DC or brushless with encoder) would eliminate this.
- **No swing-up** — the system balances from near-upright only. Lyapunov energy-based swing-up is the planned next step.
- **No integral windup protection** — under extended disturbance, integral accumulates. Anti-windup clamping not yet implemented.

---

## Repository Structure

```
pid-mujoco-furuta-pendulum/
├── Code/
│   └── furatamjcf.xml        # MuJoCo model file (MJCF)
│   └── simulation.py         # Python MuJoCo simulation + PID
├── Furata Pendulum/src/
│   └── main.cpp              # ESP32 firmware
│   └── Pid.h                 # Custom PID library
├── STLs/                     # All printable parts
└── README.md
```

---

## Build Stats

| Metric | Value |
|---|---|
| Total hours | 100-125 |
| CAD time | 50+ hours |
| Print time | 30+ hours |
| Filament used | ~500g total (prototypes + final) |
| CAD revisions | 50+ across 12 unique parts |
| Simulation time | 20+ hours |

---

## What's Next

- [ ] Lyapunov energy-based swing-up from arbitrary initial angle
- [ ] LQR controller implementation and comparison vs PID
- [ ] RL policy trained in MuJoCo, deployed on real hardware (sim-to-real) (next frontier I am to learn!)
- [ ] Higher precision motor to eliminate deadzone jitter

---

## Contact Me

📧 davesoh929@gmail.com | [GitHub](https://github.com/sohamdave95) | [Medium](https://medium.com/@davesoh929)
