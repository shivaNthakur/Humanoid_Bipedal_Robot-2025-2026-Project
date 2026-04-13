# 🤖 10-DOF Bipedal Humanoid Robot

> **Robotics Club — Motilal Nehru National Institute of Technology Allahabad**
> Prayagraj, U.P. — 211004 | Project Year: 2025–2026

A fully functional **10-Degree-of-Freedom biped humanoid robot** capable of stable quasi-static walking using inverse kinematics, cubic spline gait planning, and real-time servo control via Arduino Mega.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Demo & Simulation](#demo--simulation)
- [Robot Specifications](#robot-specifications)
- [Hardware Components](#hardware-components)
- [Software & Tech Stack](#software--tech-stack)
- [Project Structure](#project-structure)
- [Kinematics & Gait Theory](#kinematics--gait-theory)
- [Getting Started](#getting-started)
- [Arduino Firmware](#arduino-firmware)
- [Python Simulation](#python-simulation)
- [Gait Phases](#gait-phases)
- [Problems Faced](#problems-faced)
- [Real-World Applications](#real-world-applications)
- [Team](#team)
- [Mentors](#mentors)

---

## Overview

This project develops a **10-DOF biped humanoid robot** with:

- **Inverse kinematics** for precise joint angle computation
- **Cubic spline trajectory generation** for smooth, jerk-free motion
- **Quasi-static gait** — center of mass stays within the support polygon at all times
- **Python simulation** for validation before hardware deployment
- **Arduino Mega** for real-time multi-servo control

The robot has 5 degrees of freedom per leg (hip roll, hip pitch, knee, ankle pitch, ankle roll), controlled by 10 servo motors executing precomputed gait sequences.

---

## Demo & Simulation

| File | Description |
|---|---|
| `robot_simulation.py` | Full 3D animated simulation — all 10 joints shown |
| `robot_poster.py` | Poster-quality gait phase snapshots (print-ready PNG + PDF) |
| `ARDUINO_CODE/main.ino` | Production firmware for Arduino Mega |

**Run the simulation:**
```bash
pip install numpy matplotlib
python robot_simulation.py
```

**Generate the gait poster:**
```bash
python robot_poster.py
# Outputs: robot_poster.png (300 dpi) + robot_poster.pdf (vector)
```

---

## Robot Specifications

| Parameter | Value |
|---|---|
| Degrees of Freedom | 10 (5 per leg) |
| Hip width (`w`) | 9.0 cm |
| Upper hip link (`a`) | 8.3 cm |
| Thigh length (`b`) | 7.7 cm |
| Shin length (`c`) | 6.3 cm |
| Ankle link (`d`) | 4.5 cm |
| Ankle roll offset (`e`) | 1.5 cm |
| Standing height | 28.3 cm |
| Nominal leg length | 25.0 cm |
| Full step length (`X0`) | 15.0 cm |
| Lateral hip shift (`Y0`) | 4.86 cm |
| Foot lift height (`Z0`) | 3.0 cm |
| Power supply | 7.4 V Li-ion, 3000 mAh |
| Microcontroller | Arduino Mega |

---

## Hardware Components

| Component | Specification |
|---|---|
| Microcontroller | Arduino Mega 2560 |
| Servo Motors (×10) | MG-958R high-torque |
| Power Supply | Li-ion battery, 7.4 V, 3000 mAh |
| Regulation | SMPS (Switched Mode Power Supply) |
| PCB | Custom printed circuit board |
| Mechanical | Servo brackets, servo horns, nuts & bolts |
| Wiring | Jumper wires, breadboard |

### Servo-to-Joint Mapping

```
Left Leg  (servos 0–4):   S0=Hip Roll  S1=Hip Pitch  S2=Knee  S3=Ankle Pitch  S4=Ankle Roll
Right Leg (servos 5–9):   S5=Hip Roll  S6=Hip Pitch  S7=Knee  S8=Ankle Pitch  S9=Ankle Roll
```

> **Note:** Servo neutral positions: `[87, 95, 80, 90, 88, 90, 90, 108, 90, 90]` degrees

---

## Software & Tech Stack

| Tool | Purpose |
|---|---|
| **Arduino IDE** | Firmware development & serial debugging |
| **Python 3** | Simulation, kinematics, gait visualization |
| **NumPy** | Matrix operations, kinematic calculations |
| **Matplotlib** | 3D animation, joint trajectory plots, poster output |
| **SolidWorks** | Full mechanical CAD design |

---

## Project Structure

```
Humanoid_Bipedal_Robot-2025-2026-Project/
│
├── ARDUINO_CODE/
│   └── main.ino                  # Production firmware
│
├── PYTHON_FILE/
│   ├── robot_simulation.py       # 3D animated simulation (all 10 joints)
│   └── robot_poster.py           # Gait phase poster generator
│
├── CAD/
│   └── *.SLDPRT / *.SLDASM      # SolidWorks mechanical design files
│
├── docs/
│   └── Project_Report.docx       # Full project report
│
└── README.md
```

---

## Kinematics & Gait Theory

### Inverse Kinematics

Each leg is solved analytically using the 5-DOF chain. Given the hip joint position `Phip` and desired foot position `Pfoot`:

```
x = Pfoot[0] - Phip[0]
y = Pfoot[1] - Phip[1]
z = Pfoot[2] - Phip[2]

q[0] = -atan(y / (z - e))              ← Hip roll
q[4] = q[0]                             ← Ankle roll (mirrors hip roll)

a1 = a·cos(q[0]),  b1 = b·cos(q[0])
c1 = c·cos(q[0]),  d1 = d·cos(q[0])

z1 = z - a1 - d1 - e
r  = sqrt(x² + z1²)

q[3] = atan2(x, z1) + acos((b1² + r² - c1²) / (2·b1·r))   ← Hip pitch
q[2] = π - acos((b1² + c1² - r²) / (2·b1·c1))              ← Knee
q[1] = q[2] - q[3]                                           ← Ankle pitch
```

### Trajectory Generation

All joint trajectories use **cubic spline interpolation** for smooth, jerk-minimized motion:

```
θ(t) = θ_start + (θ_end − θ_start) · (3s² − 2s³),   where s = t/T
```

### Foot Lift Profile

During swing phase, foot height follows a **semi-ellipse** over the step length:

```
z_foot(x) = h − Z0 · sqrt(1 − (2x / X0)²)
```

### Quasi-Static Stability

The center of mass is always kept within the support polygon by shifting the hip laterally (`Y0 = 0.54·w`) before each swing phase — no dynamic balancing required.

---

## Getting Started

### Requirements

```bash
pip install numpy matplotlib
```

Arduino libraries required:
- `Servo.h` (built-in with Arduino IDE)

### Running the Python Simulation

```bash
cd PYTHON_FILE
python robot_simulation.py
```

Controls in the animation window:
- **Play / Pause** — freeze any frame to inspect joint angles
- **Speed slider** — slow down to 0.2× for detailed analysis
- **View selector** — Front / Side / Isometric
- **Stand / Walk** — switch modes

### Uploading Arduino Firmware

1. Open `ARDUINO_CODE/main.ino` in Arduino IDE
2. Select **Board:** Arduino Mega 2560
3. Select correct **Port**
4. Click **Upload**
5. Open Serial Monitor at **9600 baud** to observe trajectory debug output

---

## Arduino Firmware

### Key Functions

| Function | Description |
|---|---|
| `StandAtHeight(h)` | Smoothly stands to target height using cubic spline |
| `StartByLeftLeg()` | Initiates walking — left leg steps first |
| `WalkByRightLeg()` | Full right-leg swing phase |
| `WalkByLeftLeg()` | Full left-leg swing phase |
| `StopByRightLeg()` | Brings right leg back to center, stops |
| `InverseKinematics(Phip, Pfoot, q)` | Computes joint angles analytically |
| `CalculateMotorPosistion(...)` | Converts IK angles to servo degrees |
| `setMotors(angles[])` | Smoothly interpolates to target angles over 200 ms |

### Walk Sequence (loop)

```
StandAtHeight(25)
  └─► StartByLeftLeg()
        └─► WalkByRightLeg() ─► WalkByLeftLeg()  (repeat N times)
              └─► StopByRightLeg()
```

---

## Gait Phases

| Phase | Description | Duration |
|---|---|---|
| **① Neutral Stand** | Both feet flat, symmetric stance | Static |
| **② Hip Shift Left** | Weight transferred to right foot | 0.3·t₀ |
| **③ Right Leg Swing** | Right foot lifts and steps forward | 0.5·t₀ |
| **④ Double Support** | Both feet on ground, weight shifts | 0.2·t₀ |
| **⑤ Hip Shift Right** | Weight transferred to left foot | 0.3·t₀ |
| **⑥ Left Leg Swing** | Left foot lifts and steps forward | 0.5·t₀ |

One full gait cycle (`t₀ = 1.0 s`) completes a step of **15 cm** with a lateral hip shift of **4.86 cm** and foot lift of **3.0 cm**.

---

## Problems Faced

### Servo Torque Limitations
Insufficient torque caused incomplete leg lifts and motor burnout under sustained load. Resolved by selecting high-torque MG-958R servos and distributing load across the mechanical structure.

### Power Supply Instability
Simultaneous actuation of 10 servos caused voltage drops, resetting the Arduino mid-motion. Resolved by adding a dedicated SMPS and decoupling capacitors on the power rail.

### Inverse Kinematics Tuning
Small errors in link length measurements (`a`, `b`, `c`, `d`, `e`) caused jerky or impossible joint configurations. Resolved through iterative physical measurement and simulation validation.

### Gait Coordination
Synchronizing hip shift, swing, and push-off timing required careful tuning of the cubic spline breakpoints (`t1`, `t2`) to avoid tipping.

### Wire Management
Loose jumper wires during motion caused signal drops and short circuits. Resolved by routing wires along the frame and using cable ties.

---

## Real-World Applications

- **Disaster Management** — Navigate rubble, climb stairs, enter hazardous zones
- **Healthcare & Rehabilitation** — Assist elderly/disabled individuals, physiotherapy support
- **Industrial Automation** — Material handling in human-designed environments
- **Service & Hospitality** — Receptionists, guides, and interactive assistants
- **Defense** — Surveillance and equipment carrying in difficult terrain
- **Research & Education** — Teaching kinematics, control systems, and embedded programming

---

## Team

| Name | Branch | Reg. No. | Year |
|---|---|---|---|
| Abhigyan Shukla | Mechanical Engineering | 20246002 | 2nd Year |
| Aryan Kumar | Mechanical Engineering | 20246037 | 2nd Year |
| Rishabh Rawat | Electronics & Communication | 2024XXXX | 2nd Year |
| Shiva Jadaun | Electrical Engineering | 20246062 | 2nd Year |
| Surya Bhan Singh | Mechanical Engineering | 20246147 | 2nd Year |

## Mentors

| Name | Branch | Reg. No. | Year |
|---|---|---|---|
| Aaditya Gupta | Mechanical Engineering | 20236162 | 3rd Year |
| Alok Gupta | Electronics & Communication | 20236019 | 3rd Year |
| Krishna Raj Agarwal | Mechanical Engineering | 20236069 | 3rd Year |
| Pragyan Ansh | Mechanical Engineering | 20234119 | 3rd Year |

---

> **Robotics Club — MNNIT Allahabad**
> *Team Humanoid · 2025–2026*
