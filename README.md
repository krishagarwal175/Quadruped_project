# Quadruped Robot Project  
## Consolidated Technical & Progress Report

**Author:** Krish Agarwal  
**Project Duration:** June 2025 – January 2026  
**Status:** Active Development  
**Repository Purpose:** Complete technical, mechanical, electrical, and control-system documentation of the quadruped robot project.

---

## 1. Project Overview

This project documents the end-to-end design, development, and iterative refinement of a servo-actuated quadruped robot. Unlike gait-first hobby projects, this work follows a **stability-first, reflex-based control philosophy**, where standing balance, load management, and failure resilience are solved before aggressive locomotion.

The robot serves as:
- A **research testbed** for embedded control and legged stability
- A **hands-on systems engineering project** covering mechanics, electronics, and software
- A **foundation platform** for future ROS 2, perception, and learning-based extensions

---

## 2. Development Timeline Summary

### Phase 0–1: Concept & Pivot (Month 1)
- Initial work on a humanoid robot exposed critical power-to-weight limitations.
- A decisive pivot was made to a quadruped architecture.
- Learned Fusion 360, ROS, and Gazebo.
- Completed torque calculations and finalized a viable BOM.
- Began simulation-driven gait exploration.

**Key Outcome:** A mechanically and electrically feasible quadruped design.

---

### Phase 2: Mechanical Prototyping & Assembly (Month 2)
- All chassis and leg components 3D printed using PETG.
- Full mechanical assembly completed.
- Multiple redesigns to fix servo fitment, joint weakness, and infill-related failures.
- Hardware prototype became physically complete and software-ready.

**Key Outcome:** Fully assembled physical robot.

---

### Phase 3: Electrical & Platform Stabilization (Month 3)
- Introduced regulated high-current power delivery using a buck converter.
- Validated servo power distribution under load.
- Jetson Orin Nano OS corruption occurred during ROS setup → full system re-flash.
- Designed V2 leg parts after a structural failure.
- Established high-level control architecture plan.

**Key Outcome:** Stable electromechanical platform with lessons learned.

---

### Phase 4: Control Integration & Motion Refinement (Month 4)
- Full chassis rebuild to eliminate flex and rocking.
- Implemented smoother gait trajectories and reduced snap.
- Increased gait speed by ~1.3× after validation.
- Added balancing routines and removed brittle abort logic.
- Implemented configuration export and logging.
- Identified servo feedback and RL training as current hard limits.

**Key Outcome:** Stable, controllable robot with repeatable motion.

---

### Phase 5: Stability-First Control Architecture (Recent Month – Not in Prior Reports)
- Implemented **layered control architecture**:
  - Hardware abstraction
  - PD-based roll/pitch stabilization
  - Reflex behaviors and FSM-based limb control
- Developed brace reflexes and leg unloading detection.
- Enabled safe single-leg lift without collapse.
- Built real-time telemetry and manual override GUIs.
- Achieved controlled crawl-like motion under FSM supervision.

**Key Outcome:** Research-grade reflex-stabilized quadruped.

---

## 3. Mechanical System

- 3 DOF per leg (shoulder, mid-limb, foot)
- PETG 3D-printed chassis with iterative reinforcement
- Structural redesigns driven by real load failures
- Explicit mechanical limits encoded in software

**Lesson:** Mechanical stiffness directly governs control quality.

---

## 4. Electrical & Power Architecture

- 4S 6200 mAh LiPo battery
- Dedicated high-current buck converter for servos
- PCA9685 PWM driver for deterministic pulse generation
- Careful grounding and wiring to avoid I2C instability
- MPU6050 IMU for orientation feedback

**Lesson:** Power isolation is mandatory for legged robots.

---

## 5. Embedded Compute & Software Stack

- NVIDIA Jetson Orin Nano (Ubuntu + JetPack)
- Python-based control stack
- Clean separation of sensing, actuation, and control
- FSM-based behavior sequencing
- Extensive logging and visualization

A Jetson OS corruption event forced a full rebuild, resulting in a **much cleaner and more robust architecture**.

---

## 6. Control Philosophy

### Stability-First Design
- Standing solved before walking
- Reflexes before planning
- Hardware constraints drive software decisions

### Layered Control
- **Layer 0:** Servo mapping, limits, safe init
- **Layer 1:** PD roll/pitch stabilization
- **Layer 2:** Reflexes, unloading checks, FSM behaviors

### Reflex Behaviors
- Brace reflex under disturbances
- Conservative leg unloading checks
- Safe limb lift and replant phases

---

## 7. Current Capabilities

- Stable standing under external pushes
- Reflexive disturbance rejection
- Safe single-leg lift
- Controlled crawl-like motion
- Live telemetry and manual override support

---

## 8. Constraints & Known Limitations

- No joint feedback → positional drift under load
- RL gait training blocked by lack of GPU compute
- Servo thermal and current limits
- Rear-right leg slip at higher speeds
- Manual tuning slows iteration

---

## 9. Engineering Lessons

- Mechanical reality overrides theory
- Failures accelerate architectural maturity
- Visualization is not optional
- Stability-first control simplifies everything downstream

---

## 10. Future Work

- Closed-loop actuation (encoders / smart servos)
- Foot contact sensing
- ROS 2 migration
- Quantitative performance metrics
- Learning-assisted gait optimization (long-term)

---

## 11. Conclusion

This project demonstrates that **meaningful quadruped intelligence can emerge from disciplined control design without complex planners or learning**, provided stability and reflexes are prioritized early.

The robot is now a robust platform for advanced research, experimentation, and future expansion.

---

*End of Consolidated Report*
