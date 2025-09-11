# ASSIGNMENT_ARM3DOF_KEYPOINTS.md

**Title:** 3-DOF Robot Arm — Joint Position Control (no BLDC)  
**Scope:** Minimal arm model + position control to named poses in Gazebo Harmonic using ROS 2 Jazzy.  
**Style:** You figure out the details. Below are **key points** only.

---

## 1) Objectives (what you must achieve)
- Spawn a **3-DOF arm** (revolute joints: base yaw, shoulder, elbow) in Gazebo.
- Command **joint positions** from ROS 2 to move between at least **three named poses** (A→B→C).
- Keep motion stable (no oscillations, no joint explosions), with limits respected.

---

## 2) Modeling (SDF or URDF — your choice)
- **Links:** rectangular links with **sane inertias** (no zero/NaN; magnitudes consistent with mass/size).
- **Joints:**  
  - `j1_base_yaw`: axis **Z**  
  - `j2_shoulder`, `j3_elbow`: axis **Y** (planar elbow)  
  - **Limits:** choose reasonable ranges (e.g., ±π for yaw; ±~90° for shoulder/elbow).
- **Damping:** add small joint damping to avoid chatter.
- **Controller system:** attach a **Joint Position Controller** per joint (Harmonic).  
  - Expose **per-joint topics** for commands (pick a clean namespace like `/arm/jX/cmd_pos`).  
  - Decide whether to use **velocity slewing** vs. PID (document your choice).

> Deliverable: `models/` (or `urdf/`) with your arm; short note explaining your axis/limit/inertia choices.

---

## 3) ROS ↔ Gazebo interfacing
- **Bridge the joint command topics**: ROS `std_msgs/Float64` ↔ Gazebo `gz.msgs.Double`.
- Keep topic names consistent; avoid hidden defaults.
- Show the **topic graph** you end up with (diagram or `rqt_graph` screenshot).

> Deliverable: brief config or command list + one diagram/screenshot.

---

## 4) Control node (you implement)
- Single ROS 2 node (Python or C++) that publishes **target angles** for all 3 joints.  
- **Poses:** define at least **A, B, C** (radians).  
- **Timing:** publish at fixed rate with brief holds; avoid instant discontinuities.
- Optional: tiny **ramp** to setpoints if you use strict position PID (reduces jerk).

> Deliverable: source file + minimal CLI (e.g., parameter `posture:=A`).

---

## 5) Runbook (you write it)
- From fresh clone to “arm is moving” in **clear, exact steps**.  
- Include **workspace layout**, build steps, and how you launch sim + bridge + control.

> Deliverable: `README.md` (short, exact, copy-paste-able).

---

## 6) Acceptance criteria (measured, not vibes)
- **Functionality:** A→B→C sequence completes without errors.
- **Stability:** each move settles in **≤ 1.0 s** with **≤ 5°** steady-state error per joint.
- **Safety:** joint limits never exceeded; no NaNs; clean shutdown (no spammy warnings).
- **Reproducibility:** your README steps work on a fresh machine.

---

## 7) Constraints (to ensure learning)
- **No BLDC plugin.** Use joint position control only.  
- Don’t paste ready-made models; **author the arm yourself** (credit any inspiration).  
- PID gains (if you use them) must be **justified** (1–2 sentences).

---

## 8) What to hand in
- Repo with: model, launch/world, control node, README.
- **One 15–30s GIF/video** showing A→B→C.
- **One diagram** of topics/pubs/subs.  
- **One paragraph**: design notes (axes, limits, controller choice, key tuning).

---

## 9) Hints (minimal)
- If it chatters: add damping, reduce P, or switch to velocity-slew mode.
- If commands don’t reach Gazebo: **type mismatch or wrong topic name** is the usual culprit.
- Start with small angles; scale up after stability is confirmed.

---

## 10) Stretch goals (optional, pick one)
- Add a simple **planar IK** for (x, y) to `j2/j3`; keep `j1` as yaw.
- Add a tiny GUI or RViz sliders to command joints.
- Log angles and compute **overshoot/settling** automatically.

---

## 11) Review rubric (how you’ll be graded)
- **Clarity (25%)**: README/runbook precise; repo tidy.  
- **Control quality (35%)**: smooth motion, meets acceptance numerics.  
- **Model quality (25%)**: realistic inertias/limits; stable sim.  
- **Engineering hygiene (15%)**: consistent naming, comments, minimal warnings.

---

**Deadline:** 1–3 days. Keep it small, stable, and well-documented.
