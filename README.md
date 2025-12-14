# Universal 2-Axis Pan–Tilt Servo Platform

This project implements a **universal 2-axis pan–tilt servo platform** designed for experimentation, automation, and robotics projects.

It is intentionally **modular, scriptable, and protocol-driven**, rather than tied to a single application or workflow. The same firmware can be driven interactively by a human or orchestrated by higher-level automation, AI, or robotics control software.

---

## Suitable Applications

This platform is well-suited for:

- Robotic vision / AI computer vision experiments
- Face tracking or object tracking
- Security or pet cameras
- Solar tracking panels
- Nerf turrets or automated launchers
- General pan–tilt robotics projects
- Any project requiring precise, scriptable 2-axis motion

The mechanical design and firmware support **wide servo travel ranges** when properly calibrated.

---

## System Overview

The project consists of:

- A **robust 3D-printed pan–tilt mount**
- Two standard hobby servos (pan + tilt)
- An **ESP32-based controller** (ESP32-C3 tested)
- A **newline-delimited JSON (JSONL) command protocol** over USB serial
- Optional motion queueing, sweeps, and automation primitives

The firmware is designed so it can later act as a **subsystem** within a larger robot or automation stack.

---

## Servo Requirements & Setup Notes (Important)

⚠️ **Servo calibration is required before final assembly.** ⚠️  
Do not skip this step.

### Servo 1 — Pan (Side-to-Side)

- Intended for the **widest safe rotation range**
- Example tested range (MG996R-class servo):
  - **~500–2400 µs**

### Servo 2 — Tilt (Up / Down)

- Requires a **slightly reduced range** to avoid mechanical interference
- Example working range:
  - **~800–2050 µs**

⚠️ **Do not permanently mount the payload or platform to the servo horn until these limits are confirmed.** ⚠️

### Calibration Recommendation

Before final mounting:
- Use simple test code (Arduino, ESP32, RP Pico, etc.)
- Step each servo in **~50 µs increments**
- Identify the mechanical limits where binding or stress begins
- Configure those values as hard limits in firmware

This prevents damage and ensures maximum usable travel.

---

## Servo Horn Notes

- Designed around a **20 mm circular servo horn**
- MG996R / MG995R stock horns may vary slightly
- Plastic horns often work, but **metal or ceramic horns are strongly recommended** for:
  - rigidity
  - repeatability
  - long-term durability

---

## Hardware Used (Reference)

- MG996R / MG995R-class hobby servos
- ESP32 (ESP32-C3 tested, others likely compatible)
- Camera, solar panel, or custom payload
- Standard 3-wire servo connections

---

## Firmware Overview

The firmware exposes a **JSONL (newline-delimited JSON)** protocol over USB serial.

Each command is:
- A single JSON object
- Sent on one line
- Terminated with a newline (`\n`)

Example:
```json
{"cmd":"help"}
