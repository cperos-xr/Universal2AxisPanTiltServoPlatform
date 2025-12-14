# Universal 2-Axis Pan–Tilt Servo Platform

This repository contains a **complete, modular 2-axis pan–tilt servo platform**, including mechanical designs, calibration utilities, and multiple control scripts.

The project is intentionally designed to support **both human control and automation**, and to scale cleanly into larger robotics or AI systems rather than being tied to a single use case.

---

## Repository Contents (Overview)

This repository includes **four major components**:

1. **3D Models** – Printable pan–tilt mount and hardware
2. **Calibration Script** – Used to determine safe servo limits before final assembly
3. **WASD Controller** – Human-friendly interactive control for testing and tuning
4. **PanTilt_JSON Controller** – Automation- and bot-friendly JSON-based controller

Each component serves a different role and can be used independently or together.

---

## Suitable Applications

This platform is suitable for:

- Robotic vision / AI computer vision experiments
- Face or object tracking systems
- Security or pet cameras
- Solar tracking panels
- Nerf turrets or automated launchers
- General pan–tilt robotics projects
- Any application requiring precise, scriptable 2-axis motion

The design supports **wide servo travel ranges** when properly calibrated.

---

## Hardware Overview

### Supported Hardware (Reference)

- Standard hobby servos (MG996R / MG995R class tested)
- ESP32 microcontroller (ESP32-C3 tested; others likely compatible)
- Camera, solar panel, or custom payload
- Standard 3-wire servo connections

---

## Servo Requirements & Setup Notes (Important)

⚠️ **Servo calibration is required before final assembly.** ⚠️  
Do not skip this step.

### Servo 1 — Pan (Side-to-Side)

- Intended for the **widest safe rotation range**
- Example tested range (MG996R-class):
  - **~500–2400 µs**

### Servo 2 — Tilt (Up / Down)

- Requires a **slightly reduced range** to avoid mechanical interference
- Example working range:
  - **~800–2050 µs**

⚠️ **Do not permanently mount the payload or platform to the servo horn until limits are verified.** ⚠️

### Calibration Recommendation

Before final assembly:
- Use the provided calibration script or equivalent test code
- Step each servo in **~50 µs increments**
- Identify mechanical bind or stress points
- Configure those values as hard limits in firmware

This ensures maximum usable travel without damaging hardware.

---

## Servo Horn Notes

- Designed around a **20 mm circular servo horn**
- MG996R / MG995R stock horns may vary slightly
- Plastic horns often work, but **metal or ceramic horns are strongly recommended** for:
  - rigidity
  - repeatability
  - durability under load

---

## 3D Models

The mechanical pan–tilt platform is designed to be **rigid and durable**, suitable for continuous motion.

### Recommended Print Settings (Tested)

- **Infill:** 25% gyroid
- **Walls:** 5–7 perimeters
- **Layer height:** 0.2 mm
- **Material:** PLA+ or PETG  
  - PETG recommended for outdoor or solar-tracking applications

Lower infill and wall counts may work for lighter-duty use, but testing showed minimal material savings compared to the increase in stiffness and reliability with these settings.

---

## Calibration Script

The calibration script exists to determine **safe microsecond limits** for each servo *before final assembly*.

Recommended workflow:
1. Upload calibration script
2. Sweep servos in **~50 µs increments**
3. Identify mechanical limits
4. Record safe min/max values
5. Use those values in control firmware

This step is critical to avoid binding, excessive torque, or servo damage.

---

## WASD Controller (Human-Friendly Control)

The WASD controller provides **real-time keyboard-based control**, intended primarily for:

- Initial testing
- Direction verification
- Mechanical tuning
- Manual experimentation

Typical controls:
- `W / S` → one axis
- `A / D` → the other axis
- Step size can be adjusted at runtime

> Note: Arduino Serial Monitor is not ideal for raw keypress input. A proper terminal application is recommended for best results.

---

## PanTilt_JSON Controller (Automation-Friendly)

`PanTilt_JSON` is a **JSON-based serial controller** intended for bots, scripts, and higher-level control systems.

It is designed to act as a **subsystem** in larger robotics or automation projects.

### Protocol Overview

The controller uses **JSONL (newline-delimited JSON)** over USB serial:

- One JSON object per line
- Each line must end with newline (`\n`)
- Each command must include `"cmd"`

Example:
```json
{"cmd":"help"}
