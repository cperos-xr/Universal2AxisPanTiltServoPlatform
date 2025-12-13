## Universal 2-Axis Pan–Tilt Servo Platform

This is a **universal 2-axis pan–tilt servo platform** designed for experimentation, automation, and robotics projects. It is intentionally modular and adaptable rather than tied to a single use case.

### Suitable Applications

- Robotic vision / AI computer vision experiments
- Face tracking / object tracking
- Security or pet cameras
- Solar tracking panels
- Nerf turrets or automated launchers
- General pan–tilt robotics projects

The platform is designed around standard hobby servos and supports **wide travel ranges when properly calibrated**.

---

## Servo Requirements & Setup Notes (Important)

This design assumes **servo calibration before final assembly**.

### Servo 1 — Pan (Side-to-Side)

- Should be configured for an **open-ended rotation range** (as wide as your servo safely allows)
- Example tested range (MG996R): **~500–2400 µs**

### Servo 2 — Tilt (Up / Down)

- Requires a **slightly reduced range** to avoid mechanical interference
- Example working range: **~800–2050 µs**

⚠️ **Do not permanently screw the panel or payload to the servo horn until these limits are dialed in.** ⚠️

I recommend using test code (Arduino, ESP32, RP Pico, or similar) to step each servo in **~50 µs increments** to determine safe mechanical limits before final mounting.

---

## Servo Horn Notes

- Designed around a **20 mm circular ceramic or metal servo horn**
- MG996R / MG995R stock horns may vary slightly
- Stock plastic horns often work, but **metal or ceramic horns are recommended** for rigidity and durability

---

## Hardware Used (Reference)

- MG996R / MG995R-class servos
- ESP32 (for calibration and control) or similar microcontroller
- Small solar panel, camera, or custom payload
- Standard servo wiring

---

## Print Settings & Structural Notes

This design was printed with **25% gyroid infill** and **5–7 perimeters (walls)** to prioritize stiffness and durability. In this configuration, the mount is very rigid and tolerates rough handling, vibration, and repeated servo motion without flexing.

Lower infill and fewer walls will likely still work for lighter-duty applications, but in testing the **time and material savings were negligible**, so the design was intentionally printed on the conservative side for reliability.

### Recommended Starting Point

- **Infill:** 25% gyroid
- **Walls:** 5–7 perimeters
- **Layer height:** 0.2 mm (or your usual structural default)
- **Material:** PLA+ or PETG  
  - PETG recommended for outdoor or solar-tracking use

If you are weight- or material-constrained, you can experiment with reducing infill or wall count. For general-purpose builds, the above settings provide a strong balance of durability and practicality.
