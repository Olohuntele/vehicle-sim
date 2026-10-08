# Easy Drive: Simulation Demo & Capture Guide

To give fellow engineers an instant visual demonstration of the C++ Raylib physics simulation (`sim.exe`), you can capture a short GIF or MP4 video highlighting the autonomous driving features, AEB telemetry HUD, and interactive mouse controls.

---

## 🎮 What to Showcase in the Simulation Demo

When recording or demonstrating `sim.exe`, highlight the following real-time telemetry features:

1. **Autonomous Emergency Braking (AEB) & TTC:** 
   - Observe how the simulation tracks moving target vehicles and computes real-time Time-to-Collision (TTC).
   - Show the dynamic braking telemetry HUD lighting up as collision thresholds are approached.
2. **Adaptive Cruise Control (ACC):**
   - Demonstrate smooth speed matching, headway preservation, and dynamic deceleration behind leading vehicles.
3. **Interactive Mouse Controls:**
   - Click and drag or position target obstacles dynamically in real-time to test the physics engine's response and collision avoidance algorithms.

---

## 📹 Recommended Tools for Capturing GIFs & Videos

### 1. For Windows (Free & Lightweight)
- **ScreenToGif (Recommended for GIFs):**
  1. Download [ScreenToGif](https://www.screentogif.com/).
  2. Launch `sim.exe` in Windowed mode (`./build/Release/sim.exe`).
  3. Open ScreenToGif, position the capture frame over the Raylib simulation window.
  4. Hit **Record**, interact with the simulation (triggering AEB/ACC), and hit **Stop**.
  5. Export as an optimized `.gif` (under 10MB) for embedding in your GitHub `README.md`.
- **OBS Studio (Recommended for HD Videos / YouTube):**
  1. Capture Window Capture (`sim.exe`).
  2. Record at 60 FPS to capture smooth physics rendering and telemetry overlays.

---

## 🌐 Embedding the Demo in README.md

Once you record your GIF (e.g. `simulation_demo.gif`), place it in an `assets/` directory and embed it in your `README.md`:

```markdown
## 🎥 Simulation in Action

![Easy Drive Simulation Demo](assets/simulation_demo.gif)
```
