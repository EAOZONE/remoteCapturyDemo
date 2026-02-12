# remoteCapturyDemo

Demo application showing how a **remote Captury** motion-capture pipeline (Captury 8 tracking with a multi-camera setup, e.g., Blackfly cameras) can be streamed and mapped to **real-time robotic control outputs**.

This repository was used as demonstration code to show how a human performer captured in motion capture can drive:

- **Nadia** humanoid robot motion and behaviors  
- **Psyonic Ability Hands** commands  
- **Franka** robotic arm motion (Franka Emika)  

---

## Demo Videos

A full demonstration of this system — including live motion capture driving:

- Nadia humanoid robot  
- Psyonic Ability Hands  
- Franka robotic arm  

is available on my portfolio:

🔗 https://cbenpratt.wixsite.com/charles-pratt-portfo/projects  

The *Motion Capture System* section includes videos showing:

- Motion capture simulation retargeting  
- Control of Psyonic Ability Hands  
- Real-time control of IHMC's humanoid robot Nadia  
- Performer arm retargeting to Franka robotic arms  

All videos demonstrate live streaming from Captury into robotic control pipelines.

---

> **Note:** This repository is organized as a Gradle Java project, with sources under `src/main/java/us/ihmc/...` and standard Gradle wrapper scripts included.

---

## What This Project Does

At a high level, the runtime pipeline is:

1. **Captury (remote)** streams body tracking data (skeleton pose / joint transforms).  
2. The demo code ingests that stream and performs:
   - Coordinate frame handling (units, axes, handedness)  
   - Optional filtering and smoothing  
   - Retargeting / mapping from Captury skeleton joints to target device joints  
3. The mapped signals are sent to one of multiple **output adapters**:
   - Nadia humanoid control interface  
   - Psyonic hand command interface  
   - Franka arm command interface  

This repository is intentionally structured as a *demo harness*: small, readable modules that demonstrate the end-to-end pipeline from mocap → retargeting → robot/device control.

---

## Repository Layout

Typical project structure:

- `src/main/java/us/ihmc/...` — main Java source tree  
- `build.gradle.kts` / `settings.gradle.kts` — Gradle build configuration  
- `gradlew`, `gradlew.bat`, `gradle/wrapper` — Gradle wrapper tooling  

---

## Requirements

### Software

- **Java JDK 17+** (recommended)  
- Gradle wrapper included (no separate Gradle installation required)  
- Captury software stack accessible on the network  
- Target device SDKs / middleware for:
  - Nadia control stack  
  - Psyonic Ability Hand interface  
  - Franka control stack  

> Some dependencies may be proprietary or lab-specific. Without hardware access, the ingestion and mapping components can still be run with stubbed outputs.

### Hardware (Example Demo Setup)

- Captury multi-camera system (e.g., Blackfly cameras), calibrated in Captury  
- Network connectivity between:
  - Captury tracking server  
  - Demo machine  
  - Target robots/devices  

---

## Quick start

### 1) Clone
```bash
git clone https://github.com/EAOZONE/remoteCapturyDemo.git
cd remoteCapturyDemo
```
### 2) Build
```bash
Mac/Linux:
  ./gradlew build
Windows:
  gradlew.bat build
```

---
## Environment Note
This project was developed for demonstration within an IHMC research environment and may rely on IHMC-specific middleware or tooling. Running it outside that environment may require addition configuration or addapter implementaion.
