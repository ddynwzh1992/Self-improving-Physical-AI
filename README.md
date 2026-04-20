# Self-improving Physical AI — Sim2Real Manufacturing Robot System

A **self-improving robotics platform** that bridges simulation and reality. An AI agent controls both a simulated environment (NVIDIA Isaac Sim) and a real robot system, using **agent memory** to continuously learn from failures, adapt strategies, and transfer knowledge between sim and real domains.

Built on [OpenClaw](https://github.com/openclaw/openclaw) AI agent connected to Telegram for natural language control.

## Architecture

```mermaid
graph TB
    subgraph User["🧑‍💻 User Layer"]
        TG["📱 Telegram<br/>Natural Language"]
        MAC["💻 Mac / PC<br/>WebRTC Viewer"]
        DASH["📊 Dashboard<br/>Metrics & Logs"]
    end

    subgraph Agent["🤖 AI Agent Layer"]
        OC["OpenClaw Gateway<br/><i>LLM-powered Orchestrator</i>"]
        SKILL_SIM["Isaac Sim Skill<br/><i>Simulation Control</i>"]
        SKILL_REAL["Real Robot Skill<br/><i>Hardware Control</i>"]
        SKILL_S2R["Sim2Real Skill<br/><i>Transfer & Validation</i>"]
        OC --- SKILL_SIM
        OC --- SKILL_REAL
        OC --- SKILL_S2R
    end

    subgraph Memory["🧠 Agent Memory"]
        direction TB
        EP["📝 Episodic Memory<br/><i>Task execution logs<br/>Success/failure records</i>"]
        SEM["🌐 Semantic Memory<br/><i>Learned strategies<br/>Object properties · Grasp params</i>"]
        PROC["⚡ Procedural Memory<br/><i>Optimized trajectories<br/>Calibration offsets</i>"]
        DELTA["📊 Sim2Real Delta Log<br/><i>Discrepancy tracking<br/>Domain gap metrics</i>"]
    end

    subgraph Sim["🏭 Simulation (Digital Twin)"]
        ISAAC["Isaac Sim 6.0<br/><i>Docker · Headless</i>"]
        subgraph sim_scripts [" "]
            MS["🏗️ Scene Builder"]
            PP["🤖 Pick & Place"]
            RC["🎮 Robot Control"]
            SO["📦 Object Spawner"]
            CV["📸 Viewport Capture"]
        end
        ISAAC --- sim_scripts
    end

    subgraph Real["🦾 Real Robot System"]
        ROS["ROS 2<br/><i>Robot Middleware</i>"]
        DRIVER["Robot Driver<br/><i>Franka · UR · Custom</i>"]
        SENSOR["Sensors<br/><i>RGB-D Camera · F/T Sensor</i>"]
        GRIP["Gripper<br/><i>Parallel · Suction</i>"]
        ROS --- DRIVER
        ROS --- SENSOR
        ROS --- GRIP
    end

    subgraph Infra["⚙️ Infrastructure"]
        GPU["NVIDIA GPU<br/><i>L40S · 48GB VRAM</i>"]
        WEBRTC["WebRTC Stream<br/><i>49100/TCP · 47998/UDP</i>"]
        USD["USD Scene Files<br/><i>OpenUSD Format</i>"]
    end

    TG <-->|"Commands ·<br/>Images · Status"| OC
    OC -->|"Sim Commands"| ISAAC
    OC -->|"Real Commands"| ROS
    OC <-->|"Read · Write<br/>Learn · Recall"| Memory
    ISAAC -->|"Physics ·<br/>RTX Rendering"| GPU
    ISAAC <-->|"Save · Load"| USD
    ISAAC -->|"Live 3D"| WEBRTC
    WEBRTC <-->|"Real-time View"| MAC

    ISAAC <-.->|"🔄 Sim2Real<br/>Transfer"| ROS
    EP <-.->|"Compare<br/>Outcomes"| DELTA
    SENSOR -.->|"Real-world<br/>Observations"| OC
    DASH <-.- Memory

    style User fill:#E8F5E9,stroke:#4CAF50,stroke-width:2px,color:#1B5E20
    style Agent fill:#E3F2FD,stroke:#2196F3,stroke-width:2px,color:#0D47A1
    style Memory fill:#FFF9C4,stroke:#F9A825,stroke-width:2px,color:#F57F17
    style Sim fill:#FFF3E0,stroke:#FF9800,stroke-width:2px,color:#E65100
    style Real fill:#FCE4EC,stroke:#E91E63,stroke-width:2px,color:#880E4F
    style Infra fill:#F3E5F5,stroke:#9C27B0,stroke-width:2px,color:#4A148C
    style sim_scripts fill:#FFF8E1,stroke:#FFC107,stroke-width:1px
```

### Self-Improving Loop

```mermaid
graph LR
    subgraph loop ["🔄 Continuous Improvement Cycle"]
        direction LR
        A["1️⃣ Plan<br/><i>Agent plans task<br/>from memory</i>"] --> B["2️⃣ Simulate<br/><i>Execute in<br/>Isaac Sim</i>"]
        B --> C["3️⃣ Validate<br/><i>Check physics<br/>& feasibility</i>"]
        C --> D["4️⃣ Execute<br/><i>Run on real<br/>robot</i>"]
        D --> E["5️⃣ Observe<br/><i>Capture sensor<br/>data & outcome</i>"]
        E --> F["6️⃣ Learn<br/><i>Compare sim vs real<br/>Update memory</i>"]
        F --> A
    end

    style loop fill:#F3E5F5,stroke:#9C27B0,stroke-width:2px
    style A fill:#E3F2FD,stroke:#2196F3,color:#0D47A1
    style B fill:#FFF3E0,stroke:#FF9800,color:#E65100
    style C fill:#FFF9C4,stroke:#F9A825,color:#F57F17
    style D fill:#FCE4EC,stroke:#E91E63,color:#880E4F
    style E fill:#E8F5E9,stroke:#4CAF50,color:#1B5E20
    style F fill:#EDE7F6,stroke:#673AB7,color:#311B92
```

### Sim2Real Data Flow

```mermaid
sequenceDiagram
    participant U as 📱 User
    participant A as 🤖 Agent
    participant M as 🧠 Memory
    participant S as 🏭 Isaac Sim
    participant R as 🦾 Real Robot

    U->>A: "Pick up the gear from the conveyor"

    Note over A,M: 1. RECALL — Check memory for similar tasks
    A->>M: Query: "pick gear conveyor"
    M-->>A: Past attempts: grasp_offset=-0.02m,<br/>success_rate=78%, best_approach=top-down

    Note over A,S: 2. SIMULATE — Plan & validate in sim
    A->>S: Execute pick_and_place.py<br/>with learned parameters
    S-->>A: ✅ Sim success · trajectory + timing data

    Note over A,R: 3. EXECUTE — Run on real robot
    A->>R: Send trajectory via ROS 2
    R-->>A: Execution result + sensor data<br/>⚠️ Slip detected at z=0.31m

    Note over A,M: 4. LEARN — Update memory with real outcome
    A->>M: Log: sim_success=true, real_success=partial<br/>delta: grip_force +0.5N needed at z<0.35m
    A->>M: Update grasp strategy:<br/>increase_force_near_surface=true

    Note over A,S: 5. IMPROVE — Update simulation model
    A->>S: Adjust friction params:<br/>gear_surface_friction=0.4→0.35
    S-->>A: Sim model updated

    A-->>U: "Picked up the gear ✅<br/>Had to adjust grip force.<br/>Updated my model for next time."

    Note over A,M: Memory now contains improved<br/>strategy for future tasks
```

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [OpenClaw Setup & Telegram Connection](#1-openclaw-setup--telegram-connection)
3. [Isaac Sim Installation (Docker)](#2-isaac-sim-installation-docker)
4. [Deploy Demo Scripts](#3-deploy-demo-scripts)
5. [Running Simulations](#4-running-simulations)
6. [Live Viewing via WebRTC Streaming](#5-live-viewing-via-webrtc-streaming)
7. [Connecting from Mac](#6-connecting-from-mac)
8. [OpenClaw Skill Integration](#7-openclaw-skill-integration)
9. [Sim2Real Architecture & Agent Memory](#8-sim2real-architecture--agent-memory)
10. [Isaac Sim 6.0.0 API Reference](#9-isaac-sim-600-api-reference)
11. [Troubleshooting](#troubleshooting)

---

## Prerequisites

- Ubuntu 22.04/24.04 server (AWS EC2 recommended)
- NVIDIA GPU with RT Cores (L40S, RTX 4080+ or equivalent), 16GB+ VRAM
- NVIDIA Driver 580+ with CUDA 12+
- Docker with NVIDIA Container Toolkit
- Node.js 22+ (for OpenClaw)
- ~50GB free disk space

---

## 1. OpenClaw Setup & Telegram Connection

### Install OpenClaw

```bash
# Install Node.js 22 via nvm
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash
source ~/.bashrc
nvm install 22

# Install OpenClaw
npm install -g openclaw

# Start the gateway
openclaw gateway start
```

### Connect Telegram

1. Open Telegram and message [@BotFather](https://t.me/BotFather)
2. Send `/newbot`, choose a name and username (must end in `bot`)
3. Copy the bot token
4. Configure OpenClaw:

```bash
openclaw config patch '{
  "channels": {
    "telegram": {
      "enabled": true,
      "botToken": "YOUR_BOT_TOKEN_HERE"
    }
  }
}'
```

5. Restart the gateway: `openclaw gateway restart`
6. Message your bot on Telegram — it will respond!

### Set Allowed Users

Find your Telegram user ID (message [@userinfobot](https://t.me/userinfobot)), then:

```bash
openclaw config patch '{
  "channels": {
    "telegram": {
      "dmPolicy": "allowlist",
      "allowFrom": ["YOUR_TELEGRAM_USER_ID"]
    }
  }
}'
```

---

## 2. Isaac Sim Installation (Docker)

### Install Docker & NVIDIA Container Toolkit

```bash
# Docker
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
newgrp docker

# NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### Pull Isaac Sim Container

```bash
docker pull nvcr.io/nvidia/isaac-sim:6.0.0-dev2
```

This is ~28GB and may take 10-30 minutes.

### Create Cache Directories

```bash
mkdir -p ~/docker/isaac-sim/{cache/main,cache/computecache,config,data,logs,pkg}
```

### Verify GPU Access

```bash
docker run --rm --gpus all nvcr.io/nvidia/isaac-sim:6.0.0-dev2 nvidia-smi
```

You should see your GPU listed. If not, check the [Troubleshooting](#troubleshooting) section.

---

## 3. Deploy Demo Scripts

```bash
# Clone this repository
git clone https://github.com/ddynwzh1992/Self-improving-Physical-AI.git
cd Self-improving-Physical-AI

# Make run_sim.sh executable
chmod +x scripts/run_sim.sh

# Create output directory
mkdir -p output
```

### Script Overview

| Script | Purpose |
|--------|---------|
| `run_sim.sh` | Docker wrapper — runs any Python script inside Isaac Sim container |
| `manufacturing_scene.py` | Builds a full factory environment (conveyor, robot, shelves, lights) |
| `pick_and_place.py` | Robot picks objects from conveyor and places them at target positions |
| `robot_control.py` | General robot arm control (move joints, gripper, IK targets) |
| `spawn_objects.py` | Spawns boxes, spheres, or cylinders into the scene |
| `capture_viewport.py` | Captures a rendered screenshot from a configurable camera angle |
| `load_scene_streaming.py` | Loads a saved USD scene into a running streaming instance |

---

## 4. Running Simulations

### Create Manufacturing Scene & Capture Screenshot

```bash
./scripts/run_sim.sh manufacturing_scene.py --steps 50 --capture /output/factory_scene.png
```

> **Note:** First run takes 5-10 minutes due to shader compilation. Subsequent runs are much faster (~1-2 min). Shader cache is stored in `~/docker/isaac-sim/cache/`.

### Save Scene as USD File

```bash
./scripts/run_sim.sh manufacturing_scene.py --steps 0 --save-usd /output/factory_scene.usd
```

### Pick and Place Demo

```bash
./scripts/run_sim.sh pick_and_place.py --pick-index 0 --place-pos "0.5,-0.8,0.5" --capture /output/pick_place.png
```

### Robot Control

```bash
# Home position
./scripts/run_sim.sh robot_control.py --action home

# Move to target position (x, y, z)
./scripts/run_sim.sh robot_control.py --action move_to --target "0.4,0.0,0.4"

# Move specific joints (7 joint values for Franka Panda)
./scripts/run_sim.sh robot_control.py --action move_joint --joints "0.0,0.5,0.0,-1.0,0.0,1.5,0.8"

# Gripper control
./scripts/run_sim.sh robot_control.py --action gripper --state open
./scripts/run_sim.sh robot_control.py --action gripper --state close
```

### Spawn Objects

```bash
# Single box at a specific position
./scripts/run_sim.sh spawn_objects.py --type box --position "0.3,0.0,1.0" --scale "0.1,0.1,0.1"

# 10 random spheres in an area
./scripts/run_sim.sh spawn_objects.py --type sphere --count 10 --area "-1,1,-1,1" --height 1.5
```

### Capture Viewport

```bash
./scripts/run_sim.sh capture_viewport.py --output /output/scene.png --scene manufacturing \
  --eye "5.0,-5.0,4.0" --target "0.0,0.0,0.3" --resolution "1920,1080"
```

---

## 5. Live Viewing via WebRTC Streaming

Isaac Sim supports real-time WebRTC streaming so you can view and interact with the simulation from any computer with a lightweight client.

### Start Streaming Server

```bash
# Get your server's public IP
PUBLIC_IP=$(curl -s ifconfig.me)

# Start Isaac Sim with streaming enabled
docker run -d --name isaac-sim-streaming \
  --gpus all \
  -e "ACCEPT_EULA=Y" \
  -e "PRIVACY_CONSENT=Y" \
  -e "OMNI_KIT_ACCEPT_EULA=YES" \
  --network=host \
  -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
  -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
  -v $(pwd)/scripts:/scripts:ro \
  -v $(pwd)/output:/output:rw \
  nvcr.io/nvidia/isaac-sim:6.0.0-dev2

# Wait for it to be ready (1-2 minutes)
docker logs -f isaac-sim-streaming 2>&1 | grep -m1 "Streaming App is loaded"
echo "Isaac Sim streaming is ready!"
```

### Open Firewall Ports

For **AWS EC2**, add these inbound rules to your Security Group:

| Port | Protocol | Source | Purpose |
|------|----------|--------|---------|
| 49100 | TCP | Your IP | WebRTC signaling |
| 47998 | UDP | Your IP | WebRTC media stream |

For other clouds or bare metal:

```bash
sudo ufw allow 49100/tcp
sudo ufw allow 47998/udp
```

---

## 6. Connecting from Mac

### Download the Streaming Client

| Platform | Download Link |
|----------|---------------|
| **Mac (Apple Silicon)** | [Download DMG](https://downloads.isaacsim.nvidia.com/isaacsim-webrtc-streaming-client-1.1.5-macos-arm64.dmg) |
| **Mac (Intel)** | [Download DMG](https://downloads.isaacsim.nvidia.com/isaacsim-webrtc-streaming-client-1.1.5-macos-x64.dmg) |
| **Windows** | [Download EXE](https://downloads.isaacsim.nvidia.com/isaacsim-webrtc-streaming-client-1.1.5-windows-x64.exe) |
| **Linux** | [Download AppImage](https://downloads.isaacsim.nvidia.com/isaacsim-webrtc-streaming-client-1.1.5-linux-x64.AppImage) |

### Connect

1. Install and open the Streaming Client
2. Enter your server's **public IP** (e.g., `35.86.255.7`)
3. Click **Connect**
4. You should see the Isaac Sim interface rendered in real-time

### Load the Manufacturing Scene

In the Isaac Sim interface (via Streaming Client):

1. Go to **File → Open**
2. Enter path: `/output/factory_scene.usd`
3. Click **Open**

Or use the Script Editor (**Window → Script Editor**):

```python
import omni.usd
omni.usd.get_context().open_stage("/output/factory_scene.usd")
```

### Navigation Controls

| Action | Control |
|--------|---------|
| Orbit | Alt + Left Mouse Button |
| Pan | Alt + Middle Mouse Button |
| Zoom | Scroll Wheel |
| Play Simulation | Click ▶️ at bottom toolbar |

---

## 7. OpenClaw Skill Integration

Copy the skill file to enable natural language control via Telegram:

```bash
mkdir -p ~/.openclaw/workspace/skills/isaac-sim
cp skill/SKILL.md ~/.openclaw/workspace/skills/isaac-sim/SKILL.md
```

Restart OpenClaw to pick up the new skill:

```bash
openclaw gateway restart
```

Now you can control the simulation via Telegram:

| You Say | What Happens |
|---------|--------------|
| "Create a factory scene" | Builds a manufacturing environment with conveyor, robot, shelves |
| "Show me the simulation" | Captures and sends a screenshot |
| "Pick up the red box" | Robot picks object from conveyor |
| "Move robot to (0.4, 0, 0.4)" | Moves end effector to target position |
| "Open gripper" | Opens the robot gripper |
| "Close gripper" | Closes the robot gripper |
| "Spawn 5 boxes" | Adds 5 box objects to the scene |
| "Spawn 10 random spheres" | Scatters 10 spheres across the scene |
| "Move joint 3 to 0.5 radians" | Moves a specific robot joint |
| "Home the robot" | Returns robot to home position |

---

## 8. Sim2Real Architecture & Agent Memory

This project is designed as a **self-improving system** where the AI agent learns from both simulation and real-world execution, continuously closing the sim-to-real gap.

### Core Concept

Traditional robotics: program once → deploy → break when conditions change.

**Self-improving Physical AI**: simulate → execute → observe → learn → improve → repeat.

The AI agent maintains **persistent memory** across all interactions, building an ever-growing knowledge base of:
- What works (and what doesn't) for each type of task
- How simulation predictions differ from real-world outcomes
- Optimized parameters learned from experience

### Agent Memory Architecture

| Memory Type | Purpose | Example |
|---|---|---|
| **Episodic Memory** | Logs every task execution with context, parameters, and outcome | "Picked gear #3 at t=14:32, top-down approach, grip_force=8N → success" |
| **Semantic Memory** | Stores learned facts about objects, environments, and strategies | "Aluminum gears require 15% more grip force than plastic ones" |
| **Procedural Memory** | Caches optimized trajectories and calibration offsets | "For conveyor pick: approach_offset_z=+0.02m, retreat_speed=0.1m/s" |
| **Sim2Real Delta Log** | Tracks discrepancies between sim and real outcomes | "Friction in sim=0.4 but real≈0.35 for oiled metal surfaces" |

### Self-Improving Loop

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│  1. PLAN ──→ 2. SIMULATE ──→ 3. VALIDATE ──→ 4. EXECUTE        │
│    ↑                                              │             │
│    │                                              ↓             │
│  6. LEARN ←──────────────── 5. OBSERVE ←──── Real Robot         │
│    │                                                            │
│    └──→ Update Memory ──→ Refine Sim Model ──→ Next Cycle       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

**Step 1 — Plan:** Agent queries memory for similar past tasks. Retrieves best-known strategy, parameters, and potential failure modes.

**Step 2 — Simulate:** Executes the task in Isaac Sim with learned parameters. Tests edge cases with domain randomization (lighting, friction, object pose variations).

**Step 3 — Validate:** Checks simulation results — collision-free? Stable grasp? Within joint limits? If sim fails, iterate in sim (cheap & fast) before touching the real robot.

**Step 4 — Execute:** Sends validated trajectory to the real robot via ROS 2. Monitors execution in real-time via sensor feedback.

**Step 5 — Observe:** Captures real-world outcome — RGB-D images, force/torque data, success/failure, timing. Compares against simulation predictions.

**Step 6 — Learn:** Logs the episode to memory. If sim predicted success but real failed (or vice versa), records the **delta** and updates simulation parameters (friction, mass, latency offsets) to reduce future discrepancies.

### Real Robot Integration (ROS 2)

The real robot system connects via ROS 2:

```
OpenClaw Agent
    │
    ├──→ Isaac Sim (simulation)
    │       └── /isaac_sim/joint_states (simulated)
    │
    └──→ ROS 2 Bridge
            ├── /joint_commands (real robot)
            ├── /gripper/command
            ├── /camera/color/image_raw (RGB)
            ├── /camera/depth/image_raw (Depth)
            └── /force_torque_sensor/wrench
```

Supported robot platforms:
- **Franka Emika Panda** — via `franka_ros2`
- **Universal Robots (UR5/UR10)** — via `ur_robot_driver`
- **Custom robots** — any ROS 2-compatible driver

### Domain Randomization for Sim2Real

To bridge the sim-to-real gap, the simulation applies randomization:

```python
# Example: randomize lighting, object colors, friction
randomization_params = {
    "lighting_intensity": (200, 800),     # lux
    "object_friction": (0.2, 0.6),        # coefficient
    "object_color_hsv_shift": (-0.1, 0.1),
    "camera_noise_std": 0.01,             # gaussian noise
    "table_height_offset": (-0.005, 0.005) # meters
}
```

This trains the agent's perception and control to be **robust** across real-world variations.

### Memory File Structure

```
memory/
├── episodes/
│   ├── 2026-04-20_pick_gear_001.json     # Individual task logs
│   ├── 2026-04-20_pick_gear_002.json
│   └── ...
├── semantic/
│   ├── objects.json                       # Learned object properties
│   ├── strategies.json                    # Task strategies
│   └── environment.json                   # Environment model
├── procedural/
│   ├── trajectories/                      # Cached optimal trajectories
│   └── calibration.json                   # Sim2Real calibration offsets
└── sim2real/
    ├── delta_log.json                     # Sim vs Real discrepancies
    └── domain_params.json                 # Tuned simulation parameters
```

---

## 9. Isaac Sim 6.0.0 API Reference

Isaac Sim 6.0.0 introduced new namespace imports (changed from 5.x). All scripts in this repository already use the correct 6.0.0 imports.

| Old Import (5.x) | New Import (6.0.0) |
|---|---|
| `from omni.isaac.core import World` | `from isaacsim.core.api import World` |
| `from omni.isaac.core.objects import DynamicCuboid` | `from isaacsim.core.api.objects import DynamicCuboid` |
| `from omni.isaac.core.prims import XFormPrim` | `from isaacsim.core.prims import XFormPrim` |
| `from omni.isaac.core.utils.stage_utils import add_reference_to_stage` | `from isaacsim.core.utils.stage import add_reference_to_stage` |
| `from omni.isaac.core.utils.nucleus import get_assets_root_path` | `from isaacsim.storage.native.nucleus import get_assets_root_path` |
| `from omni.isaac.sensor import Camera` | `from isaacsim.sensors.camera import Camera` |
| `from omni.isaac.core.controllers import BaseController` | `from isaacsim.core.api.controllers import BaseController` |
| `from omni.isaac.manipulators.controllers import PickPlaceController` | `from isaacsim.robot.manipulators.controllers import PickPlaceController` |

---

## Troubleshooting

### EULA Error / Segfault on Startup

Always use `--entrypoint ./python.sh` when running scripts. Do **NOT** use `-u 1234:1234` (this causes permission errors and segfaults).

The `run_sim.sh` wrapper handles this correctly. If running manually, set the environment variable before imports:

```python
import os
os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"
```

### First Run is Very Slow

This is normal — Isaac Sim compiles shaders on first run, which takes 5-10 minutes. The compiled shaders are cached in `~/docker/isaac-sim/cache/`, so subsequent runs are much faster (~1-2 minutes).

### WebRTC Streaming Not Connecting

- Ensure ports **49100/TCP** and **47998/UDP** are open in your firewall/security group
- Must use `--network=host` in the Docker run command (bridge networking doesn't work with WebRTC)
- Wait for `"Isaac Sim Full Streaming App is loaded"` in the container logs before connecting
- Check logs: `docker logs isaac-sim-streaming 2>&1 | tail -50`

### GPU Not Detected

```bash
# Verify NVIDIA runtime is available
docker run --rm --gpus all nvidia/cuda:12.0-base nvidia-smi
```

If this fails:
1. Check that NVIDIA drivers are installed: `nvidia-smi`
2. Check that nvidia-container-toolkit is installed: `nvidia-ctk --version`
3. Ensure Docker was restarted after toolkit installation: `sudo systemctl restart docker`

### Container Uses Too Much Memory

Add `--shm-size=4g` to the docker run command. Isaac Sim can use significant shared memory for GPU operations.

### Container Exits Immediately

Check the logs for errors:

```bash
docker logs <container-name> 2>&1 | tail -100
```

Common causes:
- Missing EULA acceptance (ensure `-e "ACCEPT_EULA=Y"` is set)
- Insufficient GPU memory (need 16GB+ VRAM)
- Wrong entrypoint (must use `--entrypoint ./python.sh` for script execution)

---

## License

This project uses NVIDIA Isaac Sim which requires acceptance of the [NVIDIA Omniverse License Agreement](https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html).

OpenClaw is available under its own license at [github.com/openclaw/openclaw](https://github.com/openclaw/openclaw).
