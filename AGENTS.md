# AGENTS.md - Drone Project Architecture

This document describes the software agents (ROS2 nodes) and components in this multi-drone control project. The project supports two drone platforms: **Parrot ANAFI** series and **Bitcraze Crazyflie** with AI-Deck.

---

## Project Structure Overview

```
drone_project/
├── src/
│   ├── anafi_ros/              # Parrot ANAFI ROS2 bridge
│   │   ├── anafi_ros_nodes/    # Main ROS2 nodes
│   │   └── anafi_ros_interfaces/ # Custom messages and services
│   ├── anafi_ai/               # ANAFI keyboard controller
│   └── mini_drone/             # Crazyflie + AI-Deck nodes
└── docker/
    ├── ai_deck_bootloader/     # AI-Deck firmware bootloader
    └── ai_deck_examples/       # AI-Deck example applications
```

---

## ROS2 Nodes / Agents

### 1. ANAFI ROS Bridge (`anafi_ros_nodes` package)

#### 1.1 `anafi` Node
**File:** `src/anafi_ros/anafi_ros_nodes/anafi_ros_nodes/anafi.py`  
**Entry Point:** `anafi = anafi_ros_nodes.anafi:main`

The main bridge between ROS2 and Parrot ANAFI drones via the Olympe SDK.

| Aspect | Details |
|--------|---------|
| **Purpose** | Full drone control interface for ANAFI 4K/Thermal/USA/AI models |
| **SDK** | Parrot Olympe 7.5.0 |
| **Supported Models** | ANAFI 4K (2324), Thermal (2329), USA (2334), AI (2330) |

**Subscribed Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `camera/command` | `CameraCommand` | Camera zoom control |
| `drone/command` | `PilotingCommand` | Roll, pitch, yaw, gaz commands |
| `drone/moveto` | `MoveToCommand` | GPS waypoint navigation |
| `drone/moveby` | `MoveByCommand` | Relative displacement commands |
| `gimbal/command` | `GimbalCommand` | Gimbal attitude control |

**Published Topics:**
| Topic | Type | Frequency | Description |
|-------|------|-----------|-------------|
| `camera/image` | `Image` | 30 Hz | Main camera video stream |
| `drone/attitude` | `QuaternionStamped` | 30 Hz | Drone orientation (NWU) |
| `drone/altitude` | `Float32` | 30 Hz | Ground distance (m) |
| `drone/speed` | `Vector3Stamped` | 30 Hz | Body-frame velocity (m/s) |
| `drone/state` | `String` | 30 Hz | State machine status |
| `drone/rpy` | `Vector3Stamped` | 30 Hz | Roll, pitch, yaw (deg) |
| `battery/percentage` | `UInt8` | 30 Hz | Battery level (%) |
| `gimbal/attitude` | `QuaternionStamped` | 5 Hz | Gimbal orientation |

**Services:**
| Service | Type | Description |
|---------|------|-------------|
| `drone/takeoff` | `Trigger` | Take off the drone |
| `drone/land` | `Trigger` | Land the drone |
| `drone/emergency` | `Trigger` | Emergency motor cutoff |
| `drone/halt` | `Trigger` | Stop and hover |
| `drone/rth` | `Trigger` | Return to home |
| `skycontroller/offboard` | `SetBool` | Toggle offboard control mode |
| `followme/start` | `FollowMe` | Start follow-me mode |
| `POI/start` | `PilotedPOI` | Start point-of-interest tracking |

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device/ip` | string | `192.168.53.1` | Device IP address |
| `drone/model` | string | auto-detect | Drone model (4k/thermal/usa/ai) |
| `drone/max_altitude` | float | 2.0 | Max altitude (m) |
| `drone/max_horizontal_speed` | float | 1.0 | Max horizontal speed (m/s) |
| `drone/offboard` | bool | false | Offboard control enable |

---

#### 1.2 `sphinx` Node
**File:** `src/anafi_ros/anafi_ros_nodes/anafi_ros_nodes/sphinx.py`  
**Entry Point:** `sphinx = anafi_ros_nodes.sphinx:main`

Interface for Parrot Sphinx simulation environment.

| Aspect | Details |
|--------|---------|
| **Purpose** | Connect to simulated ANAFI drones in Sphinx |
| **Default IP** | `10.202.0.1` (Sphinx simulator) |

---

### 2. ANAFI Keyboard Controller (`anafi_ai` package)

#### 2.1 `anafi_keyboard_control` Node
**File:** `src/anafi_ai/anafi_ai/anafi_keyboard_control.py`  
**Entry Point:** `anafi_keyboard_control = anafi_ai.anafi_keyboard_control:main`

Terminal-based keyboard teleoperation for ANAFI drones using MoveBy commands.

| Aspect | Details |
|--------|---------|
| **Purpose** | Step-by-step keyboard control with discrete movements |
| **Namespace** | `/anafi` |
| **Control Mode** | MoveBy (relative displacement per keypress) |

**Key Mappings:**
| Key | Action |
|-----|--------|
| `w/s` | Forward/backward (±dx) |
| `a/d` | Left/right strafe (±dy) |
| `r/f` | Up/down (±dz) |
| `q/e` | Yaw left/right (±dyaw) |
| `t` | Takeoff |
| `l` | Land |
| `h` | Return to home |
| `k` | HALT (emergency stop) |
| `o` | Toggle offboard mode |
| `z/x` | Decrease/increase step size |
| `SHIFT` | Turbo mode (2x step) |

**Published Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `drone/moveby` | `MoveByCommand` | Relative movement commands |

**Subscribed Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `drone/moveby_done` | `Bool` | Movement completion feedback |
| `camera/image` | `Image` | Camera stream (status only) |

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `lin_step` | 0.5 | Linear step size (m) |
| `yaw_step_deg` | 15.0 | Yaw step size (deg) |
| `turbo_multiplier` | 2.0 | Turbo mode multiplier |
| `enable_offboard_on_start` | true | Auto-enable offboard |

---

### 3. Crazyflie Bridge (`mini_drone` package)

#### 3.1 `cf_bridge` Node
**File:** `src/mini_drone/mini_drone/cf_bridge_node.py`  
**Entry Point:** `cf_bridge = mini_drone.cf_bridge_node:main`

Main telemetry and control bridge for Bitcraze Crazyflie drones.

| Aspect | Details |
|--------|---------|
| **Purpose** | Bi-directional ROS2-Crazyflie communication |
| **SDK** | cflib (Crazyflie Python library) |
| **Protocol** | Crazy Real-Time Protocol (CRTP) via USB radio |

**Published Topics:**
| Topic | Type | Frequency | Description |
|-------|------|-----------|-------------|
| `/cf/imu` | `Imu` | 20 Hz | IMU data (orientation, angular velocity, acceleration) |
| `/cf/rpy` | `Vector3Stamped` | 20 Hz | Roll, pitch, yaw (deg) |
| `/cf/odom` | `Odometry` | 20 Hz | Position and velocity estimate |
| `/cf/battery` | `BatteryState` | 20 Hz | Battery voltage |
| `/cf/range/{dir}` | `Range` | 20 Hz | Multi-ranger distances (front/back/left/right/up/down) |
| `/cf/estop` | `Bool` | On change | E-STOP latch state |

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `uri` | `radio://0/80/2M/E7E7E7E7E7` | Crazyflie radio URI |
| `period_ms` | 100 | Telemetry logging period |
| `publish_rate_hz` | 20.0 | ROS publish rate |
| `use_state_estimate` | true | Use onboard state estimator |
| `arm_on_start` | true | Auto-arm on connection |

---

#### 3.2 `ControlManager` (Internal Component)
**File:** `src/mini_drone/mini_drone/control_logic.py`

Embedded within `cf_bridge`, handles all control logic.

**Subscribed Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/cf/cmd_hover` | `TwistStamped` | Hover setpoint (vx, vy, yawrate, z) |
| `/cf/hl/takeoff` | `Float32` | High-level takeoff (target height) |
| `/cf/hl/land` | `Float32` | High-level land (target height) |
| `/cf/hl/goto` | `PoseStamped` | High-level goto position |
| `/cf/pattern/circle` | `Float32MultiArray` | Circle pattern [radius, speed, z, duration] |
| `/cf/pattern/spin` | `Float32` | Spin in place (yaw rate rad/s) |
| `/cf/pattern/square` | `Float32MultiArray` | Square pattern [side, speed, z] |

**Services:**
| Service | Type | Description |
|---------|------|-------------|
| `/cf/stop` | `Trigger` | E-STOP (latched motor stop) |
| `/cf/estop_reset` | `Trigger` | Reset E-STOP latch |
| `/cf/notify_stop` | `Trigger` | Notify setpoint stop |
| `/cf/pattern/stop` | `Trigger` | Stop pattern execution |

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `dry_run` | false | Simulate without sending commands |
| `hover_timeout_s` | 0.2 | Hover command timeout |
| `hover_hold_forever` | true | Keep last setpoint on timeout |
| `hl_takeoff_duration_s` | 2.0 | Takeoff duration |
| `hl_land_duration_s` | 2.0 | Landing duration |

---

#### 3.3 `ai_deck_camera` Node
**File:** `src/mini_drone/mini_drone/ai_deck_camera_node.py`  
**Entry Point:** `ai_deck_camera = mini_drone.ai_deck_camera_node:main`

Streams video from the Crazyflie AI-Deck's GAP8 camera over WiFi.

| Aspect | Details |
|--------|---------|
| **Purpose** | Receive and publish AI-Deck camera images |
| **Protocol** | TCP/CPX packet protocol |
| **Formats** | RAW Bayer, JPEG |

**Published Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image` | `Image` | Decoded BGR image |
| `/camera/image/compressed` | `CompressedImage` | JPEG compressed image |

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `host` | `192.168.4.1` | AI-Deck IP address |
| `port` | 5000 | Streaming port |
| `publish_raw` | true | Publish decoded raw images |
| `decode_async` | true | Async JPEG decoding |
| `connect_timeout_s` | 3.0 | Connection timeout |
| `read_timeout_s` | 5.0 | Read timeout |

---

#### 3.4 `cf_keyboard_control_node` Node
**File:** `src/mini_drone/mini_drone/cf_keyboard_control_node.py`  
**Entry Point:** `cf_keyboard_control_node = mini_drone.cf_keyboard_control_node:main`

Terminal-based keyboard teleoperation for Crazyflie using high-level commands.

| Aspect | Details |
|--------|---------|
| **Purpose** | Keyboard control with position-based goto commands |
| **Control Mode** | High-level goto (PoseStamped) + hover (TwistStamped) |

**Key Mappings:**
| Key | Action |
|-----|--------|
| `w/s` | Forward/backward |
| `a/d` | Left/right strafe |
| `r/f` | Up/down |
| `q/e` | Yaw left/right |
| `t` | Takeoff (0.4m) |
| `l` | Land |
| `SPACE` | Emergency stop |
| `x` | Hover at current position |
| `0` | Stop sequence |
| `1` | Square flight pattern |
| `2` | Circle flight pattern |

**Published Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/cf/hl/goto` | `PoseStamped` | Goto position command |
| `/cf/hl/takeoff` | `Float32` | Takeoff command |
| `/cf/hl/land` | `Float32` | Land command |
| `/cf/cmd_hover` | `TwistStamped` | Hover velocity command |

**Subscribed Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/cf/odom` | `Odometry` | Current position/orientation |
| `/cf/battery` | `BatteryState` | Battery status |
| `/camera/image` | `Image` | Camera feed (FPS monitoring) |

---

#### 3.5 `quiz_controller` Node
**File:** `src/mini_drone/mini_drone/quiz_controller_node.py`  
**Entry Point:** `quiz_controller = mini_drone.quiz_controller_node:main`

State machine-based quiz demo controller that orchestrates both ANAFI and Crazyflie drones for quiz detection and trajectory drawing.

| Aspect | Details |
|--------|---------|
| **Purpose** | Coordinate multi-drone quiz demo (detection + trajectory drawing) |
| **Control Mode** | State machine with keyboard input |
| **Drones Controlled** | ANAFI (detection) + Crazyflie (trajectory drawing) |

**State Machine:**
```
┌──────────┐      's'        ┌──────────┐      'n'      ┌──────────┐
│  UNINIT  │ ──────────────► │   IDLE   │ ────────────► │DETECTING │
│          │   takeoff both  │          │               │          │
└──────────┘                 └──────────┘               └────┬─────┘
                                  ▲                          │
                                  │ 's' + home reached  /quiz/answer
                                  │                          │
                             ┌────┴─────┐                    │
                             │ DRAWING  │◄───────────────────┘
                             │          │   start trajectory
                             └────┬─────┘
                                  │
     ┌────────────────────────────┼────────────────────────────┐
     │ 'x' OR 5min timeout        │                            │
     │ (from IDLE/DETECTING/DRAWING)                           │
     ▼                            ▼                            ▼
┌──────────┐
│  FINISH  │  ◄─────── land both drones ───────────────────────
│          │
└──────────┘
```

**States:**
| State | Description |
|-------|-------------|
| `UNINIT` | Initial state - all drones on ground |
| `IDLE` | Both drones hovering at home positions |
| `DETECTING` | ANAFI detecting quiz, waiting for answer |
| `DRAWING` | Mini drone executing trajectory based on answer |
| `FINISH` | All drones landed, operation complete |

**Key Mappings:**
| Key | Action |
|-----|--------|
| `s` | Start (UnInit→Idle) / Return home (Drawing→Idle) |
| `n` | Next - Start detecting (Idle→Detecting) |
| `x` | Exit - Land and finish |
| `SPACE` | Emergency Stop (all drones) |
| `1` | (mini_only_mode) Manual answer 1 → figure8 |
| `2` | (mini_only_mode) Manual answer 2 → vertical_a |
| `Ctrl+C` | Force quit |

**Subscribed Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/cf/odom` | `Odometry` | Mini drone position/orientation |
| `/anafi/drone/state` | `String` | ANAFI flight state |
| `/quiz/answer` | `String` | Detection result (triggers Drawing) |

**Published Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/quiz/state` | `String` | Current controller state |
| `/cf/hl/takeoff` | `Float32` | Mini takeoff command |
| `/cf/hl/land` | `Float32` | Mini land command |
| `/cf/hl/goto` | `PoseStamped` | Mini goto position |

**Service Clients:**
| Service | Type | Description |
|---------|------|-------------|
| `/anafi/drone/takeoff` | `Trigger` | ANAFI takeoff |
| `/anafi/drone/land` | `Trigger` | ANAFI land |
| `/cf/traj/run` | `RunTrajectory` | Execute Mini trajectory |

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `mini_home_x` | 0.0 | Mini home X position (m) |
| `mini_home_y` | 0.0 | Mini home Y position (m) |
| `mini_home_z` | 0.4 | Mini home Z position (m) |
| `mini_home_yaw_deg` | 0.0 | Mini home yaw (deg) |
| `anafi_home_x` | 0.0 | ANAFI home X position (m) |
| `anafi_home_y` | 0.0 | ANAFI home Y position (m) |
| `anafi_home_z` | 0.6 | ANAFI home Z position (m) |
| `anafi_home_yaw_deg` | 0.0 | ANAFI home yaw (deg) |
| `takeoff_duration_s` | 2.0 | Takeoff duration (s) |
| `land_duration_s` | 2.0 | Landing duration (s) |
| `command_settle_time_s` | 0.2 | Extra wait after duration |
| `operation_timeout_min` | 5.0 | Auto-finish timeout (min) |
| `home_position_tolerance_m` | 0.1 | Home position tolerance (m) |
| `home_yaw_tolerance_deg` | 15.0 | Home yaw tolerance (deg) |
| `mini_only_mode` | false | Test with Mini drone only (skip ANAFI) |

**Answer to Trajectory Mapping:**
| Answer | Trajectory |
|--------|------------|
| `"1"` | `figure8` |
| `"2"` | `vertical_a` |

---

## Custom Message Types

### `anafi_ros_interfaces`

| Message | Fields | Description |
|---------|--------|-------------|
| `PilotingCommand` | roll, pitch, yaw, gaz | RPYG piloting control |
| `MoveByCommand` | dx, dy, dz, dyaw | Relative displacement |
| `MoveToCommand` | latitude, longitude, altitude, heading | GPS navigation |
| `CameraCommand` | mode, zoom | Camera control |
| `GimbalCommand` | mode, frame, roll, pitch, yaw | Gimbal control |
| `TargetTrajectory` | position, velocity | Target tracking data |

---

## Launch Files

### `crazyflie_bridge.launch.py`
**Location:** `src/mini_drone/launch/`

Launches the complete Crazyflie system:
```bash
ros2 launch mini_drone crazyflie_bridge.launch.py
```

**Nodes Started:**
1. `cf_bridge` - Telemetry and control
2. `ai_deck_camera` - Camera streaming

### `anafi_launch.py`
**Location:** `src/anafi_ros/anafi_ros_nodes/launch/`

Launches the ANAFI bridge:
```bash
ros2 launch anafi_ros_nodes anafi_launch.py
```

### `quiz_demo.launch.py`
**Location:** `src/mini_drone/launch/`

Launches the complete quiz demo system with both drones:
```bash
ros2 launch mini_drone quiz_demo.launch.py
```

With custom parameters:
```bash
ros2 launch mini_drone quiz_demo.launch.py operation_timeout_min:=3.0 mini_home_z:=0.5
```

**Nodes Started:**
1. `cf_bridge` - Crazyflie telemetry and control
2. `ai_deck_camera` - AI-Deck camera streaming
3. `quiz_controller` - State machine controller (in separate terminal)

**Note:** ANAFI bridge should be launched separately:
```bash
# Terminal 1: Launch ANAFI
ros2 launch anafi_ros_nodes anafi_launch.py namespace:='anafi'

# Terminal 2: Launch Quiz Demo (Crazyflie + Controller)
ros2 launch mini_drone quiz_demo.launch.py
```

---

## Communication Topology

```
┌─────────────────────────────────────────────────────────────────────┐
│                         ROS2 Ecosystem                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────┐     Topics/Services     ┌──────────────────┐  │
│  │  Keyboard       │◄──────────────────────► │   cf_bridge      │  │
│  │  Controller     │      /cf/hl/*           │   (Crazyflie)    │  │
│  │                 │      /cf/cmd_hover      │                  │  │
│  └─────────────────┘                         └────────┬─────────┘  │
│                                                       │ cflib      │
│  ┌─────────────────┐                                  ▼            │
│  │  AI-Deck        │                         ┌──────────────────┐  │
│  │  Camera Node    │      /camera/*          │   Crazyflie 2.1  │  │
│  │                 │─────────────────────────│   + AI-Deck      │  │
│  └────────┬────────┘      TCP/WiFi           └──────────────────┘  │
│           │                                                        │
│           └── Camera Stream (192.168.4.1:5000)                     │
│                                                                     │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────┐     Topics/Services     ┌──────────────────┐  │
│  │  ANAFI Keyboard │◄──────────────────────► │   anafi          │  │
│  │  Controller     │      /anafi/drone/*     │   (Bridge)       │  │
│  │                 │      /anafi/camera/*    │                  │  │
│  └─────────────────┘                         └────────┬─────────┘  │
│                                                       │ Olympe SDK │
│                                                       ▼            │
│                                              ┌──────────────────┐  │
│                                              │   Parrot ANAFI   │  │
│                                              │   (via SkyCtrl)  │  │
│                                              └──────────────────┘  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## AI-Deck Firmware

### Bootloader
**Location:** `docker/ai_deck_bootloader/`

Low-level bootloader for flashing the GAP8 processor on the AI-Deck.

**Key Components:**
| File | Purpose |
|------|---------|
| `main.c` | Bootloader entry point |
| `cpx.c` | CPX protocol implementation |
| `com.c` | Communication layer |
| `flash.c` | Flash memory operations |
| `bl.c` | Bootloader logic |

### Examples
**Location:** `docker/ai_deck_examples/`

Example applications for AI-Deck:
- Face detection
- Image classification
- WiFi image streaming
- Camera testing

---

## How to Build & Run ROS Nodes

### Building

#### Build All Packages (Big Drone - ANAFI)
```bash
cd ~/drone_project
colcon build
source install/setup.bash
```

#### Build Mini Drone Only (Crazyflie)
```bash
cd ~/drone_project
colcon build --packages-select mini_drone
source install/setup.bash
```

#### Build Specific Packages
```bash
# Build only ANAFI packages
colcon build --packages-select anafi_ros_interfaces anafi_ros_nodes anafi_ai
source install/setup.bash

# Build only mini drone package
colcon build --packages-select mini_drone
source install/setup.bash
```

---

### Running - Big Drone (Parrot ANAFI)

#### Terminal 1: Launch ANAFI Bridge
```bash
ros2 launch anafi_ros_nodes anafi_launch.py
```

With custom parameters:
```bash
ros2 launch anafi_ros_nodes anafi_launch.py namespace:='anafi' ip:='192.168.53.1' model:='ai'
```

#### Terminal 2: Keyboard Control
```bash
ros2 run anafi_ai anafi_keyboard_control
```

---

### Running - Mini Drone (Crazyflie)

#### Terminal 1: Launch Crazyflie Bridge
```bash
ros2 launch mini_drone crazyflie_bridge.launch.py
```

#### Terminal 2: Keyboard Control
```bash
ros2 run mini_drone cf_keyboard_control_node
```

---

## Development Notes

### Code Style
- **Formatter:** rustfmt (via `cargo fmt`) for any Rust code
- **Linter:** clippy (via `cargo clippy`) for any Rust code
- **Python:** Follow ROS2 Python conventions

### Testing
Each actor/module has corresponding test files in `test/` directories.

---

## Dependencies

| Component | Dependency | Version |
|-----------|------------|---------|
| ANAFI Bridge | parrot-olympe | 7.5.0 |
| Crazyflie Bridge | cflib | latest |
| Camera Nodes | opencv-python | latest |
| ROS2 | Humble/Foxy | recommended |

---

## Quick Reference

| Drone | Build Command | Launch Command | Keyboard Control |
|-------|---------------|----------------|------------------|
| ANAFI (Big) | `colcon build` | `ros2 launch anafi_ros_nodes anafi_launch.py` | `ros2 run anafi_ai anafi_keyboard_control` |
| Crazyflie (Mini) | `colcon build --packages-select mini_drone` | `ros2 launch mini_drone crazyflie_bridge.launch.py` | `ros2 run mini_drone cf_keyboard_control_node` |
| **Quiz Demo** | `colcon build --packages-select mini_drone` | `ros2 launch mini_drone quiz_demo.launch.py` | Built-in (s/n/x keys) |

---

### Quiz Demo Quick Start

```bash
# Terminal 1: Launch ANAFI (namespace required for quiz_controller)
ros2 launch anafi_ros_nodes anafi_launch.py namespace:='anafi'

# Terminal 2: Launch Quiz Demo
ros2 launch mini_drone quiz_demo.launch.py

# Quiz Controller will open in a separate terminal (xterm)
# Controls: [s] Start, [n] Detect, [x] Exit

# To simulate detection result (for testing):
ros2 topic pub --once /quiz/answer std_msgs/String "data: '1'"
```

---

*Last updated: December 2025*

