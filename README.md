# ROS 2 LLM Mission Control

**Natural-language mission planning for an autonomous ground robot - Prompt → LLM → Validated JSON → Deterministic Executor → Gazebo / Nav2.**

<!-- Built for the [Omokai](https://omokai.com) Robotics Engineering take-home task. -->

<!-- TODO: embed demo video/GIF here -->
<!-- ![Demo](assets/demo.gif) -->

---

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [Pipeline Walkthrough](#pipeline-walkthrough)
- [Repository Structure](#repository-structure)
- [Quick Start (Docker)](#quick-start-docker)
- [Example Prompts](#example-prompts)
- [Mission JSON Schema](#mission-json-schema)
- [Validation & Safety Guardrails](#validation--safety-guardrails)
- [Challenges Attempted](#challenges-attempted)
- [Scaling to Real-World Systems](#scaling-to-real-world-systems)
- [Cited Sources](#cited-sources)
- [Tech Stack](#tech-stack)
- [License](#license)

---

## Architecture Overview

The system implements a strict four-layer pipeline where the LLM **proposes** but never **flies**. The same validated JSON always produces the same robot behavior - the LLM is never in the control loop.

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                        LLM Mission Control Pipeline                             │
│                                                                                 │
│   ┌──────────┐    ┌──────────────┐    ┌─────────────────┐    ┌───────────────┐  │
│   │  PROMPT  │───▶│   GEMINI     │───▶│   VALIDATOR     │───▶│   EXECUTOR    │  │
│   │          │    │   (LLM)      │    │                 │    │               │  │
│   │ Natural  │    │ Structured   │    │ JSON Schema     │    │ Nav2 Action   │  │
│   │ language │    │ JSON output  │    │ Map bounds      │    │ Client        │  │
│   │ command  │    │ via Pydantic │    │ Costmap check   │    │ Waypoint nav  │  │
│   └──────────┘    └──────────────┘    └─────────────────┘    └───────┬───────┘  │
│                                                                      │          │
│                                                              ┌───────▼───────┐  │
│                                                              │  SIMULATOR    │  │
│                                                              │               │  │
│                                                              │ Gazebo +      │  │
│                                                              │ Nav2 + AMCL   │  │
│                                                              └───────────────┘  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

| Stage | Component | What It Does |
|---|---|---|
| **Prompt** | Human operator | Natural-language instruction, e.g. *"Patrol the perimeter twice at 0.3 m/s"* |
| **LLM** | Gemini 2.5 Flash | Interprets intent, emits structured JSON. Constrained at generation time via Pydantic `response_schema` |
| **Validator** | `mission_validator.py` | Double guardrail: JSON Schema (Draft-07) + live costmap occupancy check. Rejects bad coords, hallucinated fields, unsafe waypoints |
| **Executor** | `mission_executor.py` | Deterministic Nav2 `NavigateToPose` action client. Same JSON = same behavior, always |
| **Simulator** | Gazebo Harmonic + Nav2 | Full physics sim with AMCL localization, DWB local planner, A* global planner |

### Why This Architecture?

1. **Determinism** — The executor is a pure function of the validated JSON. No LLM inference runs during robot motion.
2. **Auditability** — Every mission is saved as a timestamped JSON file. You can replay any mission with `--file`.
3. **Safety** — Three validation layers catch bad plans before the robot moves: Pydantic schema at generation, JSON Schema after parsing, and live costmap check against the Nav2 global costmap.

---

## Pipeline Walkthrough

Here's what happens step-by-step when you type a command:

```
1.  rclpy.init()                          ← ROS 2 node starts
2.  Create node, costmap subscriber,      ← Plumbing setup
    Nav2 action client
3.  publish_initial_pose()                ← Seeds AMCL so map→odom TF is valid
4.  input("Enter command: ")              ← Operator types natural language
5.  call_gemini(prompt)                   ← LLM generates structured JSON
6.  validate_json_schema(raw_json)        ← Schema + defaults + return_to_start
7.  save_mission(mission)                 ← Timestamped JSON to missions/ dir
8.  nav_client.wait_for_server()          ← Wait for Nav2 to be ready
9.  costmap_validator.wait_for_costmap()  ← Wait for live costmap data
10. validate_waypoints()                  ← Map bounds + costmap occupancy check
11. execute_mission()                     ← Deterministic waypoint-by-waypoint nav
```

**Critical ordering**: `publish_initial_pose()` (step 3) must happen before waiting for Nav2/costmap (steps 8-9), because AMCL publishes the `map→odom` TF only after receiving `/initialpose`. Without this TF, the costmap's obstacle layer cannot project LiDAR scans and Nav2 never becomes ready.

---

## Repository Structure

```
ros2-llm-mission-control/
├── Dockerfile                          # Single-stage build on osrf/ros:jazzy-desktop
├── docker-compose.yml                  # Two services: sim stack + LLM bridge
├── entrypoint.sh                       # Sources ROS 2 + workspace setup
│
├── src/
│   ├── inspector_bot/                  # Robot platform (URDF, Nav2, maps, Gazebo)
│   │   ├── urdf/                       #   Parametric robot model
│   │   ├── config/                     #   Nav2 params, controllers, behavior tree
│   │   ├── launch/                     #   master_bringup.launch.py (single-command)
│   │   ├── maps/                       #   SLAM-generated warehouse occupancy grid
│   │   └── worlds/                     #   Gazebo SDF world files
│   │
│   ├── inspector_interfaces/           # Custom ROS 2 action definitions
│   │
│   ├── inspector_vision/               # Vision microservice (not used by LLM layer)
│   │
│   └── inspector_llm/                  # ★ LLM mission planning package
│       ├── inspector_llm/
│       │   ├── llm_bridge.py           #   Main entry: prompt → LLM → validate → execute
│       │   ├── mission_validator.py    #   JSON schema + map bounds + costmap checks
│       │   ├── mission_executor.py     #   Nav2 action client, initialpose, waypoint nav
│       │   └── schemas/
│       │       └── mission_schema.json #   JSON Schema Draft-07 for mission plans
│       ├── missions/                   #   Test mission files (valid + bad examples)
│       │   ├── test_valid.json
│       │   ├── test_bad_coords.json
│       │   └── test_bad_schema.json
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
│
└── missions/                           # Runtime output: saved mission JSONs
```

### Key Files

| File | Purpose |
|---|---|
| [`llm_bridge.py`](src/inspector_llm/inspector_llm/llm_bridge.py) | Orchestrates the full pipeline: prompt input → Gemini call → validation → execution |
| [`mission_validator.py`](src/inspector_llm/inspector_llm/mission_validator.py) | JSON Schema validation, map bounds check, live costmap occupancy check |
| [`mission_executor.py`](src/inspector_llm/inspector_llm/mission_executor.py) | Deterministic Nav2 `NavigateToPose` action client with initial pose seeding |
| [`mission_schema.json`](src/inspector_llm/inspector_llm/schemas/mission_schema.json) | JSON Schema Draft-07 — defines allowed fields, types, ranges, and enums |

---

## Quick Start (Docker)

### Prerequisites

- **Linux** with Docker installed
- **NVIDIA GPU** with the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) (for Gazebo rendering)
- A **Gemini API key** from [Google AI Studio](https://aistudio.google.com/apikey)

### Step 1: Clone and Build

```bash
git clone https://github.com/shubh-200/ros2-llm-mission-control.git
cd ros2-llm-mission-control

# Build the Docker image (~10 min on first build)
docker compose build
```

### Step 2: Launch the Simulation Stack

```bash
# Allow container to render on host display
xhost +local:docker

# Set your Gemini API key
export GEMINI_API_KEY="your-api-key-here"

# Terminal 1 — Launch Gazebo + Nav2 + AMCL
docker compose up inspector_stack
```

Wait ~30 seconds until you see Nav2 reporting ready in the logs.

### Step 3: Run the LLM Bridge

```bash
# Terminal 2 — Exec into the running container
docker exec -it inspector_autonomy_container bash

# Inside the container (ROS is auto-sourced via .bashrc):
export GEMINI_API_KEY="your-api-key-here"
ros2 run inspector_llm llm_bridge
```

### Step 4: Issue a Mission Command

```
=== ROS2 LLM Mission Control ===
Enter a mission command (e.g., "Patrol the perimeter twice at 0.3 m/s"):

> Patrol the warehouse perimeter once and return to start
```

The robot will:
1. LLM generates a waypoint plan as JSON
2. Validator checks schema + costmap safety
3. Mission is saved to `missions/`
4. Robot navigates each waypoint sequentially in Gazebo

### Replay a Saved Mission (No LLM Needed)

```bash
# Uses the standalone executor with a pre-validated JSON file
ros2 run inspector_llm mission_executor
```

### Standalone Validator Tests (No ROS Needed)

```bash
# These run the schema + map bounds checks without a live simulation
python3 src/inspector_llm/inspector_llm/mission_validator.py missions/test_valid.json
python3 src/inspector_llm/inspector_llm/mission_validator.py missions/test_bad_coords.json
python3 src/inspector_llm/inspector_llm/mission_validator.py missions/test_bad_schema.json
```

---

## Example Prompts

| Prompt | What Happens |
|---|---|
| `"Patrol the perimeter twice at 0.3 m/s"` | Robot visits all 4 corners (NE → SE → SW → NW), loops twice |
| `"Go to loading bay, wait 5 seconds, then return to origin"` | Robot navigates to (1.5, 0.5), waits, then goes to (0, 0) |
| `"Drive the inspection route and return to start"` | LLM picks a route through named locations, appends first waypoint at end |
| `"Visit north_east and south_west, spin 90 degrees at each"` | Robot visits two corners with spin tasks at each |
| `"Sweep the warehouse at max speed"` | LLM generates a coverage path at 0.5 m/s (schema max) |

---

## Mission JSON Schema

Every mission plan (whether LLM-generated or hand-crafted) is validated against a [JSON Schema Draft-07](src/inspector_llm/inspector_llm/schemas/mission_schema.json).

### Example Valid Mission

```json
{
  "mission_name": "Warehouse perimeter patrol",
  "description": "Two loops around the open area, return to start",
  "loop_count": 2,
  "return_to_start": true,
  "max_speed": 0.3,
  "stop_on_failure": false,
  "waypoints": [
    {"x":  2.0, "y":  1.0, "yaw":  1.57, "label": "north_east"},
    {"x":  2.0, "y": -1.0, "yaw": -1.57, "label": "south_east",
     "tasks": [{"action": "wait", "duration": 2.0}]},
    {"x": -1.0, "y": -1.0, "yaw":  3.14, "label": "south_west"},
    {"x": -1.0, "y":  1.0, "yaw":  0.0,  "label": "north_west"}
  ]
}
```

### Schema Constraints

| Field | Type | Constraints |
|---|---|---|
| `mission_name` | string | 1–100 chars |
| `loop_count` | integer | 1–10 |
| `max_speed` | number | 0.05–0.5 m/s |
| `frame_id` | string | `enum: ["map"]` — rejects `odom`/`base_link` hallucinations |
| `waypoints` | array | 1–20 items, each with required `x`, `y` |
| `waypoints[].yaw` | number | -π to π radians |
| `waypoints[].tasks[].action` | string | `enum: ["wait", "spin"]` — known commands only |
| `additionalProperties` | — | `false` at all levels — rejects any hallucinated fields |

---

## Validation & Safety Guardrails

The system uses **three independent validation layers** — no single point of failure:

### Layer 1: Pydantic Schema at Generation Time
Gemini's `response_schema=MissionPlan` constrains the LLM output at token generation. The LLM physically cannot output wrong field names or invalid types.

### Layer 2: JSON Schema Validation After Parsing
`jsonschema.validate()` runs independently on the parsed dict. Catches edge cases that Pydantic might not (e.g., `null` booleans via `Optional`, out-of-range values).

### Layer 3: Live Costmap Occupancy Check
Before the robot moves, every waypoint is checked against:
- **Static map bounds** — from `warehouse_map.yaml` resolution and origin
- **Nav2 global costmap** — live `/global_costmap/costmap` topic. Rejects waypoints that are inside obstacles, in the inscribed radius, or in unknown space.

### How the LLM Knows the Map

The LLM does NOT read the `.pgm` or `.yaml` map files. Instead, the system prompt injects:
- **Named locations** — a registry of semantic waypoints (`north_east`, `loading_bay`, etc.) with their `(x, y, yaw)` coordinates
- **Map geometry** — navigable area bounds `x=[-5.0, 5.0], y=[-3.0, 3.0]`
- **Constraints** — max speed, max waypoints, max loops, allowed task actions

This means the LLM reasons about the map using human-readable names and bounded coordinates, not raw pixel data.

---

## Challenges Attempted

### Core Task — ✅ Complete

The full pipeline works end-to-end:
- Natural language prompt → Gemini 2.5 Flash → structured JSON → schema validation → costmap validation → deterministic Nav2 execution → robot follows path in Gazebo.
- LLM is never in the control loop. Same JSON = same behavior.
- All missions are saved as timestamped, auditable JSON files.

### Senior Challenge Overviews

> The following are overviews of my approach to each challenge, as required by the assignment.

#### 1. Multi-Agent Formations

**Approach**: Extend the system prompt and JSON schema to include a `squad` field with per-agent waypoint assignments. The LLM would emit squad-level intent (e.g., `"formation": "wedge"`, `"task": "area_sweep"`) and a coordination layer would:
- Decompose the formation into per-agent offset waypoints
- Use a shared clock or heartbeat topic to synchronize motion start
- Implement a `formation_controller` node that adjusts velocities to maintain inter-agent spacing
- Handle regrouping by publishing a shared rally waypoint

**Technical choices**: Gazebo multi-robot namespacing (`/robot1/`, `/robot2/`), each with its own Nav2 stack. A central `squad_coordinator` node would consume the validated squad JSON and dispatch per-agent missions.

#### 2. SLAM / Autonomous Navigation

**Approach**: The base platform already has SLAM Toolbox integrated. To extend this:
- Replace the static map with online SLAM (`slam_toolbox` in `mapping` mode)
- Use frontier-based exploration to autonomously navigate unmapped areas
- The LLM would emit high-level goals (e.g., `"explore_area": {"bounds": [...]}`) and the executor would interface with an exploration planner
- Waypoint validation would switch from static map bounds to the live SLAM-generated costmap only

**What exists today**: The warehouse map was originally generated using SLAM Toolbox. AMCL localization against the static map is fully operational.

#### 3. Vision AI Target Detection + Follow

**Approach**: The base platform includes a lifecycle-managed vision microservice (`inspector_vision`) with RGB-D sensor fusion:
- Extend target detection from AprilTag-only to YOLOv8 for arbitrary object classes
- User specifies target class in the prompt (e.g., `"Follow the red forklift"`)
- LLM emits a `vision_task` field: `{"detect": "forklift", "action": "follow"}`
- On detection, the system: (a) publishes the camera frame to an operator topic, (b) enters a pursuit behavior that tracks the target's TF frame using a PID controller on `/cmd_vel`
- Target class would be configurable via the LLM prompt, validated against a known class list in the schema

**What exists today**: The vision node already does AprilTag detection, 2D→3D projection via organized point cloud, and TF2 broadcasting. The Nav2 Behavior Tree integration (`LocateTarget` action) is wired.

---

## Scaling to Real-World Systems

### What Would Break at Scale

1. **Hardcoded waypoint registry** — A real warehouse has hundreds of named locations. Solution: Load from a database or spatial index, not a Python dict in the system prompt.
2. **Single costmap check** — Dynamic environments need continuous replanning. Solution: Switch from pre-flight validation to runtime obstacle avoidance with Nav2's dynamic obstacle layer + recovery behaviors.
3. **Gemini API latency** — ~2s round-trip is fine for demos but blocks the operator. Solution: Async LLM calls with a mission queue, and a fast local fallback model for simple commands.
4. **Single-robot assumption** — The executor is single-threaded and single-agent. Solution: Namespaced multi-robot Nav2 stacks with a central dispatcher.

### What Would Stay the Same

- **The pipeline architecture** (Prompt → LLM → Validated JSON → Executor) scales well. Adding new capabilities means extending the JSON schema, not rewriting the executor.
- **Schema-based validation** is composable — new constraints can be added without touching the LLM or executor.
- **Deterministic execution** from validated JSON is the correct pattern for safety-critical systems. The gap between sim and real is in the executor layer (real hardware drivers), not in the planning pipeline.

---

## Cited Sources

| Source | License | What Was Used |
|---|---|---|
| [Inspector Bot](https://github.com/shubh-200/ros2-multimodal-mobile-vision-system) (own work) | Portfolio | Base robot platform: URDF, Gazebo world, Nav2 config, SLAM maps, vision node. The `inspector_bot`, `inspector_vision`, and `inspector_interfaces` packages are from this project. |
| [ROSA — Robot Operating System Agent](https://github.com/nasa-jpl/rosa) (NASA JPL) | Apache 2.0 | Architectural reference for tool-based LLM ↔ ROS integration patterns |
| [ChatDrones](https://github.com/Gaurang-1402/ChatDrones) | MIT | Inspiration for the ROSGPT pipeline pattern (prompt → structured plan → executor) |
| [ROS-LLM](https://github.com/Auromix/ROS-LLM) (Auromix) | Apache 2.0 | Reference for the "Brain-Executor" paradigm where LLM proposes and executor acts |
| [Gemini Structured Output](https://ai.google.dev/gemini-api/docs/structured-output) | — | `response_schema` parameter for constraining LLM output at generation time |
| [Nav2 Documentation](https://docs.nav2.org/) | Apache 2.0 | Nav2 action client patterns, AMCL configuration, costmap API |
| [osrf/ros:jazzy-desktop](https://hub.docker.com/r/osrf/ros) | Apache 2.0 | Base Docker image for ROS 2 Jazzy |

---

## Tech Stack

```
ROS 2 Jazzy  ·  Gazebo Harmonic  ·  Python 3  ·  Nav2 (A* + DWB + AMCL)
Gemini 2.5 Flash (google-genai)  ·  Pydantic  ·  jsonschema (Draft-07)
ros2_control  ·  diff_drive_controller  ·  SLAM Toolbox
Docker  ·  Docker Compose  ·  NVIDIA Container Toolkit
```

---

## License

<!-- This project is provided for evaluation purposes as part of the Omokai Robotics Engineering take-home task. -->

The base robot platform (`inspector_bot`, `inspector_vision`, `inspector_interfaces`) is from my own prior work: [ros2-multimodal-mobile-vision-system](https://github.com/shubh-200/ros2-multimodal-mobile-vision-system).
