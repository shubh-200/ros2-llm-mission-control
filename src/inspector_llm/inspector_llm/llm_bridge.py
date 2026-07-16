import os
import argparse
import rclpy
import json
import time
from datetime import datetime
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from pydantic import BaseModel
from typing import Optional, List
from google import genai
from google.genai import types
from ament_index_python.packages import get_package_share_directory
from inspector_llm.mission_validator import validate_json_schema, validate_waypoints, load_map_metadata, CostmapValidator
from inspector_llm.mission_executor import publish_initial_pose, navigate_to_waypoint
from inspector_llm.frontier_explorer import FrontierExplorer

MAP_YAML = os.path.join(
    get_package_share_directory('inspector_bot'),
    'maps',
    'warehouse_map.yaml'
)

# ---------------------------------------------------------------------------
# System prompts — one for each mode
# ---------------------------------------------------------------------------

SYSTEM_PROMPT_MAPPED = """You are a mission planner for a ground robot in a warehouse.

NAMED LOCATIONS (use these when the operator references them):
  origin:        x=0.0,  y=0.0
  north_east:    x=3.0,  y=2.0
  south_east:    x=3.0,  y=-2.0
  south_west:    x=-3.0, y=-2.0
  north_west:    x=-3.0, y=2.0
  loading_bay:   x=1.5,  y=0.5
  charging_dock: x=-1.0, y=0.0

MAP GEOMETRY:
  Navigable area (approximate): x=[-5.0, 5.0], y=[-3.0, 3.0]
  Robot starts at origin (0, 0).
  All waypoints are validated against the live costmap.

CONSTRAINTS:
  Max speed: 0.5 m/s
  Max waypoints: 20
  Max loops: 10
  Yaw range: [-3.14159, 3.14159] radians
  Available waypoint tasks: "wait" (with duration in seconds), "spin" (with angle in radians)

You MUST set "mode" to "mapped" in your output.
Output ONLY valid JSON. No markdown, no explanation, no code fences."""

SYSTEM_PROMPT_EXPLORE = """You are a mission planner for a ground robot exploring an UNKNOWN warehouse.

The robot has NO pre-built map. It uses SLAM (simultaneous localization and mapping)
to build the map as it drives, and a frontier-based explorer to find unexplored areas.

Your job is to interpret the operator's exploration intent and emit a mission plan
with mode="explore". You do NOT need to specify waypoints — the frontier explorer
will generate them automatically.

EXPLORE CONFIG OPTIONS:
  explore_duration_sec: How long to explore (10-600 seconds, default 120)
  max_frontiers: How many frontier waypoints to pursue per cycle (1-10, default 3)
  save_map: Whether to save the SLAM-generated map after exploration (default true)

CONSTRAINTS:
  Max speed: 0.5 m/s

You MUST set "mode" to "explore" in your output.
Output ONLY valid JSON. No markdown, no explanation, no code fences."""

SYSTEM_PROMPT_VISION = """You are a mission planner for a ground robot with a camera in a warehouse.

The robot can detect and follow colored objects using its front-facing RGB-D camera.
Your job is to interpret the operator's request and emit a vision mission plan.

AVAILABLE TARGETS (these are the objects the robot can detect by color):
  red_target   — A bright red box moving through the warehouse
  cargo_box    — A brown/tan cargo box with an AprilTag
  blue_barrel  — A blue barrel

AVAILABLE ACTIONS:
  detect  — Find the target and report its 3D position (one-shot)
  follow  — Find the target and pursue it, maintaining distance

VISION CONFIG OPTIONS:
  target: Which object to detect (one of the available targets above)
  action: "detect" or "follow"
  timeout_sec: How long to run the vision task (5-300 seconds, default 60)
  return_to_start: Whether to navigate back to the starting position after the task (default true)

CONSTRAINTS:
  Max speed: 0.5 m/s

You MUST set "mode" to "vision" in your output.
Output ONLY valid JSON. No markdown, no explanation, no code fences."""

# ---------------------------------------------------------------------------
# Pydantic models for Gemini structured output
# ---------------------------------------------------------------------------

class WaypointTask(BaseModel):
    action: str          # "wait" or "spin"
    duration: Optional[float] = None
    angle: Optional[float] = None

class Waypoint(BaseModel):
    x: float
    y: float
    yaw: Optional[float] = 0.0
    label: Optional[str] = None
    tasks: Optional[List[WaypointTask]] = None

class ExploreConfig(BaseModel):
    explore_duration_sec: Optional[float] = 120.0
    max_frontiers: Optional[int] = 3
    save_map: bool = True

class MissionPlanMapped(BaseModel):
    mode: str = "mapped"
    mission_name: Optional[str] = None
    description: Optional[str] = None
    loop_count: int = 1
    return_to_start: bool = False
    max_speed: Optional[float] = 0.3
    stop_on_failure: bool = False
    waypoints: List[Waypoint]

class MissionPlanExplore(BaseModel):
    mode: str = "explore"
    mission_name: Optional[str] = None
    description: Optional[str] = None
    max_speed: Optional[float] = 0.3
    explore_config: Optional[ExploreConfig] = None

class VisionConfig(BaseModel):
    target: Optional[str] = 'red_target'
    action: Optional[str] = 'follow'
    timeout_sec: Optional[float] = 60.0
    return_to_start: bool = True

class MissionPlanVision(BaseModel):
    mode: str = "vision"
    mission_name: Optional[str] = None
    description: Optional[str] = None
    max_speed: Optional[float] = 0.3
    vision_config: Optional[VisionConfig] = None

# ---------------------------------------------------------------------------
# LLM call
# ---------------------------------------------------------------------------

def call_gemini(user_prompt: str, mode: str) -> str:
    client = genai.Client(api_key=os.environ['GEMINI_API_KEY'])

    if mode == 'explore':
        system_prompt = SYSTEM_PROMPT_EXPLORE
        schema = MissionPlanExplore
    elif mode == 'vision':
        system_prompt = SYSTEM_PROMPT_VISION
        schema = MissionPlanVision
    else:
        system_prompt = SYSTEM_PROMPT_MAPPED
        schema = MissionPlanMapped

    response = client.models.generate_content(
        model='gemini-2.5-flash',
        contents=user_prompt,
        config=types.GenerateContentConfig(
            system_instruction=system_prompt,
            response_mime_type='application/json',
            response_schema=schema,
        ),
    )
    return response.text

# ---------------------------------------------------------------------------
# Save mission
# ---------------------------------------------------------------------------

def save_mission(mission: dict, missions_dir: str = 'missions'):
    os.makedirs(missions_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    name = mission.get('mission_name', 'unnamed').replace(' ', '_').lower()
    filename = f'{timestamp}_{name}.json'
    filepath = os.path.join(missions_dir, filename)
    with open(filepath, 'w') as f:
        json.dump(mission, f, indent=2)
    print(f'[SAVED] Mission saved to {filepath}')
    return filepath

# ---------------------------------------------------------------------------
# Exploration execution loop
# ---------------------------------------------------------------------------

def _get_robot_position(tf_buffer, node) -> tuple[float, float]:
    """Get the robot's current (x, y) position in the map frame via TF."""
    try:
        transform = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        return x, y
    except Exception as e:
        node.get_logger().warn(f'TF lookup failed, using (0,0): {e}')
        return 0.0, 0.0


def execute_exploration(nav_client, node, mission, frontier_explorer):
    """Explore until duration expires or no frontiers remain."""
    explore_config = mission.get('explore_config', {})
    duration = explore_config.get('explore_duration_sec', 120)
    max_frontiers = explore_config.get('max_frontiers', 3)
    save_map_flag = explore_config.get('save_map', True)

    # Set up TF listener for robot position tracking
    from tf2_ros import Buffer, TransformListener
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    # Wait for the SLAM map to arrive
    print('[EXPLORER] Waiting for SLAM map...')
    frontier_explorer.wait_for_map(node)

    # Give TF a moment to populate
    time.sleep(2.0)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)

    deadline = time.time() + duration
    cycle = 0
    failed_frontiers = set()  # Track unreachable frontiers to avoid retrying

    print(f'[EXPLORER] Starting exploration for {duration}s (max {max_frontiers} frontiers/cycle)')

    while time.time() < deadline:
        cycle += 1
        print(f'\n--- Exploration cycle {cycle} ---')

        # Get current robot position from TF
        robot_x, robot_y = _get_robot_position(tf_buffer, node)

        frontiers = frontier_explorer.get_frontiers(
            robot_x=robot_x, robot_y=robot_y,
            max_count=max_frontiers
        )

        # Filter out previously failed frontiers (within 1m of a failed point)
        frontiers = [
            (fx, fy) for fx, fy in frontiers
            if not any(abs(fx - bx) < 1.0 and abs(fy - by) < 1.0
                       for bx, by in failed_frontiers)
        ]

        if not frontiers:
            print('[EXPLORER] No more reachable frontiers — exploration complete.')
            break

        for fx, fy in frontiers:
            if time.time() >= deadline:
                print('[EXPLORER] Time limit reached.')
                break

            print(f'[EXPLORER] Navigating to frontier ({fx:.2f}, {fy:.2f})')
            success = navigate_to_waypoint(nav_client, node, fx, fy, yaw=0.0)
            if not success:
                node.get_logger().warn(f'Frontier ({fx:.2f}, {fy:.2f}) unreachable, skipping.')
                failed_frontiers.add((fx, fy))
                continue

    print('\n[EXPLORER] Exploration finished.')

    if save_map_flag:
        print('[EXPLORER] Saving SLAM map...')
        try:
            _save_slam_map(node)
        except Exception as e:
            node.get_logger().error(f'Failed to save map: {e}')


def _save_slam_map(node):
    """Save the SLAM-generated map via slam_toolbox's SerializePoseGraph service."""
    from slam_toolbox.srv import SerializePoseGraph

    map_name = 'explored_map'
    node.get_logger().info(f'Serializing SLAM map as {map_name}...')

    client = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn('slam_toolbox/serialize_map service not available.')
        return

    request = SerializePoseGraph.Request()
    request.filename = map_name

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.result() is not None:
        print(f'[EXPLORER] Map serialized as {map_name}.posegraph + {map_name}.data')
    else:
        node.get_logger().warn('SerializePoseGraph returned no result.')

# ---------------------------------------------------------------------------
# Vision mission execution
# ---------------------------------------------------------------------------

def execute_vision_mission(node, mission: dict):
    """
    Execute a vision detection/follow mission.

    Launches the vision_detector and visual_follower nodes as subprocesses,
    monitors the follower's status, and shuts down when done.
    """
    import subprocess
    import signal

    vc = mission.get('vision_config', {})
    target = vc.get('target', 'red_target')
    action = vc.get('action', 'follow')
    timeout = vc.get('timeout_sec', 60)
    return_flag = vc.get('return_to_start', True)

    print(f'\n[VISION] Starting vision mission:')
    print(f'  Target:  {target}')
    print(f'  Action:  {action}')
    print(f'  Timeout: {timeout}s')
    print(f'  Return:  {return_flag}\n')

    processes = []

    try:
        # Launch vision detector node
        print('[VISION] Launching vision_detector node...')
        detector_cmd = [
            'ros2', 'run', 'inspector_llm', 'vision_detector',
            '--ros-args',
            '-p', f'target_name:={target}',
            '-p', 'snapshot_dir:=detections',
            '-p', 'use_sim_time:=true',
        ]
        detector_proc = subprocess.Popen(detector_cmd)
        processes.append(('vision_detector', detector_proc))

        if action == 'follow':
            # Launch visual follower node
            print('[VISION] Launching visual_follower node...')
            follower_cmd = [
                'ros2', 'run', 'inspector_llm', 'visual_follower',
                '--ros-args',
                '-p', f'follow_timeout:={float(timeout)}',
                '-p', f'return_to_start:={str(return_flag).lower()}',
                '-p', 'use_sim_time:=true',
            ]
            follower_proc = subprocess.Popen(follower_cmd)
            processes.append(('visual_follower', follower_proc))

            # Wait for follower to finish (it exits after timeout/target-lost + return)
            print(f'[VISION] Following target for up to {timeout}s...')
            try:
                follower_proc.wait(timeout=timeout + 120)  # extra 120s for return-to-start
                print('[VISION] Follower node exited.')
            except subprocess.TimeoutExpired:
                print('[VISION] Follower timed out. Terminating.')
        else:
            # Detect-only: just wait for timeout
            print(f'[VISION] Detecting target for {timeout}s...')
            print('[VISION] Watch /detected_target and /detection_image in RViz.')
            time.sleep(timeout)
            print('[VISION] Detection timeout reached.')

    except KeyboardInterrupt:
        print('\n[VISION] Interrupted by user.')

    finally:
        # Cleanup: terminate all launched processes
        for name, proc in processes:
            if proc.poll() is None:
                print(f'[VISION] Terminating {name}...')
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()

    print('[VISION] Vision mission complete.')

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='ROS2 LLM Mission Control')
    parser.add_argument('--mode', choices=['mapped', 'explore', 'vision'], default='mapped',
                        help='Mission mode: mapped | explore | vision')
    parser.add_argument('--file', type=str, default=None,
                        help='Replay a saved mission JSON file instead of calling the LLM')
    args = parser.parse_args()

    mode = args.mode

    # --- 1. Initialize ROS ---
    rclpy.init()
    node = rclpy.create_node('llm_bridge')

    costmap_validator = CostmapValidator(node)
    nav_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')

    # In mapped mode, seed AMCL with initial pose.
    # In explore mode, SLAM Toolbox handles localization — no initialpose needed.
    # In vision mode, we use the static map (AMCL) for return-to-start.
    if mode in ('mapped', 'vision'):
        publish_initial_pose(node)

    # --- 2. Get prompt or load file ---
    if args.file:
        print(f'\n[REPLAY] Loading mission from {args.file}')
        with open(args.file, 'r') as f:
            raw_json = f.read()
    else:
        print(f'\n=== ROS2 LLM Mission Control (mode: {mode}) ===')
        if mode == 'explore':
            print('Enter an exploration command (e.g., "Explore the warehouse for 3 minutes"):\n')
        elif mode == 'vision':
            print('Enter a vision command (e.g., "Find the red box and follow it"):\n')
        else:
            print('Enter a mission command (e.g., "Patrol the perimeter twice at 0.3 m/s"):\n')
        user_prompt = input('> ')

        print('\n[LLM] Sending to Gemini...')
        raw_json = call_gemini(user_prompt, mode)
        print(f'[LLM] Response:\n{raw_json}\n')

    # --- 3. Validate JSON schema ---
    print('[VALIDATOR] Checking schema...')
    mission = validate_json_schema(raw_json)

    mission_mode = mission.get('mode', 'mapped')
    if mission_mode != mode:
        print(f'[WARNING] LLM returned mode="{mission_mode}" but bridge is in "{mode}" mode. '
              f'Using bridge mode "{mode}".')
        mission['mode'] = mode

    save_mission(mission)

    # --- 4. Wait for Nav2 ---
    node.get_logger().info('Waiting for Nav2...')
    nav_client.wait_for_server()

    # --- 5. Mode-specific execution ---
    if mode == 'explore':
        # Explore mode: use frontier explorer
        frontier_explorer = FrontierExplorer(node)
        execute_exploration(nav_client, node, mission, frontier_explorer)

    elif mode == 'vision':
        # Vision mode: launch detector + follower nodes
        execute_vision_mission(node, mission)

    else:
        # Mapped mode: validate waypoints against costmap, then execute
        node.get_logger().info('Waiting for costmap...')
        costmap_validator.wait_for_costmap(node)

        map_meta = load_map_metadata(MAP_YAML)
        waypoints = mission.get('waypoints', [])

        print(f'[VALIDATOR] Schema OK — {len(waypoints)} waypoints, '
              f'{mission.get("loop_count", 1)} loop(s)')
        print('[VALIDATOR] Checking waypoints against costmap...')
        validate_waypoints(waypoints, map_meta, costmap_validator)
        print('[VALIDATOR] All waypoints safe.')

        # Execute mapped mission
        print('\n[EXECUTOR] Starting mission...\n')
        loop_count = mission.get('loop_count', 1)

        for loop in range(loop_count):
            print(f'--- Loop {loop + 1} of {loop_count} ---')
            for wp in waypoints:
                success = navigate_to_waypoint(nav_client, node, wp['x'], wp['y'], wp.get('yaw', 0.0))
                if not success:
                    print('[EXECUTOR] Mission aborted.')
                    node.destroy_node()
                    rclpy.shutdown()
                    return

    print('\n[EXECUTOR] Mission complete.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
