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

# ---------------------------------------------------------------------------
# LLM call
# ---------------------------------------------------------------------------

def call_gemini(user_prompt: str, mode: str) -> str:
    client = genai.Client(api_key=os.environ['GEMINI_API_KEY'])

    if mode == 'explore':
        system_prompt = SYSTEM_PROMPT_EXPLORE
        schema = MissionPlanExplore
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

def execute_exploration(nav_client, node, mission, frontier_explorer):
    """Explore until duration expires or no frontiers remain."""
    explore_config = mission.get('explore_config', {})
    duration = explore_config.get('explore_duration_sec', 120)
    max_frontiers = explore_config.get('max_frontiers', 3)
    save_map_flag = explore_config.get('save_map', True)

    # Wait for the SLAM map to arrive
    print('[EXPLORER] Waiting for SLAM map...')
    frontier_explorer.wait_for_map(node)

    deadline = time.time() + duration
    cycle = 0

    print(f'[EXPLORER] Starting exploration for {duration}s (max {max_frontiers} frontiers/cycle)')

    while time.time() < deadline:
        cycle += 1
        print(f'\n--- Exploration cycle {cycle} ---')

        # Get current robot position (approximate — use odom if available)
        # For now, use (0,0) as starting reference; frontier explorer sorts by distance
        frontiers = frontier_explorer.get_frontiers(
            robot_x=0.0, robot_y=0.0,
            max_count=max_frontiers
        )

        if not frontiers:
            print('[EXPLORER] No more frontiers — exploration complete.')
            break

        for fx, fy in frontiers:
            if time.time() >= deadline:
                print('[EXPLORER] Time limit reached.')
                break

            print(f'[EXPLORER] Navigating to frontier ({fx:.2f}, {fy:.2f})')
            success = navigate_to_waypoint(nav_client, node, fx, fy, yaw=0.0)
            if not success:
                node.get_logger().warn(f'Frontier ({fx:.2f}, {fy:.2f}) unreachable, skipping.')
                continue

    print('\n[EXPLORER] Exploration finished.')

    if save_map_flag:
        print('[EXPLORER] Saving SLAM map...')
        try:
            _save_slam_map(node)
        except Exception as e:
            node.get_logger().error(f'Failed to save map: {e}')


def _save_slam_map(node):
    """Call slam_toolbox's serialize map or nav2 map_saver service."""
    from nav2_msgs.srv import SaveMap

    client = node.create_client(SaveMap, '/map_saver/save_map')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn('map_saver service not available, skipping map save.')
        return

    request = SaveMap.Request()
    request.map_url = 'explored_map'
    request.image_format = 'pgm'
    request.map_mode = 0  # TRINARY
    request.free_thresh = 0.196
    request.occupied_thresh = 0.65

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.result() is not None and future.result().result:
        print('[EXPLORER] Map saved as explored_map.pgm + explored_map.yaml')
    else:
        node.get_logger().warn('Map save returned unsuccessful result.')

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='ROS2 LLM Mission Control')
    parser.add_argument('--mode', choices=['mapped', 'explore'], default='mapped',
                        help='Mission mode: mapped (pre-built map) or explore (SLAM + frontiers)')
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
    if mode == 'mapped':
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
