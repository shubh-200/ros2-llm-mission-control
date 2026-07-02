import os
import rclpy
import json
from datetime import datetime
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from google import genai
from inspector_llm.mission_validator import validate_json_schema, validate_waypoints, load_map_metadata, CostmapValidator
from inspector_llm.mission_executor import publish_initial_pose, navigate_to_waypoint

MAP_YAML = '/home/shubham/omokai_ws/install/inspector_bot/share/inspector_bot/maps/warehouse_map.yaml'

SYSTEM_PROMPT = """You are a mission planner for a ground robot in a warehouse.

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

Output ONLY valid JSON. No markdown, no explanation, no code fences."""


def call_gemini(user_prompt: str) -> str:
    client = genai.Client(api_key=os.environ['GEMINI_API_KEY'])

    response = client.models.generate_content(
        model='gemini-2.5-flash',
        contents=user_prompt,
        config={
            'system_instruction': SYSTEM_PROMPT,
            'response_mime_type': 'application/json',
        },
    )
    return response.text

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

def main():
    # --- 1. Get prompt from user ---
    print('\n=== ROS2 LLM Mission Control ===')
    print('Enter a mission command (e.g., "Patrol the perimeter twice at 0.3 m/s"):\n')
    user_prompt = input('> ')

    # --- 2. Call LLM ---
    print('\n[LLM] Sending to Gemini...')
    raw_json = call_gemini(user_prompt)
    print(f'[LLM] Response:\n{raw_json}\n')

    # --- 3. Validate JSON schema ---
    print('[VALIDATOR] Checking schema...')
    mission = validate_json_schema(raw_json)
    print(f'[VALIDATOR] Schema OK — {len(mission["waypoints"])} waypoints, '
          f'{mission["loop_count"]} loop(s)')
    save_mission(mission)
    
    # --- 4. Initialize ROS, seed AMCL, validate against costmap ---
    rclpy.init()
    node = rclpy.create_node('llm_bridge')

    map_meta = load_map_metadata(MAP_YAML)
    costmap_validator = CostmapValidator(node)

    nav_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    publish_initial_pose(node)

    node.get_logger().info('Waiting for Nav2...')
    nav_client.wait_for_server()

    node.get_logger().info('Waiting for costmap...')
    costmap_validator.wait_for_costmap(node)

    print('[VALIDATOR] Checking waypoints against costmap...')
    validate_waypoints(mission['waypoints'], map_meta, costmap_validator)
    print('[VALIDATOR] All waypoints safe.')

    # --- 5. Execute ---
    print('\n[EXECUTOR] Starting mission...\n')
    waypoints = mission['waypoints']
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
