import yaml
import math
from PIL import Image  # pip install Pillow
import os
from nav_msgs.msg import OccupancyGrid
import rclpy
import json
import os
import jsonschema
from ament_index_python.packages import get_package_share_directory

def load_map_metadata(map_yaml_path: str) -> dict:
    with open(map_yaml_path, 'r') as f:
        meta = yaml.safe_load(f)
    
    # Load the .pgm to get pixel dimensions
    pgm_path = os.path.join(os.path.dirname(map_yaml_path), meta['image'])
    img = Image.open(pgm_path)
    width_px, height_px = img.size  # (cols, rows)

    resolution = meta['resolution']          # metres per pixel
    origin_x   = meta['origin'][0]           # map origin in world coords
    origin_y   = meta['origin'][1]

    # Compute world-space bounding box
    return {
        'resolution': resolution,
        'origin_x':   origin_x,
        'origin_y':   origin_y,
        'width_px':   width_px,
        'height_px':  height_px,
        # World extent: origin is bottom-left corner
        'world_min_x': origin_x,
        'world_max_x': origin_x + width_px  * resolution,
        'world_min_y': origin_y,
        'world_max_y': origin_y + height_px * resolution,
    }

def world_to_grid(x: float, y: float, meta: dict) -> tuple[int, int]:
    """Convert world (x, y) → (col, row) in the map grid."""
    col = int((x - meta['origin_x']) / meta['resolution'])
    row = int((y - meta['origin_y']) / meta['resolution'])
    return col, row

def is_within_map_bounds(x: float, y: float, meta: dict) -> bool:
    """Check coordinate is inside the map's real-world extent."""
    return (
        meta['world_min_x'] < x < meta['world_max_x'] and
        meta['world_min_y'] < y < meta['world_max_y']
    )

# Cost thresholds (Nav2 convention)
COST_FREE       = 0
COST_INFLATED   = 128   # near an obstacle
COST_INSCRIBED  = 253   # robot body would touch obstacle
COST_LETHAL     = 254   # obstacle cell itself
COST_UNKNOWN    = 255

class CostmapValidator:
    def __init__(self, node):
        self._costmap: OccupancyGrid | None = None
        self._sub = node.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self._costmap_callback,
            10
        )

    def _costmap_callback(self, msg: OccupancyGrid):
        self._costmap = msg  # cache the latest costmap

    def wait_for_costmap(self, node, timeout_sec=10.0):
        """Block until the first costmap arrives."""
        import time
        start = time.time()
        while self._costmap is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                raise RuntimeError('Timed out waiting for /global_costmap/costmap')

    def get_cost_at(self, x: float, y: float) -> int:
        """Return the costmap cost at world coordinate (x, y)."""
        cm = self._costmap
        info = cm.info

        # World → costmap grid cell
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)

        # Bounds check against costmap dimensions (may differ from static map)
        if not (0 <= col < info.width and 0 <= row < info.height):
            raise ValueError(f'({x}, {y}) is outside costmap bounds.')

        # OccupancyGrid data is row-major
        index = row * info.width + col
        return cm.data[index]

    def is_safe(self, x: float, y: float, threshold=COST_INSCRIBED) -> bool:
        """Returns True only if the cell cost is below the inscribed threshold."""
        cost = self.get_cost_at(x, y)
        return cost < threshold and cost != COST_UNKNOWN

def validate_waypoints(waypoints: list, map_meta: dict, costmap_validator: CostmapValidator):
    """
    Raises ValueError with a clear message if any waypoint is invalid.
    Returns the list unchanged if all pass.
    """
    for i, wp in enumerate(waypoints):
        x, y = wp['x'], wp['y']
        label = f'Waypoint {i+1} ({x}, {y})'

        # Layer 1: static map bounds
        if not is_within_map_bounds(x, y, map_meta):
            raise ValueError(
                f'{label} is outside the map. '
                f'Map bounds: x=[{map_meta["world_min_x"]:.2f}, {map_meta["world_max_x"]:.2f}] '
                f'y=[{map_meta["world_min_y"]:.2f}, {map_meta["world_max_y"]:.2f}]'
            )

        # Layer 2: costmap occupancy
        if not costmap_validator.is_safe(x, y):
            cost = costmap_validator.get_cost_at(x, y)
            raise ValueError(
                f'{label} is unsafe — costmap cost={cost}. '
                f'Cell is {"unknown" if cost == 255 else "occupied or too close to obstacle"}.'
            )

    return waypoints  # all clear

def validate_json_schema(raw_json: str) -> dict:
    """
    Parse and validate the raw JSON string from the LLM.
    Applies defaults and resolves return_to_start.
    Returns a fully resolved mission dict, or raises ValueError.
    """

    # --- Step 1: Parse JSON ---
    try:
        mission = json.loads(raw_json)
    except json.JSONDecodeError as e:
        raise ValueError(f'LLM returned invalid JSON: {e}')

    # --- Step 2: Load schema ---
    # schema_path = os.path.join(os.path.dirname(__file__), 'schemas', 'mission_schema.json')
    # schema_path = '/home/shubham/omokai_ws/src/inspector_llm/inspector_llm/schemas/mission_schema.json'
    schema_path = os.path.join(
        get_package_share_directory('inspector_llm'),
        'schemas',
        'mission_schema.json'
    )
    with open(schema_path, 'r') as f:
        schema = json.load(f)

    # --- Step 3: Validate structure against schema ---
    try:
        jsonschema.validate(instance=mission, schema=schema)
    except jsonschema.ValidationError as e:
        # e.path gives the JSON path to the bad field, e.message says what's wrong
        field_path = ' -> '.join(str(p) for p in e.absolute_path) or 'root'
        raise ValueError(f'Schema validation failed at [{field_path}]: {e.message}')

    # --- Step 4: Apply defaults for optional top-level fields ---
    mission.setdefault('mission_name', 'unnamed_mission')
    mission.setdefault('frame_id',        'map')
    mission.setdefault('loop_count',      1)
    mission.setdefault('return_to_start', False)
    mission.setdefault('max_speed',       0.3)
    mission.setdefault('stop_on_failure', False)
    mission.setdefault('description',     '')

    # Apply per-waypoint defaults
    for wp in mission['waypoints']:
        wp.setdefault('yaw',   0.0)
        wp.setdefault('label', '')
        wp.setdefault('tasks', [])

    # --- Step 5: Resolve return_to_start ---
    # Append a copy of the first waypoint to close the loop
    if mission['return_to_start'] and len(mission['waypoints']) > 1:
        first = mission['waypoints'][0].copy()
        first['label'] = first.get('label', '') + '_return'
        mission['waypoints'].append(first)

    return mission

if __name__ == '__main__':
    import sys

    # Usage: python3 mission_validator.py missions/test_valid.json
    json_file = sys.argv[1] if len(sys.argv) > 1 else 'missions/test_valid.json'

    with open(json_file, 'r') as f:
        raw = f.read()

    print(f'Testing: {json_file}')

    # Step 1: Schema validation (no ROS needed)
    try:
        mission = validate_json_schema(raw)
        print(f'✅ Schema valid — {len(mission["waypoints"])} waypoints, '
              f'{mission["loop_count"]} loop(s), return_to_start={mission["return_to_start"]}')
    except ValueError as e:
        print(f'❌ Schema FAILED: {e}')
        sys.exit(1)

    # Step 2: Map bounds check (no ROS needed)
    MAP_YAML = os.path.join(
    get_package_share_directory('inspector_bot'),
    'maps',
    'warehouse_map.yaml'
    )
    try:
        meta = load_map_metadata(MAP_YAML)
        for i, wp in enumerate(mission['waypoints']):
            if not is_within_map_bounds(wp['x'], wp['y'], meta):
                print(f'❌ Waypoint {i+1} ({wp["x"]}, {wp["y"]}) is out of bounds')
                sys.exit(1)
        print('✅ All waypoints within map bounds')
    except Exception as e:
        print(f'❌ Bounds check error: {e}')
        sys.exit(1)

    print('Skipping costmap check (requires live Nav2). Run via ros2 run for full validation.')

