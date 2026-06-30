import yaml
import math
from PIL import Image  # pip install Pillow
import os
from nav_msgs.msg import OccupancyGrid
import rclpy

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
