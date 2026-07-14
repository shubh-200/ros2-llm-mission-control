"""
Frontier-based exploration for SLAM mode.

Subscribes to the live /map topic (OccupancyGrid published by SLAM Toolbox),
detects frontier cells (free cells adjacent to unknown space), clusters them,
and returns centroid waypoints for the robot to explore.
"""

import numpy as np
from scipy import ndimage
from nav_msgs.msg import OccupancyGrid
import rclpy


# --- OccupancyGrid cell values ---
CELL_UNKNOWN = -1
CELL_FREE = 0
# Anything > 0 is occupied (higher = more certain)
CELL_OCCUPIED_THRESH = 50

# --- Frontier filtering ---
# These are BASE values — actual thresholds adapt based on map size.
# Early SLAM maps are small and fragmented, so we relax filters to bootstrap.
MIN_FRONTIER_SIZE_BASE = 5     # Min cells — reduced further for small maps
MIN_DISTANCE_BASE = 1.5        # Min distance — reduced for small maps
MAX_DISTANCE_FROM_ROBOT = 25.0 # metres — skip frontiers too far away


class FrontierExplorer:
    """Detects frontiers in a live OccupancyGrid and returns exploration waypoints."""

    def __init__(self, node):
        self._node = node
        self._map_msg: OccupancyGrid | None = None

        self._sub = node.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            10
        )

    def _map_callback(self, msg: OccupancyGrid):
        self._map_msg = msg

    def wait_for_map(self, node, timeout_sec=30.0):
        """Block until the first /map message arrives."""
        import time
        start = time.time()
        while self._map_msg is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                raise RuntimeError('Timed out waiting for /map from SLAM Toolbox')
        node.get_logger().info('Map received from SLAM Toolbox.')

    def get_frontiers(self, robot_x: float = 0.0, robot_y: float = 0.0,
                      max_count: int = 5) -> list[tuple[float, float]]:
        """
        Find frontier waypoints in the current map.

        Returns a list of (x, y) world coordinates, sorted by exploration
        value (largest frontiers preferred, with distance as tiebreaker),
        capped at max_count.
        """
        if self._map_msg is None:
            return []

        # Spin a few times to get the latest map update
        for _ in range(5):
            rclpy.spin_once(self._node, timeout_sec=0.05)

        msg = self._map_msg
        info = msg.info
        width = info.width
        height = info.height
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        # --- 1. Reshape to 2D grid ---
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # --- DEBUG: Log map composition ---
        total_cells = width * height
        num_unknown = int(np.sum(grid == CELL_UNKNOWN))
        num_free = int(np.sum((grid >= 0) & (grid <= 10)))
        num_occupied = int(np.sum(grid > CELL_OCCUPIED_THRESH))
        self._node.get_logger().info(
            f'Map: {width}x{height} ({total_cells} cells) — '
            f'free={num_free}, unknown={num_unknown}, occupied={num_occupied}'
        )

        if num_unknown == 0:
            self._node.get_logger().warn(
                'Map has ZERO unknown cells — SLAM may not be publishing unknown regions. '
                'Check that slam_toolbox is running in mapping mode.'
            )
            return []

        # --- Adaptive thresholds based on map maturity ---
        # Early SLAM: tiny free area, fragmented frontiers → relax filters
        # Mature SLAM: large free area, well-defined frontiers → tighten filters
        if num_free < 2000:
            # Bootstrap phase — accept anything
            min_frontier_size = 1
            min_distance = 0.3
            self._node.get_logger().info(
                f'Bootstrap mode (free={num_free}<2000): min_size=1, min_dist=0.3m')
        elif num_free < 10000:
            # Growing phase
            min_frontier_size = 3
            min_distance = 0.8
        else:
            # Mature map
            min_frontier_size = MIN_FRONTIER_SIZE_BASE
            min_distance = MIN_DISTANCE_BASE

        # --- 2. Create masks ---
        # Free: cells with occupancy probability 0-10 (navigable)
        free_mask = (grid >= CELL_FREE) & (grid <= 10)
        # Unknown: cells with value -1
        unknown_mask = (grid == CELL_UNKNOWN)

        # --- 3. Find frontier cells ---
        # A frontier cell is a FREE cell that has at least one UNKNOWN neighbor.
        # Use 8-connectivity to catch diagonal frontiers too.
        struct_8conn = ndimage.generate_binary_structure(2, 2)  # 8-connected
        unknown_dilated = ndimage.binary_dilation(unknown_mask, structure=struct_8conn)
        frontier_mask = free_mask & unknown_dilated

        num_frontier_cells = int(np.sum(frontier_mask))
        self._node.get_logger().info(f'Raw frontier cells: {num_frontier_cells}')

        if num_frontier_cells == 0:
            self._node.get_logger().info('No frontier cells found.')
            return []

        # --- 4. Cluster frontiers ---
        labeled, num_features = ndimage.label(frontier_mask, structure=struct_8conn)
        self._node.get_logger().info(f'Frontier clusters: {num_features}')

        # --- 5. Compute centroids and filter ---
        waypoints = []
        filtered_reasons = {'too_small': 0, 'too_close': 0, 'too_far': 0, 'occupied': 0}

        for label_id in range(1, num_features + 1):
            cluster_mask = (labeled == label_id)
            cluster_size = int(np.sum(cluster_mask))

            # Skip tiny clusters (noise)
            if cluster_size < min_frontier_size:
                filtered_reasons['too_small'] += 1
                continue

            # Centroid in grid coordinates (row, col)
            rows, cols = np.where(cluster_mask)
            centroid_row = np.mean(rows)
            centroid_col = np.mean(cols)

            # Convert grid → world coordinates
            world_x = origin_x + centroid_col * resolution
            world_y = origin_y + centroid_row * resolution

            # Distance from robot
            dist = np.sqrt((world_x - robot_x) ** 2 + (world_y - robot_y) ** 2)

            # Filter by distance
            if dist < min_distance:
                filtered_reasons['too_close'] += 1
                continue
            if dist > MAX_DISTANCE_FROM_ROBOT:
                filtered_reasons['too_far'] += 1
                continue

            # Check that the centroid itself is not in an occupied cell
            c_row = int(centroid_row)
            c_col = int(centroid_col)
            if 0 <= c_row < height and 0 <= c_col < width:
                if grid[c_row, c_col] > CELL_OCCUPIED_THRESH:
                    filtered_reasons['occupied'] += 1
                    continue

            waypoints.append((world_x, world_y, dist, cluster_size))

        # --- DEBUG: Log filtering breakdown ---
        self._node.get_logger().info(
            f'Clusters: {num_features} total, {len(waypoints)} passed filters. '
            f'Filtered out: {filtered_reasons}'
        )

        # --- 6. Sort by exploration value ---
        # Primary: largest frontier (most unexplored area). Tiebreaker: closest.
        waypoints.sort(key=lambda w: (-w[3], w[2]))

        # Return top N as (x, y) tuples
        result = [(w[0], w[1]) for w in waypoints[:max_count]]

        self._node.get_logger().info(
            f'Returning {len(result)} frontiers: '
            f'{[(f"{x:.2f}", f"{y:.2f}") for x, y in result]}'
        )

        return result
