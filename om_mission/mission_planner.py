#!/usr/bin/env python3
"""
High-level mission planner for pick and place.

1. Waits for cube poses from om_perception (/cube_poses/red|green|blue)
2. Waits for /pick and /place action servers from om_motion
3. Executes the full mission: pick red → place red → pick green → ... blue

Place positions (target zone — 10 cm closer to robot than pick zone):
  red   → (0.10,  0.08, 0.115)
  green → (0.10,  0.00, 0.115)
  blue  → (0.10, -0.08, 0.115)
"""

import threading
import time

import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from om_motion.action import Pick, Place

# Cube order for the mission
CUBE_ORDER = ['red', 'green', 'blue']

# Place target positions (world frame)
PLACE_POSITIONS: dict[str, Pose] = {
    'red':   Pose(position=Point(x=0.10, y= 0.08, z=0.115),
                  orientation=Quaternion(w=1.0)),
    'green': Pose(position=Point(x=0.10, y= 0.00, z=0.115),
                  orientation=Quaternion(w=1.0)),
    'blue':  Pose(position=Point(x=0.10, y=-0.08, z=0.115),
                  orientation=Quaternion(w=1.0)),
}


class MissionPlanner(Node):

    def __init__(self):
        super().__init__('mission_planner')

        self._poses: dict[str, Pose | None] = {c: None for c in CUBE_ORDER}
        self._lock = threading.Lock()

        # Subscribe to perception topics
        for color in CUBE_ORDER:
            self.create_subscription(
                PoseStamped,
                f'/cube_poses/{color}',
                lambda msg, c=color: self._pose_cb(c, msg),
                10,
            )

        self._pick_client  = ActionClient(self, Pick,  'pick')
        self._place_client = ActionClient(self, Place, 'place')

        # Start mission in background thread
        threading.Thread(target=self._run_mission, daemon=True).start()
        self.get_logger().info('MissionPlanner started')

    # ------------------------------------------------------------------ callbacks

    def _pose_cb(self, color: str, msg: PoseStamped) -> None:
        with self._lock:
            self._poses[color] = msg.pose

    # ------------------------------------------------------------------ blocking action helpers

    def _send_pick(self, color: str, pose: Pose) -> bool:
        goal = Pick.Goal()
        goal.color = color
        goal.pick_pose = pose

        future = self._pick_client.send_goal_async(goal)
        while not future.done():
            time.sleep(0.05)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Pick {color}: goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        success = result_future.result().result.success
        msg = result_future.result().result.message
        self.get_logger().info(f'Pick {color}: {"OK" if success else "FAILED"} — {msg}')
        return success

    def _send_place(self, color: str, pose: Pose) -> bool:
        goal = Place.Goal()
        goal.color = color
        goal.place_pose = pose

        future = self._place_client.send_goal_async(goal)
        while not future.done():
            time.sleep(0.05)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Place {color}: goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        success = result_future.result().result.success
        msg = result_future.result().result.message
        self.get_logger().info(f'Place {color}: {"OK" if success else "FAILED"} — {msg}')
        return success

    # ------------------------------------------------------------------ mission

    def _run_mission(self) -> None:
        # 1. Wait for all cube poses
        self.get_logger().info('Waiting for cube poses from perception...')
        deadline = time.time() + 30.0
        while time.time() < deadline:
            with self._lock:
                if all(p is not None for p in self._poses.values()):
                    break
            time.sleep(0.5)
        else:
            self.get_logger().error('Timeout: could not get all cube poses. Aborting.')
            return

        self.get_logger().info('All cube poses received')

        # 2. Wait for action servers
        self.get_logger().info('Waiting for pick / place action servers...')
        self._pick_client.wait_for_server()
        self._place_client.wait_for_server()
        self.get_logger().info('Action servers ready — starting mission')

        # 3. Execute pick & place for each cube
        for color in CUBE_ORDER:
            with self._lock:
                pick_pose = self._poses[color]

            self.get_logger().info(f'--- Picking {color} cube ---')
            if not self._send_pick(color, pick_pose):
                self.get_logger().warn(f'Pick {color} failed — skipping place')
                continue

            self.get_logger().info(f'--- Placing {color} cube ---')
            self._send_place(color, PLACE_POSITIONS[color])

        self.get_logger().info('=== Mission complete ===')


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
