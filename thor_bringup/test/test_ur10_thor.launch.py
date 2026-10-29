# BSD 3-Clause License
#
# Copyright (c) 2026, Nicola Pedrocchi
# All rights reserved.
#
# See the project license for redistribution terms and warranty disclaimer.

"""Integration test: UR10 with mock hardware and the THOR controller.

Run with `colcon test --packages-select thor_bringup`, or directly with
`launch_test test/test_ur10_thor.launch.py [chained:=true]`: the same test
runs with ThorTrajectoryController and with joint_trajectory_controller
chained to ThorChainableController.
"""

import math
import threading
import time
import unittest

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from control_msgs.action import FollowJointTrajectory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from thor_msgs.msg import Obstacle, ObstacleArray, ThorControllerState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HOME = [0.0, -1.2, 1.6, -1.97, -1.57, 0.0]
# The tool moves on a circle of this radius around the base, at this height.
TOOL_RADIUS = 0.88
TOOL_ANGLE_AT_HOME = math.atan2(0.164, 0.865)


@pytest.mark.launch_test
def generate_test_description():
    launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('thor_bringup'), 'launch', 'ur10_thor.launch.py']),
        launch_arguments={'rviz': 'false', 'demo': 'false',
                          'chained': LaunchConfiguration('chained')}.items())
    return LaunchDescription([
        DeclareLaunchArgument('chained', default_value='false'),
        launch,
        launch_testing.actions.ReadyToTest(),
    ])


class TestThorController(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_thor_controller')
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.lock = threading.Lock()
        self.states = []
        self.joint_state = None
        self.node.create_subscription(
            ThorControllerState, '/thor_controller/thor_state', self._on_state, 100)
        self.node.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.obstacle_pub = self.node.create_publisher(
            ObstacleArray, '/thor_controller/obstacles', 10)
        self.client = ActionClient(
            self.node, FollowJointTrajectory, '/thor_controller/follow_joint_trajectory')
        self.assertTrue(self.client.wait_for_server(timeout_sec=30.0),
                        'the action server of thor_controller is not available')
        self._wait_for(lambda: self.joint_state is not None, 10.0, 'no joint states')

    def tearDown(self):
        self.executor.shutdown()
        self.node.destroy_node()

    def _on_state(self, msg):
        with self.lock:
            self.states.append(msg)

    def _on_joints(self, msg):
        self.joint_state = dict(zip(msg.name, msg.position))

    def _wait_for(self, condition, timeout, message):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if condition():
                return
            time.sleep(0.05)
        self.fail(message)

    def _move(self, positions, duration, timeout):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(JOINTS)
        point.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points = [point]

        future = self.client.send_goal_async(goal)
        self._wait_for(future.done, 5.0, 'the goal was not answered')
        handle = future.result()
        self.assertTrue(handle.accepted, 'the goal was rejected')
        result = handle.get_result_async()
        self._wait_for(result.done, timeout, 'the goal did not finish in time')
        return result.result().result

    def _person(self, x, y):
        msg = ObstacleArray()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        body = Obstacle()
        body.position.x, body.position.y, body.position.z = x, y, 0.9
        body.half_axis.z = 0.55
        body.radius = 0.2
        msg.obstacles = [body]
        return msg

    def test_1_follows_trajectory_without_obstacles(self):
        target = list(HOME)
        target[0] = 1.0
        result = self._move(target, 2.0, timeout=10.0)
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL,
                         result.error_string)
        time.sleep(0.2)
        for name, q in zip(JOINTS, target):
            self.assertAlmostEqual(self.joint_state[name], q, delta=1e-3, msg=name)

    def test_2_keeps_distance_from_a_person_on_the_path(self):
        # From pan 1.0 (end of the previous test) back to pan 0. A person
        # stands on the path of the tool at pan 0.5 for 3 s, then walks away.
        angle = TOOL_ANGLE_AT_HOME + 0.5
        x0, y0 = 1.0 * math.cos(angle), 1.0 * math.sin(angle)
        stop = threading.Event()

        def publish_person():
            start = time.monotonic()
            while not stop.is_set():
                t = time.monotonic() - start
                away = max(0.0, t - 3.0) * 1.0  # walks away radially at 1 m/s
                r = 1.0 + away
                self.obstacle_pub.publish(self._person(x0 * r, y0 * r))
                time.sleep(0.02)

        publisher = threading.Thread(target=publish_person, daemon=True)
        publisher.start()
        try:
            time.sleep(0.3)
            with self.lock:
                self.states.clear()
            result = self._move(HOME, 2.0, timeout=20.0)
        finally:
            stop.set()
            publisher.join()

        self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL,
                         result.error_string)
        with self.lock:
            states = [s for s in self.states if s.n_obstacles > 0]
        self.assertGreater(len(states), 100)
        h_min = min(s.h_now for s in states)
        distance_min = min(s.min_distance for s in states)
        min_scaling = min(s.scaling for s in states)
        deviation = max(
            max(abs(c - r) for c, r in zip(s.command_positions, s.reference_positions))
            for s in states)
        print(f'min h {h_min:.4f} m, min distance {distance_min:.3f} m, min scaling {min_scaling:.3f}, '
              f'max deviation from the reference {deviation:.3f} rad')
        # The discrete-time barrier is linearized along the horizon: h can go
        # a few millimeters below zero, while the surfaces stay apart by about
        # the safety margin (0.05 m).
        self.assertGreaterEqual(h_min, -0.005, 'the robot got closer than the barrier allows')
        self.assertGreater(distance_min, 0.04, 'the robot got closer than the safety margin')
        self.assertTrue(min_scaling < 0.9 or deviation > 0.05,
                        'the robot neither slowed down nor moved around the person')
        self.assertTrue(all(s.status == ThorControllerState.STATUS_OK for s in states))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        # ros2_control_node may be stopped by a signal at shutdown.
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
