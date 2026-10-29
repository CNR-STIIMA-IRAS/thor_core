#!/usr/bin/env python3
# BSD 3-Clause License
#
# Copyright (c) 2026, Nicola Pedrocchi
# All rights reserved.
#
# See the project license for redistribution terms and warranty disclaimer.

"""Goals for the robot: a pick-and-place loop, or a tool target dragged in RViz.

An interactive marker at the tool (tool0) can be dragged in RViz. Its menu
(right click) has:
  Go here     inverse kinematics of the marker position, with the tool
              orientation of the home pose, and a goal to the controller
  Start loop  a pick-and-place loop between two places on the table
  Stop loop   finish the current motion and stop
Dropping the marker also sends the goal when the loop is not running.

Parameters:
  robot_description  URDF of the robot
  action             FollowJointTrajectory action of the controller
  tool_frame         default tool0
  loop               start with the loop running (default true)
  speed              fraction of the velocity limits used to time the goals (default 0.4)
"""

import threading

import numpy as np
import pinocchio as pin
import rclpy
from control_msgs.action import FollowJointTrajectory
from interactive_markers import InteractiveMarkerServer, MenuHandler
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HOME = np.array([0.0, -1.2, 1.6, -1.97, -1.57, 0.0])
# Pick-and-place loop, as tool positions in the world frame [m].
PICK = np.array([0.75, -0.45, 0.95])
PLACE = np.array([0.45, 0.70, 0.95])
LIFT = np.array([0.0, 0.0, 0.25])


class RobotGoal(Node):

    def __init__(self):
        super().__init__('robot_goal')
        urdf = self.declare_parameter('robot_description', '').value
        action = self.declare_parameter('action', '/thor_controller/follow_joint_trajectory').value
        self.tool = self.declare_parameter('tool_frame', 'tool0').value
        self.speed = self.declare_parameter('speed', 0.4).value
        self.looping = self.declare_parameter('loop', True).value

        self.model = pin.buildModelFromXML(urdf)
        self.data = self.model.createData()
        self.tool_id = self.model.getFrameId(self.tool)
        self.q_index = [self.model.joints[self.model.getJointId(j)].idx_q for j in JOINTS]
        limits = getattr(self.model, 'upperVelocityLimit', None)
        if limits is None:
            limits = self.model.velocityLimit
        self.vmax = np.array([limits[i] for i in self.q_index])
        self.tool_rotation = self.forward(HOME).rotation

        self.lock = threading.Lock()
        self.q = None
        self.goal_handle = None
        self.loop_index = 0
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.client = ActionClient(self, FollowJointTrajectory, action)

        self.server = InteractiveMarkerServer(self, 'robot_goal')
        self.menu = MenuHandler()
        self.menu.insert('Go here', callback=lambda fb: self.go_to_marker())
        self.menu.insert('Start loop', callback=lambda fb: self.start_loop())
        self.menu.insert('Stop loop', callback=lambda fb: self.stop_loop())
        self.marker_pose = None
        self.create_marker(self.forward(HOME).translation)
        self.create_timer(0.5, self.tick)
        self.get_logger().info('Drag the tool marker in RViz; right click for Go here / Start loop / Stop loop')

    # --- kinematics -------------------------------------------------------

    def full_q(self, q):
        full = pin.neutral(self.model)
        for i, k in enumerate(self.q_index):
            full[k] = q[i]
        return full

    def forward(self, q):
        pin.framesForwardKinematics(self.model, self.data, self.full_q(q))
        return self.data.oMf[self.tool_id].copy()

    def inverse(self, position, q0):
        """Damped least squares on the tool pose; None if it does not converge."""
        target = pin.SE3(self.tool_rotation, np.asarray(position))
        q = self.full_q(q0)
        for _ in range(300):
            pin.framesForwardKinematics(self.model, self.data, q)
            error = pin.log6(self.data.oMf[self.tool_id].actInv(target)).vector
            if np.linalg.norm(error) < 1e-4:
                return np.array([q[k] for k in self.q_index])
            J = pin.computeFrameJacobian(self.model, self.data, q, self.tool_id, pin.LOCAL)
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), error)
            q = pin.integrate(self.model, q, 0.5 * dq)
            q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
        return None

    # --- goals --------------------------------------------------------------

    def on_joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(j in positions for j in JOINTS):
            with self.lock:
                self.q = np.array([positions[j] for j in JOINTS])

    def duration(self, q_from, q_to):
        return max(1.0, float(np.max(np.abs(q_to - q_from) / (self.speed * self.vmax))))

    def send(self, waypoints):
        """Sends a trajectory through joint waypoints, starting from the current state."""
        with self.lock:
            q = self.q
        if q is None or not self.client.server_is_ready():
            self.get_logger().warn('The controller is not ready')
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        t, previous = 0.0, q
        for w in waypoints:
            t += self.duration(previous, w)
            point = JointTrajectoryPoint()
            point.positions = w.tolist()
            point.velocities = [0.0] * len(JOINTS)
            point.time_from_start = Duration(seconds=t).to_msg()
            goal.trajectory.points.append(point)
            previous = w
        self.client.send_goal_async(goal).add_done_callback(self.on_accepted)
        return True

    def on_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_done(handle, f))

    def on_done(self, handle, future):
        if handle is not self.goal_handle:
            return  # preempted
        self.goal_handle = None
        result = future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().warn(f'Goal finished with error {result.error_code}: {result.error_string}')

    def loop_waypoints(self, q):
        # Above the pick, down, up, above the place, down, up.
        cartesian = [PICK + LIFT, PICK, PICK + LIFT] if self.loop_index % 2 == 0 else \
                    [PLACE + LIFT, PLACE, PLACE + LIFT]
        waypoints = []
        for p in cartesian:
            q = self.inverse(p, q)
            if q is None:
                return None
            waypoints.append(q)
        return waypoints

    def tick(self):
        if not self.looping or self.goal_handle is not None:
            return
        with self.lock:
            q = self.q
        if q is None:
            return
        waypoints = self.loop_waypoints(q)
        if waypoints is None:
            self.get_logger().error('No inverse kinematics for the loop: loop stopped')
            self.looping = False
            return
        if self.send(waypoints):
            self.loop_index += 1

    def start_loop(self):
        self.get_logger().info('Loop started')
        self.looping = True

    def stop_loop(self):
        self.get_logger().info('Loop stopped')
        self.looping = False

    def go_to_marker(self):
        if self.marker_pose is None:
            return
        with self.lock:
            q = self.q
        p = self.marker_pose
        target = self.inverse([p.x, p.y, p.z], q if q is not None else HOME)
        if target is None:
            self.get_logger().warn(f'No inverse kinematics for ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')
            return
        self.looping = False
        self.get_logger().info(f'Going to ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')
        self.send([target])

    # --- interactive marker --------------------------------------------------

    def create_marker(self, position):
        marker = InteractiveMarker()
        marker.header.frame_id = 'world'
        marker.name = 'tool_goal'
        marker.description = 'tool goal (right click)'
        marker.scale = 0.25
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, position)
        marker.pose.orientation.w = 1.0

        ball = Marker()
        ball.type = Marker.SPHERE
        ball.scale.x = ball.scale.y = ball.scale.z = 0.08
        ball.color.r, ball.color.g, ball.color.b, ball.color.a = 1.0, 0.55, 0.0, 0.8
        visual = InteractiveMarkerControl()
        visual.always_visible = True
        visual.interaction_mode = InteractiveMarkerControl.MENU
        visual.markers.append(ball)
        marker.controls.append(visual)

        h = 0.5 ** 0.5
        for name, (w, x, y, z) in (('x', (h, h, 0.0, 0.0)), ('y', (h, 0.0, 0.0, h)),
                                   ('z', (h, 0.0, h, 0.0))):
            control = InteractiveMarkerControl()
            control.name = 'move_' + name
            control.orientation.w, control.orientation.x = w, x
            control.orientation.y, control.orientation.z = y, z
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            marker.controls.append(control)

        self.marker_pose = marker.pose.position
        self.server.insert(marker, feedback_callback=self.on_feedback)
        self.menu.apply(self.server, marker.name)
        self.server.applyChanges()

    def on_feedback(self, feedback):
        self.marker_pose = feedback.pose.position
        if feedback.event_type == feedback.MOUSE_UP and not self.looping:
            self.go_to_marker()


def main():
    rclpy.init()
    node = RobotGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
