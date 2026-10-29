#!/usr/bin/env python3
# BSD 3-Clause License
#
# Copyright (c) 2026, Nicola Pedrocchi
# All rights reserved.
#
# See the project license for redistribution terms and warranty disclaimer.

"""A simulated person that walks to the goals given with "2D Goal Pose" in RViz.

The person is a capsule for the body, a sphere for the head and one for each
hand. They are published as obstacles for the THOR controller
(thor_msgs/ObstacleArray, with velocity and acceleration) and as markers.

Parameters:
  obstacles_topic  topic of the obstacles (default /thor_controller/obstacles)
  goal_topic       topic of the goals (default /goal_pose)
  frame_id         frame of the floor (default world)
  start            initial position [x, y] (default [1.8, -1.2])
  max_speed        walking speed [m/s] (default 1.0)
  max_acceleration [m/s^2] (default 1.5)
  rate             publishing rate [Hz] (default 50)
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from thor_msgs.msg import Obstacle, ObstacleArray
from visualization_msgs.msg import Marker, MarkerArray

# Body parts: (name, offset [forward, left, up] from the feet, radius, half axis along z).
BODY = ('body', (0.0, 0.0, 0.9), 0.20, 0.50)
HEAD = ('head', (0.0, 0.0, 1.62), 0.12, 0.0)
LEFT_HAND = ('left_hand', (0.05, 0.30, 0.95), 0.08, 0.0)
RIGHT_HAND = ('right_hand', (0.05, -0.30, 0.95), 0.08, 0.0)
PARTS = (BODY, HEAD, LEFT_HAND, RIGHT_HAND)

SKIN = (0.93, 0.76, 0.62)
CLOTHES = (0.20, 0.35, 0.65)


class HumanSim(Node):

    def __init__(self):
        super().__init__('human_sim')
        self.frame_id = self.declare_parameter('frame_id', 'world').value
        start = self.declare_parameter('start', [1.8, -1.2]).value
        self.max_speed = self.declare_parameter('max_speed', 1.0).value
        self.max_acc = self.declare_parameter('max_acceleration', 1.5).value
        rate = self.declare_parameter('rate', 50.0).value
        obstacles_topic = self.declare_parameter('obstacles_topic', '/thor_controller/obstacles').value
        goal_topic = self.declare_parameter('goal_topic', '/goal_pose').value

        self.pos = list(start)
        self.vel = [0.0, 0.0]
        self.acc = [0.0, 0.0]
        self.yaw = math.atan2(-start[1], -start[0])  # facing the robot
        self.goal = None
        self.goal_yaw = None
        self.dt = 1.0 / rate

        self.obstacle_pub = self.create_publisher(ObstacleArray, obstacles_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', 10)
        self.create_subscription(PoseStamped, goal_topic, self.on_goal, 10)
        self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f'Person at ({start[0]:.2f}, {start[1]:.2f}): give it a goal with "2D Goal Pose" '
            f'({goal_topic})')

    def on_goal(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self.get_logger().warn(f'Goal in frame {msg.header.frame_id} ignored: expected {self.frame_id}')
            return
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        q = msg.pose.orientation
        self.goal_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.get_logger().info(f'Walking to ({self.goal[0]:.2f}, {self.goal[1]:.2f})')

    def step(self):
        # Velocity towards the goal, limited so that the person can stop there.
        desired = [0.0, 0.0]
        if self.goal is not None:
            dx, dy = self.goal[0] - self.pos[0], self.goal[1] - self.pos[1]
            dist = math.hypot(dx, dy)
            if dist > 0.01:
                speed = min(self.max_speed, math.sqrt(2.0 * self.max_acc * dist))
                desired = [speed * dx / dist, speed * dy / dist]
            else:
                self.goal = None
        dv = [desired[0] - self.vel[0], desired[1] - self.vel[1]]
        norm = math.hypot(*dv)
        limit = self.max_acc * self.dt
        if norm > limit:
            dv = [dv[0] * limit / norm, dv[1] * limit / norm]
        self.acc = [dv[0] / self.dt, dv[1] / self.dt]
        self.vel = [self.vel[0] + dv[0], self.vel[1] + dv[1]]
        self.pos = [self.pos[0] + self.vel[0] * self.dt, self.pos[1] + self.vel[1] * self.dt]

        # Face the direction of motion, then the orientation of the goal.
        if math.hypot(*self.vel) > 0.1:
            target_yaw = math.atan2(self.vel[1], self.vel[0])
        elif self.goal is None and self.goal_yaw is not None:
            target_yaw = self.goal_yaw
        else:
            target_yaw = self.yaw
        error = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
        self.yaw += max(-3.0 * self.dt, min(3.0 * self.dt, error))

        self.publish()

    def part_position(self, offset):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (self.pos[0] + c * offset[0] - s * offset[1],
                self.pos[1] + s * offset[0] + c * offset[1],
                offset[2])

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        obstacles = ObstacleArray()
        obstacles.header.stamp = stamp
        obstacles.header.frame_id = self.frame_id
        markers = MarkerArray()
        for k, (name, offset, radius, half_axis) in enumerate(PARTS):
            p = self.part_position(offset)
            o = Obstacle()
            o.position.x, o.position.y, o.position.z = p
            o.velocity.x, o.velocity.y = self.vel
            o.acceleration.x, o.acceleration.y = self.acc
            o.radius = radius
            o.half_axis.z = half_axis
            obstacles.obstacles.append(o)
            color = CLOTHES if name == 'body' else SKIN
            markers.markers.extend(self.part_markers(k, p, radius, half_axis, color, stamp))
        self.obstacle_pub.publish(obstacles)
        self.marker_pub.publish(markers)

    def part_markers(self, k, p, radius, half_axis, color, stamp):
        def marker(mid, kind, z, scale):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.ns = 'person'
            m.id = mid
            m.type = kind
            m.pose.position.x, m.pose.position.y, m.pose.position.z = p[0], p[1], z
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 1.0
            return m

        d = 2.0 * radius
        if half_axis == 0.0:
            return [marker(10 * k, Marker.SPHERE, p[2], (d, d, d))]
        return [marker(10 * k, Marker.CYLINDER, p[2], (d, d, 2.0 * half_axis)),
                marker(10 * k + 1, Marker.SPHERE, p[2] + half_axis, (d, d, d)),
                marker(10 * k + 2, Marker.SPHERE, p[2] - half_axis, (d, d, d))]


def main():
    rclpy.init()
    node = HumanSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
