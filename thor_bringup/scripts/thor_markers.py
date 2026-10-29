#!/usr/bin/env python3
# BSD 3-Clause License
#
# Copyright (c) 2026, Nicola Pedrocchi
# All rights reserved.
#
# See the project license for redistribution terms and warranty disclaimer.

"""Markers of a THOR controller for RViz.

The robot spheres of the controller, colored by their distance from the
obstacles (green far, red at the safety margin), and a text with the time
scaling, the barrier and the distance of the controller state.

Parameters:
  controllers_file  YAML file with the controller parameters
  controller        name of the controller in that file (default thor_controller)
  rate              [Hz] (default 20)
"""

import math

import numpy as np
import rclpy
import yaml
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from thor_msgs.msg import ObstacleArray, ThorControllerState
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class ThorMarkers(Node):

    def __init__(self):
        super().__init__('thor_markers')
        path = self.declare_parameter('controllers_file', '').value
        controller = self.declare_parameter('controller', 'thor_controller').value
        rate = self.declare_parameter('rate', 20.0).value

        with open(path) as f:
            thor = yaml.safe_load(f)[controller]['ros__parameters']['thor']
        spheres = thor['robot_spheres']
        self.spheres = [(n, spheres[n]['frame'], np.array(spheres[n].get('offset', [0, 0, 0])),
                         spheres[n].get('radius', 0.0)) for n in spheres['names']]
        self.margin = thor.get('cbf', {}).get('safety_margin', 0.05)
        self.frame = thor.get('obstacles', {}).get('frame_id', '') or 'world'
        topic = thor.get('obstacles', {}).get('topic', '~/obstacles').replace('~', '/' + controller)
        state_topic = thor.get('state_topic', '~/thor_state').replace('~', '/' + controller)

        self.obstacles = []
        self.state = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(ObstacleArray, topic, self.on_obstacles, 10)
        self.create_subscription(
            ThorControllerState, state_topic, self.on_state, 10)
        self.pub = self.create_publisher(MarkerArray, '~/markers', 10)
        self.create_timer(1.0 / rate, self.publish)

    def on_obstacles(self, msg):
        self.obstacles = [(np.array([o.position.x, o.position.y, o.position.z]),
                           np.array([o.half_axis.x, o.half_axis.y, o.half_axis.z]), o.radius)
                          for o in msg.obstacles]

    def on_state(self, msg):
        self.state = msg

    def distance(self, center, radius):
        best = math.inf
        for p, axis, r in self.obstacles:
            closest = p
            n = axis @ axis
            if n > 0.0:
                closest = p + np.clip((center - p) @ axis / n, -1.0, 1.0) * axis
            best = min(best, np.linalg.norm(center - closest) - radius - r)
        return best

    def publish(self):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        top = None
        for k, (name, frame, offset, radius) in enumerate(self.spheres):
            try:
                t = self.tf.lookup_transform(self.frame, frame, Time(), Duration(seconds=0.0))
            except TransformException:
                continue
            tr = t.transform.translation
            center = np.array([tr.x, tr.y, tr.z]) + quaternion_matrix(t.transform.rotation) @ offset
            top = center if top is None or center[2] > top[2] else top
            # Green beyond 0.5 m from the margin, red at the margin.
            closeness = 1.0 - min(1.0, max(0.0, (self.distance(center, radius) - self.margin) / 0.5))
            m = Marker()
            m.header.frame_id = self.frame
            m.header.stamp = now
            m.ns = 'robot_spheres'
            m.id = k
            m.type = Marker.SPHERE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = center
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 2.0 * radius
            m.color.r, m.color.g, m.color.b, m.color.a = closeness, 1.0 - closeness, 0.1, 0.35
            markers.markers.append(m)

        if self.state is not None and top is not None:
            s = self.state
            text = Marker()
            text.header.frame_id = self.frame
            text.header.stamp = now
            text.ns = 'thor_state'
            text.type = Marker.TEXT_VIEW_FACING
            text.pose.position.z = 2.0
            text.pose.orientation.w = 1.0
            text.scale.z = 0.08
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            status = 'BRAKING' if s.status == ThorControllerState.STATUS_BRAKING else ''
            barrier = f'h {s.h_now:.2f} m   distance {s.min_distance:.2f} m' if s.n_obstacles else 'no obstacles'
            text.text = f'scaling {s.scaling:.2f}   {barrier}   {status}'
            markers.markers.append(text)
        self.pub.publish(markers)


def main():
    rclpy.init()
    node = ThorMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
