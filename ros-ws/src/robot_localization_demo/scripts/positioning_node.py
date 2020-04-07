#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import argparse
import random

import rospy
from turtlesim.srv import SetPen
from turtlesim.srv import Spawn
from turtlesim.srv import TeleportAbsolute
from turtlesim.msg import Pose
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped


class PositionNode():

    def __init__(self, args):
        rospy.init_node(name='positioning_node', anonymous=False)

        self.args = args

        self.turtle1_pose = None
        self.publisher = None
        self.service_teleop_absolute = None

        rospy.Subscriber(name="/turtle1/pose", data_class=Pose, callback=self._callback, queue_size=16)  # 62.5 Hz
        self.spawn_once()
        self.publisher = rospy.Publisher(name='/turtle1/sensor/pose', data_class=PoseWithCovarianceStamped, queue_size=16)

        self.spin()

    def spawn_once(self):
        rospy.wait_for_service(service='/spawn')
        service_spawn = rospy.ServiceProxy(name='/spawn', service_class=Spawn)

        try:
            while not self.turtle1_pose:  # waiting for turtle1 initial pose
                pass
            response = service_spawn(x=self.turtle1_pose.x, y=self.turtle1_pose.y, theta=self.turtle1_pose.theta, name='')
            name_visual_turtle = response.name
        except rospy.ServiceException as e:
            rospy.logerr("OdomNode.spawn_once() service_spawn failed")
        finally:
            rospy.loginfo("OdomNode spawn " + name_visual_turtle)

        name = name_visual_turtle + '/set_pen'
        rospy.wait_for_service(service=name)
        service_setpen = rospy.ServiceProxy(name=name, service_class=SetPen)
        service_setpen(r=0, g=0, b=255, width=1, off=0)

        name = name_visual_turtle + '/teleport_absolute'
        rospy.wait_for_service(service=name)
        self.service_teleop_absolute = rospy.ServiceProxy(name=name, service_class=TeleportAbsolute)

    def _callback(self, res):
        self.turtle1_pose = res

    def spin(self):
        rate = rospy.Rate(hz=self.args.frequency)
        frame_sequence = 0
        while not rospy.is_shutdown():
            x = self.turtle1_pose.x
            y = self.turtle1_pose.y
            theta = self.turtle1_pose.theta  # radian
            x += random.gauss(mu=self.args.error_x_systematic, sigma=self.args.error_x_random)
            y += random.gauss(mu=self.args.error_y_systematic, sigma=self.args.error_y_random)
            theta += random.gauss(mu=self.args.error_yaw_systematic, sigma=self.args.error_yaw_random)
            q = quaternion_from_euler(0.0, 0.0, theta)
            current_pose = PoseWithCovarianceStamped()
            current_pose.header.seq = frame_sequence
            current_pose.header.stamp = rospy.Time.now()
            current_pose.header.frame_id = 'map'
            current_pose.pose.pose.position.x = x
            current_pose.pose.pose.position.y = y
            current_pose.pose.pose.position.z = 0.0
            current_pose.pose.pose.orientation.x = q[0]
            current_pose.pose.pose.orientation.y = q[1]
            current_pose.pose.pose.orientation.z = q[2]
            current_pose.pose.pose.orientation.w = q[3]
            current_pose.pose.covariance[0] = (self.args.error_x_systematic + self.args.error_x_random)**2.0
            current_pose.pose.covariance[7] = (self.args.error_y_systematic + self.args.error_y_random)**2.0
            current_pose.pose.covariance[35] = (self.args.error_yaw_systematic + self.args.error_yaw_random)**2.0
            self.publisher.publish(current_pose)
            self._move_visual_turtle(x=x, y=y, theta=theta)
            rate.sleep()
            frame_sequence += 1

    def _move_visual_turtle(self, x, y, theta):
        self.service_teleop_absolute(x=x, y=y, theta=theta)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--frequency", help="set measurement frequency (Hz)", default=1.0, type=float)
    parser.add_argument("-X", "--error-x-systematic", help="set systematic error on X", default=0.0, type=float)
    parser.add_argument("-x", "--error-x-random", help="set random error on X", default=0.0, type=float)
    parser.add_argument("-Y", "--error-y-systematic", help="set systematic error on Y", default=0.0, type=float)
    parser.add_argument("-y", "--error-y-random", help="set random error on Y", default=0.0, type=float)
    parser.add_argument("-T", "--error-yaw-systematic", help="set systematic error on yaw", default=0.0, type=float)
    parser.add_argument("-t", "--error-yaw-random", help="set random error on yaw", default=0.0, type=float)
    parser.add_argument("-v", "--visualize", help="visualize positioning system measurement", action='store_true')
    args, _ = parser.parse_known_args()

    position_node = PositionNode(args=args)
