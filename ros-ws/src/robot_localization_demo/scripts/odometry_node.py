#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import argparse
import random

import rospy
from turtlesim.srv import SetPen
from turtlesim.srv import Spawn
from turtlesim.srv import TeleportRelative
from turtlesim.msg import Pose
from geometry_msgs.msg import TwistWithCovarianceStamped


class OdomNode():

    def __init__(self, args):
        rospy.init_node(name='odometry_node', anonymous=False)

        self.args = args

        self.turtle1_pose = None
        self.publisher = None
        self.service_teleop_relative = None

        rospy.Subscriber(name="/turtle1/pose", data_class=Pose, callback=self._callback, queue_size=16)  # 62.5 Hz
        self.spawn_once()
        self.publisher = rospy.Publisher(name='/turtle1/sensors/twist', data_class=TwistWithCovarianceStamped, queue_size=16)

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
        service_setpen(r=255, g=0, b=0, width=1, off=0)

        name = name_visual_turtle + '/teleport_relative'
        rospy.wait_for_service(service=name)
        self.service_teleop_relative = rospy.ServiceProxy(name=name, service_class=TeleportRelative)

    def _callback(self, res):
        self.turtle1_pose = res

    def spin(self):
        rate = rospy.Rate(hz=self.args.frequency)  # dont need ros::spinOnce()
        frame_sequence = 0
        while not rospy.is_shutdown():
            vx = self.turtle1_pose.linear_velocity
            wz = self.turtle1_pose.angular_velocity
            vx *= (1.0 + random.gauss(mu=self.args.error_vx_systematic, sigma=self.args.error_vx_random))
            wz += vx*random.gauss(mu=self.args.error_wz_systematic, sigma=self.args.error_wz_random)
            current_twist = TwistWithCovarianceStamped()
            current_twist.header.seq = frame_sequence
            current_twist.header.stamp = rospy.Time.now()
            current_twist.header.frame_id = 'base_link'
            current_twist.twist.twist.linear.x = vx
            current_twist.twist.twist.linear.y = 0.0
            current_twist.twist.twist.linear.z = 0.0
            current_twist.twist.twist.angular.x = 0.0
            current_twist.twist.twist.angular.y = 0.0
            current_twist.twist.twist.angular.z = wz
            current_twist.twist.covariance[0] = (self.args.error_vx_systematic + self.args.error_vx_random)**2.0
            current_twist.twist.covariance[35] = (vx*(self.args.error_wz_systematic + self.args.error_wz_random))**2.0
            self.publisher.publish(current_twist)
            self._move_visual_turtle(vx=vx, wz=wz)
            rate.sleep()
            frame_sequence += 1

    def _move_visual_turtle(self, vx, wz):
        linear = vx / self.args.frequency
        angular = wz / self.args.frequency
        self.service_teleop_relative(linear=linear, angular=angular)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--frequency", help="set measurement frequency (Hz)", default=1.0, type=float)
    parser.add_argument("-X", "--error-vx-systematic", help="set systematic error on X velocity", default=0.0, type=float)
    parser.add_argument("-x", "--error-vx-random", help="set random error on X velocity", default=0.0, type=float)
    parser.add_argument("-T", "--error-wz-systematic", help="set systematic error on angular velocity", default=0.0, type=float)
    parser.add_argument("-t", "--error-wz-random", help="set random error on angular velocity", default=0.0, type=float)
    parser.add_argument("-v", "--visualize", help="visualize positioning system measurement", action='store_true')
    args, _ = parser.parse_known_args()

    odom_node = OdomNode(args=args)
