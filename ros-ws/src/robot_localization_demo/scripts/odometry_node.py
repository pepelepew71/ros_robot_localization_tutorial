#!/usr/bin/env python

from __future__ import print_function

import time

import rospy
from turtlesim.srv import SetPen
from turtlesim.srv import Spawn
from turtlesim.srv import TeleportAbsolute
from turtlesim.msg import Pose


class OdomNode():

    def __init__(self):
        rospy.init_node(name='odometry_node', anonymous=False)

        self.pose = None
        self.turtle_name = None

        rospy.Subscriber(name="/turtle1/pose", data_class=Pose, callback=self._callback_pose)

        self._spawn_once()

        rospy.spin()

    def _spawn_once(self):
        rospy.wait_for_service(service='/spawn')
        service_spawn = rospy.ServiceProxy(name='/spawn', service_class=Spawn)

        try:
            while not self.pose:  # waiting for turtle1 initial pose
                pass
            response = service_spawn(x=self.pose.x, y=self.pose.y, theta=self.pose.theta, name='')
            self.turtle_name = response.name
        except rospy.ServiceException as e:
            rospy.logerr("OdomNode._spawn_once() service_spawn failed")
        finally:
            rospy.loginfo("OdomNode spawn " + self.turtle_name)

        rospy.wait_for_service(service=self.turtle_name + '/set_pen')
        service_setpen = rospy.ServiceProxy(name=self.turtle_name + '/set_pen', service_class=SetPen)
        service_setpen(r=255, g=0, b=0, width=1, off=0)

    def _callback_pose(self, res):
        self.pose = res


if __name__ == "__main__":

    odom_node = OdomNode()
