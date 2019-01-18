#!/usr/bin/env python
import sys, rospy
from pimouse_ros.msg import LightSensorValues


if __name__ == "__main__":
    rospy.init_node("lightsensors")
