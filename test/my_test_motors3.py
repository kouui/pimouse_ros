#!/usr/bin/env python
#encoding: utf8

import rospy, rosnode, unittest, rostest
import time
from pimouse_ros.msg import MotorFreqs
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.srv import TimedMotion

class MotorTest(unittest.TestCase):

    def setUp(self):
        rospy.wait_for_service("/motor_on")
        rospy.wait_for_service("/motor_off")
        rospy.wait_for_service("/timed_motion")
        on = rospy.ServiceProxy("/motor_on", Trigger)
        ret = on()

    def file_check(self, dev, value, message): # does't work for /dev/rt*
        with open("/dev/"+dev, 'r') as file:
            self.assertEqual(file.readline(), str(value)+'\n', message)

    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn("/motors", nodes, "node does not exist")

    def test_put_freq(self):
        pub = rospy.Publisher("/motor_raw", MotorFreqs)
        m = MotorFreqs()
        m.left_hz = 123
        m.right_hz = 456
        pub.publish(m)
        time.sleep(2)

        # self.file_check("rtmotor_raw_l0", m.left_hz, "wrong left value from motor_raw")
        # self.file_check("rtmotor_raw_r0", m.right_hz, "wrong right value from motor_raw")

        m.left_hz = 0
        m.right_hz = 0
        pub.publish(m)

    def test_put_cmd_vel(self):
        pub = rospy.Publisher("/cmd_vel", Twist)

        m = Twist()
        m.linear.x = 0.1414
        m.angular.z = 1.57
        pub.publish(m)

        # self.file_check("rtmotor_raw_l0", 200, "wrong left value from cmd_vel")
        # self.file_check("rtmotor_raw_r0", 600, "wrong right value from cmd_vel")

        time.sleep(1.1)
        # self.file_check("rtmotor_raw_l0", 0, "don't stop after 1[s]")
        # self.file_check("rtmotor_raw_r0", 0, "don't stop after 1[s]")

    def test_on_off(self):
        off = rospy.ServiceProxy("/motor_off", Trigger)
        ret = off()
        # self.assertEqual(ret.success, True, "motor off does not succeded")
        # self.assertEqual(ret.message, "OFF", "motor off wrong message")
        on = rospy.ServiceProxy("/motor_on", Trigger)
        ret = on()

        time.sleep(4)
        off = rospy.ServiceProxy("/motor_off", Trigger)
        ret = off()

    def test_put_value_timed(self):
        tm = rospy.ServiceProxy("/timed_motion", TimedMotion)
        tm(-300, 300, 500)



if __name__ == "__main__":
    #time.sleep(3)
    rospy.init_node("my_test_motors")
    rostest.rosrun("pimouse_ros", "my_test_motors", MotorTest)
