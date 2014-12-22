#!/usr/bin/env python
import rospy
import time
import aria
import sys
from std_msgs.msg import String

class Settings:
    id_map = rospy.get_param("/id_map")
    initial_pose = rospy.get_param("/initial_pose")
    pid_gain = rospy.get_param("/pid_gain")
    setup_functions = {}
    def __init__(self):
        aria.init_publisher()
        self.setup_functions['servo_on'] = self.servo_on
        self.setup_functions['servo_off'] = self.servo_off
        self.setup_functions['get_size'] = self.get_size
        self.setup_functions['initiate'] = self.initiate
        self.setup_functions['get_exist'] = self.get_exist
        self.setup_functions['apply_gain'] = self.apply_gain
        return
    def servo_on(self):
        msg = String()
        msg.data = '{\"method\":\"sendZero\",\"params\":\"0\",\"id\":\"1\"}'
        pub = rospy.Publisher('/ros2http/socket_listener/json_string', String, latch=True)
        pub.publish(msg)
    def servo_off(self):
        for key, value in self.id_map.iteritems():
            if 'wheel' in key:
                aria.set_torque(value, 0)
            elif 'body' in key:
                print('skipping body')
            elif 'hip' in key:
                print('skipping hip')
            elif 'neck' in key:
                print('skipping neck')
            else:
                aria.set_torque(value, 0)
        print('make sure robot is connected to crane')
    def initiate(self):
        aria.set_control_modes([aria.mode['none']]*aria.joint_size)
        for key, value in self.id_map.iteritems():
            if 'wheel' in key:
                aria.set_torque(value, 0)
            else:
                aria.set_position(value, self.initial_pose[key])
    def get_size(self):
        print('%d' % (len(self.id_map)))
    def get_exist(self):
        self.servo_on()
        exist = aria.echo_joints('mode')
        okay = 0
        if len(exist) < len(self.id_map):
            print('error when applying position')
            return
        for key, value in self.id_map.iteritems():
            if not exist[value] == 1.0:
                print('joint %d is missing' % (value))
            else:
                okay = okay + 1
        print('%d' % (okay))
    def apply_gain(self):
        for key, value in self.id_map.iteritems():
                aria.init_pid_gain(value, self.pid_gain[key]['p'], self.pid_gain[key]['i'], self.pid_gain[key]['d'])
        aria.set_feedback(aria.feedback['kp'])
        kp = aria.echo_joints('debug')
        if len(kp) < len(self.id_map):
            print('error when applying position')
            return
        for key, value in self.id_map.iteritems():
            if kp[value] == 0.0:
                print('warn! joint %d gain is zero' % (value))

if __name__ == '__main__':
    rospy.init_node('setup', anonymous=True)
    settings = Settings()
    if not len(sys.argv) == 2:
        rospy.logerr('expected an argument with function name')
        sys.exit()
    settings.setup_functions[str(sys.argv[1])]()
