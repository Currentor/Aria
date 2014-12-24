#!/usr/bin/env python
import aria
import rospy
import subprocess
import sys
import thread
import time
from std_msgs.msg import String

class Demo:
    demo = {}
    brkflag = False
    def __init__(self):
        self.demo['thread_test'] = self.thread_test
        self.demo['arm_up_test'] = self.arm_up_test
        self.demo['reset'] = self.reset
        self.demo['idle'] = self.idle
        self.demo['idle_baseline'] = self.idle_baseline
        self.demo['pad_back'] = self.pad_back
        self.demo['pad_back_baseline'] = self.pad_back_baseline
        self.demo['table_wipe'] = self.table_wipe
    def tryq(self, msg="press enter to continue, q to exit   "):
        response = raw_input(msg)
        if response == 'q':
            return True
        return False
    def threadq(self, msg="enter q to break   "):
        response = raw_input(msg)
        if response == 'q':
            self.brkflag = True
    def multidemo(self, demoname):
        subprocess.check_output(['rosrun','aria_utils',demoname])
    def reset(self):
        subprocess.check_output(['rosrun','aria_utils','reset.py'])
    def thread_test(self):
        rospy.loginfo('starting test')
        thread.start_new_thread(self.threadq, ())
        while 1:
            if self.brkflag:
                self.brkflag = False
                break
        rospy.loginfo('ending test')
    def arm_up_test(self):
        rospy.loginfo('starting test')
        thread.start_new_thread(self.threadq, ())
        while 1:
            subprocess.check_output(['rosrun','aria_utils','cheers.py'])
            time.sleep(10.0)
            subprocess.check_output(['rosrun','aria_utils','reset.py'])
            if self.brkflag:
                self.brkflag = False
                break
        rospy.loginfo('ending test')
    def table_wipe(self):
        rospy.loginfo('starting demo')
        #if not self.tryq("initiate robot? q to skip   "):
        #    rospy.logwarn('initiating')
        #    self.demo['reset']()
        if not self.tryq("initiate demo pose? q to skip   "):
            rospy.logwarn('sending initial demo pose')
            subprocess.check_output(['rosrun','aria_utils','table_wipe_init.py'])
        if not self.tryq('start demo? q to skip   '):
            rospy.logwarn('starting demo')
            thread.start_new_thread(self.threadq, ())
            thread.start_new_thread(self.multidemo, ('table_wipe_voice.py',))
            while 1:
                subprocess.check_output(['rosrun','aria_utils','table_wipe.py'])
                if self.brkflag:
                    self.brkflag = False
                    break
        rospy.logwarn('ending demo')
    def pad_back(self):
        rospy.loginfo('starting demo')
        #if not self.tryq("initiate robot? q to skip   "):
        #    rospy.logwarn('initiating')
        #    self.demo['reset']()
        if not self.tryq("initiate demo pose? q to skip   "):
            rospy.logwarn('sending initial demo pose')
            subprocess.check_output(['rosrun','aria_utils','pad_back_init.py'])
        if not self.tryq('start demo? q to skip   '):
            subprocess.check_output(['rosrun','aria_utils','pad_back.py'])
        rospy.logwarn('ending demo')
    def pad_back_baseline(self):
        rospy.loginfo('starting demo')
        #if not self.tryq("initiate robot? q to skip   "):
        #    rospy.logwarn('initiating')
        #    self.demo['reset']()
        if not self.tryq("initiate demo pose? q to skip   "):
            rospy.logwarn('sending initial demo pose')
            subprocess.check_output(['rosrun','aria_utils','pad_back_init.py'])
        if not self.tryq('start demo? q to skip   '):
            subprocess.check_output(['rosrun','aria_utils','pad_back_baseline.py'])
        rospy.logwarn('ending demo')
    def idle(self):
        rospy.loginfo('starting demo')
        if not self.tryq("initiate robot? q to skip   "):
            rospy.logwarn('initiating')
            self.demo['reset']()
        #if not self.tryq("initiate demo pose? q to skip   "):
        #    rospy.logwarn('sending initial demo pose')
        #    subprocess.check_output(['rosrun','aria_utils','idle_init.py'])
        if not self.tryq('start demo? q to skip   '):
            rospy.logwarn('starting demo')
            subprocess.check_output(['rosrun','aria_utils','idle.py'])
        rospy.logwarn('ending demo')
    def idle_baseline(self):
        rospy.loginfo('starting demo')
        if not self.tryq("initiate robot? q to skip   "):
            rospy.logwarn('initiating')
            self.demo['reset']()
        #if not self.tryq("initiate demo pose? q to skip   "):
        #    rospy.logwarn('sending initial demo pose')
        #    subprocess.check_output(['rosrun','aria_utils','idle_init.py'])
        if not self.tryq('start demo? q to skip   '):
            rospy.logwarn('starting demo')
            subprocess.check_output(['rosrun','aria_utils','idle_baseline.py'])
        rospy.logwarn('ending demo')

if __name__ == '__main__':
    rospy.init_node('demo', anonymous=True)
    aria.init_publisher()
    demo = Demo()
    if not len(sys.argv) == 2:
        rospy.logerr('expected an argument with demo name')
        rospy.loginfo('\nexpected demos are:\n  reset\n  idle\n  pad_back\n  table_wipe')
        sys.exit()
    demo.demo[str(sys.argv[1])]()
