#!/usr/bin/env python
import rospy
import subprocess
import wave
import re
import time
import thread
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

joint_size = 30

mode = {'none': 0.0, 'torque': 1.0, 'position': 2.0, 'hybrid': 3.0}

feedback = {'velocity': 0.0, 'torque': 1.0, 'position': 2.0,
            'kp':3.0, 'kd': 4.0, 'ct': 5.0, 'cp':6.0}

interpolation = {'linear': 1.0, 'bezier': 2.0, 'slowout': 3.0,
                 'slowin': 4.0, 'sigmoid': 5.0, 'cubic': 6.0}

json_string_topic_name = '/ros2http/socket_listener/json_string'
mode_vector_topic_name = '/currentor_socket/request/mode_vector'
torque_vector_topic_name = '/currentor_socket/request/torque_vector'
position_vector_topic_name = '/currentor_socket/request/position_vector'
pid_vector_topic_name = '/currentor_socket/request/pid_vector'
eye_topic_name = '/2ndparty/request/eye'
gripper_topic_name = '/2ndparty/request/gripper'
#voice_topic_name = '/2ndparty/request/voice'
voice_topic_name = '/py_voice'
anticipation_topic_name = '/addons/request/anticipation'

pub_json = None
pub_mode = None
pub_torque = None
pub_position = None
pub_pid = None
pub_eye = None
pub_gripper = None
pub_voice = None
pub_anticipation = None

def init_publisher():
    global pub_json
    global pub_mode
    global pub_torque
    global pub_position
    global pub_pid
    global pub_eye
    global pub_gripper
    global pub_voice
    global pub_anticipation
    pub_json = rospy.Publisher(json_string_topic_name,
                               String, latch=True)
    pub_mode = rospy.Publisher(mode_vector_topic_name,
                               Float32MultiArray, latch=True)
    pub_torque = rospy.Publisher(torque_vector_topic_name,
                                 Float32MultiArray, latch=True)
    pub_position = rospy.Publisher(position_vector_topic_name,
                                   Float32MultiArray, latch=True)
    pub_pid = rospy.Publisher(pid_vector_topic_name,
                              Float32MultiArray, latch=True)
    pub_eye = rospy.Publisher(eye_topic_name,
                              Float32MultiArray, latch=True)
    pub_gripper = rospy.Publisher(gripper_topic_name,
                                  Float32MultiArray, latch=True)
    pub_voice = rospy.Publisher(voice_topic_name,
                                String, latch=True)
    pub_anticipation = rospy.Publisher(anticipation_topic_name,
                                       Float32MultiArray, latch=False)

def echo_joints(data_type):
    msg = subprocess.check_output(['rostopic','echo','-n1',
                                   '/currentor_socket/sensor_array/%s/data'
                                   % (str(data_type))])
    msg = re.split('\[|\]',msg)
    msg.remove('')
    msg.remove('\n---\n')
    msg = map(float, msg[0].split(','))
    return msg

def echo_joint(data_type, joint):
    msg = subprocess.check_output(['rostopic', 'echo', '-n1',
                                   '/currentor_socket/sensor_array/%s/data[%d]'
                                   % (str(data_type), joint)])
    msg = re.split('\[\|]|\\n',msg)
    return float(msg[0])

def set_feedback(fb_type):
    msg = String()
    msg.data = '{\"method\":\"%s\",\"params\":\"[%f]\",\"id\":\"0\"}' % ('setFeedback', fb_type)
    pub_json.publish(msg)
    time.sleep(0.1)

def set_control_mode(joint, mode, sleep=True):
    msg = String()
    msg.data = '{\"method\":\"setControlMode\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, mode)
    pub_json.publish(msg)
    if sleep:
        time.sleep(0.1)
    return mode

def set_torque(joint, torque, sleep=True):
    msg = String()
    msg.data = '{\"method\":\"setTorque\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, torque)
    pub_json.publish(msg)
    if sleep:
        time.sleep(0.1)
    return torque

def set_position(joint, position, sleep=True):
    msg = String()
    msg.data = '{\"method\":\"setPosition\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, position)
    pub_json.publish(msg)
    if sleep:
        time.sleep(0.1)
    return position

def set_time(joint, t, sleep=True):
    msg = String()
    msg.data = '{\"method\":\"setTime\",\"params\":\"[%d,%f]\",\"id\":\"1\"}' % (joint, t)
    pub_json.publish(msg)
    if sleep:
        time.sleep(0.1)
    return t

def set_control_modes(data):
    msg = Float32MultiArray()
    msg.data = data
    pub_mode.publish(msg)
    time.sleep(0.1)

def set_torques(data):
    msg = Float32MultiArray()
    msg.data = data
    pub_torque.publish(msg)
    time.sleep(0.1)

def set_positions(data):
    msg = Float32MultiArray()
    msg.data = data
    pub_position.publish(msg)
    time.sleep(0.1)

def init_pid_gain(joint, p, i, d):
    msg = String()
    msg.data = '{\"method\":\"initPIDGain\",\"params\":\"[%d,%f,%f,%f]\",\"id\":\"1\"}' % (joint, p, i, d)
    pub_json.publish(msg)
    time.sleep(0.1)

def set_interpolation(interpolate_type):
    msg = String()
    msg.data = '{\"method\":\"setInterpolation\",\"params\":\"[%f]\",\"id\":\"1\"}' % (interpolation[interpolate_type])
    pub_json.publish(msg)
    time.sleep(0.1)

def eye(data):
    msg = Float32MultiArray()
    msg.data = data
    pub_eye.publish(msg)
    time.sleep(0.1)

def gripper(data):
    msg = Float32MultiArray()
    msg.data = data
    pub_gripper.publish(msg)
    time.sleep(0.1)

def voice(str_data, amp='1.0'):
    dat = str_data + ', 0.0, -1, ' + amp + ', 1.0'
    msg = String()
    #msg.data = str_data
    msg.data = dat
    pub_voice.publish(msg)

def get_voice_length(voi):
    filename = subprocess.Popen(
        'echo $(rospack find aria_2ndparty)/sound/voice/',
        shell=True,
        stdout=subprocess.PIPE)
    filename = filename.communicate()[0]
    filename = re.split('\n', filename)
    filename = filename[0]
    filename = "".join((filename, voi))
    wf = wave.open(filename, "r")
    t = wf.getnframes() / float(wf.getframerate())
    return t

class Anticipation:
    rate = 100
    force_vector = []
    duration_count = 0
    max_count = 0
    play = False
    def __init__(self):
        thread.start_new_thread(self.main, ())
    def setup(self, amplitude, repeats, optional=[]):
        if repeats < 1:
            return
        self.force_vector = []
        _repeats = int(repeats)
        cur_pos = echo_joints('position')
        tmp = [[ 4.0 for i in range(30) ] for j in range(2)]
        if len(optional) > 0:
            for x in range(0,29):
                if not amplitude[x] == None:
                    tmp[0][x] = cur_pos[x] + amplitude[x]
                if not optional[x] == None:
                    tmp[1][x] = cur_pos[x] + optional[x]
            tmp[0][29] = amplitude[29]
            tmp[1][29] = optional[29]
        else:
            for x in range(0,29):
                if not amplitude[x] == None:
                    tmp[0][x] = cur_pos[x] + amplitude[x]
                    tmp[0][29] = amplitude[29] / _repeats
        self.duration_count = []
        self.max_count = _repeats
        for y in range(0, _repeats):
            self.force_vector += [tmp[y % 2]]
            self.duration_count += [tmp[y % 2][29]*self.rate]
        self.play = True
    def main(self):
        counter = 0
        loop_num = 0
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.play == True:
                if loop_num >= self.max_count:
                    counter = 0
                    loop_num = 0
                    self.play = False
                elif counter >= self.duration_count[loop_num]:
                    set_positions(self.force_vector[loop_num])
                    counter = 0
                    loop_num += 1
                counter += 1
                r.sleep()
            else:
                r.sleep()

anticipator = None

def init_anticipation():
    global anticipator
    anticipator = Anticipation()

def anticipation(amplitude, repeats, optional=[]):
    anticipator.setup(amplitude, repeats, optional)

#def anticipation(amplitude, repeats, optional=[]):
#    msg = Float32MultiArray()
#    msg.data = amplitude
#    msg.data += [repeats]
#    msg.data += optional
#    pub_anticipation.publish(msg)
#    time.sleep(1.0)

def debug():
    rospy.init_node('debug_ariapy')
    init_publisher()
    init_anticipation()

if __name__ == '__main__':
    rospy.init_node('python_webcommands')
    rospy.spin()
