#!/usr/bin/env python3

import rospy
import rosparam
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_msgs.msg import String
import numpy as np
import time
import sys
import math
from scipy.spatial.transform import Rotation
from westwood_legged_msgs.msg import MotorCmd
from westwood_legged_msgs.msg import MotorState

args = sys.argv

pub = []

command = ""
positions = {}   #dict

bfr_button6 = 0
bfr_button7 = 0

pi2 = np.pi / 2

def joyCb(data):
    global command, bfr_button6, bfr_button7
    # print("L2", data.axes[3], " R2", data.axes[4], "L2", data.buttons[6], "R2",  data.buttons[7])
    if bfr_button6 == 0 and data.buttons[6] == 1:
        command = "left"
        bfr_button6 = 1
    elif bfr_button6 == 1 and data.buttons[6] == 0:
        bfr_button6 = 0
    elif bfr_button7 == 0 and data.buttons[7] == 1:
        command = "right"
        bfr_button7 = 1
    elif bfr_button7 == 1 and data.buttons[7] == 0:
        bfr_button7 = 0

def jointCb(data):
    global positions

    for i in range(len(data.name)):
        positions[data.name[i]] = data.position[i]

    # 一覧表示
    # for key in positions.keys():
    #     print(positions[key])

    # rospy.loginfo("recieved msg")
    # print (data.name)
    # print ("position", data.position)
    # print ("velocity", data.velocity)
    # print ("effort", data.effort)


def commandCb(data):
    global command
    rospy.loginfo("recieved msg %s", data.data)
    command = data.data

def motorState1Cb(msg):
    print(msg)

def motorState2Cb(msg):
    print(msg)

def leg_control():
    global command, pub
    rospy.init_node('bear_actuator_dfm_test')
    pub.append(rospy.Publisher('/lambda_leg/L_test_controller/command', MotorCmd, queue_size=1))
    pub.append(rospy.Publisher('/lambda_leg/R_test_controller/command', MotorCmd, queue_size=1))
    rospy.Subscriber("/lambda_leg/L_test_controller/state", MotorState, motorState1Cb)
    rospy.Subscriber("/lambda_leg/R_test_controller/state", MotorState, motorState2Cb)
    rospy.Subscriber("joint_states", JointState, jointCb)
    rospy.Subscriber("leg_command", String, commandCb)
    rospy.Subscriber("joy", Joy, joyCb)
    
    r = rospy.Rate(10)  # 10Hz
    mtcmd = MotorCmd()
    mtcmd2 = MotorCmd()
    while not rospy.is_shutdown():
        if command == "zero" :
            print("command", command)
            mtcmd.q = 0.0
            mtcmd.dq = 0.0
            mtcmd.tau = 0.0
            mtcmd.Kp = 0.0
            mtcmd.Kd = 0.0
            pub[0].publish(mtcmd)
            pub[1].publish(mtcmd)
            command = ""

        elif command == "pos" :
            mtcmd.q = 1.54
            mtcmd.dq = 0.0
            mtcmd.tau = 0.0
            mtcmd.Kp = 20.0
            mtcmd.Kd = 0.5
            pub[0].publish(mtcmd)
            command = ""

        # toruque q=0としてトルクを与える。瞬間的に pos=0での位置制御が入るのでドンとトルクが入る
        elif command == "torque" :
            mtcmd.q = 0.0
            mtcmd.dq = 0.0
            mtcmd.tau = -1.0
            mtcmd.Kp = 0.0
            mtcmd.Kd = 0.0
            pub[0].publish(mtcmd)
            command = ""

        # torque2 qを変化せずにトルクを与える。瞬間的にはdfm q=1.54となるのでトルク抜けも突入トルクも入らない
        elif command == "torque2" :
            mtcmd.q = 1.54
            mtcmd.dq = 0.0
            mtcmd.tau = -1.0
            mtcmd.Kp = 0.0
            mtcmd.Kd = 0.0
            pub[0].publish(mtcmd)
            command = ""

        # dfm q=1.54のpos にtauに応じた押しが入る。Kp,Kdを小さくすると押しは大きくなるが、それほどでもない。
        elif command == "dfm" :
            mtcmd.q = 1.54
            mtcmd.dq = 0.0
            mtcmd.tau = -1.0
            mtcmd.Kp = 20.0
            mtcmd.Kd = 0.5
            pub[0].publish(mtcmd)
            command = ""

        elif command == "test" :
            kpbase = 6.0
            mtcmd.Kp = kpbase
            mtcmd.Kd = 0.5
            mtcmd.q = 3.14 / 4
            mtcmd.tau = 0
            mtcmd2.Kp = 0
            mtcmd2.Kd = 0.2
            mtcmd2.q = 0
            mtcmd2.tau = -2.0
            time.sleep(3.0)
            for i in range(10) :
                print("pos")
                pub[0].publish(mtcmd)
                # pub[1].publish(mtcmd2)
                time.sleep(0.3)
                print("effort")
                pub[0].publish(mtcmd2)
                # pub[1].publish(mtcmd)
                time.sleep(0.3)
            command = ""
        
        elif command == "quit":
            loop = False

        r.sleep()

if __name__ == '__main__':
    try:
        leg_control()
    except rospy.ROSInterruptExeception: pass
