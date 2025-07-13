#!/usr/bin/env python3

import rospy
import rosparam
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import numpy as np
import time
import sys
import math
from scipy.spatial.transform import Rotation
from westwood_legged_msgs.msg import MotorCmd
from westwood_legged_msgs.msg import MotorState
import copy

import matplotlib.pyplot as plt

args = sys.argv

pub = []
mtr_cmd = ""

command = ""
positions = {}   #dict
velocities = {}
joint_names = ["L_hip_joint", "L_hip2_joint", "L_thigh_joint", "L_calf_joint", "L_toe_joint", "R_hip_joint", "R_hip2_joint", "R_thigh_joint", "R_calf_joint", "R_toe_joint"]
joint_index = [2, 1, 3, 0, 4, 7, 6, 8, 5, 9]
indexlist = {}

bfr_button6 = 0
bfr_button7 = 0
bfr_data = Joy()
joy_first = True

motorState = []

angles = []

#data log
p = []
v = []
h = []
obs_joint = ""

for i, n in enumerate(joint_names):
    indexlist[n] = joint_index[i]
for nm in indexlist.keys():
    print(nm, indexlist[nm])

pi2 = np.pi / 2
axis = [
    np.matrix([[0], [0], [1]]),
    np.matrix([[1], [0], [0]]),
    np.matrix([[0], [1], [0]]),
    np.matrix([[0], [1], [0]]),
    np.matrix([[0], [1], [0]]),
]

limit_max = [0.4, 0.7, 1.5, 2.5, 1.5, 0.4, 0.7, 1.5, 2.5, 1.5]
limit_min = [-0.4, -0.7, -1.5, -0.05, -1.5, -0.4, -0.7, -1.5, -0.05, -1.5]

joy_x = 0.0
joy_y = 0.0
joy_rz = 0.0
joy_ry = 0.0

def joint_data_check(j_data):
    global limit_max, limit_min
    res = True
    for i in range(10):
        if j_data[i] > limit_max[i] or limit_min[i] > j_data[i]:
            res = False
    return res

def joint_publish(joint_, times, wait):
    global pub, joint_names

    kp_start = 0
    kp = [20, 20, 20, 40, 5, 20, 20, 20, 40, 5]
    kd = [2.0, 2.0, 2.0, 2.0, 0.5, 2.0, 2.0, 2.0, 2.0, 0.5]
    mtcmd = MotorCmd()
    mtcmd.dq = 0.0
    mtcmd.tau = 0.0
    mtcmd.Kp = 50.0
    mtcmd.Kd = 2.0

    if joint_data_check(joint_):
        for c in range(times):
            for i in range(10):
                mtcmd.q = joint_[i]
                mtcmd.Kp = kp_start + (kp[i] - kp_start) * (c+1) / times
                mtcmd.Kd = kd[i] * (c+1) / times
                pub[i].publish(mtcmd)
            time.sleep(wait)
    else:
        print("Joint Limit Over.")
        for i in range(10):
            print(limit_max[i], ">", joint_[i], ">", limit_min[i])

def leg_pub(tgt_left_z, tgt_right_z, times, wait):
    tgt_y = 0.0
    tgt_p = 0.0
    tgt_yw = 0.0
    ljoint, lpoints, lrots, rjoint, rpoints, rrots = get_present_pos()
    print("left")
    print(lpoints[4])
    rot = Rotation.from_matrix(lrots[4])
    print(rot.as_matrix())
    print(rot.as_euler('xyz', degrees=True))

    print("right")
    print(rpoints[4])
    rot = Rotation.from_matrix(rrots[4])
    print(rot.as_matrix())
    print(rot.as_euler('xyz', degrees=True))
    print("tgt_p", tgt_p)

    t_pitch = np.matrix([[np.cos(tgt_p), 0, np.sin(tgt_p)], \
                [0, 1, 0], \
                [-np.sin(tgt_p), 0, np.cos(tgt_p)]])
    t_yaw = np.matrix([[np.cos(tgt_yw), -np.sin(tgt_yw), 0], \
                [np.sin(tgt_yw), np.cos(tgt_yw), 0], \
                [0, 0, 1]])

    t_pos = np.matrix([[0.0], [tgt_y], [tgt_left_z]])
    print("left  : ", t_pos)
    t_pos = t_pitch * t_pos
    print("left pitch  : ", t_pos)
    t_rot = np.matrix([[np.cos(tgt_p), 0, np.sin(tgt_p)], \
                [0, 1, 0], \
                [-np.sin(tgt_p), 0, np.cos(tgt_p)]])
    t_rot = t_yaw * t_rot
    print("t_pos", t_pos)
    joint_l, res_l = ik_jac(ljoint, t_pos, t_rot)

    t_pos = np.matrix([[0.0], [-tgt_y], [tgt_right_z]])
    print("right : ", t_pos)
    t_pos = t_pitch * t_pos
    print("right pitch : ", t_pos)
    t_rot = np.matrix([[np.cos(tgt_p), 0, np.sin(tgt_p)], \
                [0, 1, 0], \
                [-np.sin(tgt_p), 0, np.cos(tgt_p)]])
    t_rot = t_yaw.T * t_rot
    print("t_pos", t_pos)
    joint_r, res_r = ik_jac(rjoint, t_pos, t_rot)

    joint_publish(joint_l + joint_r, times, wait)

def joyCb(data):
    global command, bfr_data, joy_first, bfr_button6, bfr_button7, joy_y, joy_x, joy_rz, joy_ry
    # print("L2", data.axes[3], " R2", data.axes[4], "L2", data.buttons[6], "R2",  data.buttons[7])
    if joy_first:
        bfr_data = data
        joy_first = False        
    if bfr_data.buttons[6] == 0 and data.buttons[6] == 1:
        command = ""
    elif bfr_data.buttons[7] == 0 and data.buttons[7] == 1:
        command = ""
    elif bfr_data.buttons[0] == 0 and data.buttons[0] == 1:  #□
        command = "jacob"
    elif bfr_data.buttons[3] == 0 and data.buttons[3] == 1:  #△
        command = "stand"
    elif bfr_data.buttons[8] == 0 and data.buttons[8] == 1:  #share
        command = "motoron"
    elif bfr_data.buttons[9] == 0 and data.buttons[9] == 1:  #options
        command = "motoroff"
    elif bfr_data.buttons[12] == 0 and data.buttons[12] == 1:  #PS
        command = "motorreset"
    elif bfr_data.buttons[1] == 0 and data.buttons[1] == 1:  #×
        command = "torqueoff"
    # elif bfr_data.buttons[2] == 0 and data.buttons[2] == 1:  #○
    joy_y = data.axes[0]  # LJ←→
    joy_x = data.axes[1]  # LJ↑↓ 
    joy_rz = data.axes[2]  # RJ←→
    joy_ry = data.axes[5]  # RJ↑↓

    bfr_data = data

def jointCb(data):
    global positions, velocities

    for i in range(len(data.name)):
        positions[data.name[i]] = data.position[i]
        velocities[data.name[i]] = data.velocity[i]

    # 一覧表示
    # for key in positions.keys():
    #     print(positions[key])

    # rospy.loginfo("recieved msg")
    # print (data.name)
    # print ("position", data.position)
    # print ("velocity", data.velocity)
    # print ("effort", data.effort)

def motorState0Cb(msg):
    motorState[0] = msg

def motorState1Cb(msg):
    motorState[1] = msg

def motorState2Cb(msg):
    motorState[2] = msg

def motorState3Cb(msg):
    motorState[3] = msg

def motorState4Cb(msg):
    motorState[4] = msg

def motorState5Cb(msg):
    motorState[5] = msg

def motorState6Cb(msg):
    motorState[6] = msg

def motorState7Cb(msg):
    motorState[7] = msg

def motorState8Cb(msg):
    motorState[8] = msg

def motorState9Cb(msg):
    motorState[9] = msg


def commandCb(data):
    global command
    rospy.loginfo("recieved msg %s", data.data)
    command = data.data

def genesisAnglesCb(msg):
    global angles
    angles = msg.data
    print("angles", angles)
    angles_ = [angles[i] for i in joint_index]
    print("angles_", angles_)
    joint_publish(angles_, 1, 0)

def leg_control():
    global command, pub, mtr_cmd
    rospy.init_node('lambda_leg_sample2')
    pub.append(rospy.Publisher('/lambda_leg/L_hip_controller/command', MotorCmd, queue_size=1))
    pub.append(rospy.Publisher('/lambda_leg/L_hip2_controller/command', MotorCmd, queue_size=1))
    pub.append(rospy.Publisher('/lambda_leg/L_thigh_controller/command', MotorCmd, queue_size=1))
    pub.append(rospy.Publisher('/lambda_leg/L_calf_controller/command', MotorCmd, queue_size=1))
    pub.append(rospy.Publisher('/lambda_leg/L_toe_controller/command', MotorCmd, queue_size=1))
    pub.append(rospy.Publisher('/lambda_leg/R_hip_controller/command', MotorCmd, queue_size=1))
    pub.append(rospy.Publisher('/lambda_leg/R_hip2_controller/command', MotorCmd, queue_size=1))
    pub.append(rospy.Publisher('/lambda_leg/R_thigh_controller/command', MotorCmd, queue_size=1))
    pub.append(rospy.Publisher('/lambda_leg/R_calf_controller/command', MotorCmd, queue_size=1))
    pub.append(rospy.Publisher('/lambda_leg/R_toe_controller/command', MotorCmd, queue_size=1))
    rospy.Subscriber("joint_states", JointState, jointCb)
    rospy.Subscriber("leg_command", String, commandCb)
    rospy.Subscriber("joy", Joy, joyCb)
    mtr_cmd = rospy.Publisher('pwr_cmd', String, queue_size=1)
    for i in range(10):
        motorState.append(MotorState())
    rospy.Subscriber("/lambda_leg/L_hip_controller/state", MotorState, motorState0Cb)
    rospy.Subscriber("/lambda_leg/L_hip2_controller/state", MotorState, motorState1Cb)
    rospy.Subscriber("/lambda_leg/L_thigh_controller/state", MotorState, motorState2Cb)
    rospy.Subscriber("/lambda_leg/L_calf_controller/state", MotorState, motorState3Cb)
    rospy.Subscriber("/lambda_leg/L_toe_controller/state", MotorState, motorState4Cb)
    rospy.Subscriber("/lambda_leg/R_hip_controller/state", MotorState, motorState5Cb)
    rospy.Subscriber("/lambda_leg/R_hip2_controller/state", MotorState, motorState6Cb)
    rospy.Subscriber("/lambda_leg/R_thigh_controller/state", MotorState, motorState7Cb)
    rospy.Subscriber("/lambda_leg/R_calf_controller/state", MotorState, motorState8Cb)
    rospy.Subscriber("/lambda_leg/R_toe_controller/state", MotorState, motorState9Cb)

    rospy.Subscriber("/genesis_angles", Float32MultiArray, genesisAnglesCb)

    # use command'jacob'
    # fb_coef = -30.0
    fb_coef = 0.0
    jacob_first = True
    target_height_left = 0
    target_height_right = 0
    exec_cmd = ""
    step_height = 0.0
    dt = 0.0
    phase = 0.0
    T = 0.0
    stand_height = 0.25
    test3_count = 0

    ljdefault = []
    rjdefault = []
    for i in range(5):
        ljdefault.append(0.0)
        rjdefault.append(0.0)


    r = rospy.Rate(100)  # 100Hz
    while not rospy.is_shutdown():
        if command == "zero" :
            print("command", command)
            mtcmd = MotorCmd()
            mtcmd.q = 0.0
            mtcmd.dq = 0.0
            mtcmd.tau = 0.0
            mtcmd.Kp = 10.0
            mtcmd.Kd = 0.5
            for i in range(10):
                pub[i].publish(mtcmd)
            command = ""
            jacob_first = True
        elif command == "zero2" :
            print("command", command)
            mtcmd = MotorCmd()
            mtcmd.q = 0.0
            mtcmd.dq = 0.0
            mtcmd.tau = 0.0
            mtcmd.Kp = 10.0
            mtcmd.Kd = 0.5
            qs = [0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0]
            for i in range(10):
                mtcmd.q = qs[i]
                pub[i].publish(mtcmd)
            command = ""
            jacob_first = True
        elif command == "torqueoff" :
            print("command", command)
            mtcmd = MotorCmd()
            mtcmd.q = 0.0
            mtcmd.dq = 0.0
            mtcmd.tau = 0.0
            mtcmd.Kp = 0.0
            mtcmd.Kd = 0.0
            for i in range(10):
                mtcmd.q = motorState[i].q
                pub[i].publish(mtcmd)
            command = ""
            jacob_first = True
        elif command == "motoroff" :
            command = ""
            msg = String()
            msg.data = "off"
            mtr_cmd.publish(msg)
        elif command == "motoron" :
            command = ""
            msg = String()
            msg.data = "on"
            mtr_cmd.publish(msg)
        elif command == "motorreset" :
            command = ""
            msg = String()
            msg.data = "reset"
            mtr_cmd.publish(msg)
        elif command == "test1" :
            print("command", command)
            mtcmd = MotorCmd()
            mtcmd.q = 0.0
            mtcmd.dq = 0.0
            mtcmd.tau = 0.0
            mtcmd.Kp = 6.0
            mtcmd.Kd = 0.5
            pub[0].publish(mtcmd)
            command = ""

        elif command == "test2" :
            print("command", command)   #足首（GIMactuatorのMITdriver）のテスト
            mtcmd = MotorCmd()
            mtcmd.q = 0.0
            mtcmd.dq = 0.0
            mtcmd.tau = 0.1
            mtcmd.Kp = 0.5
            mtcmd.Kd = 0.05
            pub[4].publish(mtcmd)
            pub[9].publish(mtcmd)
            command = ""

        elif command == "test3" :
            print("command", command)   #ステップ応答テスト・サンプル取得
            p.clear()
            v.clear()
            obs_joint = "L_calf_joint"
            test3_count = 100
            mtcmd = MotorCmd()
            # mtcmd.q = 0.7 # hip2
            # mtcmd.q = -1.2 # thigh
            mtcmd.q = 1.2 # calf
            mtcmd.dq = 0.0
            mtcmd.tau = 0.0
            mtcmd.Kp = 20.0
            mtcmd.Kd = 0.5
            pub[3].publish(mtcmd)    # 1: hip2 2: thigh 3: calf
            exec_cmd = command
            command = ""

        elif command == "stand" :   # set stand pose    > stand y z pitch yw  [-0.1 > z > -0.32]
            command = ""
            leg_pub(-stand_height, -stand_height, 10, 0.1)

        elif command == "left" :   # set stand pose    > stand y z pitch yw  [-0.1 > z > -0.32]
            if exec_cmd == "":
                exec_cmd = command
                command = ""
                step_height = 0.05
                dt = 2.0 / 30.0
                phase = 0.0

        elif command == "right" :   # set stand pose    > stand y z pitch yw  [-0.1 > z > -0.32]
            if exec_cmd == "":
                exec_cmd = command
                command = ""
                step_height = 0.05
                dt = 2.0 / 30.0
                phase = 0.0

        elif command == "step_motion":
            exec_cmd = command
            command = ""
            p.clear()
            v.clear()
            h.clear()
            obs_joint = "L_calf_joint"
            step_height = 0.13
            dt = 0.02
            T = 0.6
            phase = 0.0
            
        elif command == "jacob" :
            """
            DirectForceModeを使い、位置制御とトルク制御を合わせる。
            qにはデフォルト関節角度を与え、tauでトルクを与えて期待の床反力を発生させる。
            Kp,Kdを位置制御の時より下げる必要が有る。
            """
            # command = ""
            fx = joy_x * 5.0
            fy = joy_y * 5.0
            fz = -25.0
            my = joy_ry * 3.0
            mz = joy_rz * 3.0
            kp = [10, 10, 10, 10, 5, 10, 10, 10, 10, 5]
            kd = [2.0, 2.0, 2.0, 2.0, 0.5, 2.0, 2.0, 2.0, 2.0, 0.5]

            if jacob_first:
                for i in range(5):
                    ljdefault[i] = motorState[i].q
                for i in range(5, 10):
                    rjdefault[i - 5] = motorState[i].q
                ljoint, lpoints, lrots, rjoint, rpoints, rrots = get_present_pos()
                target_height_left = -lpoints[5][2, 0]
                target_height_right = -rpoints[5][2, 0]
                print("target hight left :", target_height_left)
                print("target hight right :", target_height_right)
                jacob_first = False
            ui = np.matrix([[fx, fy, fz, my, mz]]).transpose()
            ja = jacobian(lpoints, lrots)
            ltau = ja.transpose() * ui
            ui = np.matrix([[fx, fy, fz, my, mz]]).transpose()
            ja = jacobian(rpoints, rrots)
            rtau = ja.transpose() * ui
            print("ltau", ltau)
            print("rtau", rtau)
            i = 0
            mtcmd = MotorCmd()
            mtcmd.q = 0.0
            mtcmd.dq = 0.0
            mtcmd.tau = 0.0
            mtcmd.Kp = 20.0
            mtcmd.Kd = 2.0
            for t in ltau :
                mtcmd.q = ljdefault[i]
                mtcmd.tau = t[0,0]
                mtcmd.Kp = kp[i]
                mtcmd.Kd = kd[i]
                pub[i].publish(mtcmd)
                print (mtcmd.tau)
                i += 1
            for t in rtau :
                mtcmd.q = rjdefault[i - 5]
                mtcmd.tau = t[0,0]
                mtcmd.Kp = kp[i]
                mtcmd.Kd = kd[i]
                pub[i].publish(mtcmd)
                print (mtcmd.tau)
                i += 1

        elif command == "torque" :
            command = ""
            i = 0
            mtcmd = MotorCmd()
            mtcmd.q = 0.0
            mtcmd.dq = 0.0
            mtcmd.tau = 0.0
            mtcmd.Kp = 0.0
            mtcmd.Kd = 0.0
            tau = [0.0, 0.0, 0.0, -3.0, 0.0]
            for i, t in enumerate(tau) :
                mtcmd.q = motorState[i].q
                mtcmd.tau = t
                pub[i].publish(mtcmd)
                print (i, motorState[i].q, mtcmd.tau)
            for i, t in enumerate(tau, start=5) :
                mtcmd.q = motorState[i].q
                mtcmd.tau = t
                pub[i].publish(mtcmd)
                print (i, motorState[i].q, mtcmd.tau)
        elif command == "quit":
            loop = False

        if exec_cmd == "left":
            phase += dt
            phase_ = phase
            if phase > 1.0:
                phase_ = 2.0 - phase
            print(phase_)
            leg_pub(-0.3 + step_height * phase_, -0.3, 1, 0)
            if phase >= 2.0:
                exec_cmd = ""
        elif exec_cmd == "right":
            phase += dt
            phase_ = phase
            if phase > 1.0:
                phase_ = 2.0 - phase
            print(phase_)
            leg_pub(-0.3, -0.3 + step_height * phase_, 1, 0)
            if phase >= 2.0:
                exec_cmd = ""
        elif exec_cmd == "test3":
            p.append(positions[obs_joint])
            v.append(velocities[obs_joint])
            test3_count -= 1
            if test3_count <= 0:
                exec_cmd = ""
                fig = plt.figure()
                ax1 = fig.add_subplot(211)
                ax2 = fig.add_subplot(212)
                ax1.plot([t * 0.01 for t in range(len(p))], p)
                ax2.plot([t * 0.01 for t in range(len(p))], v)
                plt.show()
                with open("step_response.txt","w") as o:
                    for d in p:
                        print(d, file=o) 
                    for d in v:
                        print(d, file=o) 
                o.close()
        elif exec_cmd == "step_motion":
            phase += dt
            h_ = 4 * step_height / T * np.abs(((phase - T/4) % T) - T/2) - step_height
            if h_ > 0 :
                leg_pub(-stand_height, -stand_height + h_, 1, 0)
            else:
                leg_pub(-stand_height - h_, -stand_height, 1, 0)
            p.append(copy.deepcopy(positions))
            v.append(copy.deepcopy(velocities))
            h.append(h_)
            if phase >= 1.2:
                exec_cmd = ""
                fig = plt.figure()
                ax1 = fig.add_subplot(311)
                ax2 = fig.add_subplot(312)
                ax3 = fig.add_subplot(313)
                ax1.plot([t * 0.02 for t in range(len(p))], [d[obs_joint] for d in p])
                ax2.plot([t * 0.02 for t in range(len(p))], [d[obs_joint] for d in v])
                ax3.plot([t * 0.02 for t in range(len(p))] ,h)
                plt.show()
                with open("walk_step_record.txt","w") as o:
                    for d in p:
                        for d_ in d.values():
                            print(d_, end=',', file=o)
                        print('', file=o)
                    for d in v:
                        for d_ in d.values():
                            print(d_, end=',', file=o)
                        print('', file=o) 
                    for d in h:
                        print(d, file=o) 
                o.close()


        r.sleep()

def get_present_pos():
    global joint
    ljoint = [positions[joint_names[0]], positions[joint_names[1]], positions[joint_names[2]], positions[joint_names[3]], positions[joint_names[4]]]
    lpoints, lrots = dk(ljoint)
    rjoint = [positions[joint_names[5]], positions[joint_names[6]], positions[joint_names[7]], positions[joint_names[8]], positions[joint_names[9]]]
    rpoints, rrots = dk(rjoint)
    return ljoint, lpoints, lrots, rjoint, rpoints, rrots

def dk(joint):
    l1 = 0.1524
    l2 = 0.1542
    l3 = 0.023
    l4 = 0.04
    # ls = 0.0049

    J0 = np.matrix([[np.cos(joint[0]), -np.sin(joint[0]), 0, 0], \
                    [np.sin(joint[0]), np.cos(joint[0]), 0, 0], \
                    [0, 0, 1, 0], \
                    [0, 0, 0, 1]])
    J1 = np.matrix([[1, 0, 0, 0], \
                    [0, np.cos(joint[1]), -np.sin(joint[1]), 0], \
                    [0, np.sin(joint[1]), np.cos(joint[1]), 0], \
                    [0, 0, 0, 1]])
    J2 = np.matrix([[np.cos(joint[2]), 0, np.sin(joint[2]), 0], \
                    [0, 1, 0, 0], \
                    [-np.sin(joint[2]), 0, np.cos(joint[2]), 0], \
                    [0, 0, 0, 1]])
    J3 = np.matrix([[np.cos(joint[3]), 0, np.sin(joint[3]), 0], \
                    [0, 1, 0, 0], \
                    [-np.sin(joint[3]), 0, np.cos(joint[3]), 0], \
                    [0, 0, 0, 1]])
    J4 = np.matrix([[np.cos(joint[4]), 0, np.sin(joint[4]), 0], \
                    [0, 1, 0, 0], \
                    [-np.sin(joint[4]), 0, np.cos(joint[4]), 0], \
                    [0, 0, 0, 1]])
    J90 = np.matrix([[np.cos(pi2), 0, np.sin(pi2), 0], \
                    [0, 1, 0, 0], \
                    [-np.sin(pi2), 0, np.cos(pi2), 0], \
                    [0, 0, 0, 1]])
    Jm90 = np.matrix([[np.cos(-pi2), 0, np.sin(-pi2), 0], \
                    [0, 1, 0, 0], \
                    [-np.sin(-pi2), 0, np.cos(-pi2), 0], \
                    [0, 0, 0, 1]])
    L1 = np.matrix([[1,0,0,0],\
                    [0,1,0,0],\
                    [0,0,1,-l1],\
                    [0,0,0,1]])
    L2 = np.matrix([[1,0,0,0],\
                    [0,1,0,0],\
                    [0,0,1,-l2],\
                    [0,0,0,1]])
    L3 = np.matrix([[1,0,0,0],\
                    [0,1,0,0],\
                    [0,0,1,-l3],\
                    [0,0,0,1]])
    L4 = np.matrix([[1,0,0,0],\
                    [0,1,0,0],\
                    [0,0,1,-l4],\
                    [0,0,0,1]])
    
    P0 = J0
    P1 = J0 * J1
    P2 = P1 * J2
    P3 = P2 * L1 * J3
    P4 = P3 * L2 * J4
    toe = P4 * L3 * Jm90 * L4
    heel = P4 * L3 * J90 * L4
    return [P0[0:3,3], P1[0:3,3], P2[0:3,3], P3[0:3,3], P4[0:3,3], toe[0:3,3], heel[0:3,3]], [P0[0:3, 0:3], P1[0:3, 0:3], P2[0:3, 0:3], P3[0:3, 0:3], P4[0:3, 0:3], toe[0:3, 0:3]]
    # [p0 p1 p2 p3 p4 toe heel] [r0 r1 r2 r3 r4 rt]

def ik_jac(joint, t_pos, t_rot):
    lmd = 0.5
    points, rots = dk(joint)
    res = False
    for i in range(20):
        ja = jacobian(points, rots)
        err = CalcVWerr(t_pos, points[4], t_rot, rots[4])
        if np.linalg.norm(err) < 1e-3 :
            res = True
            break
        inv_ja = np.linalg.inv(ja)
        # print("inv_ja", inv_ja)
        # print("err", err)
        dq = lmd * inv_ja * err
        for idx in range(len(joint)):
            joint[idx] += dq[idx,0]

        points, rots = dk(joint)
        p_pos = points[4]
    return joint, res

def jacobian(p, r):
    length = len(r)-1
    ja = np.matrix([[0.0,0.0,0.0,0.0,0.0],[0.0,0.,0.,0.,0.],[0.0,0.,0.,0.,0.],[0.0,0.,0.,0.,0.],[0.0,0.,0.,0.,0.]])
    L = np.matrix([[0,0], [1,0], [0,1]])
    Z = np.matrix([[0], [0], [0]])
    for jnt in range(length):
        ja[0:3, jnt] = np.cross((r[jnt] * axis[jnt]).transpose(), (p[length-1] - p[jnt]).transpose()).transpose()
        #print((r[jnt] * axis[jnt]).transpose() * L)
        ja[3:5, jnt] = ((r[jnt] * axis[jnt]).transpose() * L).transpose()
    #print(ja)
    return ja

def CalcVWerr(t_pos, p_pos, t_rot, p_rot):
    L = np.matrix([[0,0], [1,0], [0,1]])
    perr = t_pos - p_pos
    Rerr = p_rot.transpose() * t_rot
    werr = p_rot * rot2omega(Rerr)
    # print("perr", perr)
    # print("werr", werr)
    # print("werr*L", (werr.transpose()*L).transpose())
    err = np.vstack([perr, (werr.transpose()*L).transpose()])
    return err

def rot2omega(R):
    el = np.matrix([[R[2,1]-R[1,2]], [R[0,2]-R[2,0]], [R[1,0]-R[0,1]]])
    # print("el", el)
    norm_el = np.linalg.norm(el)
    if norm_el > 0:
        w = math.atan2(norm_el, np.trace(R)-1)/norm_el * el
    elif R[0,0] > 0 and R[1,1] > 0 and R[2,2] > 0 :
        w = [[0], [0], [0]]
    else:
        w = pi2 * [[R[0,0]+1], [R[1,1]+1], R[2,2]+1]

    return w

if __name__ == '__main__':
    try:
        leg_control()
    except rospy.ROSInterruptExeception: pass
