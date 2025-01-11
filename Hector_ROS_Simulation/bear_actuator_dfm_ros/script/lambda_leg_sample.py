#!/usr/bin/env python3

import rospy
import rosparam
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import numpy as np
import time
import sys
import math
from scipy.spatial.transform import Rotation

args = sys.argv

pub = rospy.Publisher('/lambda_leg/command', JointTrajectory, queue_size=1)

command = ""
positions = {}   #dict
joint_names = ["L_hip_joint", "L_hip2_joint", "L_thigh_joint", "L_calf_joint", "L_toe_joint", "R_hip_joint", "R_hip2_joint", "R_thigh_joint", "R_calf_joint", "R_toe_joint"]
indexlist = {}

bfr_button6 = 0
bfr_button7 = 0

i = 0
for n in joint_names:
    indexlist[n] = i
    i += 1
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

t_pos_l = [
    np.matrix([[0.15], [0.], [-0.32]]),
    np.matrix([[-0.95], [0.], [-0.32]]),
    np.matrix([[0.05], [0.1], [-0.32]]),
    np.matrix([[0.05], [-0.1], [-0.32]]),
]

def joint_publish(joint_l, joint_r, sec, nsec):
    global idxlist, pub, joint_names

    joint_ = joint_l + joint_r
    tj = JointTrajectory()
    tj.joint_names = ['L_calf_joint', 'L_hip2_joint', 'L_hip_joint', 'L_thigh_joint', 'L_toe_joint', 'R_calf_joint', 'R_hip2_joint', 'R_hip_joint', 'R_thigh_joint', 'R_toe_joint']
    pnt = JointTrajectoryPoint()
    pnt.positions = [0.0] * 10
    for i, nm in enumerate(tj.joint_names):
        pnt.positions[i] = joint_[indexlist[nm]]
    for i, j in enumerate(joint_):
        print(i, j)
    for i, p in enumerate(pnt.positions):
        print(i, p, tj.joint_names[i])
    pnt.time_from_start = rospy.Duration(sec, nsec)
    tj.points.append(pnt)
    pub.publish(tj)

def leg_pub(tgt_left_z, tgt_right_z, sec, nsec):
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

    joint_publish(joint_l, joint_r, sec, nsec)

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

def leg_control():
    global command, pub
    rospy.init_node('lambda_leg_sample')
    pub = rospy.Publisher('/lambda_leg/command', JointTrajectory, queue_size=1)
    rospy.Subscriber("joint_states", JointState, jointCb)
    rospy.Subscriber("leg_command", String, commandCb)
    rospy.Subscriber("joy", Joy, joyCb)
    
    r = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        if command == "zero" :
            print("command", command)
            tj = JointTrajectory()
            # tj.joint_names = ["L_hip_joint", "L_hip2_joint", "L_thigh_joint", "L_calf_joint", "L_toe_joint", "R_hip_joint", "R_hip2_joint", "R_thigh_joint", "R_calf_joint", "R_toe_joint"]
            tj.joint_names = ['L_calf_joint', 'L_hip2_joint', 'L_hip_joint', 'L_thigh_joint', 'L_toe_joint', 'R_calf_joint', 'R_hip2_joint', 'R_hip_joint', 'R_thigh_joint', 'R_toe_joint']
            pnt = JointTrajectoryPoint()
            pnt.positions = [0,0,0,0,0,0,0,0,0,0]
            # pnt.velocities = [0,0,0,0,0,0,0,0,0,0]
            # pnt.accelerations = [0,0,0,0,0,0,0,0,0,0]
            # pnt.effort = [0,0,0,0,0,0,0,0,0,0]
            pnt.time_from_start = rospy.Duration(1, 0)
            tj.points.append(pnt)
            pub.publish(tj)
            command = ""

        elif command == "stand" :   # set stand pose    > stand y z pitch yw  [-0.1 > z > -0.32]
            command = ""
            leg_pub(-0.3, -0.3, 1, 0)

        elif command == "left" :   # set stand pose    > stand y z pitch yw  [-0.1 > z > -0.32]
            command = ""
            leg_pub(-0.2, -0.3, 0, 50000000)
            time.sleep(0.2)
            leg_pub(-0.3, -0.3, 0, 50000000)

        elif command == "right" :   # set stand pose    > stand y z pitch yw  [-0.1 > z > -0.32]
            command = ""
            leg_pub(-0.3, -0.2, 0, 50000000)
            time.sleep(0.2)
            leg_pub(-0.3, -0.3, 0, 50000000)

        elif command == "standrpy" :   # set stand pose with roll pitch and yaw    > standrpy y z roll pitch yaw [-0.1 > z > -0.32]
            # if len(inp_list) == 6 and float(inp_list[2]) < -0.1:
            # tgt_y = float(inp_list[1])
            # tgt_z = float(inp_list[2])
            # tgt_r = float(inp_list[3])
            # tgt_p = float(inp_list[4])
            # tgt_yw = float(inp_list[5])
            tgt_y = 0.0
            tgt_z = 0.0
            tgt_r = 0.0
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
            print("tgt_r", tgt_r)
            print("tgt_p", tgt_p)
            print("tgt_yw", tgt_yw)

            t_roll = np.matrix([[1, 0, 0], \
                        [0, np.cos(tgt_r), -np.sin(tgt_r)], \
                        [0, np.sin(tgt_r), np.cos(tgt_r)]])
            t_pitch = np.matrix([[np.cos(tgt_p), 0, np.sin(tgt_p)], \
                        [0, 1, 0], \
                        [-np.sin(tgt_p), 0, np.cos(tgt_p)]])
            t_yaw = np.matrix([[np.cos(tgt_yw), -np.sin(tgt_yw), 0], \
                        [np.sin(tgt_yw), np.cos(tgt_yw), 0], \
                        [0, 0, 1]])
            body_pos_l = np.matrix([[0.0], [0.053], [0.0]])
            t_pos = np.matrix([[0.0], [0.053+tgt_y], [tgt_z]])
            print("left  : ", t_pos)
            t_pos = t_yaw.T * t_pitch.T * t_roll.T * t_pos
            t_pos = t_pos - body_pos_l
            print("left roll  : ", t_pos)
            t_rot = t_yaw * t_pitch.T
            rot = Rotation.from_matrix(t_rot)
            print(rot.as_euler('xyz'))
            joint_l, res_l = ik_jac(ljoint, t_pos, t_rot)

            body_pos_r = np.matrix([[0.0], [-0.053], [0.0]])
            t_pos = np.matrix([[0.0], [-0.053-tgt_y], [tgt_z]])
            print("right : ", t_pos)
            t_pos = t_yaw.T * t_pitch.T * t_roll.T * t_pos
            t_pos = t_pos - body_pos_r
            print("right pitch : ", t_pos)
            t_rot = t_yaw * t_pitch.T
            rot = Rotation.from_matrix(t_rot)
            print(rot.as_euler('xyz'))
            joint_r, res_r = ik_jac(rjoint, t_pos, t_rot)

            joint_publish(joint_l, joint_r)

            time.sleep(1.0)
            ljoint, lpoints, lrots, rjoint, rpoints, rrots = get_present_pos()
            print("left")
            print(lpoints[4])
            rot = Rotation.from_matrix(lrots[4])
            print(rot.as_matrix())
            print(rot.as_euler('xyz', degrees=True))
            print(rot.as_euler('xyz'))

            print("right")
            print(rpoints[4])
            rot = Rotation.from_matrix(rrots[4])
            print(rot.as_matrix())
            print(rot.as_euler('xyz', degrees=True))
            print(rot.as_euler('xyz'))
        
        inp_list = ["none", 0, 0, 0, 0]
        if inp_list[0] == "bending" :    # move stand pose    > bending y z time [-0.1 > z > -0.32]
            if len(inp_list) == 4 and float(inp_list[2]) < -0.1:
                tgt_y = float(inp_list[1])
                tgt_z = float(inp_list[2])
                times = int(inp_list[3])
                if times < 1:
                    times = 50
                ljoint, lpoints, lrots, rjoint, rpoints, rrots = get_present_pos()
                print("left")
                print(lpoints[4])
                print(lrots[4])

                print("right")
                print(rpoints[4])
                print(rrots[4])

                t_pos = np.matrix([[0.0], [tgt_y], [tgt_z]])
                t_rot_l = np.matrix([[np.cos(0), 0, np.sin(0)], \
                            [0, 1, 0], \
                            [-np.sin(0), 0, np.cos(0)]])
                dt_pos_l = lpoints[4]
                joint_l = ljoint
                d_pos_l = (t_pos - lpoints[4]) / times

                t_pos = np.matrix([[0.0], [-tgt_y], [tgt_z]])
                t_rot_r = np.matrix([[np.cos(0), 0, np.sin(0)], \
                            [0, 1, 0], \
                            [-np.sin(0), 0, np.cos(0)]])
                dt_pos_r = rpoints[4]
                joint_r = rjoint
                d_pos_r = (t_pos - rpoints[4]) / times
                for i in range(times):
                    dt_pos_l = dt_pos_l + d_pos_l 
                    joint_l, res_l = ik_jac(joint_l, dt_pos_l, t_rot_l)
                    dt_pos_r = dt_pos_r + d_pos_r 
                    joint_r, res_r = ik_jac(joint_r, dt_pos_r, t_rot_r)

                    #publish
                    time.sleep(0.01)

        elif inp_list[0] == "step" :    # step movement    > step y z cycle steps [-0.1 > z > -0.32]
            if len(inp_list) == 5 and float(inp_list[2]) < -0.1:
                tgt_y = float(inp_list[1])
                tgt_z = float(inp_list[2])
                times = int(inp_list[3])
                steps = int(inp_list[4])
                if times < 1:
                    times = 50
                if steps < 1:
                    steps = 1

                ljoint, lpoints, lrots, rjoint, rpoints, rrots = get_present_pos()

                btm_pos_l = lpoints[4]
                btm_pos_r = rpoints[4]

                top_pos_l = np.matrix([[0.0], [tgt_y], [tgt_z]])
                t_rot_l = np.matrix([[np.cos(0), 0, np.sin(0)], \
                            [0, 1, 0], \
                            [-np.sin(0), 0, np.cos(0)]])
                joint_l = ljoint

                top_pos_r = np.matrix([[0.0], [-tgt_y], [tgt_z]])
                t_rot_r = np.matrix([[np.cos(0), 0, np.sin(0)], \
                            [0, 1, 0], \
                            [-np.sin(0), 0, np.cos(0)]])
                joint_r = rjoint

                for s in range(steps):
                    side = s % 2
                    for ud in range(2):    # ud == 0 -> up  ud == 1 -> down
                        if ud == 0 :
                            dt_pos_l = btm_pos_l
                            dt_pos_r = btm_pos_r
                            tgt_pos_l = top_pos_l
                            tgt_pos_r = top_pos_r
                        else :
                            dt_pos_l = top_pos_l
                            dt_pos_r = top_pos_r
                            tgt_pos_l = btm_pos_l
                            tgt_pos_r = btm_pos_r
                        d_pos_l = (tgt_pos_l - dt_pos_l) / times
                        d_pos_r = (tgt_pos_r - dt_pos_r) / times
                        print("step", s, " ud", ud)
                        for i in range(times):
                            if side == 0:
                                dt_pos_l = dt_pos_l + d_pos_l 
                                joint_l, res_l = ik_jac(joint_l, dt_pos_l, t_rot_l)
                                print(i, "left", dt_pos_l[2])
                            else :
                                dt_pos_r = dt_pos_r + d_pos_r 
                                joint_r, res_r = ik_jac(joint_r, dt_pos_r, t_rot_r)
                                print(i, "right", dt_pos_r[2])

                            #publish

                            time.sleep(0.01)

        elif inp_list[0] == "swing" :    # move swing    > swing width cycle steps
            if len(inp_list) == 4:
                width = float(inp_list[1])
                times = int(inp_list[2])
                steps = int(inp_list[3])
                if times < 1:
                    times = 50
                if steps < 1:
                    steps = 1

                ljoint, lpoints, lrots, rjoint, rpoints, rrots = get_present_pos()

                neutral_pos_l = lpoints[4]
                neutral_pos_r = rpoints[4]

                left_inc = np.matrix([[0], [width], [0]])   #左振り増分
                right_inc = np.matrix([[0], [-width], [0]])  #右振り増分

                t_rot_l = np.matrix([[np.cos(0), 0, np.sin(0)], \
                            [0, 1, 0], \
                            [-np.sin(0), 0, np.cos(0)]])
                joint_l = ljoint

                t_rot_r = np.matrix([[np.cos(0), 0, np.sin(0)], \
                            [0, 1, 0], \
                            [-np.sin(0), 0, np.cos(0)]])
                joint_r = rjoint

                for s in range(steps):
                    lr = s % 2    # lr == 0 -> left  lr == 1 -> right
                    for oi in range(2):    # oi == 0 -> outbound  oi == 1 -> inbound
                        if lr == 0 :
                            width_inc = left_inc
                        else :
                            width_inc = right_inc
                        if oi == 0 :
                            dt_pos_l = neutral_pos_l
                            dt_pos_r = neutral_pos_r
                            tgt_pos_l = neutral_pos_l + width_inc
                            tgt_pos_r = neutral_pos_r + width_inc
                        else :
                            dt_pos_l = neutral_pos_l + width_inc
                            dt_pos_r = neutral_pos_r + width_inc
                            tgt_pos_l = neutral_pos_l
                            tgt_pos_r = neutral_pos_r
                        d_pos_l = (tgt_pos_l - dt_pos_l) / times
                        d_pos_r = (tgt_pos_r - dt_pos_r) / times
                        print("step", s, " lr", lr)
                        for i in range(times):
                            dt_pos_l = dt_pos_l + d_pos_l 
                            joint_l, res_l = ik_jac(joint_l, dt_pos_l, t_rot_l)
                            print(i, "left", dt_pos_l[1])
                            dt_pos_r = dt_pos_r + d_pos_r 
                            joint_r, res_r = ik_jac(joint_r, dt_pos_r, t_rot_r)
                            print(i, "right", dt_pos_r[1])

                            #publish

                            time.sleep(0.01)

        elif inp_list[0] == "stroke" :    # move stroke    > stroke stroke cycle steps
            if len(inp_list) == 4:
                stroke = float(inp_list[1])
                times = int(inp_list[2])
                steps = int(inp_list[3])
                if times < 1:
                    times = 50
                if steps < 1:
                    steps = 1

                ljoint, lpoints, lrots, rjoint, rpoints, rrots = get_present_pos()

                neutral_pos_l = lpoints[4]
                neutral_pos_r = rpoints[4]

                front_inc = np.matrix([[stroke], [0], [0]])   #前振り増分
                back_inc = np.matrix([[-stroke], [0], [0]])  #後ろ振り増分

                t_rot_l = np.matrix([[np.cos(0), 0, np.sin(0)], \
                            [0, 1, 0], \
                            [-np.sin(0), 0, np.cos(0)]])
                joint_l = ljoint

                t_rot_r = np.matrix([[np.cos(0), 0, np.sin(0)], \
                            [0, 1, 0], \
                            [-np.sin(0), 0, np.cos(0)]])
                joint_r = rjoint

                for s in range(steps):
                    lr = s % 2    # lr == 0 -> left  lr == 1 -> right
                    for oi in range(2):    # oi == 0 -> outbound  oi == 1 -> inbound
                        if lr == 0 :
                            left_stroke_inc = front_inc
                            right_stroke_inc = back_inc
                        else :
                            left_stroke_inc = back_inc
                            right_stroke_inc = front_inc
                        if oi == 0 :
                            dt_pos_l = neutral_pos_l
                            dt_pos_r = neutral_pos_r
                            tgt_pos_l = neutral_pos_l + left_stroke_inc
                            tgt_pos_r = neutral_pos_r + right_stroke_inc
                        else :
                            dt_pos_l = neutral_pos_l + left_stroke_inc
                            dt_pos_r = neutral_pos_r + right_stroke_inc
                            tgt_pos_l = neutral_pos_l
                            tgt_pos_r = neutral_pos_r
                        d_pos_l = (tgt_pos_l - dt_pos_l) / times
                        d_pos_r = (tgt_pos_r - dt_pos_r) / times
                        print("step", s, " lr", lr)
                        for i in range(times):
                            dt_pos_l = dt_pos_l + d_pos_l 
                            joint_l, res_l = ik_jac(joint_l, dt_pos_l, t_rot_l)
                            print(i, "left", dt_pos_l[0])
                            dt_pos_r = dt_pos_r + d_pos_r 
                            joint_r, res_r = ik_jac(joint_r, dt_pos_r, t_rot_r)
                            print(i, "right", dt_pos_r[0])

                            #publish

                            time.sleep(0.01)

        elif inp_list[0] == "quit":
            loop = False

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
