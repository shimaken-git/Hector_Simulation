#!/usr/bin/env python3

import rospy
import rosparam
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import numpy as np
import time
import sys
import math
from scipy.spatial.transform import Rotation

args = sys.argv

# システムモデルのパラメータ
dt = 0.001  # 時間ステップ（秒）
A = np.array([[1, dt], [0, 1]])  # 状態遷移行列
H = np.array([[1, 0]])           # 観測行列
Q = np.array([[0.001, 0],         # プロセスノイズ共分散
              [0, 0.001]])
R = np.array([[0.25]])            # 観測ノイズ共分散
I = np.eye(2)                    # 単位行列

# 初期値
x = np.array([[0], [1]])         # 初期状態 [位置, 速度]
P = np.eye(2)                    # 初期共分散行列

pi2 = np.pi / 2

def vworldCb(msg):
    print(msg)
    out = Vector3()
    out.z = kalman_filter(msg.z)
    pub.publish(out)

def main():
    global pub
    rospy.init_node('kalman_filter_node')
    pub = rospy.Publisher('/kalman_out', Vector3, queue_size=1)
    rospy.Subscriber("/vworld", Vector3, vworldCb)
    
    r = rospy.Rate(1000)  # 1000Hz
    while not rospy.is_shutdown():
        r.sleep()

def kalman_filter(z):

# サンプルデータ生成
# np.random.seed(42)
# time = np.arange(0, 10, dt)
# true_position = np.sin(2 * np.pi * 0.2 * time)  # 真の位置
# observed_position = true_position + np.random.normal(0, np.sqrt(R[0, 0]), len(time))  # 観測値（ノイズ付き）

# # 推定結果の保存
# estimated_positions = []
# estimated_velocities = []

# カルマンフィルタの適用
    global dt, A, H, Q, R, I, x, P

    z = np.array([[z]])  # 観測値を列ベクトルに変換

    # 予測ステップ
    x = A @ x
    P = A @ P @ A.T + Q

    # 更新ステップ
    K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)  # カルマンゲイン
    x = x + K @ (z - H @ x)
    P = (I - K @ H) @ P

    # 推定結果
    return x[0, 0]


if __name__ == '__main__':
    main()
