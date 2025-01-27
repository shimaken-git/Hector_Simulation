import numpy as np
import matplotlib.pyplot as plt

# システムモデルのパラメータ
dt = 0.001  # 時間ステップ（秒）
A = np.array([[1, dt], [0, 1]])  # 状態遷移行列
H = np.array([[1, 0]])           # 観測行列
Q = np.array([[0.01, 0],         # プロセスノイズ共分散
              [0, 0.1]])
R = np.array([[0.5]])            # 観測ノイズ共分散
I = np.eye(2)                    # 単位行列

# 初期値
x = np.array([[0], [1]])         # 初期状態 [位置, 速度]
P = np.eye(2)                    # 初期共分散行列

# サンプルデータ生成
np.random.seed(42)
time = np.arange(0, 10, dt)
true_position = np.sin(2 * np.pi * 0.2 * time)  # 真の位置
observed_position = true_position + np.random.normal(0, np.sqrt(R[0, 0]), len(time))  # 観測値（ノイズ付き）

# 推定結果の保存
estimated_positions = []
estimated_velocities = []

# カルマンフィルタの適用
for z in observed_position:
    z = np.array([[z]])  # 観測値を列ベクトルに変換
    # print(z)

    # 予測ステップ
    x = A @ x
    P = A @ P @ A.T + Q

    # 更新ステップ
    K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)  # カルマンゲイン
    x = x + K @ (z - H @ x)
    P = (I - K @ H) @ P

    # 推定結果の保存
    estimated_positions.append(x[0, 0])
    estimated_velocities.append(x[1, 0])

# 結果のプロット
plt.figure(figsize=(12, 6))

# 位置のプロット
plt.subplot(2, 1, 1)
plt.plot(time, observed_position, label="Observed Position", alpha=0.6)
plt.plot(time, estimated_positions, label="Estimated Position", linewidth=2)
plt.plot(time, true_position, label="True Position", linestyle="--", linewidth=2)
plt.legend()
plt.xlabel("Time (s)")
plt.ylabel("Position")
plt.title("Position Estimation with Kalman Filter")

# 速度のプロット
plt.subplot(2, 1, 2)
plt.plot(time, [2 * np.pi * 0.2 * np.cos(2 * np.pi * 0.2 * t) for t in time], 
         label="True Velocity", linestyle="--", linewidth=2)
plt.plot(time, estimated_velocities, label="Estimated Velocity", linewidth=2)
plt.legend()
plt.xlabel("Time (s)")
plt.ylabel("Velocity")
plt.title("Velocity Estimation with Kalman Filter")

plt.tight_layout()
plt.show()
