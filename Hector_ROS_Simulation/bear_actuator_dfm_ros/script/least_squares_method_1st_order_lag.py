import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# --- 測定データ（例） ---
# 実際にはここに実測の時間と角度データを入れてください
# t = np.array([0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4])
# theta_measured = np.array([0, 0.5, 1.2, 1.7, 1.9, 2.0, 2.05, 2.07, 2.08])  # 例: [rad]

# ファイルを開く
with open('/home/kenji/step_response_1_pos.txt', 'r') as file:
    # 行単位で読み込む (readlines())
    lines = file.readlines()

# 数値に変換してリストに格納
numbers = []
for line in lines:
    # 行末の改行を取り除く
    line = line.strip()
    try:
        # floatに変換
        number = float(line)
        numbers.append(number)
    except ValueError:
        print(f"数値に変換できない行: {line}")

# 実測ステップ応答
t = np.array([i * 0.01 for i in range(len(numbers))])
print(t)
theta_measured = np.array(numbers)
print(theta_measured)


# --- ステップ応答モデル（1次遅れ系） ---
def first_order_step_response(t, Kp, tau):
    return Kp * (1 - np.exp(-t / tau))

# --- フィッティング ---
initial_guess = [2.0, 0.1]  # 初期推定値 [Kp, tau]
params, covariance = curve_fit(first_order_step_response, t, theta_measured, p0=initial_guess)

Kp_fit, tau_fit = params
print(f"推定された Kp: {Kp_fit:.4f}, tau: {tau_fit:.4f}")

# --- フィッティング結果のプロット ---
t_fit = np.linspace(0, max(t), 100)
theta_fit = first_order_step_response(t_fit, Kp_fit, tau_fit)

plt.plot(t, theta_measured, 'o', label="actual")
plt.plot(t_fit, theta_fit, '-', label="fitting")
plt.xlabel("time [s]")
plt.ylabel("angle [rad]")
plt.legend()
plt.grid(True)
plt.title("step response and fitting result")
plt.show()
