from scipy.optimize import curve_fit
import numpy as np


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
# 2次遅れ系の理論モデル
def second_order_response(t, Kp, Kd, J):
    wn = np.sqrt(Kp / J)
    zeta = Kd / (2 * np.sqrt(J * Kp))
    print("zeta:", zeta)
    # ステップ応答公式
    theta = 1 - (1 / np.sqrt(1 - zeta**2)) * np.exp(-zeta * wn * t) * np.sin(wn * np.sqrt(1 - zeta**2) * t + np.arccos(zeta))
    return theta

params, _ = curve_fit(second_order_response, t, theta_measured, p0=[10.0, 1.0, 0.01])
Kp_est, Kd_est, J_est = params

print(Kp_est, Kd_est, J_est)