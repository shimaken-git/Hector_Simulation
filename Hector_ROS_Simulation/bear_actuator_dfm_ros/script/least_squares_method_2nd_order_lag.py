import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares
import matplotlib.pyplot as plt

# # --- 実測データ（例） ---
# t_data = np.linspace(0, 1.0, 100)
# theta_cmd = 1.0  # ステップ入力目標角度
# # ここは実際には実測角度データに置き換えてください
# theta_measured = theta_cmd * (1 - np.exp(-t_data / 0.2)) * (1 + 0.05 * np.sin(10 * t_data))  # 擬似データ

# ファイルを開く
with open('/home/kenji/step_response_calf_pos.txt', 'r') as file:
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
t_data = np.array([i * 0.01 for i in range(len(numbers))])
theta_measured = np.array(numbers)
theta_cmd = 1.2  # ステップ入力目標角度


print(theta_measured)
# --- 運動方程式を解く関数（2次の非線形モデル） ---
def simulate_system(params, t, theta_cmd):
    Kp, Kd, J, b, tau_c = params

    def dynamics(t, y):
        theta, dtheta = y
        error = theta_cmd - theta
        torque = Kp * error - Kd * dtheta
        friction = b * dtheta + tau_c * np.sign(dtheta) if dtheta != 0 else 0
        ddtheta = (torque - friction) / J
        return [dtheta, ddtheta]

    y0 = [0.0, 0.0]
    sol = solve_ivp(dynamics, [t[0], t[-1]], y0, t_eval=t, method='RK45')
    return sol.y[0]  # θ(t)

# --- 誤差関数（最小二乗対象） ---
def residuals(params, t, theta_measured, theta_cmd):
    theta_sim = simulate_system(params, t, theta_cmd)
    return theta_sim - theta_measured


print("start")
# --- 初期推定パラメータ（Kp, Kd, J, b, tau_c） ---
initial_guess = [10.0, 1.0, 0.01, 0.05, 0.02]
bounds = ([0.1, 0.0, 1e-5, 0.0, 0.0], [100, 100, 1.0, 5.0, 1.0])
print("least")
# --- パラメータ同定 ---
result = least_squares(residuals, initial_guess, bounds=bounds,
                       args=(t_data, theta_measured, theta_cmd))
print("result")
# --- 結果表示 ---
Kp, Kd, J, b, tau_c = result.x
print(f"estimate result:")
print(f"  Kp     = {Kp:.3f}")
print(f"  Kd     = {Kd:.3f}")
print(f"  J      = {J:.5f}")
print(f"  b      = {b:.4f}")
print(f"  tau_c  = {tau_c:.4f}")

# --- 応答プロット ---
theta_simulated = simulate_system(result.x, t_data, theta_cmd)

plt.plot(t_data, theta_measured, 'o', label="actual data", markersize=4)
plt.plot(t_data, theta_simulated, '-', label="fitting result")
plt.xlabel("time [s]")
plt.ylabel("angle [rad]")
plt.legend()
plt.grid()
plt.title("non linear second oder fitting result")
plt.show()
