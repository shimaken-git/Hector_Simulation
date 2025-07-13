import numpy as np
import matplotlib.pyplot as plt

# パラメータ設定
T = 0.6        # 周期
h = 0.05       # 振幅 ( +h 〜 -h )
num_cycles = 3 # 表示する周期の数
samples_per_T = int(0.6/0.02)  # 1周期あたりのサンプル数

# 時間軸の作成
t = np.linspace(0, T * num_cycles, samples_per_T * num_cycles)

# 三角波の生成
# (2h / T) * (t % T) を線形に変換して -h〜+h になるよう調整
triangle_wave = 4 * h / T * np.abs(((t - T/4) % T) - T/2) - h

print(triangle_wave)

# 描画
plt.figure(figsize=(10, 4))
plt.plot(t, triangle_wave, label='Triangle Wave')
plt.title(f'Triangle Wave: ±{h}, Period={T}')
plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.grid(True)
plt.legend()
plt.show()
