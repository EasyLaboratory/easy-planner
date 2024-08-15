import numpy as np
import matplotlib.pyplot as plt

# 定义 logC2 函数
def logC2(T):
    return np.sqrt(2.0 * T - 1.0) - 1.0 if T > 1.0 else 1.0 - np.sqrt(2.0 / T - 1.0)

# 创建 T 的取值范围
T_values = np.linspace(0.1, 5, 400)  # 从 0.1 到 5 的 400 个点
logC2_values = [logC2(T) for T in T_values]  # 计算 logC2 对应的值

# 绘制函数图像
plt.figure(figsize=(10, 6))
plt.plot(T_values, logC2_values, label='logC2(T)', color='b')
plt.title('Plot of logC2 Function')
plt.xlabel('T')
plt.ylabel('logC2(T)')
plt.axhline(0, color='gray', lw=0.5, ls='--')  # 添加 y=0 的水平线
plt.axvline(1, color='gray', lw=0.5, ls='--')  # 添加 T=1 的垂直线
plt.xlim(0, 5)
plt.ylim(-2, 2)
plt.grid()
plt.legend()
plt.show()
