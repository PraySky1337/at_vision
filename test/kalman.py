import numpy as np
import matplotlib.pyplot as plt

class KalmanCV1D:
    def __init__(self, dt=0.1, q=1e-2, r=1.0):
        self.F = np.array([[1, dt],
                           [0, 1 ]], dtype=float)
        self.H = np.array([[1, 0]], dtype=float)  # 只观测位置
        self.Q = q * np.array([[dt**3/3, dt**2/2],
                               [dt**2/2, dt      ]], dtype=float)
        self.R = np.array([[r]], dtype=float)
        self.x = np.zeros((2,1))         # 初始状态 [p, v]^T
        self.P = np.eye(2) * 1e3         # 初始不确定性大

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        z = np.array([[z]], dtype=float)
        y = z - (self.H @ self.x)                          # 创新
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)           # 卡尔曼增益
        self.x = self.x + K @ y
        self.P = (np.eye(2) - K @ self.H) @ self.P
        return self.x.ravel(), y.item(), K.ravel()

# ====== 数据与滤波 ======
np.random.seed(0)
dt = 0.1
true_v = 1.5
T = 120
t = np.arange(T) * dt
true_p = true_v * t
meas = true_p + np.random.randn(T) * 2.0   # 观测噪声σ≈2

kf = KalmanCV1D(dt=dt, q=1e-2, r=2.0**2)
est_p, est_v, res, Kp, Kv = [], [], [], [], []
for z in meas:
    kf.predict()
    xhat, r_, K = kf.update(z)
    est_p.append(xhat[0]); est_v.append(xhat[1])
    res.append(r_); Kp.append(K[0]); Kv.append(K[1])

plt.figure(figsize=(10,9))

# 1) 位置
plt.subplot(3,1,1)
plt.plot(t, true_p, label="True position")
plt.plot(t, meas, 'x', alpha=0.4, label="Measurements")
plt.plot(t, est_p, label="KF estimate (pos)")
plt.ylabel("Position")
plt.title("1D Constant-Velocity Kalman Filter")
plt.legend(); plt.grid(True)

# 2) 速度
plt.subplot(3,1,2)
plt.plot(t, np.full_like(t, true_v), label="True velocity")
plt.plot(t, est_v, label="KF estimate (vel)")
plt.ylabel("Velocity")
plt.legend(); plt.grid(True)

# 3) 残差 & 增益（示意）
plt.subplot(3,1,3)
plt.plot(t, res, label="Innovation (residual)")
plt.plot(t, Kp, label="Kp")
plt.plot(t, Kv, label="Kv")
plt.xlabel("Time [s]"); plt.ylabel("Residual / Gain")
plt.legend(); plt.grid(True)

plt.tight_layout()
plt.show()
