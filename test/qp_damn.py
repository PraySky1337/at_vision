import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# ------------------- 手写 ADMM QP 求解器 -------------------

class ADMM_QP:
    """
    解决带“箱约束”（上下界）的二次规划(QP)：
        minimize   0.5 x^T H x + f^T x
        subject to lb <= x <= ub

    使用 ADMM 形式（等价于将约束写成 x = z, lb <= z <= ub），
    交替进行：
        x-update: (H + ρ I) x = -(f) + ρ (z - u)
        z-update: z = Π_[lb,ub](x + u)                # 投影到盒约束
        u-update: u = u + x - z                        # 对偶变量更新 (scaled)

    参数说明
    ----------
    H : (n,n) SPD/PSD 的 Hessian 矩阵（建议对称正定或半正定；数值不稳时可加小正则）
    f : (n,)  线性项
    lb, ub : (n,) 变量逐元素下界/上界（可含 ±np.inf）
    rho : ADMM 罚参数ρ，影响原始/对偶残差的平衡（一般 0.1~10 做初值，必要时自适应调整）
    max_iter : 最大迭代次
    eps_abs, eps_rel : 停止阈值（原始/对偶残差的绝对/相对容差）

    复杂度
    ----------
    预分解一次 Cholesky：O(n^3)
    每次迭代两个三角回代：O(n^2)
    （若 H 不变，可复用分解；若在线问题 H 固定，这部分开销摊薄后很小）

    备注
    ----------
    - 若 H 不是严格正定，(H + rho I) 仍通常是正定（rho>0），有助于稳定求解。
    - 温启动 warm-start（传入上次的解）可大幅减少迭代步数，适合 MPC 场景。
    - 可扩展：加入过松弛 α、ρ 的自适应更新、对角预条件等。
    """

    def __init__(self, H, f, lb, ub, rho=1.0, max_iter=200, eps_abs=1e-4, eps_rel=1e-4):
        # ---- 存参数（不做深拷贝，默认视为只读输入）----
        self.H = H
        self.f = f
        self.lb = lb
        self.ub = ub
        self.rho = rho
        self.max_iter = max_iter
        self.eps_abs = eps_abs
        self.eps_rel = eps_rel

        # 形成 K = H + ρ I，并做 Cholesky 分解以加速反复求解
        # 若 H 半正定/条件数较差，rho 的正数项能改善数值稳定性
        self.K = H + rho * np.eye(H.shape[0])

        # Cholesky：K = L L^T，要求 K 对称正定
        # 如分解失败，说明 K 非SPD（或数值病态），可尝试增大 rho，或对 H 加小的对角正则
        self.L = np.linalg.cholesky(self.K)
        self.LT = self.L.T

    def solve(self, warm=None):
        """
        求解器主体。

        参数
        ----------
        warm : (n,) 或 None
            温启动初值（通常来自上一次迭代/上一次时刻的解），
            若为 None 则从零向量开始。

        返回
        ----------
        z : (n,)
            QP 的近似最优解（由于强制 x=z 并投影到盒约束，返回 z 即可）
        """
        n = self.H.shape[0]

        # x 初始化：若有 warm-start 用它，无则 x=0
        x = np.zeros(n) if warm is None else warm.copy()

        # z 初始化：把 x 投影到盒约束区间 [lb, ub]
        # 注意：np.clip 支持 ±inf
        z = np.clip(x, self.lb, self.ub)

        # u 初始化（scaled dual variable），同维度零向量
        u = np.zeros_like(x)

        # 迭代主循环
        for k in range(self.max_iter):
            # ---- x-update ----
            # 解线性方程： (H + ρ I) x = -f + ρ (z - u)
            # 我们已经做了 Cholesky 分解 K = L L^T
            rhs = -self.f + self.rho * (z - u)

            # 先解 L y = rhs
            y = np.linalg.solve(self.L, rhs)
            # 再解 L^T x = y
            x = np.linalg.solve(self.LT, y)

            # ---- z-update ----
            # 先保存旧 z 用于对偶残差计算
            z_old = z
            # 在 x + u 上投影到盒约束（逐元素 clip）
            z = np.clip(x + u, self.lb, self.ub)

            # ---- u-update ----
            # scaled dual 更新：u ← u + x - z
            u = u + x - z

            # ---- 残差与停止判据 ----
            # 原始残差 r_pri = ||x - z||
            r_pri = np.linalg.norm(x - z)
            # 对偶残差 r_dual = ||ρ (z - z_old)||
            # 这是 ADMM 中“约束一致性变化”的量度
            r_dual = np.linalg.norm(self.rho * (z - z_old))

            # 停止阈值（参考 Boyd ADMM 论文表达式）
            # eps_pri = sqrt(n)*eps_abs + eps_rel*max(||x||, ||z||)
            eps_pri = np.sqrt(n) * self.eps_abs + self.eps_rel * max(np.linalg.norm(x), np.linalg.norm(z))
            # eps_dual = sqrt(n)*eps_abs + eps_rel*||ρ u||
            eps_dual = np.sqrt(n) * self.eps_abs + self.eps_rel * np.linalg.norm(self.rho * u)

            # 若原始/对偶残差均小于容差，认为收敛
            if r_pri <= eps_pri and r_dual <= eps_dual:
                break

        # 返回 z（满足盒约束），通常也可返回 x，但 x 与 z 在收敛时应接近
        return z


# ------------------- 参数与模型 -------------------
T = 0.01
N = 20
sim_steps = 800

a_max = 300.0
Q_track = 100.0
R_a = 1e-2
R_da = 1e-3

def theta_shoot_profile(t):
    base = 0.6*np.sin(0.8*t)
    if 2.0 < t < 2.5 or 4.0 < t < 4.3 or 6.0 < t < 6.4:
        base += np.pi/4
    return base

A = np.array([[1.0, T],
              [0.0, 1.0]])
B = np.array([[0.5*T**2],
              [T]])

# 预测矩阵构造
def build_prediction_mats(N, A, B, theta0, omega0):
    M = np.zeros((N,N))
    AB = np.zeros((N,2,1))
    Ap = np.eye(2)
    for i in range(N):
        AB[i] = Ap @ B
        Ap = Ap @ A
    for r in range(N):
        for c in range(r+1):
            M[r,c] = AB[r-c,0,0]
    thetas_free = np.zeros(N)
    x = np.array([theta0, omega0])
    for k in range(N):
        x = A @ x
        thetas_free[k] = x[0]
    return M, thetas_free

# ------------------- 仿真 -------------------
theta_hist, theta_ref_hist = [], []
theta_pred_N_hist = []

theta, omega = 0.0, 0.0
warm_a = None

for k in range(sim_steps):
    t_now = k*T
    theta_ref = np.array([theta_shoot_profile(t_now+(i+1)*T) for i in range(N)])
    M, c_free = build_prediction_mats(N, A, B, theta, omega)

    # 差分矩阵
    D = np.eye(N) - np.eye(N, k=1)
    D = D[:-1]

    H = (Q_track*(M.T@M) + R_a*np.eye(N) + R_da*(D.T@D))
    f = -2*Q_track*M.T@(theta_ref - c_free)

    lb = -a_max*np.ones(N)
    ub =  a_max*np.ones(N)

    solver = ADMM_QP(H, f, lb, ub, rho=1.0, max_iter=100)
    a_seq = solver.solve(warm=warm_a)
    warm_a = a_seq

    a_cmd = a_seq[0]

    omega = omega + T*a_cmd
    theta = theta + T*omega

    theta_hist.append(theta)
    theta_ref_hist.append(theta_ref[0])
    theta_pred_N_hist.append(M@a_seq + c_free)

# ---------------- 随机正弦轨迹 ----------------
class RandomSine:
    def __init__(self, base_A=0.6, base_w=0.8):
        self.A = base_A
        self.w = base_w
        self.last_update = 0.0

    def __call__(self, t):
        # 每隔 4s 随机调节一次振幅和频率
        if t - self.last_update > 4:
            self.A = 0.5 + 0.2*np.random.rand()    # 振幅 0.5~0.7
            self.w = 0.7 + 0.3*np.random.rand()    # 频率 0.7~1.0
            self.last_update = t
        return self.A * np.sin(self.w * t)

theta_shoot_profile = RandomSine()

# ---------------- 简化演示用状态更新 ----------------
T = 0.02
sim_steps = 10000
theta_hist, ref_hist = [], []
theta = 0.0

# ---------------- 动态绘制 ----------------
fig, ax = plt.subplots(figsize=(8,4))
line_hist, = ax.plot([], [], label="theta")
line_ref, = ax.plot([], [], label="ref")
ax.legend(loc="upper right")
ax.set_ylim(-1.0, 1.0)

times = []
import time
last_time = time.time()

def init():
    ax.set_xlim(0, sim_steps*T)
    return line_hist, line_ref
frame_counter = 0
start_time = time.time()

def animate(frame):
    global theta, frame_counter, start_time

    t_now = frame * T
    ref = theta_shoot_profile(t_now)
    theta += 0.1 * (ref - theta)

    theta_hist.append(theta)
    ref_hist.append(ref)

    t_arr = np.arange(len(theta_hist)) * T
    line_hist.set_data(t_arr, theta_hist)
    line_ref.set_data(t_arr, ref_hist)

    # 平均 FPS（更稳定）
    frame_counter += 1
    elapsed = time.time() - start_time
    fps = frame_counter / elapsed if elapsed > 0 else 0.0
    ax.set_title(f"Random Sine Tracking | FPS: {fps:.1f}")
    # 保持窗口宽度不变，让曲线向右滚动
    window_sec = 10.0   # 显示 5 秒宽的窗口
    t0 = max(0, t_now - window_sec)
    ax.set_xlim(t0, t0 + window_sec)

    return line_hist, line_ref

anim = FuncAnimation(fig, animate, init_func=init,
                     frames=sim_steps, interval=20, blit=True)

plt.show()