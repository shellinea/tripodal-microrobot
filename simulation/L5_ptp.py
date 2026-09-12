# l5_ptp.py —— PTP 闭环导航（Fig. 5 机理重构）：偏航补偿 + K 合成 + PID
import math, random
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False

def unit(deg):
    a = math.radians(deg); return (math.cos(a), math.sin(a))
def bearing(P, T):
    return math.degrees(math.atan2(T[1]-P[1], T[0]-P[0]))
def dist(P, T):
    return math.hypot(T[0]-P[0], T[1]-P[1])
def wrap180(a):
    while a > 180:  a -= 360
    while a <= -180: a += 360
    return a

LEG = [0.0, 120.0, 240.0]
STEP, R_ARRIVE, MAX_STEPS = 0.5, 1.0, 8000

def sector_params(g):
    if g < 60:   return 1, 2, g
    if g < 120:  return 2, 1, 120 - g
    if g < 180:  return 2, 3, g - 120
    if g < 240:  return 3, 2, 240 - g
    if g < 300:  return 3, 1, g - 240
    return 1, 3, 360 - g

def K_of(theta):
    return math.sin(math.radians(theta)) / math.sin(math.radians(120 - theta))

def synthesize(g):
    g = g % 360
    m, n, th = sector_params(g)
    K = K_of(th)
    um, un = unit(LEG[m-1]), unit(LEG[n-1])
    return math.degrees(math.atan2(um[1] + K*un[1], um[0] + K*un[0])) % 360

def cross_track(P, P0, T):
    """P 到 起点→目标 连线的垂直距离（横向偏差，轨迹品质指标）"""
    dx, dy = T[0]-P0[0], T[1]-P0[1]
    L = math.hypot(dx, dy)
    return abs(dx*(P[1]-P0[1]) - dy*(P[0]-P0[0])) / L

def ptp(P0, T, comp=True, use_pid=True, drift=0.2, fix_every=10,
        Kp=0.8, Ki=0.15, seed=1):
    """fix_every：每隔多少步才更新一次位置——模拟真机上光流/视觉的
    位置更新远比控制环慢（论文"多传感器融合"的由来）。"""
    random.seed(seed)
    P_true, P_est = P0, P0
    delta = 0.0            # 机体累积偏航（真实）
    bias = 0.0             # 缓变方向偏置（真实环境）
    I = 0.0                # PID 积分累积
    C = 0.0                # PID 输出的方向修正(度)
    g_hold = bearing(P_est, T)
    k = 0
    path_t, path_e = [P0], [P0]
    cross = [0.0]
    while dist(P_est, T) > R_ARRIVE and k < MAX_STEPS:
        if k % fix_every == 0:
            g_hold = bearing(P_est, T)         # 低频位置更新（模拟光流/视觉帧率低）
        g_cmd = g_hold + C                     # PID 修正后的期望世界方向
        d = synthesize(g_cmd) - (delta if comp else 0.0)   # 换算成体指令（IMU 预回转 δ）
        bias += random.uniform(-0.25, 0.25)    # 地面/发热等引起的缓变方向偏置
        w = bias + random.uniform(-0.3, 0.3)   # 缓变偏置 + 小抖动
        # 真实世界运动：体指令 + 真实机体偏航
        wt = unit(d + w + delta)
        P_true = (P_true[0] + STEP*wt[0], P_true[1] + STEP*wt[1])
        # 里程计：机体系测到位移 d+w，再按“假设航向”换算世界系
        assum = delta if comp else 0.0
        ue = unit(d + w + assum)
        P_est = (P_est[0] + STEP*ue[0], P_est[1] + STEP*ue[1])
        delta += drift
        # PID：拿高频的 IMU/里程计航向，对抗低频位置更新之间的航向漂移
        e = wrap180(g_hold - math.degrees(math.atan2(ue[1], ue[0])))
        if use_pid:
            I += e
            C = Kp*e + Ki*I                    # D 项在纯步进仿真里作用极小，省略
        e_prev = e
        k += 1
        path_t.append(P_true); path_e.append(P_est)
        cross.append(cross_track(P_true, P0, T))
    return dict(ok=dist(P_true, T) <= R_ARRIVE, steps=k,
                err_true=dist(P_true, T), err_est=dist(P_est, T),
                max_cross=max(cross),
                path_t=path_t, path_e=path_e)

P0, T = (0.0, 0.0), (80.0, 35.0)
runs = [("无补偿+无PID", dict(comp=False, use_pid=False)),
        ("补偿+无PID",   dict(comp=True,  use_pid=False)),
        ("补偿+PID",     dict(comp=True,  use_pid=True))]

fig, axs = plt.subplots(1, 3, figsize=(16, 5.4))
for ax, (name, kw) in zip(axs, runs):
    r = ptp(P0, T, **kw)
    ax.plot([p[0] for p in r["path_t"]], [p[1] for p in r["path_t"]], "r-", lw=1.1, label="真实")
    ax.plot([p[0] for p in r["path_e"]], [p[1] for p in r["path_e"]], "g--", lw=0.8, label="估计")
    ax.plot(*T, "b*", ms=13); ax.plot(*P0, "ko", ms=5)
    ax.plot(*r["path_t"][-1], "ks", ms=6)
    ax.set_aspect("equal"); ax.legend(fontsize=8)
    ax.set_title(f"{name}\n真实到达={r['ok']} 最大横向偏差={r['max_cross']:.2f}")
plt.tight_layout(); plt.savefig("l5_ptp.png", dpi=150); plt.show()

for name, kw in runs:
    r = ptp(P0, T, **kw)
    print(f"{name:10s} 真实到达={r['ok']} 步数={r['steps']:5d} "
          f"真实误差={r['err_true']:6.2f} 估计误差={r['err_est']:5.2f} "
          f"最大横向偏差={r['max_cross']:.2f}")

from matplotlib.animation import FuncAnimation, PillowWriter

r = ptp(P0, T, comp=True, use_pid=True)
xs = [p[0] for p in r["path_t"]]; ys = [p[1] for p in r["path_t"]]
fig, ax = plt.subplots(figsize=(5.5, 5.5))
ax.plot(xs[0], ys[0], "ko", ms=5); ax.plot(*T, "b*", ms=13)
ax.plot([P0[0], T[0]], [P0[1], T[1]], "k--", lw=0.7)
ax.set_aspect("equal")
ax.set_xlim(min(xs)-3, max(xs)+3); ax.set_ylim(min(ys)-3, max(ys)+3)
line, = ax.plot([], [], "r-", lw=1.2)
head, = ax.plot([], [], "ro", ms=6)

def update(i):
    line.set_data(xs[:i], ys[:i])
    head.set_data([xs[i]], [ys[i]])
    return line, head

ani = FuncAnimation(fig, update, frames=range(0, len(xs), 4), interval=30)
ani.save("ptp.gif", writer=PillowWriter(fps=20))
print("GIF 已保存: ptp.gif")