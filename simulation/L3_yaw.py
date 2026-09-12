# l3_yaw.py —— 偏航漂移 δ 的危害与 IMU 补偿（Fig. S4c/d）
import math
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False

def unit(deg):
    a = math.radians(deg); return (math.cos(a), math.sin(a))
def wrap180(a):
    while a > 180:  a -= 360
    while a <= -180: a += 360
    return a
def bearing(P, T):
    return math.degrees(math.atan2(T[1]-P[1], T[0]-P[0]))
def dist(P, T):
    return math.hypot(T[0]-P[0], T[1]-P[1])

LEG2_AXIS = 120.0
BASIS = [0.0, 60.0, 120.0, 180.0, 240.0, 300.0]
STEP, R_ARRIVE, MAX_STEPS = 0.5, 1.0, 8000

def alpha_of(P, T):
    return wrap180(bearing(P, T) - LEG2_AXIS)

def pick_bracket(P, T):
    g0 = bearing(P, T) % 360
    lo = max(b for b in BASIS if b <= g0)
    return lo, lo + 60.0

def interpolate_yaw(P0, T, tau, drift, compensate):
    """
    drift: 每步机体偏航漂移(度/步)；compensate: 是否用 IMU 补偿
    返回: 真实轨迹, 估计轨迹, 是否(按估计位置)到达
    """
    P_true, P_est = P0, P0          # 真实位置 / 机器人估计位置
    delta = 0.0                     # 机体累积偏航（真实值）
    B, A = pick_bracket(P_est, T)
    cur, other = B, A
    a0 = alpha_of(P_est, T)
    lo, hi = a0 - tau, a0 + tau
    k = 0; path_t = [P0]; path_e = [P0]
    while dist(P_est, T) > R_ARRIVE and k < MAX_STEPS:
        a = alpha_of(P_est, T)                       # 机器人按“它以为的位置”算 α
        ux, uy = unit(cur)
        P_try = (P_est[0] + STEP*ux, P_est[1] + STEP*uy)
        push_up = alpha_of(P_try, T) > a
        if (push_up and a >= hi) or ((not push_up) and a <= lo):
            cur, other = other, cur
        # 关键三行：
        cmd = cur - (delta if compensate else 0.0)   # IMU 补偿：指令预回转 δ（世界系意图→体指令）
        wt = unit(cmd + delta)                       # 真实世界方向 = 体指令 + 机体偏航
        we = unit(cmd + (delta if compensate else 0.0))  # 里程计按“假设航向”换算
        P_true = (P_true[0] + STEP*wt[0], P_true[1] + STEP*wt[1])
        P_est  = (P_est[0]  + STEP*we[0], P_est[1]  + STEP*we[1])
        delta += drift                               # 机体继续慢慢偏
        k += 1; path_t.append(P_true); path_e.append(P_est)
    return path_t, path_e, dist(P_true, T) <= R_ARRIVE

P0, T = (0.0, 0.0), (80.0, 35.0)
fig, axs = plt.subplots(1, 2, figsize=(13, 6))
for ax, comp in zip(axs, [False, True]):
    pt, pe, ok = interpolate_yaw(P0, T, tau=10.0, drift=0.2, compensate=comp)
    ax.plot([p[0] for p in pt], [p[1] for p in pt], "r-",  lw=1.2, label="真实轨迹")
    ax.plot([p[0] for p in pe], [p[1] for p in pe], "g--", lw=0.9, label="机器人以为的轨迹")
    ax.plot(*T, "b*", ms=14, label="目标")
    ax.plot(*pt[-1], "ks", ms=6, label="真实终点")
    ax.set_aspect("equal"); ax.legend(fontsize=8)
    ax.set_title(("补偿后：估计=真实，正常到达" if comp else
                  "无补偿：位置估计漂移，真实轨迹偏离目标") + f"（到达={ok}）")
plt.tight_layout(); plt.savefig("l3_yaw.png", dpi=150); plt.show()