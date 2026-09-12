# l4_ksynth.py —— Note 5 K(θ) 合成：公式自验 + 平滑路径对比
import math
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

LEG = [0.0, 120.0, 240.0]        # 腿1/2/3 轴线

def sector_params(g):
    """目标方向 g(度, [0,360)) -> (主腿编号, 辅腿编号, θ)
    第一扇区是论文原例；其余五个扇区按 120° 对称性推广。"""
    if g < 60:   return 1, 2, g          # 主1@0°   辅2@120°
    if g < 120:  return 2, 1, 120 - g    # 主2@120° 辅1@0°
    if g < 180:  return 2, 3, g - 120    # 主2@120° 辅3@240°
    if g < 240:  return 3, 2, 240 - g    # 主3@240° 辅2@120°
    if g < 300:  return 3, 1, g - 240    # 主3@240° 辅1@360°
    return 1, 3, 360 - g                 # 主1@0°   辅3@240°

def K_of(theta):
    """Note 5 式(8)：K = sin(θ) / sin(120°−θ)"""
    return math.sin(math.radians(theta)) / math.sin(math.radians(120 - theta))

def synthesize(g):
    """给定期望方向 g，返回 (主+辅 合成后的实际方向, K)"""
    g = g % 360
    m, n, th = sector_params(g)
    K = K_of(th)
    um, un = unit(LEG[m-1]), unit(LEG[n-1])
    rx, ry = um[0] + K*un[0], um[1] + K*un[1]     # 主1份 + 辅K份
    return math.degrees(math.atan2(ry, rx)) % 360, K

# ---------- 实验 1：公式自验（合成角 vs 期望角） ----------
gs = list(range(0, 360, 3))
errs = []
fig, axs = plt.subplots(1, 2, figsize=(12, 5.2))
for g in gs:
    got, K = synthesize(g)
    errs.append(wrap180(got - g))
axs[0].plot(gs, gs, "k--", lw=0.8, label="理想（y=x）")
axs[0].plot(gs, [wrap180(synthesize(g)[0]) for g in gs], "r.", ms=4, label="K 合成实际方向")
axs[0].set_xlabel("期望方向 (°)"); axs[0].set_ylabel("合成方向 (°)")
axs[0].set_title(f"自验：最大偏差 {max(abs(e) for e in errs):.2e}°")
axs[0].legend(); axs[0].set_aspect("equal")

ths = [t*0.5 for t in range(121)]
axs[1].plot(ths, [K_of(t) for t in ths], "b-")
axs[1].set_xlabel("θ (°)"); axs[1].set_ylabel("K(θ)")
axs[1].set_title("Note 5 式(8)：K 从 0(单驱) 连续过渡到 1(双驱)")
axs[1].grid(alpha=0.3)
plt.tight_layout(); plt.savefig("l4_verify.png", dpi=150); plt.show()

# ---------- 实验 2：K 合成平滑路径 vs L2 之字线 ----------
P0, T = (0.0, 0.0), (80.0, 35.0)
STEP, R_ARRIVE, MAX_STEPS = 0.5, 1.0, 8000

def walk_smooth(P0, T):
    P, k = P0, 0; path = [P0]
    while dist(P, T) > R_ARRIVE and k < MAX_STEPS:
        d, K = synthesize(bearing(P, T))          # 每步重算方向并合成
        u = unit(d)
        P = (P[0] + STEP*u[0], P[1] + STEP*u[1])
        k += 1; path.append(P)
    return path

def walk_zigzag(P0, T, tau):                      # 从 L2 原样搬来
    def alpha_of(P, T): return wrap180(bearing(P, T) - 120.0)
    BASIS = [0.0, 60.0, 120.0, 180.0, 240.0, 300.0]
    P = P0; B, A = None, None
    g0 = bearing(P, T) % 360
    lo = max(b for b in BASIS if b <= g0); B, A = lo, lo + 60
    cur, other = B, A
    a0 = alpha_of(P, T); blo, bhi = a0 - tau, a0 + tau
    k = 0; sw = 0; path = [P0]
    while dist(P, T) > R_ARRIVE and k < MAX_STEPS:
        a = alpha_of(P, T)
        ux, uy = unit(cur); P_try = (P[0]+STEP*ux, P[1]+STEP*uy)
        up = alpha_of(P_try, T) > a
        if (up and a >= bhi) or ((not up) and a <= blo):
            cur, other = other, cur; sw += 1
        ux, uy = unit(cur)
        P = (P[0]+STEP*ux, P[1]+STEP*uy); k += 1; path.append(P)
    return path, sw

fig, ax = plt.subplots(figsize=(7, 6))
p1 = walk_smooth(P0, T)
p2, sw2 = walk_zigzag(P0, T, tau=10.0)
ax.plot([p[0] for p in p1], [p[1] for p in p1], "b-",  lw=1.5,
        label=f"K 合成平滑路径  步数{len(p1)-1}  路径/直线={ (len(p1)-1)*STEP/dist(P0,T):.3f}")
ax.plot([p[0] for p in p2], [p[1] for p in p2], "r-", lw=0.9,
        label=f"之字线(τ=10°)  步数{len(p2)-1}  切换{sw2}  路径/直线={ (len(p2)-1)*STEP/dist(P0,T):.3f}")
ax.plot([P0[0], T[0]], [P0[1], T[1]], "k--", lw=0.8, label="直线")
ax.plot(*T, "b*", ms=14)
ax.set_aspect("equal"); ax.legend(fontsize=8); ax.set_title("两种全向实现方式对比")
plt.savefig("l4_compare.png", dpi=150); plt.show()