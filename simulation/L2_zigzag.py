# l2_zigzag.py —— Note 2 插补算法之字线 + 阈值扫描
import math
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False

# ---------- 工具函数 ----------
def unit(deg):
    a = math.radians(deg)
    return (math.cos(a), math.sin(a))

def wrap180(a):                       # 规范到 (-180, 180]
    while a > 180:  a -= 360
    while a <= -180: a += 360
    return a

def bearing(P, T):                    # P 指向 T 的方位角（度）
    return math.degrees(math.atan2(T[1]-P[1], T[0]-P[0]))

def dist(P, T):                       # P 到 T 的距离
    return math.hypot(T[0]-P[0], T[1]-P[1])

# ---------- 机器人和算法参数 ----------
LEG2_AXIS = 120.0                     # α 以腿 2 轴线为参考（论文定义）
BASIS = [0.0, 60.0, 120.0, 180.0, 240.0, 300.0]
STEP = 0.5                            # 每步距离
R_ARRIVE = 1.0                        # 到达半径
MAX_STEPS = 500000

def alpha_of(P, T):                   # 插补参考角 α
    return wrap180(bearing(P, T) - LEG2_AXIS)

def pick_bracket(P, T):
    """起点—目标连线落在哪个 60° 扇区 -> 返回 (B下, A上) 两个基向量角度"""
    g0 = bearing(P, T) % 360# 算出目标T相对于起点P的绝对角度
    lo = max(b for b in BASIS if b <= g0)   # g0 不会恰好压线，教程不处理边界
    return lo, lo + 60.0# 返回扇区的下边线B和上边线A

def interpolate(P0, T, tau):
    """Note 2 插补。tau=阈值(度)。返回 轨迹, 切换次数, 是否到达"""
    P = P0
    B, A = pick_bracket(P, T)
    cur, other = B, A                 # 论文：先沿 B 走
    a0 = alpha_of(P, T)
    lo, hi = a0 - tau, a0 + tau       # α 允许的活动带
    k = 0; switches = 0; path = [P0]
    while dist(P, T) > R_ARRIVE and k < MAX_STEPS:
        a = alpha_of(P, T)
        # 虚拟试探一步：当前基向量会推高还是压低 α？
        ux, uy = unit(cur)
        P_try = (P[0] + STEP*ux, P[1] + STEP*uy)
        push_up = alpha_of(P_try, T) > a
        # 顶到带边了就换另一个基向量
        if (push_up and a >= hi) or ((not push_up) and a <= lo):
            cur, other = other, cur
            switches += 1
        ux, uy = unit(cur)
        P = (P[0] + STEP*ux, P[1] + STEP*uy)
        k += 1; path.append(P)
    return path, switches, dist(P, T) <= R_ARRIVE

# ---------- 实验 1：单次跑，画之字线 ----------
P0, T = (0.0, 0.0), (80.0, 35.0)
path, sw, ok = interpolate(P0, T, tau=10.0)
xs = [p[0] for p in path]; ys = [p[1] for p in path]
print(f"到达: {ok} | 步数: {len(path)-1} | 切换: {sw} 次0.1 | "
      f"终点误差: {dist(path[-1], T):.2f}")

fig, ax = plt.subplots(figsize=(7, 6))
ax.plot(xs, ys, "r-", lw=1, label="插补轨迹")
ax.plot([P0[0], T[0]], [P0[1], T[1]], "k--", lw=0.8, label="直线（最短路）")
ax.plot(*P0, "go", ms=8, label="起点"); ax.plot(*T, "b*", ms=14, label="目标")
ax.set_aspect("equal"); ax.legend(); ax.set_title("插补之字线 (τ=10°)")
plt.savefig("l2_zigzag.png", dpi=150); plt.show()

# ---------- 实验 2：阈值扫描（致密化程度） ----------
# ---------- 实验 2：扩展版阈值扫描（包含极小阈值） ----------
taus = [0.1, 0.5, 2.0, 5.0, 10.0, 20.0]  # <- 你主导增加了 0.5 和 1.0
fig, axs = plt.subplots(2, 3, figsize=(15, 9))  # 改为 2行3列

for ax, tau in zip(axs.flat, taus):
    print(f"正在计算 τ={tau}° ...")
    path, sw, ok = interpolate(P0, T, tau=tau)
    xs = [p[0] for p in path]; ys = [p[1] for p in path]
    
    # 关键：极小阈值时线宽要变细，否则之字线会糊成一团黑块
    line_width = 0.3 if tau <= 2 else 0.8 
    
    ax.plot(xs, ys, "r-", lw=line_width)
    ax.plot([P0[0], T[0]], [P0[1], T[1]], "k--", lw=0.6)
    ax.plot(*T, "b*")
    ax.set_aspect("equal")
    ax.set_title(f"τ={tau}°  步数{len(path)-1}  切换{sw}次  {'到达' if ok else '未到达'}")

plt.tight_layout()
plt.savefig("l2_sweep_extended.png", dpi=150)
plt.show()


# ---------- 实验 2：交互式阈值探索（你作为决策主导） ----------
# print("\n====== 插补算法仿真控制台 ======")
# print("当前最大步数限制:", MAX_STEPS)

# while True:
#     user_input = input("\n请输入你想测试的阈值(度)，例如 0.5 或 1 (输入 q 退出): ")
#     if user_input.lower() == 'q':
#         print("退出仿真。")
#         break
    
#     try:
#         tau = float(user_input)
#         if tau <= 0:
#             print("阈值必须大于0，请重新输入！")
#             continue
#     except ValueError:
#         print("输入无效，请输入数字！")
#         continue

#     print(f"正在计算 τ={tau}° 的轨迹，请稍候...")
#     # 运行算法
#     path, sw, ok = interpolate(P0, T, tau=tau)
#     xs = [p[0] for p in path]; ys = [p[1] for p in path]
    
#     # 打印决策反馈
#     print(f"-> 结果: {'到达' if ok else '未到达'} | 步数: {len(path)-1} | 切换: {sw} 次")
#     if not ok:
#         print("⚠️ 警告：未到达目标！可能是 MAX_STEPS 不够，请考虑增大 MAX_STEPS 或提高 τ。")

#     # 绘制你主导的单一实验图
#     fig, ax = plt.subplots(figsize=(7, 6))
#     ax.plot(xs, ys, "r-", lw=0.8, label=f"轨迹 (τ={tau}°)")
#     ax.plot([P0[0], T[0]], [P0[1], T[1]], "k--", lw=0.6, label="理想直线")
#     ax.plot(*P0, "go", ms=8, label="起点"); ax.plot(*T, "b*", ms=14, label="目标")
#     ax.set_aspect("equal"); ax.legend()
#     ax.set_title(f"你的决策: τ={tau}°  切换{sw}次  {'到达' if ok else '未到达'}")
#     plt.show()











