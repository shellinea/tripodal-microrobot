# 插补与导航算法数值仿真——零基础保姆级教程

> 2026-09-03 ｜ 机理依据：文献 [1] 补充材料 Note 2（插补算法）、Note 5（K(θ) 力分配）、主文 Fig. 5（PTP 导航），References 见仓库 README
> 目标读者：会 Python 基本语法、但从未做过数值仿真的人。
> 总用时估计：环境 30 min → L1 半小时 → L2/L3/L4 各 1~1.5 h → L5 约 2 h。**按顺序做，跳关会卡住。**
> 每一关的代码都是**完整可独立运行**的：新建文件、整段复制、运行、对照"你应该看到什么"。卡住了记下报错信息再来问。

---

## 0. 我们到底要仿真什么？（先读懂再动手）

**数值仿真** = 把机器人的运动拆成一个个小时间步，每一步用公式算"这一步往哪走、走多远"，重复几千次，把轨迹画出来。没有真实机器人，全靠数学。

我们要验证的是论文的三个机制，对应三个关卡：

| 机制                                                           | 文献出处                | 仿真关卡 |
| -------------------------------------------------------------- | ----------------------- | -------- |
| 插补之字线：在两个基向量间来回切换逼近目标                     | Note 2 + Fig. S4        | L2、L3   |
| K(θ) 力分配：两条腿按 sinθ/sin(120°−θ) 配比，合成任意方向 | Note 5 式(8) + Fig. S8c | L4       |
| PTP 闭环导航：每步重算方位 + PID 修正 + 偏航补偿               | 主文 Fig. 5             | L5       |

**诚实声明**：K(θ) 公式和"第一扇区"的例题是论文原文；其余五个扇区的配比是我按对称性推广的（论文只详写了第一扇区）；切换状态机和 PID 的具体实现论文没开源，我按 Note 2 / Fig. 5 的机理重构——这正是"先仿真后固件"要自己补的部分。

## 1. 数学预备（10 分钟，看懂下面的约定才能看懂代码）

**角度**：本仿真全部用"度"思考（0°=x 正方向，逆时针为正），代码里用 `math.radians()` 转成弧度再喂给 `math.sin/cos`（这两个函数只吃弧度）。**这是新手第一大坑**。

**方位向量**：想要"沿 37° 方向走一步"，就是走 `(cos37°, sin37°)`——这个 `(cos, sin)` 对叫单位向量（长度为 1 的箭头）。代码里我们写个小工具函数 `unit(deg)` 返回它。

**atan2**：已知"从 P 点看 T 点的方向"，用 `atan2(Ty−Py, Tx−Px)` 算出方位角。为什么不用 `atan`？因为 `atan2` 能根据正负号判断象限，全 360° 都对。

**机器人的几何设定**（对应论文 Fig. S4a / S8c）：

```
        y
        │      腿2轴(120°)   双腿2+3(180°)
        │  双腿1+2(60°) ＼  ｜  ／
        │            ＼ ｜ ／
        │ 腿1轴(0°)     ＼｜／
────────●────────────────────── x
       起点          腿3轴(240°)、双腿3+1(300°) 在下半平面
```

- 三条腿的轴线固定在 **0°、120°、240°**（腿1/2/3），机体永不旋转；
- **6 个基向量**：3 个单驱方向（单开一条腿，沿该腿轴线：0°/120°/240°）+ 3 个双驱方向（开两条腿等力，合成在两腿夹角平分线上：60°/180°/300°）。相邻基向量夹 60°（论文叫"包角 envelope angle = 60°"）；
- **α（插补参考角）**：论文定义 = "当前位置—目标点连线" 相对 **腿 2 轴线** 的夹角。机器人移动时连线方向会变，α 随之变化：**沿一个基向量走 α 升、沿另一个走 α 降**——这就是开关的依据；
- **K(θ)**（Note 5 式 8）：要往与主腿轴线夹 θ 角的方向走，辅腿出力系数取 `K = sin(θ)/sin(120°−θ)`，K∈[0,1]。θ=0 → K=0（退化为单驱），θ=60° → K=1（等力双驱）。

**怎么"走一步"**：`P_new = P_old + 步长 × unit(方向角)`。仿真里步长固定 0.5（单位随便，当毫米看）。

---

## 2. 环境搭建（Windows，30 分钟）

### 2.1 装 Python

1. 打开 https://www.python.org/downloads/ ，下载 Python 3.12（或 3.10+ 都行）的 Windows installer（64-bit）；
2. 运行安装器，**第一个界面最底下务必勾选 "Add python.exe to PATH"**（不勾后面所有命令都会报"不是内部或外部命令"），然后点 Install Now；
3. 验证：按 `Win+R` 输入 `powershell` 回车，在蓝窗口里输入：

```
python --version
```

看到 `Python 3.12.x` 即成功。

### 2.2 装库（用清华镜像，快得多）

在同一个窗口输入：

```
python -m pip install matplotlib pillow -i https://pypi.tuna.tsinghua.edu.cn/simple
```

- `matplotlib`：画图 + 动画（会自动带上 numpy）；
- `pillow`：把动画存成 GIF 用。

### 2.3 建工作目录 + 冒烟测试

1. 在你的项目根目录下新建文件夹 `simulation`（后面所有 .py 都放这里）；
2. 在 `simulation` 里新建文本文件 `smoke.py`（用记事本或 VS Code 都行），内容：

```python
import math
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]   # 让图里能显示中文
plt.rcParams["axes.unicode_minus"] = False               # 让负号正常显示

plt.plot([0, 1, 2], [0, 1, 4], "b-")
plt.title("冒烟测试")
plt.savefig("smoke.png", dpi=150)
plt.show()
```

3. 在 PowerShell 里 `cd` 过去再运行：

```
cd <你的项目目录>\simulation
python smoke.py
```

**你应该看到**：弹出一个窗口，蓝色折线 + 中文标题"冒烟测试"，且文件夹里多出 `smoke.png`。

> 建议：装个 VS Code（https://code.visualstudio.com/ ）+ 扩展商店里的 Python 插件，写代码有高亮和报错提示。不装、纯记事本也行。

---

## 3. L1 几何关卡：把机器人的"骨架"画出来

**任务**：画出三条腿轴线和 6 个基向量箭头，直观理解"6 个方向、间隔 60°"。

新建 `l1_geometry.py`：

```python
# l1_geometry.py —— 画三足机器人的 6 个基向量（对应 Fig. S4a）
import math
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False

def unit(deg):
    """角度(度) -> 单位向量 (x, y)"""
    a = math.radians(deg)
    return (math.cos(a), math.sin(a))

LEG = [0.0, 120.0, 240.0]        # 腿1/腿2/腿3 的轴线方向
BASIS = [0.0, 60.0, 120.0, 180.0, 240.0, 300.0]   # 6 个基向量
NAME  = ["单驱腿1", "双驱1+2", "单驱腿2", "双驱2+3", "单驱腿3", "双驱3+1"]

fig, ax = plt.subplots(figsize=(6, 6))
for b, name in zip(BASIS, NAME):
    ux, uy = unit(b)
    color = "tab:red" if "单驱" in name else "tab:blue"   # 单驱红，双驱蓝
    ax.arrow(0, 0, ux*2, uy*2, head_width=0.08,
             length_includes_head=True, color=color)
    ax.text(ux*2.45, uy*2.45, f"{name}\n{b:.0f}°", ha="center", color=color)

ax.axhline(0, color="gray", lw=0.5); ax.axvline(0, color="gray", lw=0.5)
ax.set_xlim(-2.6, 2.6); ax.set_ylim(-2.6, 2.6)
ax.set_aspect("equal")           # 关键！不加这个圆会画成椭圆
ax.set_title("6 个基向量（相邻夹 60°）")
plt.savefig("l1_basis.png", dpi=150)
plt.show()
```

**逐行讲四个要点**：

- `unit(deg)`：全文最重要的工具函数，角度→单位向量，之后每关都靠它"朝某方向走一步"；
- `ax.arrow(0,0, ux*2, uy*2, ...)`：从原点画箭头到 `(2cosθ, 2sinθ)`，乘 2 只是为了箭头长一点；`color=` 参数让单驱/双驱颜色区分（**注意：先算好 `color` 变量再传进去——算了不用是新手常见 bug**）；
- `ax.set_aspect("equal")`：让 x、y 比例一致。**忘了它，60° 的夹角看起来就不是 60°**，新手第二大坑。
- 标签放在 2.45 倍半径处：比箭头尖端（2 倍）稍远，避免文字压住箭头。

**你应该看到**：6 支箭头均匀分布一圈，红蓝相间，相邻夹 60°。

**自查**：为什么单驱方向只有 3 个、双驱也只有 3 个，加起来正好 6 个？——单驱沿腿轴（3 条腿互成 120°），双驱等力合成在两腿平分线（也是 3 条，恰好插在单驱方向正中间）。

---

## 4. L2 插补之字线：实现 Note 2 的核心算法

**论文的算法逻辑**（Note 2 原文翻译）：

1. 起点—目标连线落在哪两个相邻基向量的 60° 扇区里，这两个就是**基向量 A（上）和 B（下）**；
2. α 定义为连线相对**腿 2 轴线**的夹角。沿一个基向量走 α 会升，沿另一个走 α 会降；
3. 先沿 B 走 → α 降到下阈值 → 切到 A → α 升到上阈值 → 切回 B……交替直到到达目标；
4. **阈值越小，之字线越密、切换越频繁**（论文叫"致密化程度"）。

实现上有一个小技巧：某个基向量到底"推高还是压低 α"，不用背，**每一步虚拟地试探一步**看 α 变大变小即可——这样无论目标在哪个扇区都不会错。

新建 `l2_zigzag.py`：

```python
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

def dist(P, T):
    return math.hypot(T[0]-P[0], T[1]-P[1])

# ---------- 机器人和算法参数 ----------
LEG2_AXIS = 120.0                     # α 以腿 2 轴线为参考（论文定义）
BASIS = [0.0, 60.0, 120.0, 180.0, 240.0, 300.0]
STEP = 0.5                            # 每步距离
R_ARRIVE = 1.0                        # 到达半径
MAX_STEPS = 20000

def alpha_of(P, T):                   # 插补参考角 α
    return wrap180(bearing(P, T) - LEG2_AXIS)

def pick_bracket(P, T):
    """起点—目标连线落在哪个 60° 扇区 -> 返回 (B下, A上) 两个基向量角度"""
    g0 = bearing(P, T) % 360
    lo = max(b for b in BASIS if b <= g0)   # g0 不会恰好压线，教程不处理边界
    return lo, lo + 60.0

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
print(f"到达: {ok} | 步数: {len(path)-1} | 切换: {sw} 次 | "
      f"终点误差: {dist(path[-1], T):.2f}")

fig, ax = plt.subplots(figsize=(7, 6))
ax.plot(xs, ys, "r-", lw=1, label="插补轨迹")
ax.plot([P0[0], T[0]], [P0[1], T[1]], "k--", lw=0.8, label="直线（最短路）")
ax.plot(*P0, "go", ms=8, label="起点"); ax.plot(*T, "b*", ms=14, label="目标")
ax.set_aspect("equal"); ax.legend(); ax.set_title("插补之字线 (τ=10°)")
plt.savefig("l2_zigzag.png", dpi=150); plt.show()

# ---------- 实验 2：阈值扫描（致密化程度） ----------
taus = [2.0, 5.0, 10.0, 20.0]
fig, axs = plt.subplots(2, 2, figsize=(11, 9))
for ax, tau in zip(axs.flat, taus):
    path, sw, ok = interpolate(P0, T, tau=tau)
    xs = [p[0] for p in path]; ys = [p[1] for p in path]
    ax.plot(xs, ys, "r-", lw=0.8)
    ax.plot([P0[0], T[0]], [P0[1], T[1]], "k--", lw=0.6)
    ax.plot(*T, "b*")
    ax.set_aspect("equal")
    ax.set_title(f"τ={tau:.0f}°  切换{sw}次  {'到达' if ok else '未到达'}")
plt.tight_layout(); plt.savefig("l2_sweep.png", dpi=150); plt.show()
```

**逐块讲解**：

- `alpha_of`：论文的 α = 连线方位角 − 腿 2 轴（120°），用 `wrap180` 规范到 ±180°，否则会出现 350° 和 −10° 其实是一回事的混乱；
- `pick_bracket`：在 6 个基向量里找夹住初始连线的上下两个。`% 360` 保证方位角非负；`lo + 60` 就是上邻（相邻基向量夹 60°）；
- **虚拟试探**（`P_try` 那三行）：这就是"沿 A 走 α 升、沿 B 走 α 降"的实现——不背方向，试一步看结果。仿真允许这么干（算力便宜），写固件时也可以在每次切换时算一次存起来；
- **切换条件**：推高 α 的基向量走到 α ≥ 上阈值就换，压低的走到 α ≤ 下阈值就换——α 永远被关在 `[a0−τ, a0+τ]` 的带子里，轨迹因此呈之字、且始终包着直线。

**你应该看到**：

1. 第一张图：红色之字线包着黑色虚线直线，最终停在三叶草✱附近，控制台显示 `到达: True | 步数: 199 | 切换: 6 次 | 终点误差: 0.62`（L2 没有随机数，你的结果应与这里完全一致）；
2. 第二张图（2×2）：τ=2°/5°/10°/20° 对应切换 **24 / 11 / 6 / 3** 次——阈值越大之字线越稀、切换越少，这就是论文"阈值决定致密化程度"的量化验证。

**自查**：τ→0 会发生什么？（轨迹趋于直线，但切换次数→无穷——真机上每次切换都有开销，所以阈值是"精度 vs 开销"的折中，这是控制器设计里典型的折中参数。）

---

## 5. L3 偏航漂移与补偿：验证 Fig. S4c/d 的机理

**论文逻辑**：机体没有旋转自由度，但加工误差会让机体慢慢偏航（偏航角 δ，IMU 实时测）。偏航后必须"基于原始插补参考角补偿 δ"（Fig. S4d）。为什么必须补偿？——因为**腿是长在机体上的**：机体偏了 δ，你命令"沿腿 3 方向走"，在世界里实际走的是"腿 3 方向 + δ"。更糟的是，机器人**用自己的里程计积分世界坐标**，航向错了，位置估计也跟着错——它连"自己在哪"都会搞错。

所以这一关我们建立更真实的模型：机器人有**两个位置**——真实位置（上帝视角，画图用）和**估计位置**（它自己以为的，靠"步长 × 体表方向 + 假设航向"积分出来）。

新建 `l3_yaw.py`（复用 l2 的工具函数，这里给全文）：

```python
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
```

**逐块讲解（只讲新的）**：

- `P_true` 与 `P_est` 分离是这一关的灵魂。里程计的原理：机器人每步知道自己在**机体系**里走了 `cur` 方向、STEP 距离，要换算成世界坐标必须乘上"我现在朝向多少"——没有 IMU 补偿时它假设朝向没变（`corr=0`），于是每一步都把位置记歪一点；
- `wt = unit(cur + delta)`：机体偏航 δ 后，机体坐标里的 `cur` 方向在世界里就是 `cur + δ`；
- `corr = delta`：补偿的做法 = **指令预回转**——我知道机体偏了 δ，那我就故意命令 `cur − δ`，走出来的世界方向正好是 `cur`。这与论文 Fig. S4d"修正插补参考角"等价。

**你应该看到**：左图（无补偿）真实红线转出一个大弧、终点离目标约 30（而它自己以为只差 0.6），绿色虚线（它以为的轨迹）却"正常"地走向目标——机器人在自我感觉良好地跑丢；右图两条线完全重合、正常到达（误差 0.6 左右）。

**自查**：为什么左图里机器人"以为"自己走对了？——它的世界坐标是从机体系位移积分出来的，航向基准（机体朝向）错了，积分出的位置就是错的。**这就是"位姿估计"会漂移的根源，也是 Movie 12 里需要视觉/同伴兜底的原因。**

---

## 6. L4 K(θ) 力分配：验证 Note 5 式(8)

**论文逻辑**（Note 5）：把 360° 均分成 6 个 60° 扇区。以第一扇区（目标方向与腿 1 轴夹 θ∈[0°,60°)）为例：腿 1 全功率（u₁=1）当主腿，腿 2 出 K(θ) 当辅腿，腿 3 收进 LBS（u₃=0），其中

**K(θ) = sin(θ) / sin(120°−θ)**　　（由力向量三角形的正弦定理推出，式 8）

K 从 0 连续变到 1，合成方向就从腿 1 轴连续扫到 60° 平分线——这正是单驱/双驱之间的无缝过渡。**这一关我们先验证公式本身**（让代码自己证明"给 θ 算出的 K，真的能合成出 θ 方向"），再看用它走出的平滑路径，与 L2 的之字线对比。

新建 `l4_ksynth.py`：

```python
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
```

**逐块讲解（只讲新的）**：

- `synthesize` 里 `rx, ry = um + K*un`：主腿 1 份力 + 辅腿 K 份力，直接矢量相加再 `atan2` 反算合成角——**这就是"验证"**：我们没有假设它对，而是让矢量加法自己给出答案；
- 实验 1 左图：所有点必须精确落在 y=x 虚线上（偏差是 1e-13 量级的浮点噪声）。如果你看到某段明显偏离，说明 `sector_params` 表抄错了——这张图就是以后改代码的回归测试；
- 实验 2：两条路径都到达，但 K 合成几乎是一条直线（路径/直线 ≈ 1.00），之字线略长且带锯齿。

**你应该看到**：左图红点全部贴着黑虚线；右图 K 曲线从 0 平滑爬升到 1；对比图两条线都到目标，蓝线明显更"丝滑"。

**自查**：既然 K 合成这么好，论文为什么还要 L2 的之字线插补？（提示：K 合成要求两条腿同时以不同电压/占空比出力，对功率电路和参数标定的要求更高；之字线只需要"每条腿非 0 即 1"的开关式控制。两者是精度与实现成本的折中，两种方案在本项目后续固件实现中会按场景取用。）

---

## 7. L5 PTP 闭环导航：实现主文 Fig. 5 机理（大结局）

**论文逻辑**（Fig. 5a）：上位机只给目标点；机器人每步用位置传感器 + IMU 自估位姿 → 算连线方位 → 选基向量/力分配（我们用 L4 的 K 合成）→ **PID 消耗角度偏差**输出修正 → 走一步 → 循环，直到进入到达半径。本文速度环仍是开环（电压/占空比直给），作者把闭环速度控制列为 future work。

这一关把 L3 的"真实/估计双位置 + 偏航漂移"、L4 的"K 合成"和一个最小 PID 组装成完整闭环。扰动模型也更贴近真实：**缓变方向偏置**（地面不均匀、陶瓷发热导致谐振频率漂移——每步随机游走的 bias）+ 小抖动；并且**位置每 10 步才更新一次**（模拟光流/视觉帧率远低于控制环——论文"多传感器融合"的由来）。看三件事：无补偿会跑丢、补偿决定成败、PID 决定轨迹品质。

新建 `l5_ptp.py`：

```python
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
```

**逐块讲解（只讲新的）**：

- `g_star = bearing(P_est, T)`：PTP 是"单周期"的——每一步只看"目标点 + 当前位置（估计）"，与论文原话一致；
- **补偿的本质 = 指令预回转**：机体偏了 δ，"沿腿 3 方向"这条体指令在世界里就跟着偏 δ。所以光修测量没用——要把世界系意图换算成体指令时先减掉 δ（`d = synthesize(g_cmd) - delta`），走出来的世界方向才真正指向目标；里程计用同一个 δ 换算，"以为的位置"才与真实位置重合。**位置信念一旦坏了（无补偿），闭环算法再好也救不了**——反馈的质量决定闭环的上限；
- **PID 的真实价值 = 桥接两种频率的传感器**：位置更新慢（每 10 步一次），IMU/里程计航向快（每步都有）。两次位置更新之间，缓变偏置会把航向带跑偏且没人管——PID 用高频的误差 e = g_hold − 实际航向 持续微调 C，把偏置抵消掉。**这就是"多传感器融合"最朴素的样子：快的传感器管平滑，慢的传感器管修正**；
- **I 项这次干的事**：缓变偏置近似"慢变的恒定误差"，纯 P 永远差一拍，积分项把它记账清零。把 `Ki` 改成 0 再跑一遍，对比横向偏差，体会更直观。D 项在无惯性的步进仿真里没有戏份，真机上才需要它阻尼振荡（用陀螺角速度原始值，别对角度做差分）。

**你应该看到**（固定了随机种子，你的数字应当几乎完全一致）：

```
无补偿+无PID 真实到达=False 步数=  173 真实误差= 25.81 估计误差= 0.83 最大横向偏差=25.12
补偿+无PID   真实到达=True  步数=  173 真实误差=  0.83 估计误差= 0.83 最大横向偏差= 0.24
补偿+PID     真实到达=True  步数=  173 真实误差=  0.82 估计误差= 0.82 最大横向偏差= 0.05
```

1. 左图（无补偿）：真实轨迹远远跑偏（终点误差 25.8），但**估计误差只有 0.8——机器人自我感觉良好地停在了错误的地方**。这就是只信航位推算的危险，也是论文要引入视觉/多机位置共享兜底的原因；
2. 中图（补偿）：到达。但两次位置更新之间的缓变偏置让路径有轻微蛇形（横向偏差最大 0.24）；
3. 右图（补偿+PID）：PID 用高频航向信息把缓变偏置抵消掉，横向偏差压到 0.05——**只有原来的 1/5**。

### 7.1 附加任务：导出 GIF 动画（选做，20 分钟）

在 `l5_ptp.py` 末尾追加（不动上面的代码）：

```python
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
```

**你应该看到**：文件夹里多出 `ptp.gif`，打开能看到红点拖着轨迹奔向蓝星。生成需要 10~60 秒，属正常。

---

## 8. 验收清单（全做完后逐项打勾）

- [ ] L1：6 支箭头均匀分布、相邻 60°，单驱红双驱蓝；
- [ ] L2：`到达: True`；2×2 扫描图里 τ 越小之字线越密、切换次数越多；
- [ ] L3：无补偿时真实轨迹画弧跑偏、绿色"自以为是"的轨迹直奔目标；补偿后两线重合；
- [ ] L4：自验图红点全部贴住 y=x（偏差 ~1e-13）；K(θ) 曲线 0→1 平滑；平滑路径比之字线路径比≈1.00；
- [ ] L5：三张对照图——无补偿跑丢 26（却自以为到了）/ 补偿到达（横向偏差 0.24）/ PID 压到 0.05；GIF 能播放；
- [ ] 六个文件 `l1~l5 + png/gif` 都留在 `simulation` 文件夹（这就是你的第一个阶段性产出物）。

## 9. 常见报错速查

| 症状                                | 原因                                | 解决                                     |
| ----------------------------------- | ----------------------------------- | ---------------------------------------- |
| `python` 不是内部或外部命令       | 装时没勾 Add to PATH                | 重装勾上，或用`py` 代替 `python`     |
| `ModuleNotFoundError: matplotlib` | 库没装进这个 Python                 | 重跑 2.2 的安装命令                      |
| 图里中文变方块                      | 字体设置缺前两行 rcParams           | 抄上第 2.3 节那两行                      |
| 图里圆变椭圆/角度看着不对           | 忘了`set_aspect("equal")`         | 补上                                     |
| 窗口一闪就没了 / 想截图             | 没调`plt.show()` 或没 `savefig` | `savefig` 放在 `show` **之前** |
| L2 显示`到达: False`              | τ 太小叠加浮点误差，或参数被改动   | 先恢复原参数；确认 STEP/R_ARRIVE 没改    |
| L5 步数打满`MAX_STEPS`            | 同上；或 drift/noise 被调得过大     | 先跑默认参数再调参                       |
| GIF 打不开                          | 保存中途关窗                        | 等控制台打印"已保存"再关图窗             |

## 10. 概念小词典

| 词               | 一句话解释                                                                  |
| ---------------- | --------------------------------------------------------------------------- |
| 弧度             | 角度的另一种单位，π 弧度 = 180°；`math.sin/cos` 只吃弧度                |
| 单位向量         | 长度为 1 的方向箭头`(cosθ, sinθ)`，乘步长=位移                          |
| 方位角 (bearing) | 从一点指向另一点的方向角，`atan2(dy, dx)`                                 |
| 航位推算         | "知道自己每步走了多少、朝向多少"，一步步积分出位置——光流+IMU 干的就是这个 |
| 开环/闭环        | 开环=只发指令不看结果；闭环=拿反馈算误差再修正                              |
| bang-bang 控制   | 只有两个开关状态、靠切换时机逼近目标——L2 的本质                           |
| 状态机           | 程序按当前状态+条件跳转的写法——插补的 A/B 切换就是两状态机                |
| PID              | P 抄当下误差、I 记历史总账、D 看变化趋势；本文 I 专治恒定漂移               |

## 11. 仿真做完之后：怎么对接固件

| 仿真里的函数        | 将来在 ESP32 固件里                                      |
| ------------------- | -------------------------------------------------------- |
| `bearing()`       | `atan2f(dy, dx)`（浮点，轻量）                         |
| `K_of()`          | 查表 + 线性插值（省掉每步两次 sin），或直接算            |
| `sector_params()` | 一张 const 查找表                                        |
| L2 的切换状态机     | 两个状态的 switch-case，阈值进配置文件（腿频率表同一份） |
| PID                 | 增量式 PID，IMU 陀螺角速度当 D 项                        |
| 偏航补偿            | IMU 航向解算后减到指令角上                               |

**项目一句话总结**："我按文献 [1] 补充材料 Note 2/5 把插补算法和 K(θ) 合成做成了数值仿真，验证了阈值-致密化关系、公式自验和偏航补偿的必要性，下一步把这几个模块移植进 ESP32 固件。"

---

*卡住时：把「哪个文件 + 完整报错信息 + 你预期看到什么/实际看到什么」三样东西带来问。*
