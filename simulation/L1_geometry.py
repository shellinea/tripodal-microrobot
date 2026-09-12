# l1_geometry.py —— 画三足机器人的 6 个基向量（对应 Fig. S4a）
import math
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False

def unit(deg):
    """角度（度） -> 单位向量 （x,y）"""
    a = math.radians(deg)#角度转弧度
    return (math.cos(a),math.sin(a))#再由余弦算出X，正弦算出Y

LEG=[0.0,120.0,240.0]#三足机器人的三个腿的初始角度
BASIS =[0.0,60.0,120.0,180.0,240.0,300.0]#六个基向量
NAME  = ["单驱腿1", "双驱1+2", "单驱腿2", "双驱2+3", "单驱腿3", "双驱3+1"]

fig, ax=plt.subplots(figsize=(6, 6))#6x6英寸的画板和画纸
for b,name in zip(BASIS,NAME):#把角度和名字打包，循环6次
    ux,uy =unit(b)#角度转单位向量
    color ="tab:red" if "单驱" in name else "tab:blue"#单驱腿用红色，双驱腿用蓝色
    ax.arrow(0, 0, ux*2, uy*2, head_width=0.08, length_includes_head=True, color=color)#画箭头
    ax.text(ux*2.45, uy*2.45, f"{name}\n{b:.0f}°", ha="center", color=color)#在箭头旁边写名字和角度

ax.axhline(0,color="gray",lw=0.5);ax.axvline(0,color="gray",lw=0.5)#画坐标轴
ax.set_xlim(-2.6, 2.6); ax.set_ylim(-2.6, 2.6)#坐标轴范围
ax.set_aspect("equal") #等比例缩放,保证x,y轴比例是一比一
ax.set_title("6 个基向量（相邻夹 60°）")#标题
plt.savefig("l1_basis.png", dpi=150)#保存图片
plt.show()