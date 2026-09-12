import math 
import matplotlib.pyplot as plt#给matplotlib.pyplot起别名为plt

#matplotlib是一个绘图库，pyplot是其中的一个模块，用于绘制二维图形
plt.rcParams["font.sans-serif"]=["Microsoft YaHei"]# 全局默认字体设为微软雅黑
plt.rcParams["axes.unicode_minus"]=False # 解决负号'-'显示成方块的问题
#以上为中文环境的配置

plt.plot([0,1,2],[0,1,4],"b-")#分别为x坐标序列和y坐标序列，b-表示蓝色blue，实线
plt.title("冒烟测试")
plt.savefig("smoke.png",dpi=150)# 保存成图片文件,150 是分辨率
plt.show()# 保存成图片文件,150 是分辨率