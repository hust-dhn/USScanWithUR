import numpy as np
from plyfile import PlyData

PLY = "camera/pc_datas/0036.ply"  # 改成你的文件

ply = PlyData.read(PLY)
v = ply["vertex"].data

print("fields:", v.dtype.names)
x = v["x"]; y = v["y"]; z = v["z"]
a3 = v["attr3"]; a4 = v["attr4"]; a5 = v["attr5"]

extra = np.vstack([a3, a4, a5]).T.astype(np.float32)
print("attr min:", extra.min(axis=0))
print("attr max:", extra.max(axis=0))
print("attr mean:", extra.mean(axis=0))

print("sample rows:")
for i in range(5):
    print(x[i], y[i], z[i], a3[i], a4[i], a5[i])



import numpy as np
from plyfile import PlyData

PLY = "camera/pc_datas/0000.ply"  # 改成你的文件
ply = PlyData.read(PLY)
v = ply["vertex"].data

x = v["x"].astype(np.float32)
y = v["y"].astype(np.float32)
z = v["z"].astype(np.float32)

a3 = v["attr3"].astype(np.float32)
a4 = v["attr4"].astype(np.float32)
a5 = v["attr5"].astype(np.float32)

dx = np.abs(x - a3)
dy = np.abs(y - a4)
dz = np.abs(z - a5)

print("abs diff quantiles x-attr3:", np.quantile(dx, [0, .5, .9, .99, 1]))
print("abs diff quantiles y-attr4:", np.quantile(dy, [0, .5, .9, .99, 1]))
print("abs diff quantiles z-attr5:", np.quantile(dz, [0, .5, .9, .99, 1]))

same_rate = np.mean((dx < 1e-6) & (dy < 1e-6) & (dz < 1e-6))
print("exact same rate:", same_rate)
