import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 读取数据
# data = np.loadtxt('ur10e/cfg/poses_traj.txt')
data = np.loadtxt('ur10e/cfg/poses_target.txt')

positions = data[:, :3]
rotations = data[:, 3:]

# 创建简单的双面板图
fig = plt.figure(figsize=(14, 10))

# 1. 3D轨迹 - 设置等比例坐标轴
ax1 = fig.add_subplot(221, projection='3d')

# 绘制轨迹
ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', alpha=0.5, linewidth=1)

# 绘制所有点
scatter = ax1.scatter(positions[:, 0], positions[:, 1], positions[:, 2], 
           c=np.arange(len(positions)), cmap='viridis', s=20, alpha=0.7)

# 标记起始点和结束点
ax1.scatter(positions[0, 0], positions[0, 1], positions[0, 2], 
           c='green', s=150, marker='o', label='Start', edgecolors='black', linewidth=1.5)
ax1.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], 
           c='red', s=150, marker='s', label='End', edgecolors='black', linewidth=1.5)

# 设置坐标轴标签
ax1.set_xlabel('X', fontsize=11)
ax1.set_ylabel('Y', fontsize=11)
ax1.set_zlabel('Z', fontsize=11)
ax1.set_title(f'3D Trajectory ({len(positions)} poses)', fontsize=12, fontweight='bold')
ax1.legend(loc='upper right', fontsize=10)

# 设置等比例坐标轴
# 计算数据范围，使三个轴有相同的缩放比例
max_range = np.array([positions[:, 0].max()-positions[:, 0].min(), 
                      positions[:, 1].max()-positions[:, 1].min(), 
                      positions[:, 2].max()-positions[:, 2].min()]).max() / 2.0

mid_x = (positions[:, 0].max()+positions[:, 0].min()) * 0.5
mid_y = (positions[:, 1].max()+positions[:, 1].min()) * 0.5
mid_z = (positions[:, 2].max()+positions[:, 2].min()) * 0.5

ax1.set_xlim(mid_x - max_range, mid_x + max_range)
ax1.set_ylim(mid_y - max_range, mid_y + max_range)
ax1.set_zlim(mid_z - max_range, mid_z + max_range)

# 添加网格
ax1.grid(True, alpha=0.3)

# 2. XY视图 - 统一坐标轴比例
ax2 = fig.add_subplot(222)
ax2.plot(positions[:, 0], positions[:, 1], 'k-', alpha=0.3, linewidth=0.5)

# 使用散点图，颜色表示索引
scatter_xy = ax2.scatter(positions[:, 0], positions[:, 1], 
           c=np.arange(len(positions)), cmap='viridis', s=30, alpha=0.7)

# 标记起始点和结束点
ax2.plot(positions[0, 0], positions[0, 1], 'go', markersize=12, 
         label='Start', markeredgecolor='black', markeredgewidth=1.5)
ax2.plot(positions[-1, 0], positions[-1, 1], 'rs', markersize=12, 
         label='End', markeredgecolor='black', markeredgewidth=1.5)

ax2.set_xlabel('X', fontsize=11)
ax2.set_ylabel('Y', fontsize=11)
ax2.set_title('XY Plane View', fontsize=12, fontweight='bold')
ax2.legend(loc='upper right', fontsize=10)

# 设置等比例坐标轴
ax2.axis('equal')

# 根据数据范围调整坐标轴限制，留出一些边距
x_margin = (positions[:, 0].max() - positions[:, 0].min()) * 0.1
y_margin = (positions[:, 1].max() - positions[:, 1].min()) * 0.1
ax2.set_xlim(positions[:, 0].min() - x_margin, positions[:, 0].max() + x_margin)
ax2.set_ylim(positions[:, 1].min() - y_margin, positions[:, 1].max() + y_margin)

# 添加网格
ax2.grid(True, alpha=0.3, linestyle='--')

# 3. 位置分量 - 统一Y轴范围
ax3 = fig.add_subplot(223)

# 计算所有位置分量的范围，统一Y轴
all_positions = positions.flatten()
pos_min, pos_max = all_positions.min(), all_positions.max()
pos_margin = (pos_max - pos_min) * 0.1

# 绘制位置分量
ax3.plot(positions[:, 0], label='X', color='blue', linewidth=1.5, alpha=0.8)
ax3.plot(positions[:, 1], label='Y', color='green', linewidth=1.5, alpha=0.8)
ax3.plot(positions[:, 2], label='Z', color='red', linewidth=1.5, alpha=0.8)

ax3.set_xlabel('Pose Index', fontsize=11)
ax3.set_ylabel('Position Value', fontsize=11)
ax3.set_title('Position Components', fontsize=12, fontweight='bold')
ax3.legend(loc='upper right', fontsize=10)

# 统一Y轴范围
ax3.set_ylim(pos_min - pos_margin, pos_max + pos_margin)

# 设置X轴范围
ax3.set_xlim(0, len(positions) - 1)

# 添加网格
ax3.grid(True, alpha=0.3, linestyle='--')

# 4. 旋转分量 - 统一Y轴范围
ax4 = fig.add_subplot(224)

# 计算所有旋转分量的范围，统一Y轴
all_rotations = rotations.flatten()
rot_min, rot_max = all_rotations.min(), all_rotations.max()
rot_margin = (rot_max - rot_min) * 0.1

# 绘制旋转分量
ax4.plot(rotations[:, 0], label='RX', color='cyan', linewidth=1.5, alpha=0.8)
ax4.plot(rotations[:, 1], label='RY', color='magenta', linewidth=1.5, alpha=0.8)
ax4.plot(rotations[:, 2], label='RZ', color='orange', linewidth=1.5, alpha=0.8)

ax4.set_xlabel('Pose Index', fontsize=11)
ax4.set_ylabel('Rotation (degrees)', fontsize=11)
ax4.set_title('Rotation Components', fontsize=12, fontweight='bold')
ax4.legend(loc='upper right', fontsize=10)

# 统一Y轴范围
ax4.set_ylim(rot_min - rot_margin, rot_max + rot_margin)

# 设置X轴范围
ax4.set_xlim(0, len(rotations) - 1)

# 添加网格
ax4.grid(True, alpha=0.3, linestyle='--')

# 调整布局，留出颜色条空间
plt.tight_layout()

# 为3D图添加颜色条
cbar_ax = fig.add_axes([0.92, 0.55, 0.02, 0.3])  # [left, bottom, width, height]
cbar = plt.colorbar(scatter, cax=cbar_ax)
cbar.set_label('Pose Index', fontsize=11)

# 为XY图添加颜色条
cbar_ax2 = fig.add_axes([0.92, 0.15, 0.02, 0.3])  # [left, bottom, width, height]
cbar2 = plt.colorbar(scatter_xy, cax=cbar_ax2)
cbar2.set_label('Pose Index', fontsize=11)

# 添加整体标题
fig.suptitle(f'Pose Visualization ({len(positions)} poses)', fontsize=14, fontweight='bold', y=0.98)

plt.show()

# 打印坐标轴范围信息
print("=== 坐标轴范围信息 ===")
print(f"3D视图坐标轴统一范围: ±{max_range:.3f}")
print(f"位置分量Y轴范围: [{pos_min-pos_margin:.3f}, {pos_max+pos_margin:.3f}]")
print(f"旋转分量Y轴范围: [{rot_min-rot_margin:.3f}, {rot_max+rot_margin:.3f}]")