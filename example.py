import numpy as np
import open3d as o3d
import pylidar  # 假设你已经安装并使用 pylidar 获取激光雷达数据
import time
import matplotlib.pyplot as plt
import os

# 创建一个空的 PointCloud 对象
pcd = o3d.geometry.PointCloud()

# 设置 colormap 使用 Jet 调色板
colormap = plt.get_cmap('jet')  # 或者你可以尝试 'viridis'，'plasma'，'inferno'，等

# 设置保存数据的路径
save_path = "saved_scans/"
os.makedirs(save_path, exist_ok=True)

# 开始循环，实时更新点云数据
while True:
    points_raw = pylidar.get_latest_points()  # 获取激光雷达的最新数据
    
    if points_raw:  # 如果获取到数据
        # 提取点数据（x, y, z, i）以及其他信息
        #xyz = np.array([[x, y, z] for x, y, z, i, ts, ch, r, theta in points_raw], dtype=np.float32)
        #intensity = np.array([i for x, y, z, i, ts, ch, r, theta in points_raw], dtype=np.float32)
        # 提取每个字段
        xyz = np.array([[x, y, z] for x, y, z, i, ts, ch, r, theta in points_raw], dtype=np.float32)
        intensity = np.array([i for x, y, z, i, ts, ch, r, theta in points_raw], dtype=np.float32)
        ts = np.array([ts for x, y, z, i, ts, ch, r, theta in points_raw], dtype=np.float64)
        ch = np.array([ch for x, y, z, i, ts, ch, r, theta in points_raw], dtype=np.int32)
        r = np.array([r for x, y, z, i, ts, ch, r, theta in points_raw], dtype=np.float32)
        theta = np.array([theta for x, y, z, i, ts, ch, r, theta in points_raw], dtype=np.float32)

        # 归一化反射率值（将反射率映射到 [0, 1] 范围）
        intensity_norm = (intensity - np.min(intensity)) / (np.max(intensity) - np.min(intensity))
        
        # 使用 Jet colormap 将归一化的反射率值映射到 RGB 颜色空间
        colors = colormap(intensity_norm)[:, :3]  # 获取 RGB 通道（去除透明度 alpha 通道）

        # 将 xyz 转换为 Open3D 的 PointCloud 格式
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(colors)  # 设置颜色
        
        # 使用 Open3D 的 `draw_geometries` 进行渲染
        o3d.visualization.draw_geometries([pcd], 
                                          zoom=0.8, 
                                          front=[0, 0, -1], 
                                          lookat=[0, 0, 0], 
                                          up=[0, -1, 0], 
                                          point_show_normal=False)
        
        # 提示用户是否保存当前点云数据

        user_input = input("Do you want to save this scan data? (y/n): ")
        if user_input.lower() == "y":
            # 保存当前的点云数据
            timestamp = int(time.time())
            save_file = os.path.join(save_path, f"points_raw_{timestamp}.npz")
            np.savez(save_file, xyz=xyz, intensity=intensity, ts=ts, ch=ch, r=r, theta=theta)
            print(f"Data saved to {save_file}")

        
    # 延时，避免过于频繁更新导致性能问题
    time.sleep(0.1)  # 每100毫秒更新一次
