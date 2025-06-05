import numpy as np
import pylidar

# 调用C++模块接口，获取最新一帧点云数据
# 返回是 NumPy ndarray，shape=(N,6)，列依次是 x,y,z,intensity,timestamp,channel
points = pylidar.get_latest_frame()
if points is None or points.size == 0:
    print("No points in latest frame or failed to retrieve data")
else:
    print(f"Got {points.shape[0]} points in latest frame")
    # 打印前5个点的坐标和信息
    for i in range(min(5, points.shape[0])):
        x, y, z, intensity, timestamp, channel = points[i]
        print(f"Point {i}: x={x:.3f}, y={y:.3f}, z={z:.3f}, intensity={intensity:.1f}, timestamp={timestamp:.3f}, channel={int(channel)}")
