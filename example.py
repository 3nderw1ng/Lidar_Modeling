import pylidar

pylidar.start()

while True:
    points = pylidar.get_latest_frame()
    if points:
        print("Got", len(points), "points")
        for p in points[:5]:  # 打印前5个点
            print(f"x={p[0]:.2f}, y={p[1]:.2f}, z={p[2]:.2f}, i={p[3]}")
