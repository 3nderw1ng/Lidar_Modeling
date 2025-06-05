import pylidar

points = pylidar.get_latest_points()
for x, y, z, intensity, timestamp, channel in points:
    print(f"x={x:.2f}, y={y:.2f}, z={z:.2f}, i={intensity}, ts={timestamp}, ch={channel}")
