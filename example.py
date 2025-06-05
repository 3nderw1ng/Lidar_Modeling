import pylidar

points = pylidar.get_latest_points()

for x, y, z, i, ts, ch, r, theta in points:
    print(f"x={x:.2f}, y={y:.2f}, z={z:.2f}, i={i}, ts={ts}, ch={ch}, r={r:.2f}, θ={theta:.2f}")
