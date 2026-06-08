import sys
import os
import csv
import math
import bisect
import numpy as np
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def tf_to_matrix(tx, ty, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([
        [c, -s, tx],
        [s,  c, ty],
        [0.0, 0.0, 1.0]
    ])


def matrix_to_pose(T):
    x = T[0, 2]
    y = T[1, 2]
    yaw = math.atan2(T[1, 0], T[0, 0])
    return x, y, yaw


def save_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def align_to_start(rows):
    if not rows:
        return np.empty((0, 4))
    arr = np.array(rows, dtype=float).copy()
    arr[:, 1] -= arr[0, 1]
    arr[:, 2] -= arr[0, 2]
    return arr


def detect_storage_id(bag_path):
    files = os.listdir(bag_path)
    if any(f.endswith(".mcap") for f in files):
        return "mcap"
    if any(f.endswith(".db3") for f in files):
        return "sqlite3"
    return "mcap"


def norm_frame(name):
    return name.lstrip("/")


def main():
    if len(sys.argv) < 2:
        print("Użycie:")
        print("python3 extract_one_bag_cartographer_real.py /sciezka/do/rosbaga")
        sys.exit(1)

    bag_path = sys.argv[1]

    if not os.path.isdir(bag_path):
        print(f"Nie znaleziono folderu rosbaga: {bag_path}")
        sys.exit(1)

    bag_name = os.path.basename(os.path.normpath(bag_path))
    output_dir = os.path.join("/home/jaokoz/ros_scripts/wyniki/cartographer_real", bag_name)
    os.makedirs(output_dir, exist_ok=True)

    odom_csv = os.path.join(output_dir, "odom.csv")
    carto_csv = os.path.join(output_dir, "cartographer.csv")
    raw_plot_png = os.path.join(output_dir, "trajektorie_surowe.png")
    aligned_plot_png = os.path.join(output_dir, "trajektorie_wyrownane.png")

    storage_id = detect_storage_id(bag_path)
    print(f"Wykryty storage_id: {storage_id}")

    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions("", "")

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    if "/odom" not in type_map or "/tf" not in type_map:
        print("Brakuje /odom albo /tf w rosbagu")
        print("Dostępne topiki:")
        for t in sorted(type_map.keys()):
            print(" ", t)
        sys.exit(1)

    odom_type = get_message(type_map["/odom"])
    tf_type = get_message(type_map["/tf"])

    odom_rows = []
    map_odom_rows = []
    map_base_rows = []

    print("Czytam rosbag...")

    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic == "/odom":
            msg = deserialize_message(data, odom_type)

            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

            odom_rows.append([stamp, x, y, yaw])

        elif topic == "/tf":
            msg = deserialize_message(data, tf_type)

            for tr in msg.transforms:
                parent = norm_frame(tr.header.frame_id)
                child = norm_frame(tr.child_frame_id)
                stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9

                tx = tr.transform.translation.x
                ty = tr.transform.translation.y
                q = tr.transform.rotation
                yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

                if parent == "map" and child == "odom":
                    map_odom_rows.append([stamp, tx, ty, yaw])

                elif parent == "map" and child == "world_1":
                    map_base_rows.append([stamp, tx, ty, yaw])

    if not odom_rows:
        print("Nie znaleziono danych /odom")
        sys.exit(1)

    carto_rows = []

    if map_base_rows:
        print(f"Znaleziono bezpośrednio map->world_1, próbek: {len(map_base_rows)}")
        carto_rows = map_base_rows

    elif map_odom_rows:
        print(f"Nie ma map->world_1, składam trajektorię z map->odom i /odom")
        print(f"Próbek map->odom: {len(map_odom_rows)}")

        tf_times = [row[0] for row in map_odom_rows]

        for stamp, ox, oy, oyaw in odom_rows:
            idx = bisect.bisect_left(tf_times, stamp)

            if idx == 0:
                nearest = map_odom_rows[0]
            elif idx >= len(tf_times):
                nearest = map_odom_rows[-1]
            else:
                before = map_odom_rows[idx - 1]
                after = map_odom_rows[idx]
                nearest = before if abs(before[0] - stamp) <= abs(after[0] - stamp) else after

            _, mx, my, myaw = nearest

            T_map_odom = tf_to_matrix(mx, my, myaw)
            T_odom_base = tf_to_matrix(ox, oy, oyaw)
            T_map_base = T_map_odom @ T_odom_base

            cx, cy, cyaw = matrix_to_pose(T_map_base)
            carto_rows.append([stamp, cx, cy, cyaw])

    else:
        print("Nie znaleziono ani map->world_1, ani map->odom w /tf")
        sys.exit(1)

    print(f"Odczytano:")
    print(f"  odom: {len(odom_rows)} próbek")
    print(f"  cartographer: {len(carto_rows)} próbek")

    save_csv(odom_csv, ["time_s", "x", "y", "yaw_rad"], odom_rows)
    save_csv(carto_csv, ["time_s", "x", "y", "yaw_rad"], carto_rows)

    print(f"Zapisano: {odom_csv}")
    print(f"Zapisano: {carto_csv}")

    odom_arr = np.array(odom_rows)
    carto_arr = np.array(carto_rows)

    plt.figure(figsize=(9, 7))
    plt.plot(odom_arr[:, 1], odom_arr[:, 2], label="odom")
    plt.plot(carto_arr[:, 1], carto_arr[:, 2], label="cartographer")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trajektorie surowe: odom vs cartographer")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.savefig(raw_plot_png, dpi=200)

    odom_aligned = align_to_start(odom_rows)
    carto_aligned = align_to_start(carto_rows)

    plt.figure(figsize=(9, 7))
    plt.plot(odom_aligned[:, 1], odom_aligned[:, 2], label="odom")
    plt.plot(carto_aligned[:, 1], carto_aligned[:, 2], label="cartographer")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trajektorie wyrównane: odom vs cartographer")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.savefig(aligned_plot_png, dpi=200)
    plt.close('all')

    print(f"Zapisano: {raw_plot_png}")
    print(f"Zapisano: {aligned_plot_png}")
    print(f"Wyniki zapisane do folderu: {output_dir}")


if __name__ == "__main__":
    main()
