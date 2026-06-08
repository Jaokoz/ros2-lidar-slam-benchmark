#!/usr/bin/env python3

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


def detect_storage_id(bag_path):
    files = os.listdir(bag_path)

    if any(f.endswith(".mcap") for f in files):
        return "mcap"

    if any(f.endswith(".db3") for f in files):
        return "sqlite3"

    return "mcap"


def save_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def normalize_time(rows):
    if not rows:
        return rows

    t0 = rows[0][0]
    out = []

    for r in rows:
        rr = list(r)
        rr[0] = rr[0] - t0
        out.append(rr)

    return out


def align_to_start(rows):
    if not rows:
        return []

    arr = np.array(rows, dtype=float).copy()
    arr[:, 1] -= arr[0, 1]
    arr[:, 2] -= arr[0, 2]

    return arr.tolist()


def find_nearest_row(rows, time_s):
    if not rows:
        return None

    times = [r[0] for r in rows]
    idx = bisect.bisect_left(times, time_s)

    if idx == 0:
        return rows[0]

    if idx >= len(rows):
        return rows[-1]

    before = rows[idx - 1]
    after = rows[idx]

    if abs(before[0] - time_s) <= abs(after[0] - time_s):
        return before

    return after


def is_reasonable_pose(x, y, limit=1000.0):
    return (
        math.isfinite(x)
        and math.isfinite(y)
        and abs(x) < limit
        and abs(y) < limit
    )


def apply_marker_pose_to_point(marker, point):
    px = point.x
    py = point.y

    pose = marker.pose
    tx = pose.position.x
    ty = pose.position.y

    q = pose.orientation
    yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    c = math.cos(yaw)
    s = math.sin(yaw)

    x = tx + c * px - s * py
    y = ty + s * px + c * py

    return x, y


def extract_cartographer_from_marker_array_messages(marker_msgs):
    """
    /trajectory_node_list ma typ visualization_msgs/msg/MarkerArray.

    W Twoim przypadku każdy marker może mieć tylko jeden punkt:
        marker.points[0]

    Dlatego bierzemy punkty ze wszystkich markerów,
    sortujemy po marker.id i budujemy trajektorię.
    """

    if not marker_msgs:
        return []

    _, last_msg = marker_msgs[-1]

    points_with_id = []

    for marker in last_msg.markers:
        if not marker.ns.startswith("Trajectory"):
            continue

        if not hasattr(marker, "points"):
            continue

        if len(marker.points) == 0:
            continue

        for local_idx, p in enumerate(marker.points):
            x, y = apply_marker_pose_to_point(marker, p)

            if is_reasonable_pose(x, y):
                points_with_id.append((int(marker.id), int(local_idx), x, y))

    if not points_with_id:
        return []

    points_with_id.sort(key=lambda r: (r[0], r[1]))

    filtered = []
    last_xy = None

    for _, _, x, y in points_with_id:
        if last_xy is None:
            filtered.append([x, y])
            last_xy = [x, y]
            continue

        dx = x - last_xy[0]
        dy = y - last_xy[1]

        if math.sqrt(dx * dx + dy * dy) > 1e-5:
            filtered.append([x, y])
            last_xy = [x, y]

    rows = []

    for i, (x, y) in enumerate(filtered):
        if i < len(filtered) - 1:
            dx = filtered[i + 1][0] - x
            dy = filtered[i + 1][1] - y
            yaw = math.atan2(dy, dx)
        elif i > 0:
            dx = x - filtered[i - 1][0]
            dy = y - filtered[i - 1][1]
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0.0

        # Tutaj time_s jest indeksem węzła trajektorii, nie rzeczywistym czasem.
        rows.append([float(i), x, y, yaw])

    return rows


def main():
    if len(sys.argv) < 2:
        print("Użycie:")
        print("python3 extract_one_bag_cartographer.py /home/jaokoz/ros2_ws/rosbag2_xxx")
        sys.exit(1)

    bag_path = sys.argv[1]

    if not os.path.isdir(bag_path):
        print(f"Nie znaleziono folderu rosbaga: {bag_path}")
        sys.exit(1)

    bag_name = os.path.basename(os.path.normpath(bag_path))

    base_output_dir = "/home/jaokoz/ros_scripts/wyniki/cartographer"
    os.makedirs(base_output_dir, exist_ok=True)

    output_dir = os.path.join(base_output_dir, bag_name)
    os.makedirs(output_dir, exist_ok=True)

    odom_csv = os.path.join(output_dir, "odom.csv")
    gt_csv = os.path.join(output_dir, "ground_truth.csv")
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

    print("Dostępne topiki w rosbagu:")
    for t in sorted(type_map.keys()):
        print(f"  {t} -> {type_map[t]}")

    required_topics = ["/odom", "/tf", "/model/four_wheel_bot/pose"]

    for topic in required_topics:
        if topic not in type_map:
            print(f"Brakuje topicu {topic} w rosbagu.")
            sys.exit(1)

    odom_type = get_message(type_map["/odom"])
    tf_type = get_message(type_map["/tf"])
    gt_type = get_message(type_map["/model/four_wheel_bot/pose"])

    has_marker_trajectory = "/trajectory_node_list" in type_map

    if has_marker_trajectory:
        trajectory_marker_type = get_message(type_map["/trajectory_node_list"])
        print("Znaleziono /trajectory_node_list.")
        print(f"Typ: {type_map['/trajectory_node_list']}")
        print("Trajektoria Cartographera będzie brana z MarkerArray.")
    else:
        trajectory_marker_type = None
        print("UWAGA: Brak /trajectory_node_list w rosbagu.")
        print("Skrypt użyje awaryjnie TF map->odom * odom.")

    odom_rows = []
    gt_rows = []
    map_odom_rows = []
    map_base_rows = []
    marker_msgs = []

    ignored_tf_count = 0

    print("Czytam rosbag...")

    while reader.has_next():
        topic, data, t = reader.read_next()
        bag_time_s = t * 1e-9

        if topic == "/odom":
            msg = deserialize_message(data, odom_type)

            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamp <= 0.0:
                stamp = bag_time_s

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            q = msg.pose.pose.orientation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

            if is_reasonable_pose(x, y):
                odom_rows.append([stamp, x, y, yaw])

        elif topic == "/model/four_wheel_bot/pose":
            msg = deserialize_message(data, gt_type)

            x = msg.position.x
            y = msg.position.y

            q = msg.orientation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

            if is_reasonable_pose(x, y):
                gt_rows.append([bag_time_s, x, y, yaw])

        elif topic == "/trajectory_node_list":
            msg = deserialize_message(data, trajectory_marker_type)
            marker_msgs.append([bag_time_s, msg])

        elif topic == "/tf":
            msg = deserialize_message(data, tf_type)

            for tr in msg.transforms:
                parent = tr.header.frame_id
                child = tr.child_frame_id

                stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                if stamp <= 0.0:
                    stamp = bag_time_s

                tx = tr.transform.translation.x
                ty = tr.transform.translation.y

                q = tr.transform.rotation
                yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

                if not is_reasonable_pose(tx, ty):
                    ignored_tf_count += 1
                    continue

                if parent == "map" and child == "odom":
                    map_odom_rows.append([stamp, tx, ty, yaw])

                elif parent == "map" and child == "base_link":
                    map_base_rows.append([stamp, tx, ty, yaw])

                elif parent == "map" and child == "four_wheel_bot/base_link":
                    map_base_rows.append([stamp, tx, ty, yaw])

    odom_rows.sort(key=lambda r: r[0])
    gt_rows.sort(key=lambda r: r[0])
    map_odom_rows.sort(key=lambda r: r[0])
    map_base_rows.sort(key=lambda r: r[0])
    marker_msgs.sort(key=lambda r: r[0])

    print("Odczytano surowe dane:")
    print(f"  /odom: {len(odom_rows)}")
    print(f"  /model/four_wheel_bot/pose: {len(gt_rows)}")
    print(f"  /trajectory_node_list wiadomości: {len(marker_msgs)}")
    print(f"  TF map->odom: {len(map_odom_rows)}")
    print(f"  TF map->base_link: {len(map_base_rows)}")
    print(f"  Odrzucone absurdalne TF: {ignored_tf_count}")

    if not odom_rows:
        print("Nie znaleziono danych /odom.")
        sys.exit(1)

    if not gt_rows:
        print("Nie znaleziono danych ground truth.")
        sys.exit(1)

    carto_rows = []

    if marker_msgs:
        print("Próbuję wyciągnąć trajektorię Cartographera z /trajectory_node_list MarkerArray...")
        carto_rows = extract_cartographer_from_marker_array_messages(marker_msgs)

        if carto_rows:
            print(f"Używam /trajectory_node_list jako trajektorii Cartographera. Punkty: {len(carto_rows)}")
        else:
            print("Nie udało się wyciągnąć punktów z /trajectory_node_list.")

    if not carto_rows and map_base_rows:
        print("Używam bezpośredniego TF map->base_link jako trajektorii Cartographera.")
        carto_rows = map_base_rows

    if not carto_rows and map_odom_rows:
        print("UWAGA: Używam awaryjnie TF map->odom * odom->base_link.")
        print("Ta trajektoria może być bardzo podobna do odometrii.")

        for odom_row in odom_rows:
            stamp, ox, oy, oyaw = odom_row

            nearest_map_odom = find_nearest_row(map_odom_rows, stamp)

            if nearest_map_odom is None:
                continue

            _, mx, my, myaw = nearest_map_odom

            T_map_odom = tf_to_matrix(mx, my, myaw)
            T_odom_base = tf_to_matrix(ox, oy, oyaw)

            T_map_base = T_map_odom @ T_odom_base

            x, y, yaw = matrix_to_pose(T_map_base)

            if is_reasonable_pose(x, y):
                carto_rows.append([stamp, x, y, yaw])

    if not carto_rows:
        print("Nie udało się wyznaczyć trajektorii Cartographera.")
        sys.exit(1)

    print("Po przetworzeniu:")
    print(f"  odom: {len(odom_rows)} próbek")
    print(f"  ground truth: {len(gt_rows)} próbek")
    print(f"  cartographer: {len(carto_rows)} próbek")

    odom_rows = normalize_time(odom_rows)
    gt_rows = normalize_time(gt_rows)
    carto_rows = normalize_time(carto_rows)

    save_csv(odom_csv, ["time_s", "x", "y", "yaw_rad"], odom_rows)
    save_csv(gt_csv, ["time_s", "x", "y", "yaw_rad"], gt_rows)
    save_csv(carto_csv, ["time_s", "x", "y", "yaw_rad"], carto_rows)

    print(f"Zapisano: {odom_csv}")
    print(f"Zapisano: {gt_csv}")
    print(f"Zapisano: {carto_csv}")

    odom_arr = np.array(odom_rows)
    gt_arr = np.array(gt_rows)
    carto_arr = np.array(carto_rows)

    plt.figure(figsize=(9, 7))
    plt.plot(gt_arr[:, 1], gt_arr[:, 2], label="ground truth Gazebo")
    plt.plot(odom_arr[:, 1], odom_arr[:, 2], label="odom")
    plt.plot(carto_arr[:, 1], carto_arr[:, 2], label="cartographer")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trajektorie surowe: ground truth, odom i Cartographer")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(raw_plot_png, dpi=200)
    plt.close()

    odom_aligned = np.array(align_to_start(odom_rows))
    gt_aligned = np.array(align_to_start(gt_rows))
    carto_aligned = np.array(align_to_start(carto_rows))

    plt.figure(figsize=(9, 7))
    plt.plot(gt_aligned[:, 1], gt_aligned[:, 2], label="ground truth Gazebo")
    plt.plot(odom_aligned[:, 1], odom_aligned[:, 2], label="odom")
    plt.plot(carto_aligned[:, 1], carto_aligned[:, 2], label="cartographer")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trajektorie wyrównane do punktu startowego")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(aligned_plot_png, dpi=200)
    plt.close()

    print(f"Zapisano: {raw_plot_png}")
    print(f"Zapisano: {aligned_plot_png}")
    print(f"Wyniki zapisane do folderu: {output_dir}")

    print("")
    print("Diagnostyka:")

    if marker_msgs and len(carto_rows) > 0:
        print("  Preferowane źródło: /trajectory_node_list MarkerArray.")
        print("  To NIE jest TF map->odom * odom.")
    elif map_base_rows:
        print("  Źródło: TF map->base_link.")
    elif map_odom_rows:
        print("  Źródło awaryjne: TF map->odom * odom->base_link.")

    print(f"  liczba próbek odom: {len(odom_rows)}")
    print(f"  liczba próbek Cartographer: {len(carto_rows)}")

    if len(carto_rows) == len(odom_rows):
        odom_np = np.array(odom_rows)
        carto_np = np.array(carto_rows)

        dx = carto_np[:, 1] - odom_np[:, 1]
        dy = carto_np[:, 2] - odom_np[:, 2]
        diff = np.sqrt(dx * dx + dy * dy)

        print(f"  średnia różnica Cartographer-odom XY: {float(np.mean(diff)):.6f} m")
        print(f"  maksymalna różnica Cartographer-odom XY: {float(np.max(diff)):.6f} m")


if __name__ == "__main__":
    main()
