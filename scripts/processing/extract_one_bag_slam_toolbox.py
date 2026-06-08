import sys
import os
import csv
import math
import numpy as np
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


GT_TOPIC = "/model/four_wheel_bot/pose"


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
        [0,  0,  1]
    ])


def matrix_to_pose(T):
    x = T[0, 2]
    y = T[1, 2]
    yaw = math.atan2(T[1, 0], T[0, 0])
    return x, y, yaw


def normalize_trajectory(rows):
    """
    Przekształca trajektorię do jej układu początkowego.
    Pierwsza poza staje się (0, 0, 0).
    """
    if not rows:
        return []

    x0, y0, yaw0 = rows[0][1], rows[0][2], rows[0][3]
    T0 = tf_to_matrix(x0, y0, yaw0)
    T0_inv = np.linalg.inv(T0)

    out = []
    for row in rows:
        t, x, y, yaw = row
        T = tf_to_matrix(x, y, yaw)
        Tn = T0_inv @ T
        xn, yn, yawn = matrix_to_pose(Tn)
        out.append([t, xn, yn, yawn])

    return out


def detect_storage_id(bag_path):
    files = os.listdir(bag_path)

    if any(f.endswith(".mcap") for f in files):
        return "mcap"

    if any(f.endswith(".db3") for f in files):
        return "sqlite3"

    return "mcap"


def save_csv(path, rows):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "x", "y", "yaw_rad"])
        writer.writerows(rows)


def plot_two_trajectories(rows_a, rows_b, label_a, label_b, title, path):
    arr_a = np.array(rows_a) if rows_a else np.empty((0, 4))
    arr_b = np.array(rows_b) if rows_b else np.empty((0, 4))

    plt.figure(figsize=(8, 6))

    if len(arr_a) > 0:
        plt.plot(arr_a[:, 1], arr_a[:, 2], label=label_a)

    if len(arr_b) > 0:
        plt.plot(arr_b[:, 1], arr_b[:, 2], label=label_b)

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(title)
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.savefig(path, dpi=200)
    plt.close()


def plot_three_trajectories(rows_a, rows_b, rows_c,
                            label_a, label_b, label_c,
                            title, path):
    arr_a = np.array(rows_a) if rows_a else np.empty((0, 4))
    arr_b = np.array(rows_b) if rows_b else np.empty((0, 4))
    arr_c = np.array(rows_c) if rows_c else np.empty((0, 4))

    plt.figure(figsize=(8, 6))

    if len(arr_a) > 0:
        plt.plot(arr_a[:, 1], arr_a[:, 2], label=label_a)

    if len(arr_b) > 0:
        plt.plot(arr_b[:, 1], arr_b[:, 2], label=label_b)

    if len(arr_c) > 0:
        plt.plot(arr_c[:, 1], arr_c[:, 2], label=label_c)

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(title)
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.savefig(path, dpi=200)
    plt.close()


def main():
    if len(sys.argv) < 2:
        print("Użycie:")
        print("python3 extract_one_bag_slam_toolbox.py /home/jaokoz/ros2_ws/rosbag2_xxx")
        sys.exit(1)

    bag_path = sys.argv[1]

    if not os.path.isdir(bag_path):
        print(f"Nie znaleziono folderu rosbaga: {bag_path}")
        sys.exit(1)

    bag_name = os.path.basename(os.path.normpath(bag_path))

    base_output_dir = "/home/jaokoz/ros_scripts/wyniki/slam_toolbox"
    os.makedirs(base_output_dir, exist_ok=True)

    output_dir = os.path.join(base_output_dir, bag_name)
    os.makedirs(output_dir, exist_ok=True)

    odom_csv = os.path.join(output_dir, "odom.csv")
    slam_csv = os.path.join(output_dir, "slam_toolbox.csv")
    gt_csv = os.path.join(output_dir, "ground_truth.csv")

    plot_odom_slam = os.path.join(output_dir, "trajektorie_odom_vs_slam.png")
    plot_gt_slam_raw = os.path.join(output_dir, "trajektorie_gt_vs_slam_surowe.png")
    plot_gt_slam_aligned = os.path.join(output_dir, "trajektorie_gt_vs_slam_wyrownane.png")
    plot_three_raw = os.path.join(output_dir, "trajektorie_odom_slam_gt_surowe.png")
    plot_three_aligned = os.path.join(output_dir, "trajektorie_odom_slam_gt_wyrownane.png")

    # Plik domyślny dla strony.
    plot_default = os.path.join(output_dir, "trajektorie.png")

    storage_id = detect_storage_id(bag_path)
    print(f"Wykryty storage_id: {storage_id}")

    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions("", "")

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    required = ["/odom", "/tf"]
    for r in required:
        if r not in type_map:
            print(f"Brakuje {r} w rosbagu")
            print("Dostępne topiki:")
            for topic_name in type_map:
                print(f" - {topic_name}")
            sys.exit(1)

    odom_type = get_message(type_map["/odom"])
    tf_type = get_message(type_map["/tf"])

    if GT_TOPIC in type_map:
        gt_type = get_message(type_map[GT_TOPIC])
        print(f"Znaleziono ground truth: {GT_TOPIC}")
    else:
        gt_type = None
        print(f"UWAGA: Brak ground truth {GT_TOPIC} w rosbagu")

    odom_rows = []
    slam_rows = []
    gt_rows = []

    latest_odom_base = None
    latest_odom_stamp = None

    map_odom_count = 0
    odom_count = 0
    slam_count = 0
    gt_count = 0

    while reader.has_next():
        topic, data, t = reader.read_next()

        if topic == "/odom":
            msg = deserialize_message(data, odom_type)

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            q = msg.pose.pose.orientation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            latest_odom_base = tf_to_matrix(x, y, yaw)
            latest_odom_stamp = stamp

            odom_rows.append([stamp, x, y, yaw])
            odom_count += 1

        elif topic == "/tf":
            msg = deserialize_message(data, tf_type)

            for tr in msg.transforms:
                parent = tr.header.frame_id
                child = tr.child_frame_id

                if parent == "map" and child == "odom":
                    tx = tr.transform.translation.x
                    ty = tr.transform.translation.y

                    q = tr.transform.rotation
                    yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

                    T_map_odom = tf_to_matrix(tx, ty, yaw)
                    map_odom_count += 1

                    # Trajektoria SLAM = map -> base_link
                    # T_map_base = T_map_odom * T_odom_base
                    if latest_odom_base is not None and latest_odom_stamp is not None:
                        T_map_base = T_map_odom @ latest_odom_base
                        sx, sy, syaw = matrix_to_pose(T_map_base)

                        slam_rows.append([latest_odom_stamp, sx, sy, syaw])
                        slam_count += 1

        elif topic == GT_TOPIC and gt_type is not None:
            msg = deserialize_message(data, gt_type)

            x = msg.position.x
            y = msg.position.y

            q = msg.orientation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

            # geometry_msgs/Pose nie ma headera, więc używamy czasu z rosbaga.
            stamp = t * 1e-9

            gt_rows.append([stamp, x, y, yaw])
            gt_count += 1

    save_csv(odom_csv, odom_rows)
    save_csv(slam_csv, slam_rows)
    save_csv(gt_csv, gt_rows)

    print(f"Liczba próbek /odom: {odom_count}")
    print(f"Liczba transformacji map->odom: {map_odom_count}")
    print(f"Liczba próbek trajektorii SLAM: {slam_count}")
    print(f"Liczba próbek ground truth: {gt_count}")

    print(f"Zapisano: {odom_csv}")
    print(f"Zapisano: {slam_csv}")
    print(f"Zapisano: {gt_csv}")

    odom_rows_norm = normalize_trajectory(odom_rows)
    slam_rows_norm = normalize_trajectory(slam_rows)
    gt_rows_norm = normalize_trajectory(gt_rows)

    # 1. Wykres pomocniczy: odometria vs SLAM
    plot_two_trajectories(
        odom_rows,
        slam_rows,
        "odom",
        "slam_toolbox",
        "Porównanie trajektorii: odom vs slam_toolbox",
        plot_odom_slam
    )

    # 2. Wykres GT vs SLAM surowy
    plot_two_trajectories(
        gt_rows,
        slam_rows,
        "ground truth Gazebo",
        "slam_toolbox",
        "Trajektorie surowe: ground truth vs slam_toolbox",
        plot_gt_slam_raw
    )

    # 3. Wykres GT vs SLAM wyrównany
    plot_two_trajectories(
        gt_rows_norm,
        slam_rows_norm,
        "ground truth Gazebo",
        "slam_toolbox",
        "Trajektorie wyrównane: ground truth vs slam_toolbox",
        plot_gt_slam_aligned
    )

    # 4. Wykres trzech trajektorii surowych
    plot_three_trajectories(
        gt_rows,
        odom_rows,
        slam_rows,
        "ground truth Gazebo",
        "odom",
        "slam_toolbox",
        "Trajektorie surowe: ground truth, odom i slam_toolbox",
        plot_three_raw
    )

    # 5. Wykres trzech trajektorii wyrównanych
    plot_three_trajectories(
        gt_rows_norm,
        odom_rows_norm,
        slam_rows_norm,
        "ground truth Gazebo",
        "odom",
        "slam_toolbox",
        "Trajektorie wyrównane: ground truth, odom i slam_toolbox",
        plot_three_aligned
    )

    # 6. Domyślny wykres dla strony:
    # ground truth + odom + slam_toolbox po wyrównaniu do początku.
    plot_three_trajectories(
        gt_rows_norm,
        odom_rows_norm,
        slam_rows_norm,
        "ground truth Gazebo",
        "odom",
        "slam_toolbox",
        "Trajektorie wyrównane: ground truth, odom i slam_toolbox",
        plot_default
    )

    print(f"Zapisano wykres pomocniczy: {plot_odom_slam}")
    print(f"Zapisano wykres GT vs SLAM surowy: {plot_gt_slam_raw}")
    print(f"Zapisano wykres GT vs SLAM wyrównany: {plot_gt_slam_aligned}")
    print(f"Zapisano wykres trzech trajektorii surowych: {plot_three_raw}")
    print(f"Zapisano wykres trzech trajektorii wyrównanych: {plot_three_aligned}")
    print(f"Zapisano wykres domyślny dla strony: {plot_default}")
    print(f"Wyniki zapisane do folderu: {output_dir}")


if __name__ == "__main__":
    main()
