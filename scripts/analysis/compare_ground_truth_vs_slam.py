import os
import csv
import argparse
import numpy as np
import matplotlib.pyplot as plt


def load_csv(path, time_col="time_s", x_col="x", y_col="y", yaw_col="yaw_rad"):
    rows = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = float(row[time_col])
                x = float(row[x_col])
                y = float(row[y_col])
                yaw = float(row[yaw_col])
                rows.append([t, x, y, yaw])
            except (ValueError, KeyError):
                continue
    if not rows:
        return np.empty((0, 4))
    return np.array(rows, dtype=float)


def normalize_time(arr):
    if len(arr) == 0:
        return arr
    out = arr.copy()
    out[:, 0] -= out[0, 0]
    return out


def normalize_start(arr):
    if len(arr) == 0:
        return arr
    out = arr.copy()
    out[:, 1] -= out[0, 1]
    out[:, 2] -= out[0, 2]
    return out


def interpolate_reference(ref_arr, query_t):
    ref_t = ref_arr[:, 0]
    ref_x = ref_arr[:, 1]
    ref_y = ref_arr[:, 2]
    ref_yaw = ref_arr[:, 3]

    x_i = np.interp(query_t, ref_t, ref_x)
    y_i = np.interp(query_t, ref_t, ref_y)

    sin_y = np.sin(ref_yaw)
    cos_y = np.cos(ref_yaw)
    sin_i = np.interp(query_t, ref_t, sin_y)
    cos_i = np.interp(query_t, ref_t, cos_y)
    yaw_i = np.arctan2(sin_i, cos_i)

    return np.column_stack([query_t, x_i, y_i, yaw_i])


def save_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def compare(gt_arr, slam_arr, odom_arr, out_dir, method_name, prefix):
    gt_arr = normalize_time(gt_arr)
    slam_arr = normalize_time(slam_arr)
    odom_arr = normalize_time(odom_arr)

    t_min = max(gt_arr[0, 0], slam_arr[0, 0], odom_arr[0, 0])
    t_max = min(gt_arr[-1, 0], slam_arr[-1, 0], odom_arr[-1, 0])

    gt_mask = (gt_arr[:, 0] >= t_min) & (gt_arr[:, 0] <= t_max)
    gt_common = gt_arr[gt_mask]

    if len(gt_common) < 5:
        raise RuntimeError("Za mało wspólnych próbek czasu.")

    slam_interp = interpolate_reference(slam_arr, gt_common[:, 0])
    odom_interp = interpolate_reference(odom_arr, gt_common[:, 0])

    err_slam = np.linalg.norm(gt_common[:, 1:3] - slam_interp[:, 1:3], axis=1)
    err_odom = np.linalg.norm(gt_common[:, 1:3] - odom_interp[:, 1:3], axis=1)

    rmse_slam = np.sqrt(np.mean(err_slam ** 2))
    rmse_odom = np.sqrt(np.mean(err_odom ** 2))

    mean_slam = np.mean(err_slam)
    mean_odom = np.mean(err_odom)

    max_slam = np.max(err_slam)
    max_odom = np.max(err_odom)

    rows = []
    for i in range(len(gt_common)):
        rows.append([
            gt_common[i, 0],
            gt_common[i, 1], gt_common[i, 2], gt_common[i, 3],
            slam_interp[i, 1], slam_interp[i, 2], slam_interp[i, 3],
            odom_interp[i, 1], odom_interp[i, 2], odom_interp[i, 3],
            err_slam[i],
            err_odom[i]
        ])

    csv_path = os.path.join(out_dir, f"{prefix}.csv")
    save_csv(
        csv_path,
        [
            "time_s",
            "gt_x", "gt_y", "gt_yaw_rad",
            "slam_x", "slam_y", "slam_yaw_rad",
            "odom_x", "odom_y", "odom_yaw_rad",
            "error_slam_m",
            "error_odom_m",
        ],
        rows
    )

    plt.figure(figsize=(10, 8))
    plt.plot(gt_arr[:, 1], gt_arr[:, 2], linewidth=2, label="ground_truth")
    plt.plot(slam_arr[:, 1], slam_arr[:, 2], linewidth=2, label=method_name)
    plt.plot(odom_arr[:, 1], odom_arr[:, 2], linewidth=2, label="odom")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(
        f"GT vs {method_name} vs odom | "
        f"RMSE {method_name}={rmse_slam:.3f} m | "
        f"RMSE odom={rmse_odom:.3f} m"
    )
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    png_abs = os.path.join(out_dir, f"{prefix}_all_abs.png")
    plt.savefig(png_abs, dpi=200)
    plt.close()

    gt_start = normalize_start(gt_arr)
    slam_start = normalize_start(slam_arr)
    odom_start = normalize_start(odom_arr)

    plt.figure(figsize=(10, 8))
    plt.plot(gt_start[:, 1], gt_start[:, 2], linewidth=2, label="ground_truth_start0")
    plt.plot(slam_start[:, 1], slam_start[:, 2], linewidth=2, label=f"{method_name}_start0")
    plt.plot(odom_start[:, 1], odom_start[:, 2], linewidth=2, label="odom_start0")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(
        f"GT vs {method_name} vs odom (start = 0,0) | "
        f"mean {method_name}={mean_slam:.3f} m | "
        f"mean odom={mean_odom:.3f} m"
    )
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    png_start0 = os.path.join(out_dir, f"{prefix}_all_start0.png")
    plt.savefig(png_start0, dpi=200)
    plt.close()

    print(f"RMSE {method_name:<12}= {rmse_slam:.4f} m")
    print(f"RMSE odom         = {rmse_odom:.4f} m")
    print(f"MEAN {method_name:<12}= {mean_slam:.4f} m")
    print(f"MEAN odom         = {mean_odom:.4f} m")
    print(f"MAX  {method_name:<12}= {max_slam:.4f} m")
    print(f"MAX  odom         = {max_odom:.4f} m")
    print(f"Zapisano: {csv_path}")
    print(f"Zapisano: {png_abs}")
    print(f"Zapisano: {png_start0}")


def main():
    parser = argparse.ArgumentParser(description="Porównanie ground truth z trajektorią SLAM i odometrią")
    parser.add_argument("--gt-csv", required=True, help="Ścieżka do ground_truth_raw.csv")
    parser.add_argument("--slam-csv", required=True, help="Ścieżka do CSV z trajektorią SLAM")
    parser.add_argument("--odom-csv", required=True, help="Ścieżka do odom.csv")
    parser.add_argument("--out-dir", required=True, help="Folder wynikowy")
    parser.add_argument("--method-name", required=True, help="Nazwa metody, np. cartographer albo slam_toolbox")
    parser.add_argument("--prefix", required=True, help="Prefiks nazw plików wynikowych")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    gt_arr = load_csv(args.gt_csv)
    slam_arr = load_csv(args.slam_csv)
    odom_arr = load_csv(args.odom_csv)

    if len(gt_arr) < 5:
        raise RuntimeError("Za mało danych w ground truth CSV.")
    if len(slam_arr) < 5:
        raise RuntimeError("Za mało danych w SLAM CSV.")
    if len(odom_arr) < 5:
        raise RuntimeError("Za mało danych w odom CSV.")

    compare(gt_arr, slam_arr, odom_arr, args.out_dir, args.method_name, args.prefix)


if __name__ == "__main__":
    main()
