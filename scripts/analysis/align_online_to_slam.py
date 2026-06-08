import cv2
import csv
import math
import json
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import faulthandler

faulthandler.enable()


def dbg(msg):
    print(f"[DEBUG] {msg}", flush=True)


# =========================================================
# NARZĘDZIA
# =========================================================

def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def load_slam_csv(path, time_col="time_s", x_col="x", y_col="y", yaw_col="yaw_rad"):
    dbg(f"load_slam_csv: {path}")
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
    dbg(f"load_slam_csv: wczytano {len(rows)} wierszy")
    return np.array(rows, dtype=float) if rows else np.empty((0, 4))


def normalize_time(arr):
    if len(arr) == 0:
        return arr
    out = arr.copy()
    out[:, 0] -= out[0, 0]
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


def fit_se2(A, B):
    centroid_A = A.mean(axis=0)
    centroid_B = B.mean(axis=0)

    AA = A - centroid_A
    BB = B - centroid_B

    H = AA.T @ BB
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_B - R @ centroid_A
    return R, t


def transform_points(points_xy, R, t):
    return (R @ points_xy.T).T + t


def save_csv(path, rows, header):
    dbg(f"save_csv: zapis do {path}, liczba wierszy={len(rows)}")
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def ensure_dir(path):
    dbg(f"ensure_dir: {path}")
    os.makedirs(path, exist_ok=True)


# =========================================================
# HOMOGRAFIA
# =========================================================

def load_homography_from_json(path):
    dbg(f"load_homography_from_json: {path}")
    with open(path, "r") as f:
        data = json.load(f)

    img_pts = np.array(data["image_points"], dtype=np.float32)
    world_pts = np.array(data["world_points"], dtype=np.float32)

    dbg(f"homography: image_points.shape={img_pts.shape}, world_points.shape={world_pts.shape}")

    if img_pts.shape[0] < 4 or world_pts.shape[0] < 4:
        raise ValueError("Potrzebne są co najmniej 4 punkty do homografii.")

    H, status = cv2.findHomography(img_pts, world_pts)
    if H is None:
        raise RuntimeError("Nie udało się wyznaczyć homografii.")

    dbg(f"homography: H=\n{H}")
    return H


def image_to_world(H, u, v):
    p = np.array([u, v, 1.0], dtype=np.float64)
    q = H @ p
    q /= q[2]
    return float(q[0]), float(q[1])


# =========================================================
# ARUCO
# =========================================================

ARUCO_DICTS = {
    "4x4_50": cv2.aruco.DICT_4X4_50,
    "4x4_100": cv2.aruco.DICT_4X4_100,
    "5x5_50": cv2.aruco.DICT_5X5_50,
    "5x5_100": cv2.aruco.DICT_5X5_100,
    "6x6_50": cv2.aruco.DICT_6X6_50,
    "6x6_100": cv2.aruco.DICT_6X6_100,
}


def detect_marker_pose_2d(frame, aruco_dict_name, marker_id, frame_idx=None):
    if frame_idx is not None and frame_idx < 5:
        dbg(f"detect_marker_pose_2d: frame_idx={frame_idx} start")

    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[aruco_dict_name])

    try:
        params = cv2.aruco.DetectorParameters()
        if frame_idx is not None and frame_idx < 5:
            dbg("aruco: użyto DetectorParameters()")
    except AttributeError:
        params = cv2.aruco.DetectorParameters_create()
        if frame_idx is not None and frame_idx < 5:
            dbg("aruco: użyto DetectorParameters_create()")

    if frame is None:
        dbg("detect_marker_pose_2d: frame is None")
        return None

    if frame_idx is not None and frame_idx < 5:
        dbg(f"frame shape={frame.shape}, dtype={frame.dtype}")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if frame_idx is not None and frame_idx < 5:
        dbg("aruco: po cvtColor")

    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

    if frame_idx is not None and frame_idx < 5:
        dbg("aruco: po detectMarkers")

    if ids is None:
        if frame_idx is not None and frame_idx < 5:
            dbg("aruco: ids is None")
        return None

    ids = ids.flatten()

    if frame_idx is not None and frame_idx < 5:
        dbg(f"aruco: znalezione ids={ids.tolist()}")

    for c, mid in zip(corners, ids):
        if int(mid) == marker_id:
            pts = c.reshape(4, 2)

            center = pts.mean(axis=0)

            p0 = pts[0]
            p1 = pts[1]
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]

            yaw_img = math.atan2(dy, dx)

            if frame_idx is not None and frame_idx < 5:
                dbg(f"aruco: wykryto marker {marker_id}, center=({center[0]:.2f},{center[1]:.2f})")

            return {
                "corners": pts,
                "u": float(center[0]),
                "v": float(center[1]),
                "yaw_img": yaw_img,
            }

    if frame_idx is not None and frame_idx < 5:
        dbg(f"aruco: nie znaleziono markera o id={marker_id} wśród wykrytych")
    return None


# =========================================================
# GŁÓWNY PIPELINE
# =========================================================

def process_video(video_path, out_dir, H, aruco_dict_name, marker_id, preview=False):
    dbg("process_video: start")
    dbg(f"process_video: video_path={video_path}")

    cap = cv2.VideoCapture(video_path)
    dbg("process_video: po VideoCapture")

    if not cap.isOpened():
        raise RuntimeError(f"Nie mogę otworzyć wideo: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    dbg(f"process_video: fps={fps}")

    if fps <= 0:
        fps = 30.0
        dbg("process_video: fps <= 0, ustawiono 30.0")

    raw_rows = []
    debug_frames = []
    detections = 0

    frame_idx = 0

    while True:
        if frame_idx < 5:
            dbg(f"read: przed cap.read(), frame_idx={frame_idx}")

        ok, frame = cap.read()

        if frame_idx < 5:
            dbg(f"read: po cap.read(), ok={ok}")

        if not ok:
            dbg(f"read: koniec wideo na frame_idx={frame_idx}")
            break

        det = detect_marker_pose_2d(frame, aruco_dict_name, marker_id, frame_idx=frame_idx)

        if det is not None:
            detections += 1
            t = frame_idx / fps
            u = det["u"]
            v = det["v"]

            if frame_idx < 5:
                dbg(f"frame {frame_idx}: marker center u={u:.2f}, v={v:.2f}")

            x, y = image_to_world(H, u, v)

            pts = det["corners"]
            p0 = pts[0]
            p1 = pts[1]

            wx0, wy0 = image_to_world(H, p0[0], p0[1])
            wx1, wy1 = image_to_world(H, p1[0], p1[1])

            yaw_world = math.atan2(wy1 - wy0, wx1 - wx0)
            yaw_world = wrap_angle(yaw_world)

            raw_rows.append([t, x, y, yaw_world])

            if frame_idx < 5:
                dbg(f"frame {frame_idx}: x={x:.4f}, y={y:.4f}, yaw={yaw_world:.4f}")

            if preview and len(debug_frames) < 3:
                frame_vis = frame.copy()
                cv2.polylines(frame_vis, [pts.astype(np.int32)], True, (0, 255, 0), 2)
                cv2.circle(frame_vis, (int(u), int(v)), 5, (0, 0, 255), -1)
                debug_frames.append(cv2.cvtColor(frame_vis, cv2.COLOR_BGR2RGB))

        frame_idx += 1

    cap.release()
    dbg("process_video: po cap.release()")

    print(f"Liczba klatek: {frame_idx}")
    print(f"Liczba wykryć markera: {detections}")

    if len(raw_rows) < 5:
        raise RuntimeError("Za mało wykryć markera w nagraniu.")

    raw_arr = np.array(raw_rows, dtype=float)
    raw_arr = normalize_time(raw_arr)

    raw_csv = os.path.join(out_dir, "ground_truth_raw.csv")
    save_csv(raw_csv, raw_arr.tolist(), ["time_s", "x", "y", "yaw_rad"])

    dbg("process_video: rysowanie ground_truth_raw.png")
    plt.figure(figsize=(8, 6))
    plt.plot(raw_arr[:, 1], raw_arr[:, 2], linewidth=2, label="ground truth z kamery")
    plt.scatter([raw_arr[0, 1]], [raw_arr[0, 2]], s=50, label="start")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trajektoria odniesienia wyznaczona z nagrania")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    traj_png = os.path.join(out_dir, "ground_truth_raw.png")
    plt.savefig(traj_png, dpi=200)
    plt.close()

    if preview and len(debug_frames) > 0:
        dbg("process_video: zapis opencv_debug_frame.png")
        plt.figure(figsize=(8, 6))
        plt.imshow(debug_frames[0])
        plt.axis("off")
        plt.tight_layout()
        frame_png = os.path.join(out_dir, "opencv_debug_frame.png")
        plt.savefig(frame_png, dpi=200, bbox_inches="tight")
        plt.close()

    dbg("process_video: koniec")
    return raw_arr


def align_to_slam(gt_arr, slam_arr, out_dir, slam_name="slam"):
    dbg("align_to_slam: start")

    gt_arr = normalize_time(gt_arr)
    slam_arr = normalize_time(slam_arr)

    t_min = max(gt_arr[0, 0], slam_arr[0, 0])
    t_max = min(gt_arr[-1, 0], slam_arr[-1, 0])

    dbg(f"align_to_slam: t_min={t_min}, t_max={t_max}")

    gt_mask = (gt_arr[:, 0] >= t_min) & (gt_arr[:, 0] <= t_max)
    gt_common = gt_arr[gt_mask]

    if len(gt_common) < 5:
        raise RuntimeError("Za mało wspólnych próbek czasu między kamerą i SLAM.")

    slam_interp = interpolate_reference(slam_arr, gt_common[:, 0])

    gt_xy = gt_common[:, 1:3]
    slam_xy = slam_interp[:, 1:3]

    R, t = fit_se2(gt_xy, slam_xy)

    all_gt_xy = gt_arr[:, 1:3]
    all_gt_aligned_xy = transform_points(all_gt_xy, R, t)

    alpha = math.atan2(R[1, 0], R[0, 0])
    all_gt_aligned_yaw = np.array([wrap_angle(yaw + alpha) for yaw in gt_arr[:, 3]])

    gt_common_aligned_xy = transform_points(gt_common[:, 1:3], R, t)
    errors = np.linalg.norm(gt_common_aligned_xy - slam_xy, axis=1)
    rmse = np.sqrt(np.mean(errors ** 2))
    max_err = np.max(errors)

    aligned_arr = np.column_stack([
        gt_arr[:, 0],
        all_gt_aligned_xy[:, 0],
        all_gt_aligned_xy[:, 1],
        all_gt_aligned_yaw
    ])

    aligned_csv = os.path.join(out_dir, f"ground_truth_aligned_to_{slam_name}.csv")
    save_csv(aligned_csv, aligned_arr.tolist(), ["time_s", "x", "y", "yaw_rad"])

    dbg("align_to_slam: rysowanie wykresu porównania")
    plt.figure(figsize=(8, 6))
    plt.plot(slam_arr[:, 1], slam_arr[:, 2], linewidth=2, label=slam_name)
    plt.plot(gt_arr[:, 1], gt_arr[:, 2], linewidth=1, label="kamera surowa")
    plt.plot(aligned_arr[:, 1], aligned_arr[:, 2], linewidth=2, label="kamera wyrównana")
    plt.scatter([slam_arr[0, 1]], [slam_arr[0, 2]], s=50, label=f"{slam_name} start")
    plt.scatter([aligned_arr[0, 1]], [aligned_arr[0, 2]], s=50, label="kamera start")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(f"Wyrównanie kamery -> {slam_name} | RMSE={rmse:.3f} m | max={max_err:.3f} m")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plot_png = os.path.join(out_dir, f"ground_truth_vs_{slam_name}.png")
    plt.savefig(plot_png, dpi=200)
    plt.close()

    print("\n=== WYNIK DOPASOWANIA ===")
    print(f"slam_name   = {slam_name}")
    print(f"alpha [rad] = {alpha}")
    print(f"alpha [deg] = {math.degrees(alpha)}")
    print(f"tx          = {t[0]}")
    print(f"ty          = {t[1]}")
    print(f"RMSE        = {rmse:.4f} m")
    print(f"MAX         = {max_err:.4f} m")
    print(f"Zapisano: {aligned_csv}")
    print(f"Zapisano: {plot_png}")

    dbg("align_to_slam: koniec")
    return aligned_arr


# =========================================================
# MAIN
# =========================================================

def main():
    dbg("START PROGRAMU")
    dbg(f"OpenCV version: {cv2.__version__}")
    dbg(f"hasattr(cv2.aruco, 'detectMarkers') = {hasattr(cv2.aruco, 'detectMarkers')}")
    dbg(f"hasattr(cv2.aruco, 'ArucoDetector') = {hasattr(cv2.aruco, 'ArucoDetector')}")

    parser = argparse.ArgumentParser(
        description="Wideo z markerem ArUco -> trajektoria ground truth -> opcjonalne porównanie ze SLAM"
    )
    parser.add_argument("--video", required=True, help="Ścieżka do pliku wideo")
    parser.add_argument("--homography", required=True, help="JSON z punktami do homografii")
    parser.add_argument("--marker-id", type=int, required=True, help="ID markera ArUco")
    parser.add_argument("--dict", default="5x5_50", choices=list(ARUCO_DICTS.keys()), help="Słownik ArUco")
    parser.add_argument("--out-dir", default="out_ground_truth", help="Folder wynikowy")
    parser.add_argument("--slam-csv", default="", help="Opcjonalnie trajektoria SLAM do porównania")
    parser.add_argument("--slam-name", default="slam", help="Nazwa trajektorii SLAM")
    parser.add_argument("--preview", action="store_true", help="Zapisz przykładową klatkę debug")

    args = parser.parse_args()

    dbg(f"args.video={args.video}")
    dbg(f"args.homography={args.homography}")
    dbg(f"args.marker_id={args.marker_id}")
    dbg(f"args.dict={args.dict}")
    dbg(f"args.out_dir={args.out_dir}")
    dbg(f"args.preview={args.preview}")

    ensure_dir(args.out_dir)

    H = load_homography_from_json(args.homography)

    gt_arr = process_video(
        video_path=args.video,
        out_dir=args.out_dir,
        H=H,
        aruco_dict_name=args.dict,
        marker_id=args.marker_id,
        preview=args.preview
    )

    print(f"Zapisano: {os.path.join(args.out_dir, 'ground_truth_raw.csv')}")
    print(f"Zapisano: {os.path.join(args.out_dir, 'ground_truth_raw.png')}")

    if args.slam_csv:
        slam_arr = load_slam_csv(args.slam_csv)
        if len(slam_arr) < 5:
            raise RuntimeError("Za mało danych w pliku SLAM CSV.")
        align_to_slam(gt_arr, slam_arr, args.out_dir, slam_name=args.slam_name)

    dbg("KONIEC PROGRAMU")


if __name__ == "__main__":
    main()
