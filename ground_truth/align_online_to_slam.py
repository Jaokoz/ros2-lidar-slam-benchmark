#!/usr/bin/env python3

import cv2
import csv
import math
import json
import os
import argparse
import time
import numpy as np
import matplotlib.pyplot as plt
import faulthandler

faulthandler.enable()


def dbg(msg: str):
    print(f"[DEBUG] {msg}", flush=True)


def format_seconds(seconds: float) -> str:
    seconds = max(0, int(seconds))
    h = seconds // 3600
    m = (seconds % 3600) // 60
    s = seconds % 60
    if h > 0:
        return f"{h:d}:{m:02d}:{s:02d}"
    return f"{m:02d}:{s:02d}"


def print_progress(current_frame: int, total_frames: int, detections: int, start_time: float):
    if total_frames <= 0:
        return

    elapsed = time.time() - start_time
    progress = min(max(current_frame / total_frames, 0.0), 1.0)
    percent = 100.0 * progress

    if progress > 1e-9:
        eta = elapsed * (1.0 - progress) / progress
    else:
        eta = 0.0

    msg = (
        f"\rPostęp: {percent:6.2f}% | "
        f"klatka {current_frame}/{total_frames} | "
        f"wykrycia: {detections} | "
        f"czas: {format_seconds(elapsed)} | "
        f"ETA: {format_seconds(eta)}"
    )
    print(msg, end="", flush=True)


# =========================================================
# NARZĘDZIA OGÓLNE
# =========================================================

def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def save_csv(path, rows, header):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def normalize_time(arr):
    if len(arr) == 0:
        return arr
    out = arr.copy()
    out[:, 0] -= out[0, 0]
    return out


def normalize_pose_start(arr):
    if len(arr) == 0:
        return arr
    out = arr.copy()
    out[:, 1] -= out[0, 1]
    out[:, 2] -= out[0, 2]
    return out


def load_slam_csv(path, time_col="time_s", x_col="x", y_col="y", yaw_col="yaw_rad"):
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
    return np.array(rows, dtype=float) if rows else np.empty((0, 4), dtype=float)


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


def save_plot(path, arr_xy, title, label="ground truth z kamery"):
    plt.figure(figsize=(8, 6))
    plt.plot(arr_xy[:, 0], arr_xy[:, 1], linewidth=2, label=label)
    plt.scatter([arr_xy[0, 0]], [arr_xy[0, 1]], s=50, label="start")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(title)
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=200)
    plt.close()


# =========================================================
# HOMOGRAFIA
# =========================================================

def load_homography_from_json(path):
    with open(path, "r") as f:
        data = json.load(f)

    img_pts = np.array(data["image_points"], dtype=np.float32)
    world_pts = np.array(data["world_points"], dtype=np.float32)

    if img_pts.shape[0] < 4 or world_pts.shape[0] < 4:
        raise ValueError("Potrzebne są co najmniej 4 punkty do homografii.")

    H, _ = cv2.findHomography(img_pts, world_pts)
    if H is None:
        raise RuntimeError("Nie udało się wyznaczyć homografii.")

    dbg(f"homography H=\n{H}")
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
    "4x4_250": cv2.aruco.DICT_4X4_250,
    "4x4_1000": cv2.aruco.DICT_4X4_1000,

    "5x5_50": cv2.aruco.DICT_5X5_50,
    "5x5_100": cv2.aruco.DICT_5X5_100,
    "5x5_250": cv2.aruco.DICT_5X5_250,
    "5x5_1000": cv2.aruco.DICT_5X5_1000,

    "6x6_50": cv2.aruco.DICT_6X6_50,
    "6x6_100": cv2.aruco.DICT_6X6_100,
    "6x6_250": cv2.aruco.DICT_6X6_250,
    "6x6_1000": cv2.aruco.DICT_6X6_1000,

    "7x7_50": cv2.aruco.DICT_7X7_50,
    "7x7_100": cv2.aruco.DICT_7X7_100,
    "7x7_250": cv2.aruco.DICT_7X7_250,
    "7x7_1000": cv2.aruco.DICT_7X7_1000,
}


def get_predefined_dictionary(dict_name: str):
    dict_id = ARUCO_DICTS[dict_name]
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)


def create_detector_parameters():
    # Stabilna ścieżka dla OpenCV 4.6.0
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()

    params.adaptiveThreshWinSizeMin = 5
    params.adaptiveThreshWinSizeMax = 41
    params.adaptiveThreshWinSizeStep = 4
    params.minMarkerPerimeterRate = 0.01
    params.maxMarkerPerimeterRate = 2.0
    params.polygonalApproxAccuracyRate = 0.05
    params.minCornerDistanceRate = 0.01
    params.minDistanceToBorder = 2
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    return params


def build_detector(dict_name: str):
    dictionary = get_predefined_dictionary(dict_name)
    params = create_detector_parameters()
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    subpix_criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.01
    )
    return dictionary, params, clahe, subpix_criteria


def make_search_rect_from_prev(prev_state, frame_shape):
    h, w = frame_shape[:2]

    cx = int(round(prev_state["cx"]))
    cy = int(round(prev_state["cy"]))
    size = float(prev_state["size"])

    half = int(max(90, size * 2.8))

    x0 = max(0, cx - half)
    y0 = max(0, cy - half)
    x1 = min(w, cx + half)
    y1 = min(h, cy + half)

    return x0, y0, x1, y1


def detect_on_crop(
    crop_bgr,
    offset_x,
    offset_y,
    dictionary,
    params,
    clahe,
    subpix_criteria,
    marker_id,
    scale,
    frame_idx=None,
    search_name="full"
):
    if crop_bgr is None or crop_bgr.size == 0:
        return None

    gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)

    if scale <= 0.0:
        scale = 1.0

    if scale != 1.0:
        interp = cv2.INTER_CUBIC if scale > 1.0 else cv2.INTER_AREA
        gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=interp)

    gray = np.ascontiguousarray(gray)
    gray_clahe = clahe.apply(gray)

    variants = [
        ("gray", gray),
        ("clahe", gray_clahe),
    ]

    for variant_name, gray_used in variants:
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray_used,
            dictionary,
            parameters=params
        )

        if ids is None or len(ids) == 0:
            continue

        ids = ids.flatten().astype(int)

        for c, mid in zip(corners, ids):
            if int(mid) != int(marker_id):
                continue

            c2 = c.reshape(-1, 1, 2).astype(np.float32)

            try:
                cv2.cornerSubPix(
                    gray_used,
                    c2,
                    winSize=(3, 3),
                    zeroZone=(-1, -1),
                    criteria=subpix_criteria
                )
            except cv2.error:
                pass

            pts = c2.reshape(4, 2)

            if scale != 1.0:
                pts = pts / scale

            pts[:, 0] += offset_x
            pts[:, 1] += offset_y

            center = pts.mean(axis=0)

            edge01 = np.linalg.norm(pts[1] - pts[0])
            edge12 = np.linalg.norm(pts[2] - pts[1])
            size_px = max(edge01, edge12)

            p0 = pts[0]
            p1 = pts[1]
            yaw_img = math.atan2(p1[1] - p0[1], p1[0] - p0[0])

            dbg(
                f"frame {frame_idx}: marker znaleziony | "
                f"search={search_name} | variant={variant_name} | id={mid}"
            )

            return {
                "corners": pts.astype(np.float32),
                "u": float(center[0]),
                "v": float(center[1]),
                "yaw_img": yaw_img,
                "size_px": float(size_px),
                "search_name": search_name,
            }

    return None


def detect_marker_pose_2d(
    frame,
    dictionary,
    params,
    clahe,
    subpix_criteria,
    marker_id,
    prev_state=None,
    frame_idx=None,
    full_scale=1.0,
    local_scale=1.8,
    force_full=False
):
    if frame is None:
        return None

    h, w = frame.shape[:2]

    if prev_state is not None and not force_full:
        x0, y0, x1, y1 = make_search_rect_from_prev(prev_state, frame.shape)
        crop = frame[y0:y1, x0:x1]
        det = detect_on_crop(
            crop_bgr=crop,
            offset_x=x0,
            offset_y=y0,
            dictionary=dictionary,
            params=params,
            clahe=clahe,
            subpix_criteria=subpix_criteria,
            marker_id=marker_id,
            scale=local_scale,
            frame_idx=frame_idx,
            search_name="local"
        )
        if det is not None:
            det["search_rect"] = (x0, y0, x1, y1)
            return det

    det = detect_on_crop(
        crop_bgr=frame,
        offset_x=0,
        offset_y=0,
        dictionary=dictionary,
        params=params,
        clahe=clahe,
        subpix_criteria=subpix_criteria,
        marker_id=marker_id,
        scale=full_scale,
        frame_idx=frame_idx,
        search_name="full"
    )
    if det is not None:
        det["search_rect"] = (0, 0, w, h)
        return det

    return None


def save_debug_frame(frame_bgr, pts, u, v, frame_idx, out_dir, x=None, y=None, yaw=None, search_rect=None):
    debug_dir = os.path.join(out_dir, "debug_frames")
    ensure_dir(debug_dir)

    vis = frame_bgr.copy()

    if search_rect is not None:
        x0, y0, x1, y1 = search_rect
        if not (x0 == 0 and y0 == 0 and x1 == frame_bgr.shape[1] and y1 == frame_bgr.shape[0]):
            cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 255), 2)

    cv2.polylines(vis, [pts.astype(np.int32)], True, (0, 255, 0), 2)
    cv2.circle(vis, (int(round(u)), int(round(v))), 6, (0, 0, 255), -1)

    p0 = pts[0].astype(int)
    p1 = pts[1].astype(int)
    cv2.arrowedLine(vis, tuple(p0), tuple(p1), (255, 0, 0), 2, tipLength=0.2)

    text_lines = [f"frame={frame_idx}", f"u={u:.1f}, v={v:.1f}"]
    if x is not None and y is not None:
        text_lines.append(f"x={x:.3f}, y={y:.3f}")
    if yaw is not None:
        text_lines.append(f"yaw={yaw:.3f} rad")

    y0 = 30
    for line in text_lines:
        cv2.putText(
            vis,
            line,
            (20, y0),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )
        y0 += 30

    out_path = os.path.join(debug_dir, f"frame_{frame_idx:06d}.png")
    cv2.imwrite(out_path, vis)


# =========================================================
# GŁÓWNY PIPELINE
# =========================================================

def process_video(
    video_path,
    out_dir,
    H,
    aruco_dict_name,
    marker_id,
    preview=False,
    min_detections=5,
    full_scale=1.0,
    local_scale=1.8,
    save_debug_frames=False,
    debug_every_n=1,
    max_debug_frames=50,
    max_jump_m=0.15,
    frame_step=1,
    progress_every=50,
    relocalize_every=20
):
    dbg("process_video: start")
    dbg(f"video_path={video_path}")
    dbg(f"dict={aruco_dict_name}, marker_id={marker_id}")

    dictionary, params, clahe, subpix_criteria = build_detector(aruco_dict_name)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Nie mogę otworzyć wideo: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0:
        fps = 30.0

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    dbg(f"fps={fps}")
    dbg(f"total_frames={total_frames}")

    raw_rows_abs = []
    debug_frames = []
    detections = 0
    saved_debug_count = 0
    frame_idx = -1
    start_time = time.time()
    track_state = None

    if frame_step < 1:
        frame_step = 1
    if progress_every < 1:
        progress_every = 1
    if relocalize_every < 1:
        relocalize_every = 999999999

    first_frame_saved = False

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            break

        frame_idx += 1

        if not first_frame_saved:
            cv2.imwrite(os.path.join(out_dir, "first_frame.png"), frame)
            first_frame_saved = True

        if total_frames > 0 and frame_idx % progress_every == 0:
            print_progress(frame_idx, total_frames, detections, start_time)

        if frame_idx % frame_step != 0:
            continue

        t_ms = cap.get(cv2.CAP_PROP_POS_MSEC)
        if t_ms is not None and t_ms > 0:
            t = t_ms / 1000.0
        else:
            t = frame_idx / fps

        force_full = (frame_idx % relocalize_every == 0)

        det = detect_marker_pose_2d(
            frame=frame,
            dictionary=dictionary,
            params=params,
            clahe=clahe,
            subpix_criteria=subpix_criteria,
            marker_id=marker_id,
            prev_state=track_state,
            frame_idx=frame_idx,
            full_scale=full_scale,
            local_scale=local_scale,
            force_full=force_full
        )

        if det is None:
            continue

        u = det["u"]
        v = det["v"]
        pts = det["corners"]

        x, y = image_to_world(H, u, v)

        p0 = pts[0]
        p1 = pts[1]
        wx0, wy0 = image_to_world(H, p0[0], p0[1])
        wx1, wy1 = image_to_world(H, p1[0], p1[1])

        yaw_world = wrap_angle(math.atan2(wy1 - wy0, wx1 - wx0))

        if len(raw_rows_abs) > 0:
            prev_x = raw_rows_abs[-1][1]
            prev_y = raw_rows_abs[-1][2]
            dist = math.hypot(x - prev_x, y - prev_y)

            if dist > max_jump_m:
                dbg(f"odrzucono skok w klatce {frame_idx}: dist={dist:.3f} m")
                track_state = None
                continue

        detections += 1
        raw_rows_abs.append([t, x, y, yaw_world])

        center = pts.mean(axis=0)
        edge01 = np.linalg.norm(pts[1] - pts[0])
        edge12 = np.linalg.norm(pts[2] - pts[1])
        size_px = max(edge01, edge12)

        track_state = {
            "cx": float(center[0]),
            "cy": float(center[1]),
            "size": float(size_px),
        }

        if preview and len(debug_frames) < 3:
            frame_vis = frame.copy()
            cv2.polylines(frame_vis, [pts.astype(np.int32)], True, (0, 255, 0), 2)
            cv2.circle(frame_vis, (int(round(u)), int(round(v))), 5, (0, 0, 255), -1)
            debug_frames.append(cv2.cvtColor(frame_vis, cv2.COLOR_BGR2RGB))

        if save_debug_frames and saved_debug_count < max_debug_frames:
            if debug_every_n <= 1 or (detections % debug_every_n == 0):
                save_debug_frame(
                    frame_bgr=frame,
                    pts=pts,
                    u=u,
                    v=v,
                    frame_idx=frame_idx,
                    out_dir=out_dir,
                    x=x,
                    y=y,
                    yaw=yaw_world,
                    search_rect=det.get("search_rect", None)
                )
                saved_debug_count += 1

    cap.release()

    if total_frames > 0:
        print_progress(total_frames, total_frames, detections, start_time)
        print()

    print(f"Liczba przetworzonych klatek: {frame_idx + 1}")
    print(f"Liczba wykryć markera: {detections}")
    print(f"Liczba zapisanych debug PNG: {saved_debug_count}")

    if len(raw_rows_abs) < min_detections:
        raise RuntimeError(
            f"Za mało wykryć markera w nagraniu. "
            f"Wykryto: {len(raw_rows_abs)}, wymagane minimum: {min_detections}."
        )

    raw_arr_abs = np.array(raw_rows_abs, dtype=float)
    raw_arr_abs = normalize_time(raw_arr_abs)

    raw_arr_start0 = normalize_pose_start(raw_arr_abs)

    raw_csv_abs = os.path.join(out_dir, "ground_truth_raw.csv")
    raw_csv_start0 = os.path.join(out_dir, "ground_truth_start0.csv")

    save_csv(raw_csv_abs, raw_arr_abs.tolist(), ["time_s", "x", "y", "yaw_rad"])
    save_csv(raw_csv_start0, raw_arr_start0.tolist(), ["time_s", "x", "y", "yaw_rad"])

    traj_png_abs = os.path.join(out_dir, "ground_truth_raw.png")
    traj_png_start0 = os.path.join(out_dir, "ground_truth_start0.png")

    save_plot(
        traj_png_abs,
        raw_arr_abs[:, 1:3],
        "Trajektoria odniesienia wyznaczona z nagrania (układ absolutny)"
    )

    save_plot(
        traj_png_start0,
        raw_arr_start0[:, 1:3],
        "Trajektoria odniesienia wyznaczona z nagrania (start = 0,0)"
    )

    if preview and len(debug_frames) > 0:
        plt.figure(figsize=(8, 6))
        plt.imshow(debug_frames[0])
        plt.axis("off")
        plt.tight_layout()
        frame_png = os.path.join(out_dir, "opencv_debug_frame.png")
        plt.savefig(frame_png, dpi=200, bbox_inches="tight")
        plt.close()

    return raw_arr_abs, raw_arr_start0


def align_to_slam(gt_arr, slam_arr, out_dir, slam_name="slam"):
    gt_arr = normalize_time(gt_arr)
    slam_arr = normalize_time(slam_arr)

    t_min = max(gt_arr[0, 0], slam_arr[0, 0])
    t_max = min(gt_arr[-1, 0], slam_arr[-1, 0])

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

    return aligned_arr


def main():
    dbg(f"OpenCV version: {cv2.__version__}")

    parser = argparse.ArgumentParser(
        description="Wideo z markerem ArUco -> trajektoria ground truth -> opcjonalne porównanie ze SLAM"
    )
    parser.add_argument("--video", required=True, help="Ścieżka do pliku wideo")
    parser.add_argument("--homography", required=True, help="JSON z punktami do homografii")
    parser.add_argument("--marker-id", type=int, required=True, help="ID markera ArUco")
    parser.add_argument("--dict", default="5x5_1000", choices=list(ARUCO_DICTS.keys()), help="Słownik ArUco")
    parser.add_argument("--out-dir", default="out_ground_truth", help="Folder wynikowy")
    parser.add_argument("--slam-csv", default="", help="Opcjonalnie trajektoria SLAM do porównania")
    parser.add_argument("--slam-name", default="slam", help="Nazwa trajektorii SLAM")
    parser.add_argument("--preview", action="store_true", help="Zapisz przykładową klatkę debug")
    parser.add_argument("--min-detections", type=int, default=5, help="Minimalna liczba wykryć markera")
    parser.add_argument("--save-debug-frames", action="store_true", help="Zapisuj klatki PNG z wykrytym markerem")
    parser.add_argument("--debug-every-n", type=int, default=1, help="Zapisuj co n-te poprawne wykrycie")
    parser.add_argument("--max-debug-frames", type=int, default=50, help="Maksymalna liczba debug PNG")
    parser.add_argument("--max-jump-m", type=float, default=0.15, help="Maksymalny dopuszczalny skok pozycji")
    parser.add_argument("--frame-step", type=int, default=1, help="Przetwarzaj co n-tą klatkę")
    parser.add_argument("--progress-every", type=int, default=50, help="Odświeżaj postęp co n klatek")
    parser.add_argument("--full-scale", type=float, default=1.0, help="Skala dla pełnej klatki")
    parser.add_argument("--local-scale", type=float, default=1.8, help="Skala dla lokalnego śledzenia")
    parser.add_argument("--relocalize-every", type=int, default=20, help="Co ile klatek wymuś pełne przeszukanie")

    args = parser.parse_args()

    ensure_dir(args.out_dir)

    H = load_homography_from_json(args.homography)

    gt_arr_abs, gt_arr_start0 = process_video(
        video_path=args.video,
        out_dir=args.out_dir,
        H=H,
        aruco_dict_name=args.dict,
        marker_id=args.marker_id,
        preview=args.preview,
        min_detections=args.min_detections,
        full_scale=args.full_scale,
        local_scale=args.local_scale,
        save_debug_frames=args.save_debug_frames,
        debug_every_n=args.debug_every_n,
        max_debug_frames=args.max_debug_frames,
        max_jump_m=args.max_jump_m,
        frame_step=args.frame_step,
        progress_every=args.progress_every,
        relocalize_every=args.relocalize_every
    )

    print(f"Zapisano: {os.path.join(args.out_dir, 'ground_truth_raw.csv')}")
    print(f"Zapisano: {os.path.join(args.out_dir, 'ground_truth_start0.csv')}")
    print(f"Zapisano: {os.path.join(args.out_dir, 'ground_truth_raw.png')}")
    print(f"Zapisano: {os.path.join(args.out_dir, 'ground_truth_start0.png')}")
    print(f"Zapisano: {os.path.join(args.out_dir, 'first_frame.png')}")

    if args.slam_csv:
        slam_arr = load_slam_csv(args.slam_csv)
        if len(slam_arr) < 5:
            raise RuntimeError("Za mało danych w pliku SLAM CSV.")
        align_to_slam(gt_arr_abs, slam_arr, args.out_dir, slam_name=args.slam_name)


if __name__ == "__main__":
    main()
