import os
import csv
import numpy as np

from utils_csv import load_trajectory_csv, densify_two_trajectories
from utils_metrics import build_metrics, timing_metrics
from utils_plot import plot_real


def normalize_xy_start(x, y):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)

    if len(x) == 0:
        return x, y

    return x - x[0], y - y[0]


def find_combined_csv(input_folder):
    candidates = [
        "trajectories.csv",
        "comparison.csv",
        "compare_ground_truth_vs_slam_toolbox.csv",
        "compare_ground_truth_vs_cartographer.csv",
    ]

    for name in candidates:
        path = os.path.join(input_folder, name)
        if os.path.isfile(path):
            return path

    for name in os.listdir(input_folder):
        if not name.endswith(".csv"):
            continue

        path = os.path.join(input_folder, name)

        with open(path, "r", newline="") as f:
            reader = csv.reader(f)
            header = next(reader, [])

        required = {
            "time_s",
            "gt_x", "gt_y",
            "slam_x", "slam_y",
            "odom_x", "odom_y",
        }

        if required.issubset(set(header)):
            return path

    return None


def load_combined_real_csv(path):
    t = []
    gt_x = []
    gt_y = []
    slam_x = []
    slam_y = []
    odom_x = []
    odom_y = []

    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)

        required = [
            "time_s",
            "gt_x", "gt_y",
            "slam_x", "slam_y",
            "odom_x", "odom_y",
        ]

        for col in required:
            if col not in reader.fieldnames:
                raise ValueError(f"Brak kolumny {col} w pliku {path}")

        for row in reader:
            try:
                t.append(float(row["time_s"]))
                gt_x.append(float(row["gt_x"]))
                gt_y.append(float(row["gt_y"]))
                slam_x.append(float(row["slam_x"]))
                slam_y.append(float(row["slam_y"]))
                odom_x.append(float(row["odom_x"]))
                odom_y.append(float(row["odom_y"]))
            except (ValueError, TypeError, KeyError):
                continue

    if len(t) == 0:
        raise ValueError(f"Plik {path} nie zawiera poprawnych danych.")

    t = np.asarray(t, dtype=float)
    gt_x = np.asarray(gt_x, dtype=float)
    gt_y = np.asarray(gt_y, dtype=float)
    slam_x = np.asarray(slam_x, dtype=float)
    slam_y = np.asarray(slam_y, dtype=float)
    odom_x = np.asarray(odom_x, dtype=float)
    odom_y = np.asarray(odom_y, dtype=float)

    idx = np.argsort(t)

    t = t[idx]
    gt_x = gt_x[idx]
    gt_y = gt_y[idx]
    slam_x = slam_x[idx]
    slam_y = slam_y[idx]
    odom_x = odom_x[idx]
    odom_y = odom_y[idx]

    t = t - t[0]

    return t, gt_x, gt_y, slam_x, slam_y, odom_x, odom_y


def load_real_input(input_folder):
    gt_path = os.path.join(input_folder, "ground_truth.csv")
    slam_path = os.path.join(input_folder, "slam.csv")
    odom_path = os.path.join(input_folder, "odom.csv")

    if (
        os.path.isfile(gt_path)
        and os.path.isfile(slam_path)
        and os.path.isfile(odom_path)
    ):
        gt_t, gt_x, gt_y = load_trajectory_csv(gt_path)
        slam_t, slam_x, slam_y = load_trajectory_csv(slam_path)
        odom_t, odom_x, odom_y = load_trajectory_csv(odom_path)

        return {
            "mode": "separate",
            "gt_t": gt_t,
            "gt_x": gt_x,
            "gt_y": gt_y,
            "slam_t": slam_t,
            "slam_x": slam_x,
            "slam_y": slam_y,
            "odom_t": odom_t,
            "odom_x": odom_x,
            "odom_y": odom_y,
        }

    combined_path = find_combined_csv(input_folder)

    if combined_path is None:
        raise FileNotFoundError(
            f"Nie znaleziono ani plików ground_truth.csv/slam.csv/odom.csv, "
            f"ani wspólnego CSV z kolumnami gt_x, slam_x, odom_x w folderze: {input_folder}"
        )

    t, gt_x, gt_y, slam_x, slam_y, odom_x, odom_y = load_combined_real_csv(
        combined_path
    )

    return {
        "mode": "combined",
        "combined_path": combined_path,
        "gt_t": t,
        "gt_x": gt_x,
        "gt_y": gt_y,
        "slam_t": t,
        "slam_x": slam_x,
        "slam_y": slam_y,
        "odom_t": t,
        "odom_x": odom_x,
        "odom_y": odom_y,
    }


def save_timing_block(f, title, timing):
    f.write(f"{title}\n")
    f.write(f"Czas trwania [s]: {timing['duration_s']:.6f}\n")
    f.write(f"Liczba próbek: {timing['num_samples']}\n")
    f.write(f"Średnie dt [s]: {timing['mean_dt_s']:.6f}\n")
    f.write(f"Odchylenie dt [s]: {timing['std_dt_s']:.6f}\n")
    f.write(f"Minimalne dt [s]: {timing['min_dt_s']:.6f}\n")
    f.write(f"Maksymalne dt [s]: {timing['max_dt_s']:.6f}\n")
    f.write(f"Średnia częstotliwość [Hz]: {timing['mean_hz']:.3f}\n\n")


def save_metrics_txt(
    filepath,
    method,
    trial_name,
    input_mode,
    slam_metrics,
    odom_metrics,
    slam_timing,
    odom_timing,
    gt_timing,
    combined_path=None,
):
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("SCENARIUSZ: rzeczywiste\n")
        f.write(f"METODA: {method}\n")
        f.write(f"PROBA: {trial_name}\n")
        f.write(f"TRYB WEJŚCIA: {input_mode}\n")

        if combined_path:
            f.write(f"PLIK WEJŚCIOWY: {combined_path}\n")

        f.write("\n")

        f.write("POROWNANIE 1: SLAM vs Ground Truth\n")
        f.write(f"Liczba punktow: {slam_metrics['num_points']}\n")
        f.write(f"RMSE [m]: {slam_metrics['rmse']:.6f}\n")
        f.write(f"MAE [m]: {slam_metrics['mae']:.6f}\n")
        f.write(f"Max error [m]: {slam_metrics['max_error']:.6f}\n")
        f.write(f"Final error [m]: {slam_metrics['final_error']:.6f}\n")
        f.write(f"Dlugosc trajektorii Ground Truth [m]: {slam_metrics['ref_length']:.6f}\n")
        f.write(f"Dlugosc trajektorii SLAM [m]: {slam_metrics['test_length']:.6f}\n\n")

        f.write("POROWNANIE 2: ODOM vs Ground Truth\n")
        f.write(f"Liczba punktow: {odom_metrics['num_points']}\n")
        f.write(f"RMSE [m]: {odom_metrics['rmse']:.6f}\n")
        f.write(f"MAE [m]: {odom_metrics['mae']:.6f}\n")
        f.write(f"Max error [m]: {odom_metrics['max_error']:.6f}\n")
        f.write(f"Final error [m]: {odom_metrics['final_error']:.6f}\n")
        f.write(f"Dlugosc trajektorii Ground Truth [m]: {odom_metrics['ref_length']:.6f}\n")
        f.write(f"Dlugosc trajektorii ODOM [m]: {odom_metrics['test_length']:.6f}\n\n")

        f.write("METRYKI CZASOWE\n")
        f.write("Uwaga: metryki czasowe oznaczają częstotliwość publikacji danych w CSV,\n")
        f.write("a nie bezpośredni czas obliczeń CPU algorytmu.\n\n")

        save_timing_block(f, "CZAS AKTUALIZACJI SLAM", slam_timing)
        save_timing_block(f, "CZAS AKTUALIZACJI ODOM", odom_timing)
        save_timing_block(f, "CZAS PRÓBKOWANIA GROUND TRUTH", gt_timing)


def analyze_real_folder(input_folder, output_folder, method, trial_name):
    data = load_real_input(input_folder)

    gt_t = data["gt_t"]
    gt_x = data["gt_x"]
    gt_y = data["gt_y"]

    slam_t = data["slam_t"]
    slam_x = data["slam_x"]
    slam_y = data["slam_y"]

    odom_t = data["odom_t"]
    odom_x = data["odom_x"]
    odom_y = data["odom_y"]

    # Porównanie start = 0,0.
    # To usuwa globalny offset kamery/homografii względem układu SLAM.
    gt_x, gt_y = normalize_xy_start(gt_x, gt_y)
    slam_x, slam_y = normalize_xy_start(slam_x, slam_y)
    odom_x, odom_y = normalize_xy_start(odom_x, odom_y)

    _, gt_x_slam_i, gt_y_slam_i, slam_x_i, slam_y_i = densify_two_trajectories(
        gt_t, gt_x, gt_y,
        slam_t, slam_x, slam_y,
        dt=0.01
    )

    _, gt_x_odom_i, gt_y_odom_i, odom_x_i, odom_y_i = densify_two_trajectories(
        gt_t, gt_x, gt_y,
        odom_t, odom_x, odom_y,
        dt=0.01
    )

    slam_metrics = build_metrics(gt_x_slam_i, gt_y_slam_i, slam_x_i, slam_y_i)
    odom_metrics = build_metrics(gt_x_odom_i, gt_y_odom_i, odom_x_i, odom_y_i)

    slam_timing = timing_metrics(slam_t)
    odom_timing = timing_metrics(odom_t)
    gt_timing = timing_metrics(gt_t)

    os.makedirs(output_folder, exist_ok=True)

    txt_path = os.path.join(output_folder, "metryki.txt")
    save_metrics_txt(
        txt_path,
        method,
        trial_name,
        data["mode"],
        slam_metrics,
        odom_metrics,
        slam_timing,
        odom_timing,
        gt_timing,
        combined_path=data.get("combined_path"),
    )

    plot_path = os.path.join(output_folder, "trajektoria.png")
    title = f"Porownanie trajektorii - rzeczywiste - {method} - {trial_name}"

    plot_real(
        gt_x,
        gt_y,
        slam_x,
        slam_y,
        odom_x,
        odom_y,
        title,
        plot_path
    )
