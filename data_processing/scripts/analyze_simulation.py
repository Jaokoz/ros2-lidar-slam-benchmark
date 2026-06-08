import os

from utils_csv import load_trajectory_csv, densify_two_trajectories
from utils_metrics import build_metrics, timing_metrics
from utils_plot import plot_simulation


def save_timing_block(f, title, timing):
    f.write(f"{title}\n")
    f.write(f"Czas trwania [s]: {timing['duration_s']:.6f}\n")
    f.write(f"Liczba próbek: {timing['num_samples']}\n")
    f.write(f"Średnie dt [s]: {timing['mean_dt_s']:.6f}\n")
    f.write(f"Odchylenie dt [s]: {timing['std_dt_s']:.6f}\n")
    f.write(f"Minimalne dt [s]: {timing['min_dt_s']:.6f}\n")
    f.write(f"Maksymalne dt [s]: {timing['max_dt_s']:.6f}\n")
    f.write(f"Średnia częstotliwość [Hz]: {timing['mean_hz']:.3f}\n\n")


def save_metrics_txt(filepath, method, trial_name, metrics, slam_timing, gt_timing):
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("SCENARIUSZ: symulacja\n")
        f.write(f"METODA: {method}\n")
        f.write(f"PROBA: {trial_name}\n\n")

        f.write("POROWNANIE: SLAM vs Ground Truth\n")
        f.write(f"Liczba punktow: {metrics['num_points']}\n")
        f.write(f"RMSE [m]: {metrics['rmse']:.6f}\n")
        f.write(f"MAE [m]: {metrics['mae']:.6f}\n")
        f.write(f"Max error [m]: {metrics['max_error']:.6f}\n")
        f.write(f"Final error [m]: {metrics['final_error']:.6f}\n")
        f.write(f"Dlugosc trajektorii Ground Truth [m]: {metrics['ref_length']:.6f}\n")
        f.write(f"Dlugosc trajektorii SLAM [m]: {metrics['test_length']:.6f}\n\n")

        f.write("METRYKI CZASOWE\n")
        f.write("Uwaga: metryki czasowe oznaczają częstotliwość publikacji danych w CSV,\n")
        f.write("a nie bezpośredni czas obliczeń CPU algorytmu.\n\n")

        save_timing_block(f, "CZAS AKTUALIZACJI SLAM", slam_timing)
        save_timing_block(f, "CZAS PRÓBKOWANIA GROUND TRUTH", gt_timing)


def analyze_simulation_folder(input_folder, output_folder, method, trial_name):
    gt_path = os.path.join(input_folder, "ground_truth.csv")
    slam_path = os.path.join(input_folder, "slam.csv")

    gt_t, gt_x, gt_y = load_trajectory_csv(gt_path)
    slam_t, slam_x, slam_y = load_trajectory_csv(slam_path)

    _, gt_x_i, gt_y_i, slam_x_i, slam_y_i = densify_two_trajectories(
        gt_t, gt_x, gt_y,
        slam_t, slam_x, slam_y,
        dt=0.01
    )

    metrics = build_metrics(gt_x_i, gt_y_i, slam_x_i, slam_y_i)

    slam_timing = timing_metrics(slam_t)
    gt_timing = timing_metrics(gt_t)

    os.makedirs(output_folder, exist_ok=True)

    txt_path = os.path.join(output_folder, "metryki.txt")

    save_metrics_txt(
        txt_path,
        method,
        trial_name,
        metrics,
        slam_timing,
        gt_timing
    )

    plot_path = os.path.join(output_folder, "trajektoria.png")
    title = f"Porownanie trajektorii - symulacja - {method} - {trial_name}"

    plot_simulation(
        gt_x,
        gt_y,
        slam_x,
        slam_y,
        title,
        plot_path
    )
