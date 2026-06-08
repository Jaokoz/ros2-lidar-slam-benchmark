import numpy as np
import matplotlib.pyplot as plt


def _downsample_for_plot(x, y, max_points=2500):
    """
    Przerzedza punkty tylko do wizualizacji.
    Dzięki temu Ground Truth będzie wyglądał jak zbiór punktów,
    a nie jak prawie ciągła linia.
    """
    x = np.asarray(x)
    y = np.asarray(y)

    n = len(x)
    if n <= max_points:
        return x, y

    step = int(np.ceil(n / max_points))
    return x[::step], y[::step]


def plot_simulation(gt_x, gt_y, slam_x, slam_y, title, output_path):
    plt.figure(figsize=(8, 6))

    gt_x_p, gt_y_p = _downsample_for_plot(gt_x, gt_y, max_points=2500)
    slam_x_p, slam_y_p = _downsample_for_plot(slam_x, slam_y, max_points=2500)

    plt.scatter(gt_x_p, gt_y_p, s=10, marker="o", label="Ground Truth")
    plt.scatter(slam_x_p, slam_y_p, s=10, marker="o", label="SLAM")

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()


def plot_real(gt_x, gt_y, slam_x, slam_y, odom_x, odom_y, title, output_path):
    plt.figure(figsize=(8, 6))

    gt_x_p, gt_y_p = _downsample_for_plot(gt_x, gt_y, max_points=2500)
    slam_x_p, slam_y_p = _downsample_for_plot(slam_x, slam_y, max_points=2500)
    odom_x_p, odom_y_p = _downsample_for_plot(odom_x, odom_y, max_points=2500)

    plt.scatter(gt_x_p, gt_y_p, s=10, marker="o", label="Ground Truth")
    plt.scatter(slam_x_p, slam_y_p, s=10, marker="o", label="SLAM")
    plt.scatter(odom_x_p, odom_y_p, s=10, marker="o", label="Odometry")

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
