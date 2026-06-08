import numpy as np


def point_errors(x_ref, y_ref, x_test, y_test):
    x_ref = np.asarray(x_ref, dtype=float)
    y_ref = np.asarray(y_ref, dtype=float)
    x_test = np.asarray(x_test, dtype=float)
    y_test = np.asarray(y_test, dtype=float)

    return np.sqrt((x_ref - x_test) ** 2 + (y_ref - y_test) ** 2)


def rmse(errors):
    return np.sqrt(np.mean(errors ** 2))


def mae(errors):
    return np.mean(errors)


def max_error(errors):
    return np.max(errors)


def final_error(errors):
    return errors[-1]


def trajectory_length(x, y):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)

    dx = np.diff(x)
    dy = np.diff(y)

    return np.sum(np.sqrt(dx ** 2 + dy ** 2))


def build_metrics(x_ref, y_ref, x_test, y_test):
    errors = point_errors(x_ref, y_ref, x_test, y_test)

    return {
        "num_points": int(len(errors)),
        "rmse": float(rmse(errors)),
        "mae": float(mae(errors)),
        "max_error": float(max_error(errors)),
        "final_error": float(final_error(errors)),
        "ref_length": float(trajectory_length(x_ref, y_ref)),
        "test_length": float(trajectory_length(x_test, y_test)),
    }


def compute_metrics(x_ref, y_ref, x_test, y_test):
    """
    Alias dla zgodności wstecznej.
    """
    return build_metrics(x_ref, y_ref, x_test, y_test)


def timing_metrics(t):
    """
    Liczy metryki czasowe trajektorii.

    To nie jest bezpośredni czas obliczeń CPU algorytmu,
    tylko praktyczna częstotliwość publikacji estymaty trajektorii.
    """
    t = np.asarray(t, dtype=float)

    if len(t) < 2:
        return {
            "duration_s": 0.0,
            "num_samples": int(len(t)),
            "mean_dt_s": 0.0,
            "std_dt_s": 0.0,
            "min_dt_s": 0.0,
            "max_dt_s": 0.0,
            "mean_hz": 0.0,
        }

    dt = np.diff(t)

    positive_dt = dt[dt > 0.0]

    if len(positive_dt) == 0:
        mean_dt = 0.0
        mean_hz = 0.0
    else:
        mean_dt = float(np.mean(positive_dt))
        mean_hz = float(1.0 / mean_dt)

    return {
        "duration_s": float(t[-1] - t[0]),
        "num_samples": int(len(t)),
        "mean_dt_s": mean_dt,
        "std_dt_s": float(np.std(positive_dt)) if len(positive_dt) > 0 else 0.0,
        "min_dt_s": float(np.min(positive_dt)) if len(positive_dt) > 0 else 0.0,
        "max_dt_s": float(np.max(positive_dt)) if len(positive_dt) > 0 else 0.0,
        "mean_hz": mean_hz,
    }
