import csv
import numpy as np


def load_trajectory_csv(filepath):
    times = []
    xs = []
    ys = []

    with open(filepath, "r", newline="") as f:
        reader = csv.DictReader(f)

        fieldnames = reader.fieldnames
        if fieldnames is None:
            raise ValueError(f"Plik {filepath} nie ma naglowka.")

        if "time_s" in fieldnames:
            time_col = "time_s"
        elif "time" in fieldnames:
            time_col = "time"
        else:
            raise ValueError(f"Plik {filepath} nie ma kolumny time_s ani time.")

        if "x" not in fieldnames or "y" not in fieldnames:
            raise ValueError(f"Plik {filepath} nie ma wymaganych kolumn x,y.")

        for row in reader:
            try:
                t = float(row[time_col])
                x = float(row["x"])
                y = float(row["y"])
            except (ValueError, TypeError, KeyError):
                continue

            times.append(t)
            xs.append(x)
            ys.append(y)

    if len(times) == 0:
        raise ValueError(f"Plik {filepath} nie zawiera poprawnych danych.")

    times = np.asarray(times, dtype=float)
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)

    idx = np.argsort(times)
    times = times[idx]
    xs = xs[idx]
    ys = ys[idx]

    # czas od startu
    times = times - times[0]

    return times, xs, ys


def interpolate_to_reference_time(ref_t, src_t, src_x, src_y):
    """
    Zachowana dla zgodności wstecznej.
    Interpoluje trajektorię źródłową do czasów trajektorii referencyjnej
    tylko w części wspólnej.
    """
    common_start = max(ref_t[0], src_t[0])
    common_end = min(ref_t[-1], src_t[-1])

    mask = (ref_t >= common_start) & (ref_t <= common_end)
    ref_t_common = ref_t[mask]

    if len(ref_t_common) == 0:
        raise ValueError("Brak wspolnego zakresu czasu do interpolacji.")

    interp_x = np.interp(ref_t_common, src_t, src_x)
    interp_y = np.interp(ref_t_common, src_t, src_y)

    return ref_t_common, interp_x, interp_y, mask


def make_dense_common_time(t1, t2, dt=0.01):
    """
    Tworzy gęstą wspólną siatkę czasu na części wspólnej dwóch trajektorii.
    dt=0.01 oznacza 10 ms.
    """
    common_start = max(t1[0], t2[0])
    common_end = min(t1[-1], t2[-1])

    if common_end <= common_start:
        raise ValueError("Brak wspolnego zakresu czasu do interpolacji.")

    n = int(np.floor((common_end - common_start) / dt)) + 1
    return common_start + np.arange(n, dtype=float) * dt


def interpolate_trajectory(t_src, x_src, y_src, t_dst):
    x_i = np.interp(t_dst, t_src, x_src)
    y_i = np.interp(t_dst, t_src, y_src)
    return x_i, y_i


def densify_two_trajectories(t1, x1, y1, t2, x2, y2, dt=0.01):
    """
    Zagęszcza obie trajektorie przez interpolację do wspólnej gęstej osi czasu.
    To jest właściwa baza do liczenia metryk.
    """
    t_dense = make_dense_common_time(t1, t2, dt=dt)

    x1_i, y1_i = interpolate_trajectory(t1, x1, y1, t_dense)
    x2_i, y2_i = interpolate_trajectory(t2, x2, y2, t_dense)

    return t_dense, x1_i, y1_i, x2_i, y2_i
