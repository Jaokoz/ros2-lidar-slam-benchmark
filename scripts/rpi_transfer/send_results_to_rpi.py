import sys
import os
import mimetypes
import requests


def iter_files_recursive(root_dir, allowed_ext):
    result = []
    for current_root, _, files in os.walk(root_dir):
        for name in sorted(files):
            path = os.path.join(current_root, name)
            ext = os.path.splitext(name)[1].lower()
            if ext not in allowed_ext:
                continue

            rel_path = os.path.relpath(path, root_dir)
            result.append((path, rel_path))
    return result


def main():
    if len(sys.argv) < 4:
        print("Użycie:")
        print("python3 send_results_to_rpi.py slam_toolbox /sciezka/do/folderu_wynikow 192.168.0.57")
        print("python3 send_results_to_rpi.py cartographer /sciezka/do/folderu_wynikow 192.168.0.57")
        print("python3 send_results_to_rpi.py ground_truth /sciezka/do/folderu_wynikow 192.168.0.57")
        print("python3 send_results_to_rpi.py comparison_cartographer /sciezka/do/folderu_wynikow 192.168.0.57")
        print("python3 send_results_to_rpi.py comparison_slam_toolbox /sciezka/do/folderu_wynikow 192.168.0.57")
        sys.exit(1)

    method = sys.argv[1].strip()
    results_dir = sys.argv[2].strip()
    rpi_ip = sys.argv[3].strip()

    if method not in [
        "slam_toolbox",
        "cartographer",
        "ground_truth",
        "comparison_cartographer",
        "comparison_slam_toolbox"
    ]:
        print("Nieznana metoda")
        sys.exit(1)

    if not os.path.isdir(results_dir):
        print(f"Nie znaleziono folderu: {results_dir}")
        sys.exit(1)

    bag_name = os.path.basename(os.path.normpath(results_dir))
    url = f"http://{rpi_ip}:5000/upload_results"

    allowed_ext = {".csv", ".png", ".pgm", ".yaml", ".json", ".mp4"}
    file_entries = iter_files_recursive(results_dir, allowed_ext)

    if not file_entries:
        print("Brak plików do wysłania w folderze wyników")
        sys.exit(1)

    files = []
    handles = []

    try:
        for abs_path, rel_path in file_entries:
            mime_type = mimetypes.guess_type(abs_path)[0] or "application/octet-stream"
            f = open(abs_path, "rb")
            handles.append(f)
            files.append(("files", (rel_path, f, mime_type)))

        data = {
            "method": method,
            "bag_name": bag_name
        }

        r = requests.post(url, data=data, files=files, timeout=180)

        print("Status:", r.status_code)
        print("Odpowiedź serwera:")
        print(r.text)

    finally:
        for f in handles:
            f.close()


if __name__ == "__main__":
    main()
