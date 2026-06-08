#!/bin/bash

set -e

WS="$HOME/ros2_ws"
RESULT_BASE="$HOME/ros_scripts/wyniki/slam_toolbox"
RPI_IP="192.168.0.57"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

echo "=== Szukanie najnowszego rosbaga w $WS ==="

# Szuka najnowszego katalogu rosbaga po czasie modyfikacji, a nie po nazwie.
BAG_PATH=$(find "$WS" -maxdepth 1 -type d \( -name "rosbag2_*" -o -name "slam_s*" \) -printf "%T@ %p\n" | sort -n | tail -n 1 | cut -d' ' -f2-)

if [ -z "$BAG_PATH" ]; then
    echo "Nie znaleziono żadnego rosbaga w $WS"
    exit 1
fi

BAG_NAME=$(basename "$BAG_PATH")
RESULT_DIR="$RESULT_BASE/$BAG_NAME"

echo "Najnowszy bag: $BAG_PATH"
echo "Folder wyników: $RESULT_DIR"

mkdir -p "$RESULT_DIR"

echo "=== Sprawdzenie zawartości rosbaga ==="
ros2 bag info "$BAG_PATH" || true

echo "=== Ekstrakcja trajektorii i wykresów ==="
python3 "$HOME/ros_scripts/extract_one_bag_slam_toolbox.py" "$BAG_PATH"

echo "=== Ekstrakcja OSTATNIEJ mapy /map z rosbaga ==="

python3 - "$BAG_PATH" "$RESULT_DIR" <<'PY'
import sys
import os
import math
import yaml
import numpy as np

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def detect_storage_id(bag_path):
    files = os.listdir(bag_path)
    if any(f.endswith(".mcap") for f in files):
        return "mcap"
    if any(f.endswith(".db3") for f in files):
        return "sqlite3"
    return "mcap"


def save_pgm(filename, grid, width, height):
    """
    Zapis OccupancyGrid do PGM zgodnego z map_server.
    Wartości:
    - 0   -> wolne
    - 100 -> zajęte
    - -1  -> nieznane
    """
    data = np.array(grid, dtype=np.int16).reshape((height, width))

    # Odwrócenie osi Y, bo PGM zapisuje obraz od góry, a mapa ROS od dołu.
    data = np.flipud(data)

    img = np.zeros((height, width), dtype=np.uint8)

    # Standard map_server:
    # wolne = jasne, zajęte = ciemne, nieznane = szare
    img[data == -1] = 205
    img[data >= 65] = 0
    img[(data >= 0) & (data < 65)] = 254

    with open(filename, "wb") as f:
        f.write(b"P5\n")
        f.write(f"{width} {height}\n".encode())
        f.write(b"255\n")
        f.write(img.tobytes())


def quat_to_yaw(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


bag_path = sys.argv[1]
result_dir = sys.argv[2]

storage_id = detect_storage_id(bag_path)

storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
converter_options = ConverterOptions("", "")

reader = SequentialReader()
reader.open(storage_options, converter_options)

topic_types = reader.get_all_topics_and_types()
type_map = {topic.name: topic.type for topic in topic_types}

if "/map" not in type_map:
    print("UWAGA: W rosbagu nie ma topicu /map. Nie można zapisać mapy.")
    sys.exit(0)

map_type = get_message(type_map["/map"])

last_map = None
last_stamp = None
map_count = 0

while reader.has_next():
    topic, data, t = reader.read_next()

    if topic == "/map":
        msg = deserialize_message(data, map_type)
        last_map = msg
        last_stamp = t
        map_count += 1

print(f"Liczba wiadomości /map w rosbagu: {map_count}")

if last_map is None:
    print("UWAGA: Nie znaleziono żadnej wiadomości /map.")
    sys.exit(0)

width = last_map.info.width
height = last_map.info.height
resolution = last_map.info.resolution

origin = last_map.info.origin
origin_x = origin.position.x
origin_y = origin.position.y
origin_yaw = quat_to_yaw(origin.orientation)

pgm_path = os.path.join(result_dir, "map.pgm")
yaml_path = os.path.join(result_dir, "map.yaml")

save_pgm(pgm_path, last_map.data, width, height)

map_yaml = {
    "image": "map.pgm",
    "mode": "trinary",
    "resolution": float(resolution),
    "origin": [float(origin_x), float(origin_y), float(origin_yaw)],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.25,
}

with open(yaml_path, "w") as f:
    yaml.dump(map_yaml, f, default_flow_style=False, sort_keys=False)

print(f"Zapisano ostatnią mapę z rosbaga:")
print(f" - {pgm_path}")
print(f" - {yaml_path}")
PY

echo "=== Konwersja mapy PGM -> PNG ==="

if [ -f "$RESULT_DIR/map.pgm" ]; then
    if command -v convert >/dev/null 2>&1; then
        convert "$RESULT_DIR/map.pgm" "$RESULT_DIR/map.png"
        echo "Zapisano: $RESULT_DIR/map.png"
    else
        echo "Brak programu convert. Pomijam konwersję map.pgm do map.png."
    fi
else
    echo "UWAGA: Nie znaleziono $RESULT_DIR/map.pgm"
    echo "Mapa nie została zapisana."
fi

echo "=== Sprawdzenie plików wynikowych ==="
ls -lah "$RESULT_DIR"

echo "=== Wysyłka na stronę ==="
python3 "$HOME/ros_scripts/send_results_to_rpi.py" \
slam_toolbox \
"$RESULT_DIR" \
"$RPI_IP"

echo "=== Gotowe ==="
echo "Bag: $BAG_PATH"
echo "Wyniki: $RESULT_DIR"
