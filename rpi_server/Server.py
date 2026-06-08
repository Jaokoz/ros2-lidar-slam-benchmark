from flask import Flask, request, jsonify, send_file, redirect, url_for
import json
import os
import shutil
from datetime import datetime

app = Flask(__name__)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_FILE = os.path.join(BASE_DIR, "trajectory.json")
UPLOAD_BASE = os.path.join(BASE_DIR, "uploads")

os.makedirs(UPLOAD_BASE, exist_ok=True)

robot_state = {
    "x": 0.0,
    "y": 0.0,
    "theta": 0.0,
    "status": "start",
    "v": 0.0,
    "omega": 0.0,
    "time": ""
}

trajectory = []


def now_str():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def is_valid_method(method):
    return method in ["slam_toolbox", "cartographer"]


def save_to_file():
    with open(DATA_FILE, "w", encoding="utf-8") as f:
        json.dump(trajectory, f, ensure_ascii=False, indent=2)


def load_from_file():
    global trajectory
    try:
        with open(DATA_FILE, "r", encoding="utf-8") as f:
            trajectory = json.load(f)
    except FileNotFoundError:
        trajectory = []
    except json.JSONDecodeError:
        trajectory = []


def get_latest_result_dir(method):
    method_dir = os.path.join(UPLOAD_BASE, method)

    if not os.path.isdir(method_dir):
        return None, None

    bag_dirs = []
    for d in os.listdir(method_dir):
        full = os.path.join(method_dir, d)
        if os.path.isdir(full):
            bag_dirs.append(full)

    if not bag_dirs:
        return None, None

    bag_dirs.sort(key=os.path.getmtime, reverse=True)
    latest_dir = bag_dirs[0]
    latest_bag = os.path.basename(latest_dir)

    return latest_bag, latest_dir


def find_first_existing_file(folder, candidates):
    for name in candidates:
        path = os.path.join(folder, name)
        if os.path.isfile(path):
            return name
    return None


def reset_robot_state(message="historia wyczyszczona"):
    robot_state["x"] = 0.0
    robot_state["y"] = 0.0
    robot_state["theta"] = 0.0
    robot_state["status"] = message
    robot_state["v"] = 0.0
    robot_state["omega"] = 0.0
    robot_state["time"] = now_str()


def build_results_html(method, bag_name, bag_dir, ts):
    if not bag_name or not bag_dir:
        return f"<p>Brak wyników dla {method}</p>"

    if method == "slam_toolbox":
        plot_file = find_first_existing_file(
            bag_dir,
            ["trajektorie.png"]
        )
    else:
        plot_file = find_first_existing_file(
            bag_dir,
            ["trajektorie_wyrownane.png", "trajektorie_surowe.png"]
        )

    map_file = find_first_existing_file(
        bag_dir,
        ["map.png", "map.pgm"]
    )

    html = f"""
    <div class="result-block">
        <div class="result-header">
            <h3>{method}: {bag_name}</h3>
            <div class="result-actions">
                <form action="/delete_latest_results/{method}" method="post" style="display:inline;">
                    <button class="btn danger" type="submit">Usuń najnowszy komplet</button>
                </form>
                <form action="/delete_all_results/{method}" method="post" style="display:inline;">
                    <button class="btn danger-dark" type="submit">Usuń wszystkie {method}</button>
                </form>
            </div>
        </div>
    """

    if plot_file or map_file:
        html += '<div class="image-row">'

        # wykres
        html += '<div class="image-card">'
        html += '<h4>Wykres</h4>'
        if plot_file:
            html += f'''
            <img src="/uploaded/{method}/{bag_name}/{plot_file}?t={ts}" class="result-image">
            <div class="file-actions">
                <a href="/uploaded/{method}/{bag_name}/{plot_file}" target="_blank">Otwórz</a>
                <form action="/delete_file/{method}/{bag_name}/{plot_file}" method="post" style="display:inline;">
                    <button class="btn small" type="submit">Usuń plik</button>
                </form>
            </div>
            '''
        else:
            html += "<p>Brak wykresu.</p>"
        html += '</div>'

        # mapa
        html += '<div class="image-card">'
        html += '<h4>Mapa</h4>'
        if map_file:
            html += f'''
            <img src="/uploaded/{method}/{bag_name}/{map_file}?t={ts}" class="result-image">
            <div class="file-actions">
                <a href="/uploaded/{method}/{bag_name}/{map_file}" target="_blank">Otwórz</a>
                <form action="/delete_file/{method}/{bag_name}/{map_file}" method="post" style="display:inline;">
                    <button class="btn small" type="submit">Usuń plik</button>
                </form>
            </div>
            '''
        else:
            html += "<p>Brak mapy.</p>"
        html += '</div>'

        html += '</div>'

    html += '<div class="file-list"><h4>Wszystkie pliki</h4>'

    file_names = sorted(
        [name for name in os.listdir(bag_dir) if os.path.isfile(os.path.join(bag_dir, name))]
    )

    if not file_names:
        html += "<p>Brak plików.</p>"
    else:
        for name in file_names:
            html += f'''
            <div class="file-row">
                <span>{name}</span>
                <div class="file-actions">
                    <a href="/uploaded/{method}/{bag_name}/{name}" target="_blank">Otwórz</a>
                    <form action="/delete_file/{method}/{bag_name}/{name}" method="post" style="display:inline;">
                        <button class="btn small" type="submit">Usuń</button>
                    </form>
                </div>
            </div>
            '''

    html += '</div></div>'
    return html


@app.route("/")
def home():
    points_js = json.dumps(trajectory)
    ts = int(datetime.now().timestamp())

    slam_bag, slam_dir = get_latest_result_dir("slam_toolbox")
    carto_bag, carto_dir = get_latest_result_dir("cartographer")

    slam_results_html = build_results_html("slam_toolbox", slam_bag, slam_dir, ts)
    carto_results_html = build_results_html("cartographer", carto_bag, carto_dir, ts)

    html = f"""
    <html>
    <head>
        <title>Robot Panel</title>
        <meta http-equiv="refresh" content="3">
        <style>
            body {{
                font-family: Arial, sans-serif;
                margin: 0;
                padding: 20px;
                background: #f5f5f5;
            }}
            .container {{
                display: flex;
                gap: 20px;
                align-items: flex-start;
            }}
            .left, .right {{
                background: white;
                padding: 20px;
                border-radius: 12px;
                box-shadow: 0 2px 8px rgba(0,0,0,0.08);
            }}
            .left {{
                width: 360px;
                position: sticky;
                top: 20px;
            }}
            .right {{
                flex: 1;
            }}
            h1, h2, h3, h4 {{
                margin-top: 0;
            }}
            .value {{
                margin: 8px 0;
                font-size: 16px;
            }}
            .btn {{
                padding: 10px 16px;
                font-size: 14px;
                border: none;
                border-radius: 8px;
                cursor: pointer;
                background: #0275d8;
                color: white;
            }}
            .btn:hover {{
                opacity: 0.92;
            }}
            .btn.danger {{
                background: #d9534f;
            }}
            .btn.danger-dark {{
                background: #a94442;
            }}
            .btn.small {{
                padding: 6px 10px;
                font-size: 12px;
                background: #d9534f;
            }}
            canvas {{
                border: 1px solid #ccc;
                background: #fff;
                max-width: 100%;
            }}
            a {{
                text-decoration: none;
            }}
            .section {{
                margin-top: 30px;
                padding-top: 20px;
                border-top: 1px solid #ddd;
            }}
            .result-block {{
                margin-top: 20px;
                padding: 20px;
                border: 1px solid #ddd;
                border-radius: 12px;
                background: #fafafa;
            }}
            .result-header {{
                display: flex;
                justify-content: space-between;
                gap: 20px;
                align-items: flex-start;
                flex-wrap: wrap;
            }}
            .result-actions {{
                display: flex;
                gap: 10px;
                flex-wrap: wrap;
            }}
            .image-row {{
                display: grid;
                grid-template-columns: 1fr 1fr;
                gap: 20px;
                margin-top: 20px;
            }}
            .image-card {{
                background: white;
                border: 1px solid #ddd;
                border-radius: 10px;
                padding: 12px;
            }}
            .result-image {{
                width: 100%;
                border: 1px solid #ccc;
                border-radius: 8px;
                display: block;
            }}
            .file-list {{
                margin-top: 20px;
                background: white;
                border: 1px solid #ddd;
                border-radius: 10px;
                padding: 12px;
            }}
            .file-row {{
                display: flex;
                justify-content: space-between;
                gap: 20px;
                align-items: center;
                padding: 8px 0;
                border-bottom: 1px solid #eee;
                flex-wrap: wrap;
            }}
            .file-row:last-child {{
                border-bottom: none;
            }}
            .file-actions {{
                display: flex;
                gap: 10px;
                align-items: center;
                flex-wrap: wrap;
                margin-top: 8px;
            }}
            .control-box {{
                margin-top: 20px;
                padding: 15px;
                border: 1px solid #ddd;
                border-radius: 10px;
                background: #fafafa;
            }}
            @media (max-width: 1100px) {{
                .container {{
                    flex-direction: column;
                }}
                .left {{
                    width: auto;
                    position: static;
                }}
                .image-row {{
                    grid-template-columns: 1fr;
                }}
            }}
        </style>
    </head>
    <body>
        <h1>Panel robota</h1>

        <div class="container">
            <div class="left">
                <h2>Status</h2>
                <div class="value"><b>Status:</b> {robot_state["status"]}</div>
                <div class="value"><b>Czas:</b> {robot_state["time"]}</div>
                <div class="value"><b>X:</b> {robot_state["x"]}</div>
                <div class="value"><b>Y:</b> {robot_state["y"]}</div>
                <div class="value"><b>Theta:</b> {robot_state["theta"]}</div>
                <div class="value"><b>v:</b> {robot_state["v"]}</div>
                <div class="value"><b>omega:</b> {robot_state["omega"]}</div>
                <div class="value"><b>Liczba punktów trajektorii:</b> {len(trajectory)}</div>

                <div class="control-box">
                    <h3>Trajektoria online</h3>
                    <form action="/clear_trajectory" method="post">
                        <button class="btn danger" type="submit">Usuń wszystkie punkty trajektorii</button>
                    </form>
                </div>

                <div class="control-box">
                    <h3>Podgląd danych</h3>
                    <div><a href="/state" target="_blank">/state</a></div>
                    <div><a href="/trajectory" target="_blank">/trajectory</a></div>
                </div>
            </div>

            <div class="right">
                <h2>Trajektoria online 2D</h2>
                <canvas id="trajCanvas" width="900" height="650"></canvas>

                <div class="section">
                    <h2>Wyniki przesłane z komputera</h2>

                    <div class="section">
                        {slam_results_html}
                    </div>

                    <div class="section">
                        {carto_results_html}
                    </div>
                </div>
            </div>
        </div>

        <script>
            const points = {points_js};
            const canvas = document.getElementById("trajCanvas");
            const ctx = canvas.getContext("2d");

            function drawTrajectory(points) {{
                ctx.clearRect(0, 0, canvas.width, canvas.height);

                if (!points || points.length === 0) {{
                    ctx.fillStyle = "#333";
                    ctx.font = "18px Arial";
                    ctx.fillText("Brak punktów trajektorii", 30, 40);
                    return;
                }}

                let minX = points[0].x;
                let maxX = points[0].x;
                let minY = points[0].y;
                let maxY = points[0].y;

                for (const p of points) {{
                    if (p.x < minX) minX = p.x;
                    if (p.x > maxX) maxX = p.x;
                    if (p.y < minY) minY = p.y;
                    if (p.y > maxY) maxY = p.y;
                }}

                const pad = 60;

                let rangeX = maxX - minX;
                let rangeY = maxY - minY;

                if (rangeX < 1.0) rangeX = 1.0;
                if (rangeY < 1.0) rangeY = 1.0;

                minX -= 0.5;
                maxX += 0.5;
                minY -= 0.5;
                maxY += 0.5;

                rangeX = maxX - minX;
                rangeY = maxY - minY;

                const scaleX = (canvas.width - 2 * pad) / rangeX;
                const scaleY = (canvas.height - 2 * pad) / rangeY;
                const scale = Math.min(scaleX, scaleY);

                function mapX(x) {{
                    return pad + (x - minX) * scale;
                }}

                function mapY(y) {{
                    return canvas.height - (pad + (y - minY) * scale);
                }}

                function drawAxis(minX, maxX, minY, maxY) {{
                    ctx.strokeStyle = "#dddddd";
                    ctx.lineWidth = 1;
                    ctx.font = "12px Arial";
                    ctx.fillStyle = "#444";

                    const step = 1;

                    for (let x = Math.floor(minX); x <= Math.ceil(maxX); x += step) {{
                        const px = mapX(x);

                        ctx.beginPath();
                        ctx.moveTo(px, 0);
                        ctx.lineTo(px, canvas.height);
                        ctx.stroke();

                        ctx.fillText(x.toFixed(1), px + 2, canvas.height - 8);
                    }}

                    for (let y = Math.floor(minY); y <= Math.ceil(maxY); y += step) {{
                        const py = mapY(y);

                        ctx.beginPath();
                        ctx.moveTo(0, py);
                        ctx.lineTo(canvas.width, py);
                        ctx.stroke();

                        ctx.fillText(y.toFixed(1), 5, py - 4);
                    }}

                    ctx.strokeStyle = "#888";
                    ctx.lineWidth = 2;

                    if (minX <= 0 && maxX >= 0) {{
                        ctx.beginPath();
                        ctx.moveTo(mapX(0), 0);
                        ctx.lineTo(mapX(0), canvas.height);
                        ctx.stroke();
                    }}

                    if (minY <= 0 && maxY >= 0) {{
                        ctx.beginPath();
                        ctx.moveTo(0, mapY(0));
                        ctx.lineTo(canvas.width, mapY(0));
                        ctx.stroke();
                    }}

                    ctx.fillStyle = "black";
                    ctx.font = "16px Arial";
                    ctx.fillText("X [m]", canvas.width - 80, canvas.height - 20);
                    ctx.fillText("Y [m]", 10, 20);
                }}

                drawAxis(minX, maxX, minY, maxY);

                ctx.strokeStyle = "#007bff";
                ctx.lineWidth = 2;
                ctx.beginPath();

                ctx.moveTo(mapX(points[0].x), mapY(points[0].y));

                for (let i = 1; i < points.length; i++) {{
                    ctx.lineTo(mapX(points[i].x), mapY(points[i].y));
                }}
                ctx.stroke();

                ctx.fillStyle = "#222";
                for (const p of points) {{
                    ctx.beginPath();
                    ctx.arc(mapX(p.x), mapY(p.y), 3, 0, 2 * Math.PI);
                    ctx.fill();
                }}

                ctx.fillStyle = "green";
                ctx.beginPath();
                ctx.arc(mapX(points[0].x), mapY(points[0].y), 6, 0, 2 * Math.PI);
                ctx.fill();

                const last = points[points.length - 1];
                ctx.fillStyle = "red";
                ctx.beginPath();
                ctx.arc(mapX(last.x), mapY(last.y), 6, 0, 2 * Math.PI);
                ctx.fill();

                const dirLength = 0.25;
                const dirX = last.x + Math.cos(last.theta) * dirLength;
                const dirY = last.y + Math.sin(last.theta) * dirLength;

                ctx.strokeStyle = "black";
                ctx.lineWidth = 3;
                ctx.beginPath();
                ctx.moveTo(mapX(last.x), mapY(last.y));
                ctx.lineTo(mapX(dirX), mapY(dirY));
                ctx.stroke();

                const arrowAngle = Math.atan2(dirY - last.y, dirX - last.x);
                const arrowSize = 10;

                const endPx = mapX(dirX);
                const endPy = mapY(dirY);

                ctx.beginPath();
                ctx.moveTo(endPx, endPy);
                ctx.lineTo(
                    endPx - arrowSize * Math.cos(arrowAngle - Math.PI / 6),
                    endPy + arrowSize * Math.sin(arrowAngle - Math.PI / 6)
                );
                ctx.lineTo(
                    endPx - arrowSize * Math.cos(arrowAngle + Math.PI / 6),
                    endPy + arrowSize * Math.sin(arrowAngle + Math.PI / 6)
                );
                ctx.closePath();
                ctx.fillStyle = "black";
                ctx.fill();

                ctx.fillStyle = "black";
                ctx.font = "14px Arial";
                ctx.fillText("θ = " + last.theta.toFixed(2) + " rad", mapX(last.x) + 10, mapY(last.y) - 10);
            }}

            drawTrajectory(points);
        </script>
    </body>
    </html>
    """
    return html


@app.route("/state")
def state():
    return jsonify(robot_state)


@app.route("/trajectory")
def traj():
    return jsonify(trajectory)


@app.route("/update", methods=["POST"])
def update():
    data = request.get_json(silent=True)

    if data is None:
        return jsonify({"status": "error", "message": "Brak JSON"}), 400

    if "x" in data:
        robot_state["x"] = data["x"]
    if "y" in data:
        robot_state["y"] = data["y"]
    if "theta" in data:
        robot_state["theta"] = data["theta"]
    if "status" in data:
        robot_state["status"] = data["status"]
    if "v" in data:
        robot_state["v"] = data["v"]
    if "omega" in data:
        robot_state["omega"] = data["omega"]

    robot_state["time"] = now_str()

    point = {
        "time": robot_state["time"],
        "x": robot_state["x"],
        "y": robot_state["y"],
        "theta": robot_state["theta"]
    }

    trajectory.append(point)
    save_to_file()

    return jsonify({"status": "ok"})


@app.route("/upload_results", methods=["POST"])
def upload_results():
    method = request.form.get("method", "").strip()
    bag_name = request.form.get("bag_name", "").strip()
    uploaded_files = request.files.getlist("files")

    if not is_valid_method(method):
        return jsonify({"status": "error", "message": "Nieznana metoda"}), 400

    if not bag_name:
        return jsonify({"status": "error", "message": "Brak bag_name"}), 400

    if not uploaded_files:
        return jsonify({"status": "error", "message": "Brak plików"}), 400

    save_dir = os.path.join(UPLOAD_BASE, method, bag_name)
    os.makedirs(save_dir, exist_ok=True)

    saved_files = []

    for f in uploaded_files:
        if f and f.filename:
            filename = os.path.basename(f.filename)
            file_path = os.path.join(save_dir, filename)
            f.save(file_path)
            saved_files.append(filename)

    return jsonify({
        "status": "ok",
        "method": method,
        "bag_name": bag_name,
        "saved_files": saved_files
    })


@app.route("/uploaded/<method>/<bag_name>/<filename>")
def uploaded_file(method, bag_name, filename):
    if not is_valid_method(method):
        return "Nieznana metoda", 404

    filename = os.path.basename(filename)
    bag_name = os.path.basename(bag_name)

    file_path = os.path.join(UPLOAD_BASE, method, bag_name, filename)

    if not os.path.isfile(file_path):
        return "Brak pliku", 404

    return send_file(file_path)


@app.route("/clear_trajectory", methods=["POST"])
def clear_trajectory():
    global trajectory
    trajectory = []
    reset_robot_state("trajektoria online wyczyszczona")
    save_to_file()
    return redirect(url_for("home"))


@app.route("/clear", methods=["POST"])
def clear_history():
    return clear_trajectory()


@app.route("/delete_latest_results/<method>", methods=["POST"])
def delete_latest_results(method):
    if not is_valid_method(method):
        return "Nieznana metoda", 404

    bag_name, bag_dir = get_latest_result_dir(method)
    if bag_dir and os.path.isdir(bag_dir):
        shutil.rmtree(bag_dir, ignore_errors=True)

    return redirect(url_for("home"))


@app.route("/delete_all_results/<method>", methods=["POST"])
def delete_all_results(method):
    if not is_valid_method(method):
        return "Nieznana metoda", 404

    method_dir = os.path.join(UPLOAD_BASE, method)
    if os.path.isdir(method_dir):
        shutil.rmtree(method_dir, ignore_errors=True)
        os.makedirs(method_dir, exist_ok=True)

    return redirect(url_for("home"))


@app.route("/delete_file/<method>/<bag_name>/<filename>", methods=["POST"])
def delete_file(method, bag_name, filename):
    if not is_valid_method(method):
        return "Nieznana metoda", 404

    bag_name = os.path.basename(bag_name)
    filename = os.path.basename(filename)

    file_path = os.path.join(UPLOAD_BASE, method, bag_name, filename)
    if os.path.isfile(file_path):
        os.remove(file_path)

    bag_dir = os.path.join(UPLOAD_BASE, method, bag_name)
    if os.path.isdir(bag_dir) and not os.listdir(bag_dir):
        os.rmdir(bag_dir)

    return redirect(url_for("home"))


if __name__ == "__main__":
    load_from_file()
    app.run(host="0.0.0.0", port=5000)


