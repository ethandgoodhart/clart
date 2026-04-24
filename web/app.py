import json
import os
import time

from flask import Flask, jsonify, render_template

STATE_FILE = os.environ.get("CART_STATE_FILE", "/tmp/cart_state.json")
STATE_FRESH_S = 1.0

app = Flask(__name__)


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/state")
def state():
    try:
        with open(STATE_FILE) as f:
            data = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return jsonify({
            "mph": 0, "gas": 0, "brake": 0, "stale": True,
            "controller_connected": False,
            "arduino_connected": False,
            "motor_connected": False,
        })
    if time.time() - data.get("ts", 0) > STATE_FRESH_S:
        data["mph"] = 0
        data["stale"] = True
        # ps5_drive.py only writes this file while it's running, so stale
        # state means the driver exited — usually a controller drop. Treat
        # everything as disconnected so the UI dots turn gray.
        data["controller_connected"] = False
        data["arduino_connected"] = False
        data["motor_connected"] = False
    return jsonify(data)


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5050, debug=False)
