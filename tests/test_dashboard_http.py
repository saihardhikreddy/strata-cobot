"""The dashboard's web handler, served for real but with ROS and the robots stubbed out."""
import http.client
import json
import os
import sys
import threading
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest import mock

import pytest

for name in ("rclpy", "rclpy.node", "rclpy.executors", "rclpy.signals", "rclpy.qos", "cv_bridge",
             "nav_msgs", "nav_msgs.msg", "sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg"):
    sys.modules.setdefault(name, mock.MagicMock())
sys.modules["rclpy.node"].Node = type("Node", (), {})
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "dashboard"))

import app  # noqa: E402


@pytest.fixture
def server():
    runner = mock.Mock()
    runner.start.return_value = (True, "started")
    runner.snapshot.return_value = {"mode": "lap", "phase": "running"}
    srv = ThreadingHTTPServer(("127.0.0.1", 0), app.make_handler(mock.Mock(), runner))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield srv, runner
    srv.shutdown()


def request(srv, method, path, body=None):
    conn = http.client.HTTPConnection("127.0.0.1", srv.server_address[1], timeout=5)
    conn.request(method, path, body=body)
    res = conn.getresponse()
    return res.status, res.read()


def test_run_accepts_a_known_mission(server):
    srv, runner = server
    status, body = request(srv, "POST", "/api/run", json.dumps({"mode": "lap"}))
    assert status == 200 and json.loads(body)["ok"]
    runner.start.assert_called_once_with("lap", False)


@pytest.mark.parametrize("body", [b"{not json", b"[1, 2]", b'"lap"'])
def test_bad_json_gets_a_400_not_a_dropped_connection(server, body):
    srv, runner = server
    status, reply = request(srv, "POST", "/api/run", body)
    assert status == 400 and json.loads(reply)["ok"] is False
    runner.start.assert_not_called()


def test_obstacle_without_gazebo_reports_an_error(server):
    srv, _ = server
    with mock.patch.object(app, "drop_rock", side_effect=FileNotFoundError("gz")):
        status, reply = request(srv, "POST", "/api/obstacle", b"{}")
    assert status == 200
    assert json.loads(reply) == {"ok": False, "message": "Could not reach Gazebo: gz"}


def test_static_files_stay_inside_the_static_folder(server, tmp_path, monkeypatch):
    srv, _ = server
    static = tmp_path / "static"
    static.mkdir()
    (static / "index.html").write_text("<h1>hi</h1>")
    sibling = tmp_path / "static_old"
    sibling.mkdir()
    (sibling / "secret.txt").write_text("not for the web")
    monkeypatch.setattr(app, "STATIC", str(static))
    assert request(srv, "GET", "/")[0] == 200
    assert request(srv, "GET", "/static/index.html")[1] == b"<h1>hi</h1>"
    status, body = request(srv, "GET", "/static/../static_old/secret.txt")
    assert status == 404 and b"not for the web" not in body
    assert request(srv, "GET", "/static/" + os.path.join("..", "..", "etc", "passwd"))[0] == 404
