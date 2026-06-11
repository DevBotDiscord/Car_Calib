from __future__ import annotations

from fastapi.testclient import TestClient

from runtime import https_stream
from runtime.https_stream import HttpsMjpegServer, SharedFrameStore


class _FakeSteeringController:
    PARAM_BOUNDS = {"kp": [0.0, 10.0]}

    def __init__(self) -> None:
        self.params = {"kp": 1.0}

    def get_params(self) -> dict[str, float]:
        return dict(self.params)

    def update_params(self, body: dict[str, float]) -> dict[str, float]:
        self.params.update(body)
        return dict(self.params)


def _client(controller: _FakeSteeringController | None = None, restart_callback=None) -> TestClient:
    server = HttpsMjpegServer(
        host="127.0.0.1",
        port=8443,
        stream_path="/stream",
        snapshot_path="/snapshot",
        status_path="/status",
        token="",
        cert_file="cert.pem",
        key_file="key.pem",
        frame_store=SharedFrameStore(),
        steering_controller=controller or _FakeSteeringController(),
        restart_callback=restart_callback,
    )
    return TestClient(server._build_app())


def test_post_control_params_accepts_json_body() -> None:
    client = _client()

    response = client.post("/control/params", json={"kp": 2.0})

    assert response.status_code == 200
    assert response.json()["params"]["kp"] == 2.0


def test_post_control_params_invokes_restart_callback() -> None:
    calls: list[bool] = []

    def fake_restart() -> dict[str, object]:
        calls.append(True)
        return {"requested": True, "deferred": False}

    client = _client(restart_callback=fake_restart)

    response = client.post("/control/params", json={"kp": 1.5})

    assert response.status_code == 200
    body = response.json()
    assert body["params"]["kp"] == 1.5
    assert body["restart"] == {"requested": True, "deferred": False}
    assert calls == [True]


def test_control_preset_save_accepts_json_body(monkeypatch) -> None:
    saved: dict[str, object] = {}

    def fake_save(name: str, params: dict[str, float]) -> None:
        saved["name"] = name
        saved["params"] = params

    def fake_load(name: str) -> dict[str, object]:
        return {"name": name, "params": saved["params"]}

    monkeypatch.setattr(https_stream, "_tune_preset_save", fake_save)
    monkeypatch.setattr(https_stream, "_tune_preset_load", fake_load)
    client = _client()

    response = client.put("/control/presets/demo", json={"params": {"kp": 3.0}})

    assert response.status_code == 200
    assert response.json()["preset"]["params"]["kp"] == 3.0
    assert saved == {"name": "demo", "params": {"kp": 3.0}}
