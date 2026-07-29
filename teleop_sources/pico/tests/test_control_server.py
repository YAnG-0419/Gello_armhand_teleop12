import json
import socket

import pytest

from pico_bimanual_franka_teleop.control_server import (
    OperatorConsole,
    OperatorControlServer,
)


def test_console_matches_the_input_source_contract():
    console = OperatorConsole()
    assert console.poll() == {"left": False, "right": False}
    console.active["left"] = True
    assert console.poll()["left"] is True
    console.deny("left", "tracker missing")
    assert console.poll()["left"] is False
    console.disable_all("robot state missing")
    console.set_status("STATE | trackers: ok")
    snapshot = console.snapshot()
    assert snapshot["status_line"] == "STATE | trackers: ok"
    assert any("tracker missing" in line for line in snapshot["feedback"])
    assert any("robot state missing" in line for line in snapshot["feedback"])


def test_console_requests_are_one_shot():
    console = OperatorConsole()
    console.requests["open_hands"] = True
    taken = console.take_requests()
    assert taken["open_hands"] is True
    assert console.take_requests()["open_hands"] is False


def make_server():
    console = OperatorConsole()
    server = OperatorControlServer(("127.0.0.1", 0), console)
    return server, console


def test_dispatch_maps_commands_onto_the_console():
    server, console = make_server()
    try:
        server.dispatch("engage", {"side": "left"})
        assert console.active["left"] is True
        server.dispatch("disengage", {"side": "left"})
        assert console.active["left"] is False
        server.dispatch("engage", {"side": "right"})
        server.dispatch("disengage_all", {})
        assert console.active == {"left": False, "right": False}
        server.dispatch("open_hands", {})
        server.dispatch("reset", {"side": "left"})
        server.dispatch("reset", {})
        requests = console.take_requests()
        assert requests["open_hands"] and requests["reset_left"] and requests["reset"]
        with pytest.raises(ValueError):
            server.dispatch("engage", {"side": "middle"})
        with pytest.raises(ValueError):
            server.dispatch("warp", {})
    finally:
        server.server_close()


def test_socket_round_trip_speaks_line_json():
    server, console = make_server()
    server.start()
    try:
        with socket.create_connection(server.server_address, timeout=2.0) as client:
            stream = client.makefile("rw", encoding="utf-8")
            for request_id, command, arguments in (
                (1, "engage", {"side": "right"}),
                (2, "status", {}),
                (3, "nonsense", {}),
            ):
                stream.write(
                    json.dumps(
                        {"id": request_id, "command": command, "arguments": arguments}
                    )
                    + "\n"
                )
                stream.flush()
            replies = [json.loads(stream.readline()) for _ in range(3)]
    finally:
        server.close()
    assert replies[0] == {"id": 1, "ok": True, "result": {}}
    assert replies[1]["ok"] and replies[1]["result"]["active"]["right"] is True
    assert not replies[2]["ok"] and "Unknown command" in replies[2]["error"]
    assert console.active["right"] is True
