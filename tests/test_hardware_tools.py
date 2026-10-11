import json
import math
import os
import sys

import pytest

from tools.hardware.telemetry import Recorder, replay, timing_summary
from tools.hardware.commission import inject_fault, mock_record, step_plan, step_response
from tools.hardware.deploy import select, stage
from tools.hardware.profile import profile_command


def test_create_only_and_provenance(tmp_path):
    path = tmp_path/"data.jsonl"
    with Recorder(path, "real_sim", "CARLA", "simulation") as writer:
        writer.write(0., "encoder", {"ticks": 10}, {"ticks": "count"})
        writer.write(.1, "imu", {"yaw": .3}, {"yaw": "rad"})
    assert len(list(replay(path))) == 2
    with pytest.raises(FileExistsError):
        Recorder(path, "mock_io", "fixture", "simulation")
    assert list(replay(path))[0]["source"]["mode"] == "real_sim"


@pytest.mark.parametrize("stamp", [float("nan"), float("inf"), -.1])
def test_bad_timestamp_rejected_without_poisoning_writer(tmp_path, stamp):
    with Recorder(tmp_path/"log", "mock_io", "fixture", "sim") as writer:
        with pytest.raises(ValueError):
            writer.write(stamp, "state", {}, {})
        writer.write(0., "state", {}, {})


@pytest.mark.parametrize("mutation", ["time", "sequence", "source", "nan"])
def test_replay_detects_corruption(tmp_path, mutation):
    path = tmp_path/"log"
    with Recorder(path, "mock_io", "fixture", "sim") as writer:
        writer.write(1., "state", {"x": 1.}, {"x": "m"})
        writer.write(2., "state", {"x": 2.}, {"x": "m"})
    lines = [json.loads(x) for x in path.read_text().splitlines()]
    if mutation == "time": lines[1]["time_s"] = .5
    if mutation == "sequence": lines[1]["sequence"] = 0
    if mutation == "source": lines[1]["source"]["mode"] = "hardware"
    if mutation == "nan": lines[1]["payload"]["x"] = float("nan")
    path.write_text("\n".join(json.dumps(x) for x in lines))
    with pytest.raises(ValueError): list(replay(path))


def test_step_bounds_and_stop():
    plan = step_plan(.1, .2, .01, 4.)
    assert plan[-1][1] == 0
    assert max(v for _, v in plan) == .1
    for args in [(1., .2, .01, 4.), (.1, .2, 0., 4.), (.1, .2, .01, .1), (math.nan, .2, .01, 4.)]:
        with pytest.raises(ValueError): step_plan(*args)


def test_identify_actual_fixture_response_and_reverse_step(tmp_path):
    for amplitude in (.1, -.1):
        path = tmp_path/str(amplitude)
        mock_record(path, amplitude=amplitude, tau=.15, delay=.05)
        data = list(replay(path))
        result = step_response(data, "joint_rad")
        assert result["gain_estimate"] == pytest.approx(1., abs=1e-5)
        assert result["rise_10_to_90_s"] == pytest.approx(.15*math.log(9), abs=.02)
        assert result["delay_to_10pct_s"] >= .05
        assert result["physical_validation"] == "PENDING_HARDWARE"
        assert result["provenance"]["mode"] == "mock_io"


def test_zero_or_absent_response_cannot_be_identified(tmp_path):
    path = tmp_path/"log"
    mock_record(path)
    data = list(replay(path))
    for r in data:
        if r["kind"] == "state": r["payload"]["joint_rad"] = 0.
    with pytest.raises(ValueError, match="zero measured"): step_response(data, "joint_rad")
    with pytest.raises(ValueError, match="insufficient"): step_response([], "absent")


def test_faults_are_offline_replay_and_resquenced(tmp_path):
    path = tmp_path/"log"
    mock_record(path)
    data = list(replay(path))
    dropped = inject_fault(data, "drop_state", 1., .2)
    assert len(dropped) < len(data)
    assert all(r["sequence"] == i and r["source"]["mode"] == "replay" for i, r in enumerate(dropped))
    stale = inject_fault(data, "stale_state", 1., .2)
    assert any(r["payload"].get("measurement_time_s", r["time_s"]) < r["time_s"] for r in stale)
    assert all(r["source"]["mode"] == "mock_io" for r in data)


def test_measured_latency_quantiles_and_jitter(tmp_path):
    path = tmp_path/"log"
    with Recorder(path, "mock_io", "fixture", "one_clock") as writer:
        writer.write(0., "state", {}, {})
        writer.write(.01, "command", {"command_id": 1}, {})
        writer.write(.03, "receipt", {"command_id": 1}, {})
        writer.write(.10, "state", {}, {})
        writer.write(.22, "state", {}, {})
    result = timing_summary(list(replay(path)))
    assert result["roundtrip"]["p99_s"] == pytest.approx(.02)
    assert result["period_jitter_rms_s"] == pytest.approx(.01)
    assert result["unacknowledged_commands"] == 0


def test_orphan_receipt_is_rejected(tmp_path):
    path = tmp_path/"log"
    with Recorder(path, "mock_io", "fixture", "one_clock") as writer:
        writer.write(0., "receipt", {"command_id": 4}, {})
    with pytest.raises(ValueError, match="matching prior command"):
        timing_summary(list(replay(path)))


def test_deployment_selection_requires_tested_release_and_rolls_back(tmp_path):
    root = tmp_path/"deployment"
    for commit, status in [("a"*40, "STAGED_TESTED"), ("b"*40, "STAGED_TESTED"), ("c"*40, "STAGED_UNVERIFIED")]:
        release = root/"releases"/commit
        release.mkdir(parents=True)
        (release/"release.json").write_text(json.dumps({"commit": commit, "status": status, "hardware_enabled": False}))
    first = select(root, "a"*40)
    second = select(root, "b"*40)
    assert first["hardware_enabled"] is False and first["process_started"] is False
    assert second["previous_commit"] == "a"*40
    with pytest.raises(ValueError, match="only tested"):
        select(root, "c"*40)
    assert json.loads((root/"selection.json").read_text())["commit"] == "b"*40
    rollback = select(root, second["previous_commit"], rollback=True)
    assert rollback["operation"] == "rollback" and rollback["commit"] == "a"*40
    assert (root/"releases"/("b"*40)).exists()
    with pytest.raises(ValueError): select(root, "../unsafe")


def test_stage_rejects_inside_source_directory(tmp_path):
    with pytest.raises(ValueError, match="outside source"):
        stage(tmp_path, tmp_path/"releases", verify=False)


@pytest.mark.skipif(os.name != "posix", reason="Linux/WSL process profiler")
@pytest.mark.parametrize("code", [0, 3])
def test_process_profiler_preserves_real_exit_code(tmp_path, code):
    pytest.importorskip("psutil")
    path = tmp_path/"profile.json"
    result = profile_command([sys.executable, "-c", f"print('real subprocess'); raise SystemExit({code})"], path)
    assert result["exit_code"] == code
    assert result["status"] == ("PASS_PROCESS" if code == 0 else "FAIL")
    assert result["wall_s"] > 0 and result["sampled_tree_rss_max_bytes"] > 0
    assert "real subprocess" in path.with_suffix(".stdout.txt").read_text()
    with pytest.raises(FileExistsError):
        profile_command([sys.executable, "-c", "pass"], path)


def test_profiler_platform_or_invalid_arguments(tmp_path):
    if os.name != "posix":
        with pytest.raises(RuntimeError, match="Linux/WSL"):
            profile_command([], tmp_path/"profile")
    else:
        with pytest.raises(ValueError): profile_command([], tmp_path/"profile")
        with pytest.raises(ValueError): profile_command(["true"], tmp_path/"profile", interval=0)
