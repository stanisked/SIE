"""Source-level safeguards for the opt-in, motor-off LD19 bring-up."""

import ast
from pathlib import Path


LAUNCH_PATH = Path(__file__).resolve().parents[1] / "launch" / "sie_pipeline.launch.py"


def _calls(name):
    tree = ast.parse(LAUNCH_PATH.read_text(encoding="utf-8"))
    return [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == name
    ]


def _keyword(call, name):
    return next((item.value for item in call.keywords if item.arg == name), None)


def _literal(node):
    return ast.literal_eval(node)


def test_ld19_is_disabled_by_default():
    arguments = _calls("DeclareLaunchArgument")
    ld19 = next(
        call for call in arguments
        if call.args and _literal(call.args[0]) == "enable_ld19"
    )
    assert _literal(_keyword(ld19, "default_value")) == "false"


def test_only_optional_ld19_nodes_use_enable_ld19_condition():
    nodes = _calls("Node")
    expected_optional_packages = {"ldlidar_stl_ros2", "tf2_ros"}
    optional = []

    for node in nodes:
        condition = _keyword(node, "condition")
        if condition is None:
            continue

        assert isinstance(condition, ast.Call)
        assert isinstance(condition.func, ast.Name)
        assert condition.func.id == "IfCondition"
        assert len(condition.args) == 1

        switch = condition.args[0]
        assert isinstance(switch, ast.Name)
        assert switch.id == "enable_ld19"

        optional.append(_literal(_keyword(node, "package")))

    assert set(optional) == expected_optional_packages
    assert len(optional) == 2


def test_original_phase_one_contract_nodes_remain_unconditional():
    nodes = _calls("Node")
    contract_executables = {
        "perception_contract_node",
        "navigation_contract_node",
        "supervisor_contract_node",
    }
    actual = {
        _literal(_keyword(node, "executable"))
        for node in nodes
        if _keyword(node, "condition") is None
    }
    assert actual == contract_executables


def test_ld19_scan_and_frame_remain_raw_diagnostic_inputs():
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    assert '"/sie/lidar/scan_raw"' in source
    assert '"ld19_sensor"' in source
    assert '"base_link"' in source
    assert '"--x", "-0.023"' in source
    assert '"--z", "0.212"' in source
    assert '"--yaw", "0"' in source
    assert "motor" not in source.lower()
