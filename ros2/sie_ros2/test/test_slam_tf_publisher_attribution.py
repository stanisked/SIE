"""Exercise attribution using real DDS metadata, including the Jazzy fallback."""
import os
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.node import Node
from rclpy.time import Time

from tools.diagnostics import step75_5a_slam as slam


@pytest.mark.parametrize("duplicate", [False, True])
def test_native_attribution_with_two_endpoints_on_mapper_node(tmp_path, duplicate):
    executable = os.environ.get("SIE_TF_AUDIT_EXECUTABLE")
    if not executable:
        pytest.skip("Set SIE_TF_AUDIT_EXECUTABLE to the built diagnostic subscriber")

    rclpy.init(args=[])
    check = slam.Check("synthetic", {}, tmp_path, None, native_tf_audit=True)
    mapper = Node(slam.MAPPER.lstrip("/"))
    processes = slam.Processes(tmp_path)
    journal = tmp_path / "tf_receipts.txt"
    try:
        odom = check.create_publisher(slam.TFMessage, "/tf", 30)
        map_primary = mapper.create_publisher(slam.TFMessage, "/tf", 30)
        map_secondary = mapper.create_publisher(slam.TFMessage, "/tf", 30)
        processes.start("tf_audit", [str(Path(executable).resolve(strict=True)), str(journal)])
        assert slam.spin(check, processes, 5, lambda: all(
            publisher.get_subscription_count() >= 3 for publisher in (odom, map_primary, map_secondary)
        ))
        stamp = check.get_clock().now().nanoseconds
        odom.publish(slam.fixture_tf(Time(nanoseconds=stamp), 0.0))
        message = slam.fixture_tf(Time(nanoseconds=stamp + 1), 0.0)
        message.transforms[0].header.frame_id, message.transforms[0].child_frame_id = slam.MAP_EDGE
        map_primary.publish(message)
        if duplicate:
            # Same node name and identical frame/stamp/value, different DDS writer.
            map_secondary.publish(message)
        assert slam.spin(check, processes, 5, lambda: journal.is_file() and
                         journal.read_text().count("TF_RECEIPT ") == (3 if duplicate else 2))
        assert slam.spin(check, processes, 3, lambda: stamp + 1 in check.tf_observed_stamps[slam.MAP_EDGE])
        assert processes.stop()["tf_audit"]["exit_code"] == 0
        check.reconcile_native_tf()
        assert check.edge_owners[slam.ODOM_EDGE] == {slam.CHECK}
        assert check.edge_owners[slam.MAP_EDGE] == {slam.MAPPER}
        assert len(check.edge_gids[slam.MAP_EDGE]) == (2 if duplicate else 1)
        assert check.failure == (f"MULTIPLE_TF_AUTHORITIES:{slam.MAP_EDGE}" if duplicate else None)
    finally:
        processes.stop()
        mapper.destroy_node()
        check.destroy_node()
        rclpy.try_shutdown()


def test_missing_native_receipt_refuses_endpoint_only_attribution(tmp_path):
    rclpy.init(args=[])
    check = slam.Check("synthetic", {}, tmp_path, None, native_tf_audit=True)
    try:
        (tmp_path / "tf_receipts.txt").touch()
        check.tf_observed_stamps[slam.MAP_EDGE].add(42)
        check.tf_endpoints["/tf"]["a1"] = slam.MAPPER
        check.reconcile_native_tf()
        assert check.failure == f"TF_RECEIPT_COVERAGE_INCOMPLETE:{slam.MAP_EDGE}"
        assert not check.edge_owners
    finally:
        check.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize(("receipt_case", "refusal"), [
    ("legitimate", None),
    ("same_node_duplicate", f"MULTIPLE_TF_AUTHORITIES:{slam.MAP_EDGE}"),
    ("unknown_gid", "UNEXPECTED_TF_AUTHORITY:"),
    ("zero_gid", "TF_PUBLISHER_GID_UNAVAILABLE:"),
    ("incomplete_record", "INVALID_NATIVE_TF_RECEIPT"),
    ("missing_stamp", "TF_RECEIPT_COVERAGE_INCOMPLETE:"),
])
def test_receipt_validation_offline(tmp_path, receipt_case, refusal):
    from collections import defaultdict
    from types import MethodType, SimpleNamespace

    wheel_gid, mapper_gid, extra_mapper_gid = (part * 16 for part in ("ab", "cd", "ef"))
    check = SimpleNamespace(
        evidence=tmp_path, failure=None, edge_owners=defaultdict(set), edge_gids=defaultdict(set),
        tf_endpoints={"/tf": {wheel_gid: slam.CHECK, mapper_gid: slam.MAPPER, extra_mapper_gid: slam.MAPPER}},
        tf_observed_stamps={slam.ODOM_EDGE: {100}, slam.MAP_EDGE: {101}},
        tf_attributed_stamps=defaultdict(set),
    )
    for method in (slam.Check.refuse, slam.Check.attribute_tf, slam.Check.reconcile_native_tf):
        setattr(check, method.__name__, MethodType(method, check))
    rows = [f'TF_RECEIPT /tf {wheel_gid} 100 "odom" "base_link"\n',
            f'TF_RECEIPT /tf {mapper_gid} 101 "map" "odom"\n']
    if receipt_case == "same_node_duplicate":
        rows.append(f'TF_RECEIPT /tf {extra_mapper_gid} 101 "map" "odom"\n')
    elif receipt_case == "unknown_gid":
        rows[1] = rows[1].replace(mapper_gid, "12" * 16)
    elif receipt_case == "zero_gid":
        rows[1] = rows[1].replace(mapper_gid, "00" * 16)
    elif receipt_case == "incomplete_record":
        rows[1] = rows[1].rstrip("\n")
    elif receipt_case == "missing_stamp":
        rows.pop()
    (tmp_path / "tf_receipts.txt").write_text("".join(rows))
    if receipt_case == "incomplete_record":
        with pytest.raises(ValueError, match=refusal):
            check.reconcile_native_tf()
        return
    check.reconcile_native_tf()
    assert check.edge_owners[slam.ODOM_EDGE] == {slam.CHECK}
    if refusal is None:
        assert check.failure is None
        assert check.edge_owners[slam.MAP_EDGE] == {slam.MAPPER}
        assert check.edge_gids[slam.MAP_EDGE] == {mapper_gid}
    else:
        assert check.failure.startswith(refusal)
