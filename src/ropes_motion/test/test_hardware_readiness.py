from ropes_motion.hardware_readiness import HardwareHeartbeat, HardwareReadiness


def heartbeat(source_ns, received_ns, armed=True, referenced=True):
    return HardwareHeartbeat(source_ns, received_ns, "test", "", referenced, armed)


def test_requires_fresh_armed_and_referenced_heartbeat():
    gate = HardwareReadiness(timeout_ns=100, clock_tolerance_ns=10)

    assert "No hardware heartbeat" in gate.problem(1000, 1000)
    gate.receive(heartbeat(991, 1000, referenced=False), 1000)
    assert "unreferenced" in gate.problem(1000, 1000)
    gate.receive(heartbeat(993, 1001, armed=False), 1001)
    assert "disarmed" in gate.problem(1001, 1001)
    gate.receive(heartbeat(995, 1002), 1002)
    assert gate.problem(1002, 1002) is None
    assert "stale" in gate.problem(1096, 1002)
    assert "stale" in gate.problem(1002, 1103)


def test_rejects_invalid_future_and_out_of_order_heartbeats():
    gate = HardwareReadiness(timeout_ns=100, clock_tolerance_ns=10)
    gate.receive(heartbeat(0, 1000), 1000)
    gate.receive(heartbeat(1011, 1000), 1000)
    assert "No hardware heartbeat" in gate.problem(1000, 1000)
    gate.receive(heartbeat(1000, 1000, armed=False), 1000)
    gate.receive(heartbeat(999, 1001), 1001)
    gate.receive(heartbeat(1000, 1002), 1002)
    assert "disarmed" in gate.problem(1002, 1002)


def test_requires_post_stop_disarm_then_a_fresh_armed_status():
    gate = HardwareReadiness(timeout_ns=100, clock_tolerance_ns=10)
    gate.receive(heartbeat(995, 1000), 1000)
    gate.wait_for_disarm(1005)
    gate.receive(heartbeat(1001, 1010, armed=False), 1010)
    gate.receive(heartbeat(1010, 1011), 1011)
    assert "Waiting" in gate.problem(1011, 1011)
    gate.receive(heartbeat(1012, 1012, armed=False), 1012)
    assert "disarmed" in gate.problem(1012, 1012)
    gate.receive(heartbeat(1014, 1014), 1014)
    assert gate.problem(1014, 1014) is None
