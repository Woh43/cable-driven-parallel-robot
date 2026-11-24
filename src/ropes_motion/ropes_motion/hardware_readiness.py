"""Readiness and rearming state for trajectories sent to a physical actuator."""

from dataclasses import dataclass


@dataclass(frozen=True)
class HardwareHeartbeat:
    source_ns: int
    received_ns: int
    state: str
    detail: str
    referenced: bool
    armed: bool


class HardwareReadiness:
    def __init__(self, timeout_ns, clock_tolerance_ns):
        self.timeout_ns = timeout_ns
        self.clock_tolerance_ns = clock_tolerance_ns
        self._heartbeat: HardwareHeartbeat | None = None
        self._disarm_after_ns: int | None = None

    def _fresh(self, heartbeat, now_ns, steady_ns):
        source_age = now_ns - heartbeat.source_ns
        receive_age = steady_ns - heartbeat.received_ns
        return (
            heartbeat.source_ns > 0
            and -self.clock_tolerance_ns <= source_age <= self.timeout_ns
            and 0 <= receive_age <= self.timeout_ns
        )

    def receive(self, heartbeat, now_ns):
        if not self._fresh(heartbeat, now_ns, heartbeat.received_ns):
            return
        previous = self._heartbeat
        if (previous is not None and heartbeat.source_ns <= previous.source_ns
                and self._fresh(previous, now_ns, heartbeat.received_ns)):
            return
        self._heartbeat = heartbeat
        if (self._disarm_after_ns is not None and not heartbeat.armed
                and heartbeat.source_ns >= self._disarm_after_ns):
            self._disarm_after_ns = None

    def wait_for_disarm(self, now_ns):
        """Block new motion until a subsequent heartbeat confirms the hardware disarmed."""
        self._disarm_after_ns = now_ns

    def problem(self, now_ns, steady_ns):
        if self._disarm_after_ns is not None:
            return "Waiting for hardware to confirm it is disarmed after a trajectory stop"
        heartbeat = self._heartbeat
        if heartbeat is None:
            return "No hardware heartbeat received on hardware/status"
        if not self._fresh(heartbeat, now_ns, steady_ns):
            return "Hardware heartbeat became stale"
        if heartbeat.referenced and heartbeat.armed:
            return None
        reason = "Hardware is unreferenced" if not heartbeat.referenced else "Hardware is disarmed"
        state = f" ({heartbeat.state})" if heartbeat.state else ""
        detail = f". {heartbeat.detail}" if heartbeat.detail else ""
        return f"{reason}{state}{detail}"
