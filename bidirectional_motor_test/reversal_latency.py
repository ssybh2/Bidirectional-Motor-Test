"""Observed reversal: first opposite DSHOT publish -> sustained thrust sign.

This detector intentionally uses the receiver's monotonic UDP packet arrival
timestamps, NOT the G10 sequence-paced *estimated* ADC times. The latter
were observed to be ahead of receipt in the field; they cannot establish
absolute command-to-force latency without hardware synchronization.

One datum is the median of the 40 actual ADC forces in a UDP packet (~4 ms).
A short spike can never satisfy the >=200 ms default persistence criterion.
The reported crossing is the receipt of the FIRST packet in the confirmed
target-sign run; confirmed_ns is the end of the run. This is a host-observed
latency (an upper-bound proxy with unknown sensor/network bias), not the
instant of ESC reception or proof of rotor RPM reversal.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DirectionChangeResult:
    event_id: int
    metric: str
    status: str
    command_ns: int
    observed_ns: int | None
    confirmed_ns: int | None
    latency_ms: float | None
    baseline: float | None
    observed_force: float | None
    from_direction: int
    to_direction: int


class ReversalDirectionTracker:
    """Measure ONLY an established +1 -> -1 or -1 -> +1 command transition."""

    def __init__(self, sign_threshold=4.0, stable_sec=0.2,
                 timeout_sec=5.0, max_packet_gap_sec=0.05):
        if (not math.isfinite(sign_threshold) or sign_threshold <= 0 or
                not math.isfinite(stable_sec) or stable_sec <= 0 or
                not math.isfinite(timeout_sec) or timeout_sec <= 0 or
                not math.isfinite(max_packet_gap_sec) or
                max_packet_gap_sec <= 0):
            raise ValueError("reversal thresholds and time spans must be > 0")
        self.sign_threshold = float(sign_threshold)
        self.stable_ns = round(stable_sec * 1e9)
        self.timeout_ns = round(timeout_sec * 1e9)
        self.max_gap_ns = round(max_packet_gap_sec * 1e9)
        self.pending = None

    @staticmethod
    def _result(p, status, observed_ns=None, confirmed_ns=None,
                force=None):
        return DirectionChangeResult(
            event_id=p["id"], metric="force_direction_change",
            status=status, command_ns=p["t0"], observed_ns=observed_ns,
            confirmed_ns=confirmed_ns,
            latency_ms=(
                (observed_ns - p["t0"]) / 1e6
                if observed_ns is not None else None),
            baseline=p["baseline"], observed_force=force,
            from_direction=p["from"], to_direction=p["to"])

    def start(self, event_id, command_ns, from_direction, to_direction,
              baseline_force=None):
        """The controller MUST already have published opposite nonzero DSHOT.

        Never synthesize a response if pre-command force was not observed or
        was already steadily on the target side: such a t0 is ambiguous.
        """
        if (from_direction not in (-1, 1) or
                to_direction != -from_direction):
            raise ValueError("expected exactly +1 <-> -1 transition")
        p = {
            "id": int(event_id), "t0": int(command_ns),
            "from": int(from_direction), "to": int(to_direction),
            "baseline": baseline_force, "first": None,
            "last": None, "n": 0,
        }
        out = []
        if self.pending is not None:
            out.append(self._result(self.pending, "superseded"))
        self.pending = None
        if baseline_force is None or not math.isfinite(baseline_force):
            out.append(self._result(p, "no_pre_command_force"))
        elif to_direction * baseline_force >= self.sign_threshold:
            out.append(self._result(p, "already_target_before_command"))
        else:
            self.pending = p
        return out

    def observe_packet(self, receive_ns, median_force):
        """Use monotonic HOST packet receipt, never future-dated ADC estimates."""
        p = self.pending
        if p is None:
            return []
        receive_ns = int(receive_ns)
        if receive_ns <= p["t0"]:
            return []
        if receive_ns - p["t0"] > self.timeout_ns:
            self.pending = None
            return [self._result(p, "timeout")]
        if not math.isfinite(median_force):
            p["first"] = p["last"] = None
            p["n"] = 0
            return []
        if (p["last"] is not None and
                (receive_ns <= p["last"] or
                 receive_ns - p["last"] > self.max_gap_ns)):
            p["first"] = p["last"] = None
            p["n"] = 0
        if p["to"] * median_force < self.sign_threshold:
            p["first"] = p["last"] = None
            p["n"] = 0
            return []
        if p["first"] is None:
            p["first"] = receive_ns
        p["last"] = receive_ns
        p["n"] += 1
        if (p["last"] - p["first"] >= self.stable_ns and
                p["n"] >= 10):
            self.pending = None
            return [self._result(
                p, "detected", p["first"], receive_ns, median_force)]
        return []

    def expire(self, now_ns):
        p = self.pending
        if p is None or int(now_ns) - p["t0"] <= self.timeout_ns:
            return []
        self.pending = None
        return [self._result(p, "timeout")]

    def cancel(self, reason="stream_fault"):
        p = self.pending
        self.pending = None
        return [self._result(p, reason)] if p is not None else []
