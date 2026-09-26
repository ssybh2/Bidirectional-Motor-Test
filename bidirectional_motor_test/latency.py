"""Thrust-onset estimator on a common host monotonic receive-time axis.

This is an *observed ROS command-publish to force-receipt* delay, not a
hardware-triggered ESC-to-physical-force response time. Sensor and transport
latencies are included, and ongoing freewheel decay may produce false onset.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Result:
    event_id: int
    metric: str
    status: str
    command_ns: int
    observed_ns: int | None
    latency_ms: float | None
    baseline: float
    observed_force: float | None


class ThrustLatency:
    def __init__(self, delta_threshold=0.03, sign_threshold=0.03,
                 confirm_samples=3, timeout_sec=5.0):
        if delta_threshold <= 0 or sign_threshold <= 0:
            raise ValueError("force thresholds must be positive")
        if confirm_samples < 1:
            raise ValueError("confirm_samples must be at least 1")
        if timeout_sec <= 0:
            raise ValueError("timeout_sec must be positive")
        self.delta_threshold = delta_threshold
        self.sign_threshold = sign_threshold
        self.confirm_samples = confirm_samples
        self.timeout_ns = round(timeout_sec * 1e9)
        self.pending = None

    def start(self, event_id, now_ns, desired_sign, baseline):
        """Register first nonzero command after start or after a reversal hold."""
        if desired_sign not in (-1, 1):
            raise ValueError("desired_sign must be +1 or -1")
        if baseline is None:
            return []
        self.pending = {
            "id": event_id, "ns": now_ns, "sign": desired_sign,
            "baseline": float(baseline), "delta_count": 0,
            "delta_first": None, "sign_count": 0, "sign_first": None,
            "delta_done": False,
            # If force is already in the target direction at command time,
            # a subsequent sign crossing is not a meaningful delay.
            "sign_done": baseline * desired_sign >= self.sign_threshold,
        }
        if self.pending["sign_done"]:
            return [Result(event_id, "target_sign", "already_reached",
                           now_ns, None, None, float(baseline), None)]
        return []

    def observe(self, now_ns, force):
        pending = self.pending
        if pending is None or now_ns < pending["ns"]:
            return []
        if now_ns - pending["ns"] > self.timeout_ns:
            return self.expire(now_ns)

        out = []
        b = pending["baseline"]
        sign = pending["sign"]
        if not pending["delta_done"]:
            # Look for a force change *toward* the new desired direction.
            # A residual force decay after zero command can also satisfy this.
            if sign * (force - b) >= self.delta_threshold:
                if pending["delta_count"] == 0:
                    pending["delta_first"] = now_ns
                pending["delta_count"] += 1
                if pending["delta_count"] >= self.confirm_samples:
                    pending["delta_done"] = True
                    first = pending["delta_first"]
                    out.append(Result(pending["id"], "force_onset", "detected",
                                      pending["ns"], first,
                                      (first - pending["ns"]) / 1e6, b, force))
            else:
                pending["delta_count"] = 0
                pending["delta_first"] = None
        if not pending["sign_done"]:
            if sign * force >= self.sign_threshold:
                if pending["sign_count"] == 0:
                    pending["sign_first"] = now_ns
                pending["sign_count"] += 1
                if pending["sign_count"] >= self.confirm_samples:
                    pending["sign_done"] = True
                    first = pending["sign_first"]
                    out.append(Result(pending["id"], "target_sign", "detected",
                                      pending["ns"], first,
                                      (first - pending["ns"]) / 1e6, b, force))
            else:
                pending["sign_count"] = 0
                pending["sign_first"] = None
        if pending["sign_done"] and pending["delta_done"]:
            self.pending = None
        return out

    def expire(self, now_ns):
        p = self.pending
        if p is None or now_ns - p["ns"] <= self.timeout_ns:
            return []
        out = []
        for metric, done in (("force_onset", p["delta_done"]),
                             ("target_sign", p["sign_done"])):
            if not done:
                out.append(Result(p["id"], metric, "timeout", p["ns"],
                                  None, None, p["baseline"], None))
        self.pending = None
        return out

    def cancel(self):
        self.pending = None
