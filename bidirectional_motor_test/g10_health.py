"""Pure functions for G10 data readiness and event baseline acceptance.

Neither a UDP packet reception time nor ROS publish time represents a
hardware-triggered ADC/DSHOT timestamp. These checks only assess whether
the host-side observations are usable.
"""


def baseline_ready(g10_enabled, force_topic, zero_ready,
                   sample_ns, command_ns, max_age_ns):
    """A fresh, calibrated-or-raw, *same-host* force sample exists at t0."""
    return (
        bool(g10_enabled or force_topic)
        and bool(zero_ready)
        and sample_ns is not None
        and 0 <= command_ns - sample_ns <= max_age_ns
    )


def g10_health(now_ns, last_recv_ns, zero_ready, timeout_ns,
               backlog, backlog_limit, error=None):
    """Return (ready, reason); no fabricated measurement when stream is stale."""
    if error is not None:
        return False, "receiver_error"
    if backlog > backlog_limit:
        return False, "queue_backlog"
    if last_recv_ns is None:
        return False, "no_packets"
    if now_ns < last_recv_ns or now_ns - last_recv_ns > timeout_ns:
        return False, "stale_packets"
    if not zero_ready:
        return False, "auto_zero_incomplete"
    return True, "ready"
