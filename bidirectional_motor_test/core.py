"""Hardware-independent switch interlock and sinusoidal 3D DSHOT mapping."""

from dataclasses import dataclass
import math


DISARM = 2
ARM = 3
RUN = 1


class SwitchInterlock:
    """Require an observed DISARM -> ARM -> RUN sequence.

    Returning from RUN to ARM stops the wave without losing arm authorization.
    Faults and invalid values revoke authorization.
    """

    def __init__(self):
        self.primed = False
        self.armed = False

    def fault(self):
        self.primed = False
        self.armed = False

    def mode(self, switch):
        if switch == DISARM:
            self.primed = True
            self.armed = False
            return "DISARM"
        if switch == ARM:
            if self.primed:
                self.armed = True
                return "ARMED"
            return "WAIT_FOR_DISARM"
        if switch == RUN:
            return "SINE" if self.armed else "WAIT_FOR_ARM"
        self.fault()
        return "INVALID_SWITCH"


@dataclass(frozen=True)
class WaveOutput:
    dshot: int
    sine: float
    direction: int
    phase_rad: float
    event: str = ""


class SineDshot:
    """Phase-frozen sinusoidal speed demand with guarded zero-output reversals.

    Values for 3D DSHOT:
      positive: 1048..2047
      negative: 48..1047
      zero: 0
    A zero-output pause *changes the wall-clock waveform* and is intentional.
    It does not prove the rotor has actually stopped.
    """

    def __init__(self, frequency_hz=0.05, deadband=0.08,
                 positive_peak=1250, negative_peak=250,
                 reversal_pause_sec=2.0, invert_direction=False):
        if not (0 < frequency_hz <= 1.0):
            raise ValueError("frequency_hz must be in (0, 1]")
        if not (0.0 <= deadband < 0.5):
            raise ValueError("deadband must be in [0, 0.5)")
        if not (1048 <= positive_peak <= 2047):
            raise ValueError("positive_peak must be in [1048, 2047]")
        if not (48 <= negative_peak <= 1047):
            raise ValueError("negative_peak must be in [48, 1047]")
        if reversal_pause_sec < 0:
            raise ValueError("reversal_pause_sec must be >= 0")
        self.frequency_hz = float(frequency_hz)
        self.deadband = float(deadband)
        self.positive_peak = int(positive_peak)
        self.negative_peak = int(negative_peak)
        self.reversal_pause_ns = round(reversal_pause_sec * 1e9)
        self.invert_direction = bool(invert_direction)
        self.stop()

    def stop(self):
        self.running = False
        self.phase_rad = 0.0
        self.last_ns = None
        self.last_direction = 0
        self.hold_until_ns = None
        self.waiting_for_rpm = False

    def start(self, now_ns):
        self.stop()
        self.running = True
        self.last_ns = now_ns

    def step(self, now_ns, rpm_ready=True):
        if not self.running:
            return WaveOutput(0, 0.0, 0, self.phase_rad)
        if now_ns < self.last_ns:
            # Monotonic clock must not run backwards, but handle testing and resets.
            now_ns = self.last_ns

        release = False
        if self.hold_until_ns is not None:
            # Freeze sine phase for the whole pause; the opposite half-cycle
            # begins near its zero crossing rather than jumping to a high demand.
            self.last_ns = now_ns
            if now_ns < self.hold_until_ns:
                return WaveOutput(0, math.sin(self.phase_rad), 0,
                                  self.phase_rad)
            if not rpm_ready:
                event = "" if self.waiting_for_rpm else "waiting_for_rpm"
                self.waiting_for_rpm = True
                return WaveOutput(0, math.sin(self.phase_rad), 0,
                                  self.phase_rad, event)
            self.hold_until_ns = None
            self.waiting_for_rpm = False
            release = True
        else:
            seconds = (now_ns - self.last_ns) / 1e9
            self.phase_rad = (self.phase_rad +
                              math.tau * self.frequency_hz * seconds) % math.tau
            self.last_ns = now_ns

        sine = math.sin(self.phase_rad)
        direction = (1 if sine > self.deadband else
                     -1 if sine < -self.deadband else 0)
        if direction == 0:
            return WaveOutput(0, sine, 0, self.phase_rad)

        if self.last_direction and direction != self.last_direction and not release:
            self.hold_until_ns = now_ns + self.reversal_pause_ns
            return WaveOutput(0, sine, 0, self.phase_rad, "reversal_pause")

        event = ("reversal_command" if release else
                 "first_command" if not self.last_direction else "")
        self.last_direction = direction
        magnitude = (abs(sine) - self.deadband) / (1.0 - self.deadband)
        magnitude = min(1.0, max(0.0, magnitude))
        high_range = (direction > 0) != self.invert_direction
        if high_range:
            dshot = round(1048 + magnitude * (self.positive_peak - 1048))
        else:
            dshot = round(48 + magnitude * (self.negative_peak - 48))
        return WaveOutput(dshot, sine, direction, self.phase_rad, event)



class StepDshot:
    """Fixed-amplitude 3D DSHOT alternating at a configurable dwell time.

    hold_sec is the time at each NONZERO command, not the whole cycle.
    Optional zero-command pause lengthens the total half-cycle. A deliberate
    allow_direct_reversal opt-in is required for 0-second pauses.

    The machine emits the exact same first_command/reversal_command events
    as SineDshot. The sine field in WaveOutput is a normalized +/-1 STEP
    indicator here, NOT a sinusoidal sample.
    """

    def __init__(self, forward_dshot=1250, reverse_dshot=250,
                 hold_sec=2.0, reversal_pause_sec=2.0,
                 allow_direct_reversal=False, invert_direction=False):
        if not (1048 <= int(forward_dshot) <= 2047):
            raise ValueError("step_forward_dshot must be in [1048, 2047]")
        if not (48 <= int(reverse_dshot) <= 1047):
            raise ValueError("step_reverse_dshot must be in [48, 1047]")
        # The bidirectional 3D ranges are separated by EXACTLY 1000:
        # e.g. forward 1250 and reverse 250 encode equal command offsets.
        if int(forward_dshot) - 1048 != int(reverse_dshot) - 48:
            raise ValueError(
                "step_forward_dshot and step_reverse_dshot must encode "
                "equal 3D magnitudes (forward - reverse == 1000)")
        if not math.isfinite(hold_sec) or hold_sec <= 0:
            raise ValueError("step_hold_sec must be positive and finite")
        if (not math.isfinite(reversal_pause_sec) or
                reversal_pause_sec < 0):
            raise ValueError("step_reversal_pause_sec must be >= 0 and finite")
        if reversal_pause_sec == 0 and not allow_direct_reversal:
            raise ValueError(
                "zero step reversal pause requires explicit "
                "step_allow_direct_reversal: true")
        self.forward_dshot = int(forward_dshot)
        self.reverse_dshot = int(reverse_dshot)
        self.hold_ns = round(float(hold_sec) * 1e9)
        self.reversal_pause_ns = round(float(reversal_pause_sec) * 1e9)
        self.allow_direct_reversal = bool(allow_direct_reversal)
        self.invert_direction = bool(invert_direction)
        self.stop()

    def stop(self):
        self.running = False
        self.last_ns = None
        self.segment_start_ns = None
        self.last_direction = 0
        self.hold_until_ns = None
        self.pending_direction = 0
        self.waiting_for_rpm = False
        self.phase_rad = 0.0

    def start(self, now_ns):
        self.stop()
        self.running = True
        self.last_ns = int(now_ns)

    def _command(self, event=""):
        direction = self.last_direction
        self.phase_rad = 0.0 if direction > 0 else math.pi
        high_range = (direction > 0) != self.invert_direction
        dshot = (self.forward_dshot if high_range
                 else self.reverse_dshot)
        return WaveOutput(
            dshot=dshot, sine=float(direction), direction=direction,
            phase_rad=self.phase_rad, event=event)

    def step(self, now_ns, rpm_ready=True):
        if not self.running:
            return WaveOutput(0, 0.0, 0, self.phase_rad)
        now_ns = max(int(now_ns), self.last_ns)
        self.last_ns = now_ns

        if self.hold_until_ns is not None:
            # The 0-command pause is a WAIT state. Do not skip the
            # optional real-RPM stopped interlock after the timer elapses.
            if now_ns < self.hold_until_ns:
                return WaveOutput(0, 0.0, 0, self.phase_rad)
            if not rpm_ready:
                event = "" if self.waiting_for_rpm else "waiting_for_rpm"
                self.waiting_for_rpm = True
                return WaveOutput(0, 0.0, 0, self.phase_rad, event)
            self.waiting_for_rpm = False
            self.hold_until_ns = None
            self.last_direction = self.pending_direction
            self.pending_direction = 0
            self.segment_start_ns = now_ns
            return self._command("reversal_command")

        if self.last_direction == 0:
            if not rpm_ready:
                event = "" if self.waiting_for_rpm else "waiting_for_rpm"
                self.waiting_for_rpm = True
                return WaveOutput(0, 0.0, 0, self.phase_rad, event)
            self.waiting_for_rpm = False
            self.last_direction = 1
            self.segment_start_ns = now_ns
            return self._command("first_command")

        if now_ns - self.segment_start_ns < self.hold_ns:
            return self._command()

        next_direction = -self.last_direction
        if (self.reversal_pause_ns == 0 and
                self.allow_direct_reversal and rpm_ready):
            # Experimental command step: no intervening DSHOT 0.
            # The DSHOT publish in the node is the authoritative t0.
            self.last_direction = next_direction
            self.segment_start_ns = now_ns
            return self._command("reversal_command")

        # An RPM stop interlock may be false while spinning. Emit zero
        # and wait for the guard, even if direct switching was requested.
        self.pending_direction = next_direction
        self.hold_until_ns = now_ns + self.reversal_pause_ns
        return WaveOutput(
            0, 0.0, 0, self.phase_rad, "reversal_pause")
