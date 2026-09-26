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
