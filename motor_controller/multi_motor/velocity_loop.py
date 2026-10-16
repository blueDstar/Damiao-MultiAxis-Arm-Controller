"""Per-axis feedback trim of the MIT velocity command, without altering readings."""
import math


class VelocityLoop:
    def __init__(self, integral_gain=1.0, trim_limit=2.0):
        self.integral_gain, self.trim_limit = integral_gain, trim_limit
        self.trim = 0.0
        self.last_time = None
        self.last_reference = 0.0

    def reset(self):
        self.trim, self.last_time, self.last_reference = 0.0, None, 0.0

    def command(self, reference, measured, timestamp, limit):
        if reference == 0.0:
            self.reset()
            return 0.0
        if self.last_reference * reference < 0:
            self.reset()
        elapsed = 0.0 if self.last_time is None else max(0.0, min(0.2, timestamp - self.last_time))
        error = reference - measured
        # Avoid integral windup during startup and large direction changes.
        if abs(error) <= max(0.5, abs(reference) * 0.25):
            proposal = max(-self.trim_limit, min(self.trim_limit, self.trim + self.integral_gain * error * elapsed))
            if abs(reference + proposal) <= limit or (reference + proposal) * error <= 0:
                self.trim = proposal
        self.last_time, self.last_reference = timestamp, reference
        return max(-limit, min(limit, reference + self.trim))
