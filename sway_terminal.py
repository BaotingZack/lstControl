"""Terminal entry and full-period settling; position arrival alone is not settling."""

from collections import deque


def terminal_distance(speed, period, minimum, max_speed, acceleration):
    """Leave a swing period plus braking distance; cap noisy speed at the limit."""
    speed = min(abs(speed), max_speed)
    return max(minimum, speed * period + speed * speed / (2 * max(acceleration, .01)))


class TerminalSwayMonitor:
    def __init__(self):
        self.samples = deque()

    def reset(self):
        self.samples.clear()

    def update(self, *, t, theta, error, velocity, period, angle_tol, pos_tol, vel_tol):
        if self.samples and (t <= self.samples[-1][0] or t - self.samples[-1][0] > .5):
            self.reset()
        self.samples.append((t, *theta, *error, *velocity))
        while len(self.samples) > 2 and self.samples[1][0] <= t - period:
            self.samples.popleft()
        if len(self.samples) < 8 or t - self.samples[0][0] < period:
            return False
        return (all(max(s[a] for s in self.samples) - min(s[a] for s in self.samples) <= angle_tol for a in (1, 2))
                and all(abs(s[a]) <= pos_tol for s in self.samples for a in (3, 4))
                and all(abs(s[a]) <= vel_tol for s in self.samples for a in (5, 6)))
