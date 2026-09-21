"""Conservative per-axis measurement check using natural support motion.

This does not calibrate the frame sensor or identify the real crane. A positive
model match is a necessary gate for the optional feedback, not a stability proof.
The fit separates unknown initial free sway and bias from the *forced* response;
without independent forcing, an arbitrary free-sway phase cannot prove polarity.
Only measured support positions in the controller coordinate frame are used.
"""

from collections import deque
import math

import numpy as np

from sway_settle import GRAVITY


class SwayFeedbackQualifier:
    def __init__(self):
        self.reset()

    def reset(self, *, preserve_revoked=False):
        revoked = self._revoked if preserve_revoked else [False, False]
        self._revoked = revoked
        self._allowed = [False, False]
        self._previous = None
        self._time = self._last_sample = self._last_check = 0.
        # Forced X/Y and the two free solutions; state h = theta_dot + v/L
        # avoids taking acceleration (second differences) of noisy positions.
        self._states = np.array([[0., 0.], [0., 0.], [1., 0.], [0., 1.]])
        self._samples = deque(maxlen=800)
        self._predictors = [None, None]
        self._passes = [0, 0]
        self._residuals = [deque(), deque()]
        self.diagnostics = [dict(reason='observing'), dict(reason='observing')]

    @property
    def allowed(self):
        return tuple(self._allowed)

    def update(self, theta_x, theta_y, x, y, length, dt):
        values = (theta_x, theta_y, x, y, length, dt)
        if not all(math.isfinite(v) for v in values) or length <= 0 or dt <= 0 or dt > .5:
            self.reset(preserve_revoked=True)
            return self.allowed
        if self._previous is None:
            self._previous = (x, y, length)
            return self.allowed
        px, py, pl = self._previous
        velocity = np.array([(x - px) / dt, (y - py) / dt, 0., 0.])
        length_rate = (length - pl) / dt
        steps = max(1, math.ceil(dt / .025))
        h = dt / steps
        for k in range(steps):
            L = pl + (k + .5) / steps * (length - pl)
            wn = math.sqrt(GRAVITY / L)
            damping = 2 * length_rate / L + .03 * wn

            def derivative(state):
                return np.column_stack((state[:, 1] - velocity / L,
                    -wn**2 * state[:, 0] - damping * state[:, 1]
                    + (damping / L - length_rate / L**2) * velocity))

            a = derivative(self._states)
            b = derivative(self._states + h * a / 2)
            c = derivative(self._states + h * b / 2)
            d = derivative(self._states + h * c)
            self._states += h * (a + 2*b + 2*c + d) / 6
        self._previous = (x, y, length)
        self._time += dt
        if self._time - self._last_sample < .05 - 1e-9:
            return self.allowed
        self._last_sample = self._time
        wn = math.sqrt(GRAVITY / length)
        period = 2 * math.pi / wn
        common = [1., self._time, *self._states[2:, 0]]
        rows = [np.array([*common, self._states[a, 0],
                          (self._states[a, 1] - velocity[a] / length) / wn]) for a in (0, 1)]
        self._samples.append((self._time, rows, (theta_x, theta_y)))
        for axis, theta in enumerate((theta_x, theta_y)):
            if not self._allowed[axis]:
                continue
            residual = theta - float(rows[axis] @ self._predictors[axis])
            samples = self._residuals[axis]
            samples.append((self._time, residual, theta))
            while len(samples) > 1 and samples[1][0] < self._time - period / 2:
                samples.popleft()
            if samples[-1][0] - samples[0][0] >= period / 4:
                rms = math.sqrt(sum(s[1]**2 for s in samples) / len(samples))
                amplitude = np.std([s[2] for s in samples])
                if rms > max(math.radians(.10), .35 * amplitude):
                    self._allowed[axis] = False
                    self._revoked[axis] = True
                    self.diagnostics[axis]['reason'] = 'prediction_mismatch'
        if self._time < 2 * period or self._time - self._last_check < period / 4:
            return self.allowed
        self._last_check = self._time
        for axis in (0, 1):
            if self._allowed[axis] or self._revoked[axis]:
                continue
            design = np.array([s[1][axis] for s in self._samples])
            observed = np.array([s[2][axis] for s in self._samples])
            coeff, diagnostic = self._fit(design, observed)
            self.diagnostics[axis] = diagnostic
            self._passes[axis] = self._passes[axis] + 1 if coeff is not None else 0
            if self._passes[axis] >= 2:
                self._predictors[axis] = coeff
                self._allowed[axis] = True
                self.diagnostics[axis]['reason'] = 'qualified'
        return self.allowed

    @staticmethod
    def _fit(design, observed):
        # Remove free modes and DC from both forced columns and measurements.
        nuisance, forced = design[:, :4], design[:, 4:]
        projected = forced - nuisance @ np.linalg.lstsq(nuisance, forced, rcond=None)[0]
        response = observed - nuisance @ np.linalg.lstsq(nuisance, observed, rcond=None)[0]
        excitation = np.linalg.svd(projected, compute_uv=False)[-1] / math.sqrt(len(observed))
        info = dict(reason='insufficient_excitation', excitation_deg=float(np.degrees(excitation)))
        if excitation < math.radians(.02) or np.std(response) < math.radians(.03):
            return None, info
        coeff = np.linalg.lstsq(design, observed, rcond=None)[0]
        error = observed - design @ coeff
        fit = 1 - float(error @ error) / max(float(response @ response), 1e-18)
        gain = math.hypot(*coeff[4:])
        phase = math.degrees(math.atan2(coeff[5], coeff[4]))
        # A chronological holdout prevents an in-window correlation alone from
        # qualifying an arbitrary initial phase or a poorly observed response.
        split = int(len(observed) * .75)
        train = np.linalg.lstsq(design[:split], observed[:split], rcond=None)[0]
        validation = float(np.sqrt(np.mean((observed[split:] - design[split:] @ train)**2)))
        info.update(gain=gain, phase_deg=phase, forced_r2=fit,
                    validation_error_deg=math.degrees(validation), reason='transfer_mismatch')
        passed = (.4 <= gain <= 1.8 and abs(phase) <= 35 and fit >= .85
                  and validation <= max(math.radians(.04), .2 * np.std(observed[split:])))
        return (coeff if passed else None), info

    def snapshot(self):
        out = {}
        for axis, name in enumerate('xy'):
            out[f'antisway_qualified_{name}'] = self._allowed[axis]
            for key, value in self.diagnostics[axis].items():
                out[f'antisway_model_{name}_{key}'] = value
        return out
