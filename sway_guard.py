"""按轴观察倾角振荡增长并撤回反馈。

这不是绳摆真值估计，也不能证明增长由反馈导致。它在持续增长时保守地退回
PD，避免在一个作业内跨阶段反复注入修正。触发前的摆动仍须计入验收。
"""

from collections import deque
import math


class SwayGrowthGuard:
    def __init__(self, *, growth_ratio=1.25, min_pp_rad=math.radians(.6),
                 min_increase_rad=math.radians(.15)):
        if not (math.isfinite(growth_ratio) and growth_ratio > 1):
            raise ValueError('growth_ratio must be finite and greater than one')
        if any(not math.isfinite(v) or v < 0 for v in (min_pp_rad, min_increase_rad)):
            raise ValueError('growth floors must be finite and non-negative')
        self.growth_ratio = growth_ratio
        self.min_pp_rad = min_pp_rad
        self.min_increase_rad = min_increase_rad
        self._blocked = [False, False]
        self.events = []
        self._time = 0.
        self.reset_window()

    @property
    def blocked(self):
        return tuple(self._blocked)

    def reset_window(self):
        """新阶段重新建立观测窗；已撤回轴只能通过新建整个作业的 guard 清除。"""
        self._samples = deque()
        self._last_check = None
        self._baseline = [None, None]
        self._engaged = [False, False]
        self._violations = [0, 0]
        self.last_pp = [None, None]

    def pause(self, dt):
        """缺测区间不能压缩成相邻采样，清观测但保留已撤回状态。"""
        self._time += dt
        self.reset_window()

    def update(self, theta_x, theta_y, *, dt, period_s, commanded, feedback_allowed=True):
        self._time += dt
        if dt > .5:
            self.reset_window()
        self._samples.append((self._time, theta_x, theta_y))
        cutoff = self._time - period_s
        while len(self._samples) > 2 and self._samples[1][0] <= cutoff:
            self._samples.popleft()
        for axis in (0, 1):
            # 限幅和活动门限会让正常反馈短时为零；这不能抬高基准、抹掉增长。
            # 只有阶段门控明确关闭反馈，才重新用纯观测建立基准。
            self._engaged[axis] = (self._engaged[axis] or bool(commanded[axis])) if feedback_allowed else False
        # 满一个周期再计算；每四分之一周期检查，防止残缺窗自身填充被当成增长。
        if len(self._samples) < 8 or self._time - self._samples[0][0] < period_s - 1e-9:
            return self.blocked
        if self._last_check is not None and self._time - self._last_check < period_s / 4:
            return self.blocked
        self._last_check = self._time
        for axis in (0, 1):
            values = [s[axis + 1] for s in self._samples]
            pp = max(values) - min(values)
            self.last_pp[axis] = pp
            baseline = self._baseline[axis]
            if baseline is None or not self._engaged[axis]:
                self._baseline[axis] = pp
                self._violations[axis] = 0
            elif not self._blocked[axis]:
                growing = (pp > baseline * self.growth_ratio and pp > self.min_pp_rad
                           and pp - baseline > self.min_increase_rad)
                self._violations[axis] = self._violations[axis] + 1 if growing else 0
                if self._violations[axis] >= 2:
                    self._blocked[axis] = True
                    self.events.append(dict(t=self._time, axis='xy'[axis],
                                            baseline_deg=math.degrees(baseline), pp_deg=math.degrees(pp)))
                self._baseline[axis] = min(baseline, pp)
        return self.blocked

    def snapshot(self):
        return {
            'antisway_blocked_x': self._blocked[0],
            'antisway_blocked_y': self._blocked[1],
            'antisway_raw_pp_x_deg': None if self.last_pp[0] is None else math.degrees(self.last_pp[0]),
            'antisway_raw_pp_y_deg': None if self.last_pp[1] is None else math.degrees(self.last_pp[1]),
        }
