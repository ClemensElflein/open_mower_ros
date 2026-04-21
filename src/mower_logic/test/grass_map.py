"""Load a sensor-log CSV and expose grass_thickness(s) along the recorded path.

Used by the closed-loop blade_speed_adapter replay test. The CSV columns are:

    time_unix, time_iso, x (m), y (m), mow_rpm (RPM), mow_current (A), load_ratio

Each row is assumed to have been recorded at ~SPEED_MAX (see
.hypertask/task/blade_speed_adapter_analysis.md section 3: the pre-change
adapter was barely engaging, so the observed sag is essentially "what this
grass does to the blade at full speed"). We derive a continuous
grass_thickness in [0, 1] per row and interpolate between rows by arclength.
"""

import csv
import math

from motor_model import RPM_NOMINAL

RPM_MAX_LOAD = 2000.0   # |rpm| <= this counts as fully bogged; matches adapter default


def _rpm_sag_ratio(rpm):
    """Replicate blade_speed_adapter's own formula so 'truth' stays consistent."""
    mag = abs(rpm)
    if mag >= RPM_NOMINAL:
        return 0.0
    ratio = (RPM_NOMINAL - mag) / (RPM_NOMINAL - RPM_MAX_LOAD)
    if ratio < 0.0:
        return 0.0
    if ratio > 1.0:
        return 1.0
    return ratio


class GrassMap:
    """Arclength-indexed grass_thickness lookup along a CSV-recorded path."""

    def __init__(self, csv_path):
        self.x = []
        self.y = []
        self.grass = []
        with open(csv_path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                x = float(row["x (m)"])
                y = float(row["y (m)"])
                rpm = float(row["mow_rpm (RPM)"])
                load_ratio = float(row["load_ratio"])
                thickness = max(_rpm_sag_ratio(rpm), load_ratio)
                self.x.append(x)
                self.y.append(y)
                self.grass.append(thickness)

        if len(self.x) < 2:
            raise ValueError("CSV has too few rows: %d" % len(self.x))

        # Cumulative arclength at each sample
        self.s = [0.0]
        for i in range(1, len(self.x)):
            dx = self.x[i] - self.x[i - 1]
            dy = self.y[i] - self.y[i - 1]
            self.s.append(self.s[-1] + math.hypot(dx, dy))
        self.total_length = self.s[-1]

    def sample(self, s):
        """Linearly interpolate (x, y, grass) at arclength s (clamped to path)."""
        if s <= 0.0:
            return self.x[0], self.y[0], self.grass[0]
        if s >= self.total_length:
            return self.x[-1], self.y[-1], self.grass[-1]

        # Binary search for the bracketing segment
        lo, hi = 0, len(self.s) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.s[mid] <= s:
                lo = mid
            else:
                hi = mid

        seg = self.s[hi] - self.s[lo]
        if seg <= 0.0:
            return self.x[lo], self.y[lo], self.grass[lo]
        t = (s - self.s[lo]) / seg
        x = self.x[lo] + t * (self.x[hi] - self.x[lo])
        y = self.y[lo] + t * (self.y[hi] - self.y[lo])
        g = self.grass[lo] + t * (self.grass[hi] - self.grass[lo])
        return x, y, g

    def thick_arclength_windows(self, threshold):
        """Return list of (s_start, s_end, n_thick_rows) ranges where grass >= threshold.

        n_thick_rows distinguishes brief single-row spikes (transient, adapter
        cannot realistically reach full-stop in the time it takes to cross
        them) from sustained events spanning multiple rows. The test uses
        n_thick_rows to pick a stricter or looser speed-reduction threshold.
        """
        windows = []
        start = None
        start_idx = None
        for i, g in enumerate(self.grass):
            if g >= threshold and start is None:
                start = self.s[i]
                start_idx = i
            elif g < threshold and start is not None:
                windows.append((start, self.s[i], i - start_idx))
                start = None
                start_idx = None
        if start is not None:
            windows.append((start, self.s[-1], len(self.grass) - start_idx))
        return windows
