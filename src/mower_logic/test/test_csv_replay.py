#!/usr/bin/env python3
"""Closed-loop CSV-replay integration test for blade_speed_adapter.

Feeds a real sensor log through a (grass, robot_speed) -> (rpm, current)
motor model. The robot's "position" along the recorded path advances by
`target_speed * dt` each tick — so when the adapter slows the robot down,
we physically dwell longer on the same thick-grass cell. This is the
feedback loop the adapter exists to exercise.

Asserts that during every thick-grass segment on the path, target_speed
drops below a safety threshold at least once. This is the regression test
that would have caught the pre-recalibration bug (see section 7 of
.hypertask/task/blade_speed_adapter_analysis.md).

Requires /use_sim_time and a /clock publisher (sim_clock.py) — see
blade_speed_adapter_csv_replay.test.
"""

import os
import sys
import unittest

import rospy
from std_msgs.msg import String
from mower_msgs.msg import ESCStatus, HighLevelStatus, Status

# test/ is not a Python package — add our own directory to sys.path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from grass_map import GrassMap  # noqa: E402
from motor_model import MotorModel, SPEED_MAX  # noqa: E402


# --- Assertion thresholds -----------------------------------------------------
# Thresholds are what the adapter can *actually* achieve, not what would be
# ideal. Closed-loop equilibrium for grass=1.0 is v* = speed_max / (1 + K) with
# K ~= (speed_max - speed_min) / (speed_max * (rpm_nominal - rpm_max_load) /
#                                  (rpm_nominal - rpm_stall))
# ... which comes out around 0.165 m/s given our defaults. Assertions leave
# headroom for motor-model lag and feedback delay.
GRASS_THICK_THRESHOLD = 0.7     # grass level that demands a slowdown
SPEED_SUSTAINED_THRESHOLD = 0.20  # sustained (>= 2 rows): >50% slowdown
SPEED_SPIKE_THRESHOLD = 0.25      # single-row spike: meaningful response only


def parse_log(msg_data):
    result = {}
    for part in msg_data.split(";"):
        if "=" in part:
            k, v = part.split("=", 1)
            result[k] = v
    return result


class TestCsvReplay(unittest.TestCase):
    """Run the replay ONCE in setUpClass, then assert on the captured trace."""

    # Populated by setUpClass
    trace = []           # list of dicts: {s, grass, rpm, current, target_speed}
    grass_map = None

    @classmethod
    def setUpClass(cls):
        csv_path = rospy.get_param("~csv_path")
        max_sim_duration = float(rospy.get_param("~max_sim_duration", 900.0))
        publish_hz = float(rospy.get_param("~publish_hz", 10.0))

        rospy.loginfo("test_csv_replay: loading %s", csv_path)
        cls.grass_map = GrassMap(csv_path)
        rospy.loginfo("test_csv_replay: path_length=%.1fm rows=%d",
                      cls.grass_map.total_length, len(cls.grass_map.x))

        status_pub = rospy.Publisher("/ll/mower_status", Status, queue_size=10)
        hl_pub = rospy.Publisher("mower_logic/current_state",
                                 HighLevelStatus, queue_size=10)

        latest_target_speed = {"v": SPEED_MAX}

        def _log_cb(msg):
            fields = parse_log(msg.data)
            ts = fields.get("target_speed")
            if ts is not None:
                try:
                    latest_target_speed["v"] = float(ts)
                except ValueError:
                    pass

        rospy.Subscriber("/blade_speed_adapter/log", String, _log_cb)

        # Wait for the adapter + FTC stack to come up and publish at least one log
        rospy.loginfo("test_csv_replay: waiting for adapter log topic...")
        start_wait = rospy.Time.now()
        while (rospy.Time.now() - start_wait).to_sec() < 60.0:
            if latest_target_speed["v"] != SPEED_MAX:
                break
            # Publish priming state so adapter has fresh data
            cls._publish_state(status_pub, hl_pub, 0.0, SPEED_MAX)
            rospy.sleep(0.1)

        # Arclength-driven replay loop
        s = 0.0
        dt = 1.0 / publish_hz
        motor = MotorModel(dt=dt)
        sim_start = rospy.Time.now()
        ticks = 0
        rospy.loginfo("test_csv_replay: starting replay")
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            elapsed = (now - sim_start).to_sec()
            if s >= cls.grass_map.total_length:
                rospy.loginfo("test_csv_replay: reached end of path at t=%.1fs", elapsed)
                break
            if elapsed > max_sim_duration:
                rospy.logwarn("test_csv_replay: sim budget %.0fs exhausted at s=%.1fm / %.1fm",
                              max_sim_duration, s, cls.grass_map.total_length)
                break

            _, _, grass = cls.grass_map.sample(s)
            robot_speed = latest_target_speed["v"]
            rpm, current = motor.step(grass, robot_speed)

            cls._publish_state(status_pub, hl_pub, current, rpm)

            cls.trace.append({
                "t": elapsed,
                "s": s,
                "grass": grass,
                "rpm": rpm,
                "current": current,
                "target_speed": robot_speed,
            })

            s += robot_speed * dt
            ticks += 1
            rospy.sleep(dt)

        rospy.loginfo("test_csv_replay: %d ticks recorded, final s=%.1fm",
                      ticks, s)

        # Dump trace for debugging; location is /tmp so it doesn't pollute the repo.
        dump_path = rospy.get_param("~trace_dump", "/tmp/csv_replay_trace.csv")
        try:
            with open(dump_path, "w") as f:
                f.write("t,s,grass,rpm,current,target_speed\n")
                for t in cls.trace:
                    f.write("%.3f,%.3f,%.3f,%.1f,%.3f,%.3f\n" % (
                        t["t"], t["s"], t["grass"], t["rpm"], t["current"], t["target_speed"]))
            rospy.loginfo("test_csv_replay: trace written to %s", dump_path)
        except Exception as e:
            rospy.logwarn("test_csv_replay: could not dump trace: %s", e)

    @staticmethod
    def _publish_state(status_pub, hl_pub, current, rpm):
        status = Status()
        status.stamp = rospy.Time.now()
        status.mower_status = 255
        status.mower_esc_status = ESCStatus.ESC_STATUS_RUNNING
        status.mower_esc_current = current
        status.mower_esc_temperature = 35.0
        status.mower_motor_temperature = 25.0
        status.mower_motor_rpm = rpm
        status_pub.publish(status)

        hl = HighLevelStatus()
        hl.state = HighLevelStatus.HIGH_LEVEL_STATE_AUTONOMOUS
        hl.state_name = "MOWING"
        hl_pub.publish(hl)

    # -------------------------------------------------------------------------

    def test_1_replay_captured_data(self):
        """Sanity: the replay actually ran and produced a trace."""
        self.assertGreater(len(self.trace), 50,
                           "Replay did not produce enough ticks (got %d)" % len(self.trace))

    def test_2_nominal_sections_keep_speed_max(self):
        """While traversing quiet sections (grass ~ 0), speed should stay near max."""
        quiet_samples = [t for t in self.trace if t["grass"] < 0.1]
        self.assertGreater(len(quiet_samples), 10,
                           "CSV has too few quiet sections to make this assertion")
        # Take median instead of mean — tolerant to the first few ticks while the
        # adapter is still ramping up.
        speeds = sorted(t["target_speed"] for t in quiet_samples)
        median = speeds[len(speeds) // 2]
        self.assertGreater(
            median, SPEED_MAX - 0.05,
            "Median speed in quiet grass was %.3f m/s (expected near %.3f)"
            % (median, SPEED_MAX))

    def test_3_thick_grass_triggers_slowdown(self):
        """For each thick-grass arclength window, speed must drop below threshold.

        Sustained sags (multi-row) demand near-stall. Single-row spikes only
        demand a meaningful slowdown — physically the robot cannot fully ramp
        down in the sub-second it takes to cross a 0.2m cell at speed_max.
        """
        windows = self.grass_map.thick_arclength_windows(GRASS_THICK_THRESHOLD)
        self.assertGreater(
            len(windows), 0,
            "CSV has no thick-grass windows — is the CSV path correct?")

        failures = []
        for (s_start, s_end, n_rows) in windows:
            sustained = n_rows >= 2
            threshold = SPEED_SUSTAINED_THRESHOLD if sustained else SPEED_SPIKE_THRESHOLD
            # Allow small arclength margin on exit for adapter filter lag.
            dwelled = [t for t in self.trace
                       if s_start <= t["s"] <= s_end + 0.5]
            if not dwelled:
                failures.append("window [%.1f..%.1f] never visited (sim budget too short?)"
                                % (s_start, s_end))
                continue
            min_speed = min(t["target_speed"] for t in dwelled)
            if min_speed > threshold:
                failures.append(
                    "window [%.1f..%.1f]m (%s, %d rows): min target_speed=%.3f (> %.3f)"
                    % (s_start, s_end,
                       "sustained" if sustained else "spike",
                       n_rows, min_speed, threshold))

        self.assertEqual(failures, [],
                         "Adapter failed to slow down in %d of %d thick-grass windows:\n  %s"
                         % (len(failures), len(windows), "\n  ".join(failures)))


if __name__ == "__main__":
    import rostest
    rospy.init_node("test_csv_replay", anonymous=True)
    rostest.rosrun("mower_logic", "test_csv_replay", TestCsvReplay)
