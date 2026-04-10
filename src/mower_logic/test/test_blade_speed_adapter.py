#!/usr/bin/env python3
"""
Integration test for blade_speed_adapter node.

Launched via blade_speed_adapter.test with the full simulation stack.
The test verifies:
1. The node starts and publishes diagnostics on its log topic
2. When idle (not mowing), the adapter does not change planner speed
3. When mowing with low current, speed stays at max
4. When mowing with high current (thick grass), speed decreases
5. When mowing stops, speed is restored to original
"""

import unittest
import rospy
import dynamic_reconfigure.client
from std_msgs.msg import String
from mower_msgs.msg import ESCStatus, HighLevelStatus, Status


def wait_for(condition_fn, timeout_sec, poll_hz=10):
    """Poll condition_fn() until it returns True or timeout."""
    rate = rospy.Rate(poll_hz)
    deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if condition_fn():
            return True
        rate.sleep()
    return False


def parse_log(msg_data):
    """Parse 'key=value;key=value' log string into dict."""
    result = {}
    for part in msg_data.split(";"):
        if "=" in part:
            k, v = part.split("=", 1)
            result[k] = v
    return result


class TestBladeSpeedAdapter(unittest.TestCase):
    """Integration tests for the blade_speed_adapter node."""

    @classmethod
    def setUpClass(cls):
        """Wait for the simulation stack and adapter to be ready."""
        cls.log_messages = []
        cls.log_sub = rospy.Subscriber(
            "/blade_speed_adapter/log", String, cls._log_cb)

        # Wait for the adapter's log topic to appear (node is alive)
        rospy.loginfo("Waiting for blade_speed_adapter log topic...")
        wait_for(lambda: len(cls.log_messages) > 0 or
                 rospy.Time.now() > rospy.Time(0), 30)

        # Publishers to fake mower state
        cls.status_pub = rospy.Publisher(
            "/ll/mower_status", Status, queue_size=10)
        cls.hl_pub = rospy.Publisher(
            "mower_logic/current_state", HighLevelStatus, queue_size=10)

        # Wait a bit for publishers to register
        rospy.sleep(2.0)

    @classmethod
    def _log_cb(cls, msg):
        cls.log_messages.append(msg.data)

    def _clear_logs(self):
        """Clear collected logs for a fresh assertion window."""
        self.__class__.log_messages = []

    def _get_ftc_speed_fast(self):
        """Read current speed_fast from FTC planner via dynamic_reconfigure."""
        try:
            client = dynamic_reconfigure.client.Client(
                "/move_base_flex/FTCPlanner", timeout=10)
            config = client.get_configuration()
            return config.get("speed_fast", None)
        except Exception as e:
            rospy.logwarn("Could not read FTC config: %s" % e)
            return None

    def _make_esc_status(self, running, current=1.0):
        """Create an ESCStatus message."""
        esc = ESCStatus()
        esc.status = ESCStatus.ESC_STATUS_RUNNING if running else ESCStatus.ESC_STATUS_OK
        esc.current = current
        esc.temperature_motor = 25.0
        esc.temperature_pcb = 35.0
        esc.tacho = 0
        return esc

    def _publish_mowing_state(self, blade_current=1.0):
        """Publish fake status and high-level state as if mowing."""
        status = Status()
        status.stamp = rospy.Time.now()
        status.mower_status = 255  # MOWER_STATUS_OK
        status.mow_esc_status = self._make_esc_status(True, blade_current)
        status.left_esc_status = self._make_esc_status(True, 0.5)
        status.right_esc_status = self._make_esc_status(True, 0.5)
        status.v_battery = 26.0
        status.v_charge = 0.0
        status.charge_current = 0.0
        self.status_pub.publish(status)

        hl = HighLevelStatus()
        hl.state = HighLevelStatus.HIGH_LEVEL_STATE_AUTONOMOUS
        hl.state_name = "MOWING"
        self.hl_pub.publish(hl)

    def _publish_idle_state(self):
        """Publish fake status and high-level state as if idle."""
        status = Status()
        status.stamp = rospy.Time.now()
        status.mower_status = 255
        status.mow_esc_status = self._make_esc_status(False, 0.0)
        status.left_esc_status = self._make_esc_status(False, 0.0)
        status.right_esc_status = self._make_esc_status(False, 0.0)
        status.v_battery = 26.0
        status.v_charge = 0.0
        status.charge_current = 0.0
        self.status_pub.publish(status)

        hl = HighLevelStatus()
        hl.state = HighLevelStatus.HIGH_LEVEL_STATE_IDLE
        hl.state_name = "IDLE"
        self.hl_pub.publish(hl)

    # ------------------------------------------------------------------
    # Test 1: Node starts and captures original FTC config
    # ------------------------------------------------------------------

    def test_1_node_starts(self):
        """blade_speed_adapter should start and connect to FTC planner."""
        speed = self._get_ftc_speed_fast()
        self.assertIsNotNone(speed, "Could not read FTC planner speed_fast")
        self.assertAlmostEqual(speed, 0.4, places=1,
                               msg="Initial speed_fast should be ~0.4")

    # ------------------------------------------------------------------
    # Test 2: Idle state — adapter should not change speed
    # ------------------------------------------------------------------

    def test_2_idle_no_speed_change(self):
        """When idle, adapter should not modify planner speed."""
        original_speed = self._get_ftc_speed_fast()
        self.assertIsNotNone(original_speed)

        for _ in range(20):
            self._publish_idle_state()
            rospy.sleep(0.1)

        current_speed = self._get_ftc_speed_fast()
        self.assertAlmostEqual(current_speed, original_speed, places=3,
                               msg="Speed should not change while idle")

    # ------------------------------------------------------------------
    # Test 3: Mowing at nominal current — speed stays at max
    # ------------------------------------------------------------------

    def test_3_nominal_current_max_speed(self):
        """Mowing at nominal current should keep speed_fast at maximum."""
        self._clear_logs()

        # Publish mowing state with nominal current (1.0A)
        for _ in range(20):
            self._publish_mowing_state(blade_current=1.0)
            rospy.sleep(0.1)

        speed = self._get_ftc_speed_fast()
        self.assertIsNotNone(speed)
        self.assertAlmostEqual(speed, 0.4, places=2,
                               msg="speed_fast should be 0.4 at nominal current")

        # Verify log messages were produced
        self.assertGreater(len(self.log_messages), 0,
                           "Should have received log messages while mowing")

        log = parse_log(self.log_messages[-1])
        self.assertIn("current", log, "Log should contain 'current' field")
        self.assertIn("actual_speed", log, "Log should contain 'actual_speed'")
        self.assertIn("load_ratio", log, "Log should contain 'load_ratio'")

    # ------------------------------------------------------------------
    # Test 4: High current — speed should decrease
    # ------------------------------------------------------------------

    def test_4_high_current_reduces_speed(self):
        """When blade current rises (thick grass), speed_fast should decrease."""
        # First mow at nominal to establish baseline
        for _ in range(10):
            self._publish_mowing_state(blade_current=1.0)
            rospy.sleep(0.1)

        # Now raise current to 3.0A (midway between nominal=1 and max_load=5)
        for _ in range(20):
            self._publish_mowing_state(blade_current=3.0)
            rospy.sleep(0.1)

        speed = self._get_ftc_speed_fast()
        self.assertIsNotNone(speed)
        self.assertLess(speed, 0.3,
                        "speed_fast should drop below 0.3 when current is 3.0A")
        self.assertGreater(speed, 0.0,
                           "speed_fast should still be positive")

    # ------------------------------------------------------------------
    # Test 5: Max load current — speed should be near minimum
    # ------------------------------------------------------------------

    def test_5_max_load_minimum_speed(self):
        """When current is at max load (5A), speed should be near minimum."""
        for _ in range(20):
            self._publish_mowing_state(blade_current=5.0)
            rospy.sleep(0.1)

        speed = self._get_ftc_speed_fast()
        self.assertIsNotNone(speed)
        self.assertLess(speed, 0.15,
                        "speed_fast should be < 0.15 at max load current")

    # ------------------------------------------------------------------
    # Test 6: Recovery is gradual
    # ------------------------------------------------------------------

    def test_6_recovery_is_gradual(self):
        """After current drops, speed should increase gradually, not jump."""
        # Drive current up first
        for _ in range(20):
            self._publish_mowing_state(blade_current=5.0)
            rospy.sleep(0.1)

        low_speed = self._get_ftc_speed_fast()
        self.assertIsNotNone(low_speed)

        # Now drop current back to nominal
        self._clear_logs()
        for _ in range(10):
            self._publish_mowing_state(blade_current=1.0)
            rospy.sleep(0.1)

        mid_speed = self._get_ftc_speed_fast()
        self.assertIsNotNone(mid_speed)

        self.assertGreater(mid_speed, low_speed,
                           "Speed should increase when current drops")
        self.assertLess(mid_speed, 0.35,
                        "Speed should recover gradually, not jump to max "
                        "(got %.3f after 1s of recovery)" % mid_speed)

    # ------------------------------------------------------------------
    # Test 7: Mowing stops — speed restored
    # ------------------------------------------------------------------

    def test_7_stop_mowing_restores_speed(self):
        """When mowing stops, speed_fast should be restored to original."""
        # Mow at high current to change speed
        for _ in range(20):
            self._publish_mowing_state(blade_current=4.0)
            rospy.sleep(0.1)

        speed_during = self._get_ftc_speed_fast()
        self.assertLess(speed_during, 0.4,
                        "Speed should be reduced while mowing at high current")

        # Stop mowing
        for _ in range(20):
            self._publish_idle_state()
            rospy.sleep(0.1)

        speed_after = self._get_ftc_speed_fast()
        self.assertIsNotNone(speed_after)
        self.assertAlmostEqual(speed_after, 0.4, places=2,
                               msg="speed_fast should be restored to 0.4 after mowing stops")

    # ------------------------------------------------------------------
    # Test 8: Log format validation
    # ------------------------------------------------------------------

    def test_8_log_format(self):
        """Log messages should contain all expected fields."""
        self._clear_logs()

        for _ in range(10):
            self._publish_mowing_state(blade_current=2.0)
            rospy.sleep(0.1)

        self.assertGreater(len(self.log_messages), 0,
                           "Should have log messages")

        log = parse_log(self.log_messages[-1])
        expected_fields = ["current", "current_avg", "load_ratio",
                           "target_speed", "actual_speed", "mode", "state"]
        for field in expected_fields:
            self.assertIn(field, log,
                          "Log missing field: %s (got: %s)" % (field, log))

        self.assertEqual(log["mode"], "live",
                         "Mode should be 'live' in test configuration")


if __name__ == "__main__":
    import rostest
    rospy.init_node("test_blade_speed_adapter", anonymous=True)
    rostest.rosrun("mower_logic", "test_blade_speed_adapter",
                   TestBladeSpeedAdapter)
