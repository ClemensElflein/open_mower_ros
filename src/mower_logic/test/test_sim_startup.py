#!/usr/bin/env python3
"""
Integration test: Verify simulation startup and basic state machine behavior.

Launches the simulator + mower_logic and checks:
1. /mower/status is published (hardware simulation is running)
2. /mower_logic/current_state is published (state machine is running)
3. Initial state is IDLE (state=1) since no mowing area is recorded
"""

import unittest
import rospy
from mower_msgs.msg import HighLevelStatus, Status


class TestSimStartup(unittest.TestCase):
    """Test that the mower simulation starts up correctly."""

    def setUp(self):
        """Initialize ROS node for testing."""
        self.hw_status_received = False
        self.hw_status_msg = None
        self.hl_status_received = False
        self.hl_status_msg = None

    def _hw_status_cb(self, msg):
        self.hw_status_msg = msg
        self.hw_status_received = True

    def _hl_status_cb(self, msg):
        self.hl_status_msg = msg
        self.hl_status_received = True

    def test_hardware_status_published(self):
        """Mower simulation should publish /mower/status within 15 seconds."""
        sub = rospy.Subscriber("/mower/status", Status, self._hw_status_cb)
        timeout = rospy.Time.now() + rospy.Duration(15)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.hw_status_received:
                break
            rate.sleep()
        sub.unregister()

        self.assertTrue(self.hw_status_received,
                        "Did not receive /mower/status within 15 seconds")
        self.assertIsNotNone(self.hw_status_msg)
        # Battery voltage should be > 0 in simulation
        self.assertGreater(self.hw_status_msg.v_battery, 0.0,
                           "Simulated battery voltage should be > 0")

    def test_high_level_state_published(self):
        """Mower logic should publish /mower_logic/current_state within 20 seconds."""
        sub = rospy.Subscriber("/mower_logic/current_state",
                               HighLevelStatus, self._hl_status_cb)
        timeout = rospy.Time.now() + rospy.Duration(20)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.hl_status_received:
                break
            rate.sleep()
        sub.unregister()

        self.assertTrue(self.hl_status_received,
                        "Did not receive /mower_logic/current_state within 20 seconds")
        self.assertIsNotNone(self.hl_status_msg)

    def test_initial_state_is_idle(self):
        """Initial state should be IDLE (1) since no mowing area is configured."""
        sub = rospy.Subscriber("/mower_logic/current_state",
                               HighLevelStatus, self._hl_status_cb)
        timeout = rospy.Time.now() + rospy.Duration(20)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.hl_status_received:
                break
            rate.sleep()
        sub.unregister()

        if not self.hl_status_received:
            self.skipTest("No HighLevelStatus received - cannot check initial state")

        # Mask out sub-state bits (state is in lower 5 bits)
        state = self.hl_status_msg.state & 0x1F
        self.assertEqual(state, HighLevelStatus.HIGH_LEVEL_STATE_IDLE,
                         f"Expected IDLE state (1), got {state}")


if __name__ == "__main__":
    import rostest
    rospy.init_node("test_sim_startup", anonymous=True)
    rostest.rosrun("mower_logic", "test_sim_startup", TestSimStartup)
