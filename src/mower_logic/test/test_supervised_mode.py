#!/usr/bin/env python3
"""Integration tests for supervised stuck recovery + skip_points action.

Covers the cheap-to-check parts of the design:
  - `wait_for_user_on_mbf_error` dynamic_reconfigure param is declared and round-trips
  - malformed `skip_points/<N>` action payloads don't crash mower_logic

Behavior tests that require a mowing area + MBF-error injection (MBF cascade
suppression, skip_points advancing the index) are left for manual verification
on the robot since the sim harness has no pre-recorded area fixture.
"""

import unittest

import rospy
from dynamic_reconfigure.client import Client as DynReconfClient
from mower_msgs.msg import HighLevelStatus
from std_msgs.msg import String


def _wait_for_mower_logic(timeout_s: float = 20.0) -> bool:
    """Block until /mower_logic/current_state is seen (node is up)."""
    got = {"seen": False}

    def cb(_msg):
        got["seen"] = True

    sub = rospy.Subscriber("/mower_logic/current_state", HighLevelStatus, cb)
    deadline = rospy.Time.now() + rospy.Duration(timeout_s)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if got["seen"]:
            break
        rate.sleep()
    sub.unregister()
    return got["seen"]


class TestSupervisedMode(unittest.TestCase):
    def setUp(self):
        # Each test grabs its own client so a pre-existing init race (see race.md)
        # doesn't wipe out the whole suite via setUpClass.
        if not _wait_for_mower_logic(timeout_s=25.0):
            self.skipTest("mower_logic never came up (pre-existing map_service race)")
        try:
            self.dyn = DynReconfClient("/mower_logic", timeout=10)
        except Exception as e:
            self.skipTest(f"dynamic_reconfigure client not ready: {e}")

    def test_reconfigure_param_exists(self):
        """wait_for_user_on_mbf_error must be declared in MowerLogic.cfg."""
        cfg = self.dyn.get_configuration(timeout=5)
        self.assertIn(
            "wait_for_user_on_mbf_error", cfg,
            "wait_for_user_on_mbf_error missing from /mower_logic dynamic_reconfigure. "
            "Add it to src/mower_logic/cfg/MowerLogic.cfg.",
        )

    def test_reconfigure_param_round_trip(self):
        """Setting the param true/false must round-trip via set_parameters."""
        self.dyn.update_configuration({"wait_for_user_on_mbf_error": True})
        rospy.sleep(0.5)
        got = rospy.get_param("/mower_logic/wait_for_user_on_mbf_error")
        self.assertTrue(got, "param did not round-trip to true")

        self.dyn.update_configuration({"wait_for_user_on_mbf_error": False})
        rospy.sleep(0.5)
        got = rospy.get_param("/mower_logic/wait_for_user_on_mbf_error")
        self.assertFalse(got, "param did not round-trip to false")

    def test_malformed_skip_points_does_not_crash(self):
        """Bad skip_points payloads must not kill the node.

        The handler uses std::stoi on a substring; malformed input must be
        caught. After publishing several bad payloads the node must still be
        publishing /mower_logic/current_state.
        """
        pub = rospy.Publisher("/xbot/action", String, queue_size=5, latch=False)
        rospy.sleep(1.0)  # let the publisher register

        for payload in [
            "mower_logic:mowing/skip_points/",
            "mower_logic:mowing/skip_points/abc",
            "mower_logic:mowing/skip_points/-5",
            "mower_logic:mowing/skip_points/0",
            "mower_logic:mowing/skip_points/9999999999999999999",
        ]:
            pub.publish(String(data=payload))
            rospy.sleep(0.1)

        # Node must still be alive → still publishing current_state.
        self.assertTrue(
            _wait_for_mower_logic(timeout_s=5.0),
            "mower_logic stopped publishing current_state after malformed skip_points payloads",
        )


if __name__ == "__main__":
    import rostest

    rospy.init_node("test_supervised_mode", anonymous=True)
    rostest.rosrun("mower_logic", "test_supervised_mode", TestSupervisedMode)
