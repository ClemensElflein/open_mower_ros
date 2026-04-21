#!/usr/bin/env python3
"""Accelerated /clock publisher for the CSV-replay rostest.

With /use_sim_time:=true, every ROS node reads time from /clock. We publish
clock ticks at a high wall-rate and advance the sim timestamp by
(wall_dt * speedup) each tick, so all ros::Time-based timers/filters fire
at `speedup` x wall rate. This lets a 9-minute trajectory complete in
~1 minute of wall time without changing any node's tunables.

Params (private):
    ~speedup     : float, sim_time_rate / wall_time_rate  (default 10.0)
    ~wall_hz     : float, publish rate in wall Hz         (default 200.0)
    ~start_time  : float, initial sim time (sec)          (default 1.0)
                   (Non-zero so rospy.Time.now() != Time(0), which some
                   ROS APIs treat as "uninitialized".)
"""

import time

import rospy
from rosgraph_msgs.msg import Clock


def main():
    # Do NOT rospy.init_node() with default args — that would try to read /clock
    # which we are the publisher for. Use disable_rostime to avoid the chicken/egg.
    rospy.init_node("sim_clock", disable_rostime=True, anonymous=False)

    speedup = float(rospy.get_param("~speedup", 10.0))
    wall_hz = float(rospy.get_param("~wall_hz", 200.0))
    sim_time = float(rospy.get_param("~start_time", 1.0))

    pub = rospy.Publisher("/clock", Clock, queue_size=10)

    wall_dt = 1.0 / wall_hz
    sim_step = wall_dt * speedup

    rospy.loginfo("sim_clock: speedup=%.2fx  wall_hz=%.1f  sim_step=%.4fs",
                  speedup, wall_hz, sim_step)

    next_wall = time.monotonic()
    while not rospy.is_shutdown():
        msg = Clock()
        secs = int(sim_time)
        nsecs = int((sim_time - secs) * 1e9)
        msg.clock.secs = secs
        msg.clock.nsecs = nsecs
        pub.publish(msg)

        sim_time += sim_step
        next_wall += wall_dt
        sleep_for = next_wall - time.monotonic()
        if sleep_for > 0.0:
            time.sleep(sleep_for)
        else:
            # We fell behind; resync so we don't spin-lock.
            next_wall = time.monotonic()


if __name__ == "__main__":
    main()
