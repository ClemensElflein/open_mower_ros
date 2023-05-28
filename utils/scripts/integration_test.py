#!/usr/bin/env python3
import shlex, subprocess

import rospy
import time
from mower_msgs.msg import Status
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField

last_status: Status = None
last_imu: Imu = None
last_mag: MagneticField = None

forward_cmd = Twist()
back_cmd = Twist()
stop_cmd = Twist()
forward_cmd.linear.x = 1
back_cmd.linear.x = -1

START_MOWER_MOTOR_CMD = '/opt/ros/noetic/bin/rosservice call /mower_service/mow_enabled "mow_enabled: 1"'
STOP_MOWER_MOTOR_CMD = '/opt/ros/noetic/bin/rosservice call /mower_service/mow_enabled "mow_enabled: 0"'


def callback_status(data):
    global last_status
    last_status = data


def callback_imu(data):
    global last_imu
    last_imu = data


def callback_mag(data):
    global last_mag
    last_mag = data


def test_script():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/mower/status', Status, callback_status)
    rospy.Subscriber('/imu/data_raw', Imu, callback_imu)
    rospy.Subscriber('/imu/mag', MagneticField, callback_mag)

    while not (last_status and last_imu and last_mag):
        time.sleep(0.1)

    rospy.loginfo(last_status)

    # Initial state
    assert not last_status.rain_detected, "Rain sensor is shorting"
    assert last_status.right_esc_status.status == 200, "Right xESC is not 200"
    assert last_status.left_esc_status.status == 200, "Left xESC is not 200"
    assert last_status.mow_esc_status.status == 200, "Mower xESC is not 200"

    assert 14 < last_status.right_esc_status.temperature_pcb < 50, "Right xESC temperature_pcb looks off"
    assert 14 < last_status.left_esc_status.temperature_pcb < 50, "Left xESC temperature_pcb looks off"
    assert 14 < last_status.mow_esc_status.temperature_pcb < 50, "Mower xESC temperature_pcb looks off"

    assert 14 < last_status.mow_esc_status.temperature_motor < 50, "Mower xESC temperature_motor looks off"

    assert last_status.right_esc_status.current == 0, "Right xESC current should be zero"
    assert last_status.left_esc_status.current == 0, "Left xESC current should be zero"
    assert last_status.mow_esc_status.current == 0, "Mower xESC current should be zero"

    test_motors()
    test_accel()
    # test_mag()
    test_rain()
    test_charging()

    exit(0)


def test_rain():
    print("Testing rain sensor, short it with finger or jumper...")
    while not last_status.rain_detected:
        time.sleep(0.01)
    print("Passed")


def test_mag():
    fixed_mag = last_mag
    print("Testing magnetometer, lift rotate mower on all axis, will compare to current position")

    x_done = y_done = z_done = False
    while (True):
        abs_x = abs(last_mag.magnetic_field.x - fixed_mag.magnetic_field.x) / 0.004
        abs_y = abs(last_mag.magnetic_field.y - fixed_mag.magnetic_field.y) / 0.004
        abs_z = abs(last_mag.magnetic_field.z - fixed_mag.magnetic_field.z) / 0.0004

        if abs_x > 1:
            x_done = True
        if abs_y > 1:
            y_done = True
        if abs_z > 1:
            z_done = True

        if x_done and y_done and z_done:
            break

        print(f"x: {abs_x * 100:.2f}%,{x_done} y:{abs_y * 100:.2f}%,{y_done} z:{abs_z * 100:.2f}%,{z_done}")

        time.sleep(0.5)
    print("Passed")


def test_accel():
    print("Testing accelerometer, move mower over axis X")
    while not (abs(last_imu.angular_velocity.x) > 0.1 and abs(last_imu.linear_acceleration.x) > 3):
        time.sleep(0.01)
    print("Testing accelerometer, move mower over axis Y")
    while not (abs(last_imu.angular_velocity.y) > 0.1 and abs(last_imu.linear_acceleration.y) > 3):
        time.sleep(0.01)
    print("Testing accelerometer, move mower over axis Z")
    while not (abs(last_imu.angular_velocity.z) > 0.1 and abs(last_imu.linear_acceleration.z) > 9.8 + 3):
        time.sleep(0.01)
    print("Passed")


def test_charging():
    print("Testing charging controls. Connect both battery and charger, we expect charge current to be >100mA")
    while not (last_status.v_charge > 20 and last_status.v_battery > 20 and last_status.charge_current > 0.1):
        time.sleep(0.01)
    print("Disconnect battery")
    while not (last_status.v_battery < 3):
        time.sleep(0.01)
    print("Connect battery and disconnect charger")
    while not (last_status.v_charge < 20 and last_status.charge_current < 0.1):
        time.sleep(0.01)
    print("Passed")


def test_motors():
    wheels = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    stopped_status = last_status
    for lp in range(100):
        wheels.publish(forward_cmd)
        time.sleep(0.01)
    time.sleep(1)
    rospy.loginfo(last_status)

    try:
        assert last_status.right_esc_status.current > 0.035, "Right xESC current should be more than 0.035"
        assert last_status.left_esc_status.current > 0.035, "Left xESC current should be more than 0.035"

        assert last_status.right_esc_status.tacho != stopped_status.right_esc_status.tacho, "Right xESC tacho should be changing"
        assert last_status.left_esc_status.tacho != stopped_status.left_esc_status.tacho, "Left xESC tacho should be changing"

        # Go back for fun
        for lp in range(100):
            wheels.publish(back_cmd)
            time.sleep(0.01)
    finally:
        for lp in range(100):
            wheels.publish(stop_cmd)
            time.sleep(0.01)
    print("Passed")

    # TODO Potentially could be done cleaner
    # from mower_msgs.srv import MowerControlSrv, MowerControlSrvRequest
    # rospy.wait_for_service('/mower_service/mow_enabled')
    #
    # mower_control_srv = rospy.ServiceProxy('/mower_service/mow_enabled', MowerControlSrv)
    # mower_control_srv(1)
    # mower_control_srv(MowerControlSrvRequest(mow_enabled=0))
    try:
        print("Running the Mower motor")
        subprocess.run(["bash", "-c", START_MOWER_MOTOR_CMD], check=True, capture_output=True)
        time.sleep(1)
        rospy.loginfo(last_status)

        assert last_status.mow_esc_status.current > 0.3, "Mower xESC current should be more than 0.3"
        assert last_status.mow_esc_status.tacho != stopped_status.mow_esc_status.tacho, "Mower xESC tacho should be changing"
    finally:
        subprocess.run(["bash", "-c", STOP_MOWER_MOTOR_CMD], check=True, capture_output=True)

    print("Passed")


if __name__ == '__main__':
    test_script()
