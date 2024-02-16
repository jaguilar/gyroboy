#!/usr/bin/env pybricks-micropython

import config
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (
    Motor,
    TouchSensor,
    ColorSensor,
    InfraredSensor,
    UltrasonicSensor,
    GyroSensor,
)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.messaging import Mailbox, BluetoothMailboxServer
from pybricks.media.ev3dev import SoundFile, ImageFile
from pid import PID, IntPID, Smoother, make_log2_ratio, log2_ratio_mul
import micropython
from micropython import const
import struct
import _thread
import utime

# Ports:
# Color: 1
# Touch: 2
# Gyro: 3
# Ultrasound: 4

# Motors
# Rleg: A
# LLeg: D
# Arm motor: C

# Create your objects here.
ev3 = EV3Brick()

color = ColorSensor(Port.S1)
touch = TouchSensor(Port.S2)
gyro = GyroSensor(Port.S3)
ultra = UltrasonicSensor(Port.S4)

rleg = Motor(Port.A)
lleg = Motor(Port.D)
arm = Motor(Port.C)

target_loop_ms = 5


def clamp(x, min, max):
    if x < min:
        return min
    if x > max:
        return max
    return x


ANGLE_SCALE = const(10)  # Decidegrees


# Returns a function that, given a dt, reports the angle of the gyro in millidegrees.
def get_calibrated_angle_speed():
    """Returns a tuple of (angle, speed) of the gyro, both floats."""
    CALIBRATION_TIME_MS = 2000
    CALIBRATION_SAMPLE_TIME_MS = 15
    EXTRA_PRECISION = const(4)

    sum, count = 0, int(CALIBRATION_TIME_MS / CALIBRATION_SAMPLE_TIME_MS)
    for _ in range(count):
        sum += gyro.speed()
        wait(CALIBRATION_SAMPLE_TIME_MS)

    # Within this function, angles and speeds are stored as value << 10.
    # We scale this down to deci-degrees on output.
    calibration_time_s = CALIBRATION_SAMPLE_TIME_MS / 1000
    drift_speed = round(sum << EXTRA_PRECISION) / calibration_time_s
    angle = 0  # Stored as 1024 * angle-in-millidegrees

    # Every time we sample the speed of the gyro, we'll update the average drift
    # as an exponentially weighted moving average of the speed.
    DRIFT_UPDATE_NUM, DRIFT_UPDATE_DENOM = make_log2_ratio(
        0.00005, 90 * EXTRA_PRECISION
    )

    def sample(dt: int):
        nonlocal angle, drift_speed
        speed = gyro.speed() << EXTRA_PRECISION

        if drift_speed != 0:
            # Gradually update the drift rate with the average of observed speeds.
            # The assumption here is that the average speed should be zero so to the
            # extent we see non-average speeds, we're drifting. If we found absolutely
            # zero drift during initial calibration, don't update the fraction.
            speed -= drift_speed
            drift_speed = (
                drift_speed
                - log2_ratio_mul(drift_speed, DRIFT_UPDATE_NUM, DRIFT_UPDATE_DENOM)
            ) + log2_ratio_mul(speed, DRIFT_UPDATE_NUM, DRIFT_UPDATE_DENOM)

        # Unlikely to overflow here -- speed might be on the order of 360000 at the high
        # end and dt could be as much as 100, but that will still be well under 16B.
        angle += (speed * dt) >> EXTRA_PRECISION
        return angle

    return sample


left_right = 0
forward_backward = 0

if False:
    server = BluetoothMailboxServer()
    print("waiting for connection")
    server.wait_for_connection(1)
    mailbox = Mailbox(config.direction_channel, server)


def serve():
    global left_right, forward_backward
    i = 0
    while True:
        data = mailbox.read()
        if data is not None:
            (
                left_right,
                forward_backward,
            ) = struct.unpack_from(config.direction_format, data)
            if i == 10:
                print(left_right, forward_backward)
                i = 0
        i += 1

        wait(100)


def main_loop():
    sw = StopWatch()

    get_angle = get_calibrated_angle_speed()
    ev3.speaker.beep()

    # We want the motors to rotate forward one full revolution. I.e. the robot should
    # roll itself off the stand. The travel is the sum of the leg travel so.
    desired_motor_angle_sum = 720

    # All angles are deci (tenth) degrees.
    # Angular velocities are tenth degrees per second.
    # All times are milliseconds, except ticks, which are microseconds.

    def motor_angle_sum():
        return 1000 * (lleg.angle() + rleg.angle())

    angle = 0
    motor_angle_history = Smoother()
    motor_angle_history_length = len(motor_angle_history)

    def get_smoothed_motor_angular_velocity(dt: int):
        newest = motor_angle_sum()
        oldest = motor_angle_history.oldest()
        motor_angle_history.add(newest)
        # Note: velocity is tenth degrees per second. The angle diff is already in
        # millidegrees. dt is milliseconds, dividing the two gives degrees per second.
        # To get decidegrees, we multiply by 10. This gives us a bit more precision
        # without making us use too low of a gain in the PID.
        return 10 * (newest - oldest) // (motor_angle_history_length * dt)

    # For initial testing, we want a motorspeed of zero. The angle
    # will controll the motor speed.
    angle_by_motorspeed = IntPID(
        0,
        gain=1,
        integral_time=100,  # 100ms
        min=-450,  # -45 degrees
        max=450,  # 45 degrees
        max_expected_abs_err=3600,  # 360 degrees/sec
        logname="anglepid",
    )

    target_angle = 5  # 0.5 deg
    pid = IntPID(
        target_angle,
        gain=100 / 2500,  # We want 100% power when we're off by more than 2.5 degrees
        integral_time=500,  # ms integration time
        derivative_time=0,  # ms derivative time
        min=-100,
        max=100,
        maxdt=100,  # 100ms
        max_expected_abs_err=90000,  # 90 deg
        # logname="mainpid",
    )

    first_t = t = utime.ticks_us()
    last_t_sub100pct = t
    nloops = 0

    while True:
        new_t = utime.ticks_us()
        dt_us = utime.ticks_diff(new_t, t)
        dt_ms = dt_us // 1000
        if dt_ms == 0:
            wait(1)
            continue
        t = utime.ticks_add(t, dt_ms * 1000)

        angle = get_angle(dt_ms)

        # Compute the desired motor angular velocity. We want have a revolution per
        # second toward the desired position when the motor is 4 full revolutions away
        # from the desired position, and no more. We linearly interpolate between that
        # speed and zero when the motor is close to its initial position.
        # (This is just a P controller, but it's simpler to do it here in plain code.)
        if False:
            revolutions_from_desired_position = (
                desired_motor_angle_sum - motor_angle_sum()
            ) / (4 * 720.0)
            # Note that 360 means half a revolution, since it's the sum of the angles.
            desired_speed = float(
                clamp(revolutions_from_desired_position * 360, -360, 360)
            )

        angle_by_motorspeed.setpoint = 0
        target_angle = angle_by_motorspeed.update(
            get_smoothed_motor_angular_velocity(dt_ms), dt_ms
        )

        # If we want to tilt toward the positive angle, we need to drive the wheels
        # in reverse, hence the negative.
        pid.setpoint = target_angle
        output = -pid.update(angle, dt_ms)

        lleg.dc(output)
        rleg.dc(output)

        if abs(output) < 100:
            last_t_sub100pct = t
        elif utime.ticks_diff(t, last_t_sub100pct) > 1000000:
            print(
                "motor at max for 1 sec, we crashed! %d loop time ms avg %d most recent"
                % (utime.ticks_diff(t, first_t) / 1000 / nloops, dt_ms)
            )
            return
        if angle > 20000:
            print(
                "angle limit exceeded, we crashed! %d loop time ms avg %d most recent"
                % (utime.ticks_diff(t, first_t) / 1000 / nloops, dt_ms)
            )
            return

        wait_ms = target_loop_ms - dt_ms
        if wait_ms > 0:
            wait(wait_ms)
        nloops += 1


if __name__ == "__main__":
    # _thread.start_new_thread(serve, ())
    main_loop()
