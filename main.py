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
    CALIBRATION_TIME_MS = const(2000)
    CALIBRATION_SAMPLE_TIME_MS = const(15)
    PRECBITS = const(
        10
    )  # Internal state is represented with 10 extra bits of precision.
    SCALE = const(1000)

    # This ratio scales speed into millidegrees/sec in the extra precision level
    # given by PRECBITS. (Dividing by scale gives millidegrees.)
    SPEED_SCALE_NUM, SPEED_SCALE_SHR = make_log2_ratio(PRECBITS / SCALE, 360)

    sum, count = 0, 0
    for _ in range(count):
        sum += gyro.speed() * SCALE
        wait(CALIBRATION_SAMPLE_TIME_MS)

    # drift is degrees/sec (or millidegrees/ms) << PRECBITS
    # Note that during the angle update, this number (if non-zero)
    # is multiplied by dt, which is in milliseconds. That means
    # we can add it to speed, and then multiply the result by dt
    # to get the angle change per loop.
    drift_rate = (sum << PRECBITS) // (CALIBRATION_TIME_MS // 1000)
    print(drift_rate)

    # angle is in millidegrees << PRECBITS
    angle = 0  # Stored as 1024 * angle-in-decidegrees

    # Scale millidegrees << PRECBITS to decidegrees (10), which is our output unit.
    # We are allowing a higher error here -- being off by 5% is not too bad when
    # our total angles are so small and compensated for by PID tuning anyway.
    ANGLE_OUT_NUM, ANGLE_OUT_SHR = make_log2_ratio(
        10 / (SCALE << PRECBITS), 360 * SCALE << PRECBITS, goodrelerr=0.05
    )

    DRIFT_UPDATE_FRAC = 0.0001
    DRIFT_UPDATE_FREQ = const(20)

    until_drift_update = DRIFT_UPDATE_FREQ

    @micropython.native
    def sample(dt: int):
        nonlocal angle, drift_rate, until_drift_update
        # Speed is millidegrees/ms.
        speed = (gyro.speed() * SPEED_SCALE_NUM) >> SPEED_SCALE_SHR

        if drift_rate != 0:
            # Gradually update the drift rate with the average of observed speeds.
            # The assumption here is that the average speed should be zero so to the
            # extent we see non-average speeds, we're drifting. If we found absolutely
            # zero drift during initial calibration, don't update the fraction.
            speed -= drift_rate
            until_drift_update -= 1
            if until_drift_update == 0:
                until_drift_update = DRIFT_UPDATE_FREQ
                new_drift_rate = float(drift_rate)
                new_drift_rate *= 1 - DRIFT_UPDATE_FRAC
                new_drift_rate += DRIFT_UPDATE_FRAC * speed
                drift_rate = int(new_drift_rate)

        angle += speed * dt

        # Scale angle to the output unit, including shifting away the precision.
        return (angle * ANGLE_OUT_NUM) >> ANGLE_OUT_SHR

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

    @micropython.native
    def current_motor_angle_sum() -> int:
        # Motor angles are measured in whole degrees, since there's not
        # as much precision here.
        return lleg.angle() + rleg.angle()

    angle = 0
    motor_angle_history = Smoother()
    motor_angle_history_length = len(motor_angle_history)

    @micropython.native
    def motor_velocity_deg_per_sec(dt: int) -> int:
        newest = current_motor_angle_sum()
        oldest = motor_angle_history.oldest()
        motor_angle_history.add(newest)
        angle_diff = newest - oldest
        # If we're doing less than 1 degree per sec, we'll report zero.
        # We can't actually drive these motors that slow consistently so
        # zero is probably more right than wrong.
        # We're assuming dt is relatively steady when we multiply it here.
        return (
            angle_diff * dt * motor_angle_history_length // 1000
        )  # Note dt is milli so divide by 1000.

    # For initial testing, we want a motorspeed of zero. The angle
    # will control the motor speed.
    angle_by_motorspeed = IntPID(
        0,
        # Tilt 2 degrees for every 100 degrees/sec we're running more than target.
        gain=2 / 100,
        min=-450,  # -45 decidegrees -- target angle is measured in higher precision.
        max=450,  # 45 decidegrees
        max_expected_abs_err=720,  # 360 degrees/sec
        # logname="anglepid",
    )

    target_angle = 0  # 0.5 deg
    pid = IntPID(
        target_angle,
        gain=100 / 250,  # We want 100% power when we're off by more than 2.5 degrees
        integral_time=500,  # ms integration time
        derivative_time=0,  # ms derivative time
        min=-100,
        max=100,
        maxdt=100,  # 100ms
        max_expected_abs_err=90000,  # 90 deg
        logname="mainpid",
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

        angle_decideg = get_angle(dt_ms)

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
            motor_velocity_deg_per_sec(dt_ms), dt_ms
        )

        # If we want to tilt toward the positive angle, we need to drive the wheels
        # in reverse, hence the negative.
        # pid.setpoint = target_angle
        pid.setpoint = 5  # .5 degrees forward
        output = -pid.update(angle_decideg, dt_ms)

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
