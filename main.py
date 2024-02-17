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


@micropython.viper
def clamp(x: int, min: int, max: int) -> int:
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
    DRIFTPREC = const(10)
    SCALE = const(10)

    sum, count = 0, 0
    for _ in range(count):
        sum += gyro.speed()
        wait(CALIBRATION_SAMPLE_TIME_MS)

    # drift is decidegrees/sec (or millidecidegrees/ms) << PRECBITS
    # Note that during the angle update, this number (if non-zero)
    # is multiplied by dt, which is in milliseconds. That means
    # we can add it to speed, and then multiply the result by dt
    # to get the angle change per loop.
    drift_rate = (sum // (CALIBRATION_TIME_MS // 1000)) << DRIFTPREC
    print(drift_rate)

    # angle is in millidegrees << PRECBITS
    angle = 0  # Stored as 1024 * angle-in-decidegrees

    DRIFT_UPDATE_FRAC = 0.0001
    DRIFT_UPDATE_FREQ = const(20)

    until_drift_update = DRIFT_UPDATE_FREQ

    @micropython.native
    def sample(dt: int):
        nonlocal angle, drift_rate, until_drift_update
        # Modulo the bonus precision, speed is multiplied by 10 (decidegrees/sec).
        speed = gyro.speed()

        if False:  # drift_rate != 0:
            # Gradually update the drift rate with the average of observed speeds.
            # The assumption here is that the average speed should be zero so to the
            # extent we see non-average speeds, we're drifting. If we found absolutely
            # zero drift during initial calibration, don't update the fraction.
            speed -= drift_rate >> DRIFTPREC
            until_drift_update -= 1
            if until_drift_update == 0:
                until_drift_update = DRIFT_UPDATE_FREQ
                new_drift_rate = float(drift_rate)
                new_drift_rate *= 1 - DRIFT_UPDATE_FRAC
                new_drift_rate += DRIFT_UPDATE_FRAC * (speed << DRIFTPREC)
                drift_rate = int(new_drift_rate)

        angle += speed * dt

        # Scale angle to the output unit, including shifting away the precision.
        return angle // 100

    return sample


left_right = 0
forward_backward = 0


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
            left_right //= 10  # Proportional drive max 10%
            forward_backward = 0 - forward_backward
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

    @micropython.viper
    def current_motor_angle_sum() -> int:
        # Motor angles are measured in whole degrees, since there's not
        # as much precision here.
        return int(lleg.angle()) + int(rleg.angle())

    angle = 0
    motor_angle_history = Smoother()
    motor_angle_history_length = len(motor_angle_history)

    @micropython.viper
    def motor_velocity_deg_per_sec(dt: int) -> int:
        newest = int(current_motor_angle_sum())
        oldest = int(motor_angle_history.oldest())
        motor_angle_history.add(newest)
        angle_diff = newest - oldest
        # If we're doing less than 1 degree per sec, we'll report zero.
        # We can't actually drive these motors that slow consistently so
        # zero is probably more right than wrong.

        return angle_diff * 1000 // (dt * int(motor_angle_history_length))

    # For initial testing, we want a motorspeed of zero. The angle
    # will control the motor speed.
    angle_by_motorspeed = IntPID(
        0,
        # Tilt 2 degrees for every 100 degrees/sec we're running more than target.
        gain=0.75 / 100,
        integral_time=500,
        derivative_time=75,
        min=-150,  # -45 decidegrees -- target angle is measured in higher precision.
        max=150,  # 45 decidegrees
        max_expected_abs_err=720,  # 360 degrees/sec
        # logname="anglepid",
    )

    target_angle = 5
    pid = IntPID(
        target_angle,
        gain=1.25,  # We want 100% power when we're off by more than .75 degrees
        integral_time=125,  # ms integration time
        derivative_time=50,  # ms derivative time
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

        forward_rate = forward_backward * 7 * dt_ms // 1000
        desired_motor_angle_sum += forward_rate

        angle_decideg = get_angle(dt_ms)

        # Compute the desired motor angular velocity. We want have a revolution per
        # second toward the desired position when the motor is 4 full revolutions away
        # from the desired position, and no more. We linearly interpolate between that
        # speed and zero when the motor is close to its initial position.
        # (This is just a P controller, but it's simpler to do it here in plain code.)

        deci_revolutions_from_desired_position = (
            10 * (desired_motor_angle_sum - current_motor_angle_sum()) // (4 * 720)
        )
        # Note that 360 means half a revolution, since it's the sum of the angles.
        desired_speed = clamp(deci_revolutions_from_desired_position * 40, -720, 720)

        angle_by_motorspeed.setpoint = int(desired_speed)
        target_angle = angle_by_motorspeed.update(
            motor_velocity_deg_per_sec(dt_ms), dt_ms
        )

        # If we want to tilt toward the positive angle, we need to drive the wheels
        # in reverse, hence the negative.
        pid.setpoint = target_angle
        # pid.setpoint = 5  # .5 degrees forward
        output = -pid.update(angle_decideg, dt_ms)

        lleg.dc(output + left_right)
        rleg.dc(output - left_right)

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
    _thread.start_new_thread(serve, ())
    main_loop()
