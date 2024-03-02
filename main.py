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
from pid import Smoother
from pid_wrap import PID
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

_TARGET_LOOP_MS = const(5)


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
    angle = 0  # Stored as angle-in-decidegrees

    @micropython.native
    def sample(dt: int):
        nonlocal angle
        # Modulo the bonus precision, speed is multiplied by 10 (decidegrees/sec).
        speed = gyro.speed()
        angle += speed * dt
        # Scale angle to the output unit, including shifting away the precision.
        return angle // 100

    return sample


left_right = 0
forward_backward = 0


# Honestly the controller doesn't work that well. It has a detectable effect, but
# I haven't been able to find a tune of the PID controller that is both responsive
# and doesn't lead to wild oscillations. Possibly because the system we're controlling
# is not very linear and PID is a linear controller? I don't know.
def serve():
    global left_right, forward_backward

    server = BluetoothMailboxServer()
    print("waiting for connection")
    server.wait_for_connection(1)
    mailbox = Mailbox(config.direction_channel, server)

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
                i = 0
        i += 1

        wait(100)


def main_loop():
    sw = StopWatch()

    get_angle = get_calibrated_angle_speed()
    ev3.speaker.beep()

    _DEGREES_PER_REVOLUTION = const(720)

    # We want the motors to rotate forward a bit to get the machine off its pylon.
    # Because of the latencies in the controllers, this doesn't work as well as
    # you might hope, at least not on my unlevel basement floor.
    desired_motor_angle_sum = 2 * _DEGREES_PER_REVOLUTION

    def current_motor_angle_sum() -> int:
        # Motor angles are measured in whole degrees, since there's not
        # as much precision here.
        return int(lleg.angle()) + int(rleg.angle())

    motor_angle_history = Smoother()
    motor_angle_history_length = len(motor_angle_history)

    _MAX_REQUESTED_SPEED = const(400)  # 200 degrees per second per wheel.

    def motor_velocity_deg_per_sec(dt: int) -> int:
        newest = int(current_motor_angle_sum())
        oldest = int(motor_angle_history.oldest())
        motor_angle_history.add(newest)
        angle_diff = newest - oldest
        # If we're doing less than 1 degree per sec, we'll report zero.
        # We can't actually drive these motors that slow consistently so
        # zero is probably more right than wrong.
        return angle_diff * 1000 // (dt * int(motor_angle_history_length))

    angle_by_motorspeed = PID(
        gain=1.25 / 100,
        integral_time=800,
        derivative_time=100,
        min=-150,  # -45 decidegrees -- target angle is measured in higher precision.
        max=150,  # 45 decidegrees
        max_expected_abs_err=_MAX_REQUESTED_SPEED * 4,
        # logname="anglepid",
    )

    target_angle = 5
    pid = PID(
        gain=1.5,
        integral_time=190,  # ms integration time
        derivative_time=20,  # ms derivative time
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

        # If forward is 100, then we'll ask for 200 degrees per second forward.
        angle_decideg = get_angle(dt_ms)
        desired_speed = forward_backward * 2

        current_motor_sum = current_motor_angle_sum()
        if desired_speed < 5:
            # No speed request, try to stay at the same position.
            sum_diff = desired_motor_angle_sum - current_motor_sum

            millirevolutions_err = sum_diff * 1000 // _DEGREES_PER_REVOLUTION

            _1_PCT_SPEED_MILLIREVOLUTIONS = 30  # 3 revolutions for max / 100 pct
            _1_PCT_MAX_SPEED = 4

            desired_speed = (
                millirevolutions_err * _1_PCT_MAX_SPEED // _1_PCT_SPEED_MILLIREVOLUTIONS
            )
            if desired_speed > _MAX_REQUESTED_SPEED:
                desired_speed = _MAX_REQUESTED_SPEED
            if desired_speed < -_MAX_REQUESTED_SPEED:
                desired_speed = -_MAX_REQUESTED_SPEED

        else:
            # Speed is controlled by the controller. Update the desired position to
            # our current position.
            desired_motor_angle_sum = current_motor_sum

        target_angle = angle_by_motorspeed.update(
            dt=dt_ms,
            setpoint=int(desired_speed),
            measurement=motor_velocity_deg_per_sec(dt_ms),
        )

        # If we want to tilt toward the positive angle, we need to drive the wheels
        # in reverse, hence the negative.
        output = -pid.update(dt=dt_ms, setpoint=target_angle, measurement=angle_decideg)

        angle_err = target_angle - angle_decideg

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
        if angle_err > 100 or angle_err < -100:
            print(
                "angle limit exceeded, we crashed! %d loop time ms avg %d most recent"
                % (utime.ticks_diff(t, first_t) / 1000 / nloops, dt_ms)
            )
            return

        wait_ms = _TARGET_LOOP_MS - dt_ms
        if wait_ms > 0:
            wait(wait_ms)
        nloops += 1


if __name__ == "__main__":
    _thread.start_new_thread(serve, ())
    main_loop()
