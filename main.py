#!/usr/bin/env pybricks-micropython

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
from pybricks.media.ev3dev import SoundFile, ImageFile


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


class Smoother:
    def __init__(self, nelements=10):
        self._ring = [0] * nelements
        self._index = 0

    def update(self, n):
        self._ring[self._index] = n
        self._index += 1
        if self._index == len(self._ring):
            self._index = 0

    def diff(self):
        """Returns the difference between the latest observation and the earliest."""
        latest = self._index - 1 if self._index != 0 else len(self._ring) - 1
        return self._ring[latest] - self._ring[self._index]


def errintegral(errf):
    sw = StopWatch()
    sum = 0
    last_t = 0
    while True:
        t = sw.time()
        dt = t - last_t
        last_t = t
        sum += (errf() * t) / 1000.0
        yield sum


def clamp(x, min, max):
    if x < min:
        return min
    if x > max:
        return max
    return x


def pid_controller(
    errf,  # Callable[[], int]
    initial_value: float = 0.0,
    kp: float = 0,
    ki: float = 0,
    kd: float = 0,
    output_range=(None, None),
    logname=None,
):
    smoothed_err = Smoother()
    smoothed_t = Smoother()

    err_integral = 0.0
    max_abs_err_integral = None
    if ki != 0 and output_range[0] is not None and output_range[1] is not None:
        # Configure a maximum error integral such that the correction applied
        # per 100ms will be the maximum span of the output.
        output_width = output_range[1] - output_range[0]
        max_abs_err_integral = output_width / ki

    # We apply corrections according to dt, which will be in millseconds. But we want
    # callers to reason in terms of per-second rates. Therefore, we divide all our constants
    # by 1000 so that the total correction applied per second is as expected. For example,
    # if the target angle setpoint is 1 and the actual angle is 0 and kp is 1, then we would expect
    # an output adjustment of 1 per second while the error remains fixed, no matter what sample
    # rate we have.
    kp /= 1000.0
    ki /= 1000.0
    kd /= 1000.0

    last_t = 0
    timer = StopWatch()

    log_index = 0
    log_timer = StopWatch()
    output = initial_value
    log = (
        DataLog(
            "index",
            "err",
            "p",
            "i",
            "d",
            "correction",
            "output",
            name=logname,
            timestamp=False,
        )
        if logname
        else None
    )
    while True:
        err = errf()
        t = timer.time()
        dt = t - last_t
        last_t = t

        if dt == 0:
            # We do not update if no time has passed
            yield output

        p = err * kp

        i = 0
        if ki != 0:
            err_integral += err * dt
            if max_abs_err_integral is not None:
                err_integral = clamp(
                    err_integral, -max_abs_err_integral, max_abs_err_integral
                )
            i = ki * err_integral

        d = 0
        if kd != 0:
            # Record the smoothed error and time, then calculate d.
            smoothed_err.update(err)
            smoothed_t.update(t)
            sderr = smoothed_err.diff()
            sdt = smoothed_t.diff()
            d = kd * float(sderr) / sdt

        correction = p + i + d
        output += dt * correction
        output = clamp(output, output_range[0], output_range[1])

        if log and log_timer.time() > 50:
            # Note that we multiply p i d and correction by 1000 to get the per-second
            # values.
            log.log(
                log_index,
                err,
                p * 1000,
                i * 1000,
                d * 1000,
                correction * 1000,
                output,
            )
            log_timer.reset()
            log_index += 1
        yield output


def calibrate_gyro():
    sw = StopWatch()
    start_angle = gyro.angle()
    drift_rate_sum = 0.0
    drift_rate_samples = 0
    while sw.time() < 2000:
        wait(5)
        drift = gyro.angle() - start_angle
        drift_per_milli = float(drift) / sw.time()
        drift_rate_sum += drift_per_milli
        drift_rate_samples += 1
    average_drift_per_milli = drift_rate_sum / drift_rate_samples
    print(average_drift_per_milli)

    sw.reset()
    gyro.reset_angle(0)

    def true_angle():
        return gyro.angle() - sw.time() * average_drift_per_milli

    return true_angle


def main_loop():
    sw = StopWatch()

    # During the time we're going to observe the rate of gyro drift. We will subsequently
    # measure the actual angle as angle() - drift_rate * time.
    angle = calibrate_gyro()
    ev3.speaker.beep()

    # We want the motors to rotate forward one full revolution. I.e. the robot should
    # roll itself off the stand. The travel is the sum of the leg travel so.
    desired_total_travel = 720

    def motor_error():
        return desired_total_travel - (lleg.angle() + rleg.angle())

    # output_range is assuming we started within 10 degrees of vertical.
    anglepid = pid_controller(
        motor_error,
        kp=0.0001,
        ki=0.0000005,
        kd=0,
        logname="anglepid",
        output_range=(-10, 10),
    )

    # Minimize the difference between the total travel of the motors and the desired total
    # travel. This is going to be interpreted as an increment on the desire angle.
    # So, for example, if we haven't travelled as far as we want to, then we want put upward
    # pressure on the target angle.
    #
    # Note that this does not feed directly into the angle error function! This is an *increment*
    # on the target angle each time through the loop. So the values should be fairly small. We
    # structure it this way so that this error can act as a calibration function. If we are steady
    # at our desired level of motor rotation, and the difference between the current angle and the
    # target angle is zero, then this will be zero (ish, not counting the integral component), but
    # the target angle may still be positive, since the robot starts leaning slightly back from its
    # center of gravity.
    target_angle = 0.0

    def difference_from_desired_angle():
        target_angle = 1
        return target_angle - angle()

    pid = pid_controller(
        difference_from_desired_angle,
        kp=25,
        ki=0.1,
        kd=0,
        output_range=(-100, 100),
        logname="mainpid",
    )

    sw100 = StopWatch()
    loopwatch = StopWatch()
    nloops = 0
    total_loop_time = StopWatch()

    while True:
        nloops += 1

        # If we want to tilt toward the positive angle, we need to drive the wheels
        # in reverse, hence the negative.
        target_angle = next(anglepid)
        output = -int(next(pid))
        lleg.dc(round(output))
        rleg.dc(round(output))

        if abs(output) < 100:
            sw100.reset()
        elif sw100.time() >= 1000:
            print(
                "motor at max for 1 sec, we crashed! %f loop time ms avg"
                % (float(total_loop_time.time()) / nloops,)
            )
            return
        if abs(angle()) > 45:
            print(
                "gyro detected at >45 offset, we crashed! %f loop time ms avg"
                % (float(total_loop_time.time()) / nloops,)
            )
            return
        wait(max(0, target_loop_ms - loopwatch.time()))
        loopwatch.reset()


if __name__ == "__main__":
    main_loop()
