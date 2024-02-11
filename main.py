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


def get_calibrated_angle_speed():
    """Returns a tuple of (angle, speed) of the gyro, both floats."""
    CALIBRATION_TIME_MS = 2000
    CALIBRATION_SAMPLE_TIME_MS = 15

    sum, count = 0, int(CALIBRATION_TIME_MS / CALIBRATION_SAMPLE_TIME_MS)
    for _ in range(count):
        sum += gyro.speed()
        wait(CALIBRATION_SAMPLE_TIME_MS)

    drift_rate = sum / count
    angle = 0

    # Every time we sample the speed of the gyro, we'll update the average drift
    # as an exponentially weighted moving average of the speed, with an alpha of 0.001
    DRIFT_RATE_UPDATE_FRACTION = 0.001

    def sample(dt: float):
        nonlocal angle, drift_rate
        raw_speed = gyro.speed()
        speed = raw_speed - drift_rate

        angle += speed * dt / 1000

        # Gradually update the drift rate with the average of observed speeds.
        # The assumption here is that the average speed should be zero so to the
        # extent we see non-average speeds, we're drifting.
        drift_rate = (
            drift_rate * (1 - DRIFT_RATE_UPDATE_FRACTION)
            + speed * DRIFT_RATE_UPDATE_FRACTION
        )

        return angle

    return sample


class Smoother:

    def __init__(self, nelements=7):
        self._ring = [0] * nelements
        self._index = 0

    def update(self, n):
        self._ring[self._index] = n
        self._index += 1
        if self._index == len(self._ring):
            self._index = 0

    def avg(self):
        """Returns the difference between the latest observation and the earliest."""
        # The earliest observation is the ring index we're currently pointing at.
        latest = self._index - 1 if self._index != 0 else len(self._ring) - 1
        return (self._ring[latest] - self._ring[self._index]) / len(self._ring)


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
    dtf,
    initial_value: float = 0.0,
    kp: float = 0,
    ki: float = 0,
    kd: float = 0,
    output_range=(None, None),
    logname=None,
):
    smoothed_err = Smoother()

    # Note that the integral component is multiplied by dt in milliseconds, which means
    # that without scaling ki down by 1000 it will be of a different scale than kd and kp.
    ki /= 1000.0

    err_integral = 0.0
    max_abs_err_integral = None
    if ki != 0 and output_range[0] is not None and output_range[1] is not None:
        # Configure a maximum error integral such that the correction applied
        # per 100ms will be the maximum span of the output.
        output_width = output_range[1] - output_range[0]
        max_abs_err_integral = output_width / ki

    # Note that our raw derr/dt is calculated per millisecond. To allow the calculation
    # to be of the same scale as ki and kp, multiply by 1000.
    kd *= 1000.0

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
            "output",
            name=logname,
            timestamp=False,
        )
        if logname
        else None
    )

    while True:
        err = errf()

        p = err * kp
        dt = dtf()

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
            smoothed_err.update(err)
            sderr = smoothed_err.avg()
            d = kd * float(sderr) / dt

        output = p + i + d
        output = clamp(output, output_range[0], output_range[1])

        if log and log_timer.time() > 50:
            log.log(
                log_index,
                err,
                p,
                i,
                d,
                output,
            )
            log_timer.reset()
            log_index += 1
        yield output


def main_loop():
    sw = StopWatch()

    get_angle = get_calibrated_angle_speed()
    ev3.speaker.beep()

    # We want the motors to rotate forward one full revolution. I.e. the robot should
    # roll itself off the stand. The travel is the sum of the leg travel so.
    desired_total_travel = 720

    def motor_error():
        return desired_total_travel - (lleg.angle() + rleg.angle())

    avg_loop_ms = target_loop_ms

    angle = 0
    anglelog = DataLog("angle", timestamp=False, name="anglelog")

    # output_range is assuming we started within 10 degrees of vertical.
    anglepid = pid_controller(
        motor_error,
        dtf=lambda: avg_loop_ms,
        kp=1 / 720,
        ki=0,
        kd=0,
        logname="anglepid",
        output_range=(-5, 5),
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
        return target_angle - angle

    pid = pid_controller(
        difference_from_desired_angle,
        dtf=lambda: avg_loop_ms,
        kp=22.5,
        ki=200,
        kd=0.75,
        output_range=(-100, 100),
        logname="mainpid",
    )

    sw100 = StopWatch()
    loopwatch = StopWatch()
    nloops = 0
    total_loop_time = StopWatch()

    while True:
        angle = get_angle(avg_loop_ms)
        anglelog.log(angle)
        target_angle = next(anglepid)

        # If we want to tilt toward the positive angle, we need to drive the wheels
        # in reverse, hence the negative.

        output = -round(next(pid))
        lleg.dc(output)
        rleg.dc(output)

        if abs(output) < 100:
            sw100.reset()
        elif sw100.time() >= 1000:
            print(
                "motor at max for 1 sec, we crashed! %f loop time ms avg"
                % (float(total_loop_time.time()) / nloops,)
            )
            return
        if angle > 20:
            print(
                "angle limit exceeded, we crashed! %f loop time ms avg"
                % (float(total_loop_time.time()) / nloops,)
            )
            return
        wait(max(0, target_loop_ms - loopwatch.time()))
        loopwatch.reset()

        nloops += 1
        avg_loop_ms = total_loop_time.time() / nloops


if __name__ == "__main__":
    main_loop()
