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


def clamp(x, min, max):
    if x < min:
        return min
    if x > max:
        return max
    return x


class Smoother:

    def __init__(self, nelements=4):
        self._ring = [0] * nelements
        self._index = 0

    def add(self, n):
        self._ring[self._index] = n
        self._index += 1
        if self._index == len(self._ring):
            self._index = 0

    def diff(self):
        """Returns the difference between the latest observation and the earliest."""
        # The earliest observation is the ring index we're currently pointing at.
        latest = self._index - 1 if self._index != 0 else len(self._ring) - 1
        return self._ring[latest] - self._ring[self._index]

    def __len__(self):
        return len(self._ring)


class PID:

    def __init__(self, setpoint, kp, ki, kd, min, max, logname=None, logfreq=0.05):
        """Initializes the PID controller.

        setpoint: the target value.
        kp: The proportional coefficient.
        ki: The integral coefficient
        kd: The differential coefficient
        min: The minimum output value.
        max: The maximum output value.
        """
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._min = min
        self._max = max
        self.setpoint = float(setpoint)

        # Data for integral part.
        if self._ki != 0:
            self._errsum = 0
            minmax_diff = max - min
            self._errsum_cap = minmax_diff / self._ki

        # Data for derivative part.
        if self._kd != 0:
            self._err_tracker = Smoother()

        self._log = None
        self._t = 0.0
        if logname is not None:
            self._log = (
                DataLog(
                    "t",
                    "setpoint",
                    "observation",
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
            self._logfreq = logfreq
            self._logindex = 0
            self._until_log = 0.0
            self._t = 0.0

    def update(self, measured, dt):
        """Update the PID controller.

        measured: The measured value of the system.
        dt: The change in time, in seconds.
        """
        err = self.setpoint - measured
        self._t += dt

        p = self._kp * err

        i = 0
        if self._ki != 0:
            self._errsum = clamp(
                self._errsum + err * dt, -self._errsum_cap, self._errsum_cap
            )
            i = self._errsum * self._ki

        d = 0.0
        if self._kd != 0:
            self._err_tracker.add(err)
            derr_dt = self._err_tracker.diff() / (len(self._err_tracker) * dt)
            d = derr_dt * self._kd

        output = clamp(p + i + d, self._min, self._max)

        if self._log is not None:
            self._until_log -= dt
            if self._until_log < 0:
                self._until_log = self._logfreq
                self._log.log(self._t, self.setpoint, measured, p, i, d, output)
                self._logindex += 1

        return output


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
    DRIFT_RATE_UPDATE_FRACTION = 0.000005

    def sample(dt: float):
        nonlocal angle, drift_rate
        raw_speed = gyro.speed()
        speed = raw_speed - drift_rate

        angle += speed * dt

        # Gradually update the drift rate with the average of observed speeds.
        # The assumption here is that the average speed should be zero so to the
        # extent we see non-average speeds, we're drifting. If we found absolutely
        # zero drift during initial calibration, don't update the fraction.
        if drift_rate != 0.0:
            drift_rate = (
                drift_rate * (1 - DRIFT_RATE_UPDATE_FRACTION)
                + speed * DRIFT_RATE_UPDATE_FRACTION
            )

        return angle

    return sample


def main_loop():
    sw = StopWatch()

    get_angle = get_calibrated_angle_speed()
    ev3.speaker.beep()

    # We want the motors to rotate forward one full revolution. I.e. the robot should
    # roll itself off the stand. The travel is the sum of the leg travel so.
    desired_motor_angle_sum = 720

    def motor_angle_sum():
        return lleg.angle() + rleg.angle()

    avg_loop_s = target_loop_ms / 1000

    angle = 0
    motor_angle_history = Smoother()

    def get_smoothed_motor_angular_velocity(dt):
        motor_angle_history.add(motor_angle_sum())
        return motor_angle_history.diff() / (dt * len(motor_angle_history))

    # For initial testing, we want a motorspeed of zero. The angle
    # will controll the motor speed.
    angle_by_motorspeed = PID(
        0, kp=1 / 1500, ki=1 / 100, kd=1 / 10000, min=-15, max=15, logname="anglepid"
    )

    target_angle = 0.0

    def difference_from_desired_angle():
        return target_angle - angle

    pid = PID(
        target_angle, kp=40 / 3, ki=100, kd=1.0, min=-100, max=100, logname="mainpid"
    )

    sw100 = StopWatch()
    loopwatch = StopWatch()
    nloops = 0
    total_loop_time = StopWatch()

    logger = DataLog("desired_speed", name="desired_speed_log", timestamp=False)
    while True:
        angle = get_angle(avg_loop_s)

        # Compute the desired motor angular velocity. We want have a revolution per
        # second toward the desired position when the motor is 4 full revolutions away
        # from the desired position, and no more. We linearly interpolate between that
        # speed and zero when the motor is close to its initial position.
        # (This is just a P controller, but it's simpler to do it here in plain code.)
        revolutions_from_desired_position = (
            desired_motor_angle_sum - motor_angle_sum()
        ) / (4 * 720.0)
        # Note that 360 means half a revolution, since it's the sum of the angles.
        desired_speed = float(clamp(revolutions_from_desired_position * 360, -360, 360))

        angle_by_motorspeed.setpoint = desired_speed
        target_angle = angle_by_motorspeed.update(
            get_smoothed_motor_angular_velocity(avg_loop_s), avg_loop_s
        )

        # If we want to tilt toward the positive angle, we need to drive the wheels
        # in reverse, hence the negative.
        pid.setpoint = target_angle
        output = -round(pid.update(angle, avg_loop_s))

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
        avg_loop_s = total_loop_time.time() / nloops / 1000


if __name__ == "__main__":
    main_loop()
