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

target_loop_ms = 15


def errderivative(errf):  # Callable[[], int]
    """Yields the change in errf per second over the last window_ms milliseconds, with samples taken every 1000 / sample_hz milliseconds."""
    # We're using 250ms as the window for derr/dt
    bufsize = int(250 / target_loop_ms)
    errring = [errf()] * bufsize
    timering = [0] * bufsize
    i = 0
    sw = StopWatch()
    while True:
        t = sw.time()

        errring[i] = errf()
        timering[i] = t

        windowstart = i + 1
        if windowstart >= len(errring):
            windowstart = 0

        derr = errring[i] - errring[windowstart]
        dt = timering[i] - timering[windowstart]
        if dt == 0:
            yield 0
            continue
        derr_dt = derr * 1000.0 / dt
        i = windowstart
        yield derr_dt


def errintegral(errf):
    sw = StopWatch()
    sum = 0
    while True:
        t = sw.time()
        sw.reset()
        sum += (errf() * t) / 1000.0
        yield sum


def pid_controller(
    errf,  # Callable[[], int]
    kp: float = 0,
    ki: float = 0,
    kd: float = 0,
    printit: bool = False,
):
    derivative = errderivative(errf)
    integral = errintegral(errf)
    index = 0
    while True:
        err = errf()
        p = err * kp
        i = next(integral) * ki
        d = next(derivative) * kd
        pid = p + i + d
        if printit:
            print("%d,%f,%f,%f,%f,%f" % (index, p, i, d, err, pid))
            index += 1
        yield pid


def main_loop():
    sw = StopWatch()

    def motor_total_travel():
        return lleg.angle() + rleg.angle()

    # We want to minimize the total motor angle.
    anglepid = pid_controller(motor_total_travel, kp=0.0025, ki=0.005, printit=True)

    def difference_from_desired_angle():
        desired_angle = next(anglepid)
        print(desired_angle, gyro.angle())
        return gyro.angle() + desired_angle

    pid = pid_controller(
        difference_from_desired_angle,
        kp=15.0,
        ki=60,
        kd=0,
    )

    while sw.time() < 2000:
        output = next(pid)

    ev3.speaker.beep()

    sw100 = StopWatch()
    loopwatch = StopWatch()
    nloops = 0

    while True:
        nloops += 1

        output = int(next(pid))
        lleg.dc(output)
        rleg.dc(output)

        if abs(output) < 100:
            sw100.reset()
        elif sw100.time() >= 1000:
            print("motor at max for 1 sec, we crashed!", nloops)
            return
        if abs(gyro.angle()) > 45:
            print("gyro detected at >45 offset, we crashed!", nloops)
            return
        wait(max(0, target_loop_ms - loopwatch.time()))
        loopwatch.reset()


if __name__ == "__main__":
    main_loop()
