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


def derr_dt_gen(errf, window_ms: int = 25, sample_ms: int = 2):  # Callable[[], int]
    """Yields the change in errf per second over the last window_ms milliseconds, with samples taken every 1000 / sample_hz milliseconds."""
    bufsize = int(window_ms / sample_ms)
    errring = [errf()] * bufsize
    timering = [0] * bufsize
    i = 0
    sw = StopWatch()
    derr_dt = 0
    while True:
        t = sw.time()

        prev = i - 1 if i > 0 else len(timering) - 1
        d_last_sample = t - timering[prev]

        if d_last_sample < sample_ms:
            yield derr_dt
            continue

        errring[i] = errf()
        timering[i] = t

        windowstart = i + 1
        if windowstart >= len(errring):
            windowstart = 0

        derr = errring[i] - errring[windowstart]
        dt = timering[i] - timering[windowstart]
        derr_dt = derr * 1000.0 / dt
        i = windowstart
        yield derr_dt


def errintegral(errf, sample_ms: int = 2):
    sw = StopWatch()
    sum = 0
    while True:
        t = sw.time()
        if t < sample_ms:
            yield sum
        sum += errf() * t / 1000.0
        sw.reset()


def pid_controller(
    errf,  # Callable[[], int]
    pcoeff: float = 0,
    icoeff: float = 0,
    dcoeff: float = 0,
):
    derr_dt = derr_dt_gen(errf)
    integral = errintegral(errf)
    sw = StopWatch()
    while True:
        err = errf()
        p = err * pcoeff
        i = next(integral) * icoeff
        d = next(derr_dt) * dcoeff
        pid = p + i + d
        if sw.time() > 5:
            print(p, i, d, pid)
            sw.reset()
        yield pid


def main_loop():
    sw = StopWatch()
    pid = pid_controller(gyro.angle, pcoeff=20.0, icoeff=1500, dcoeff=0)
    while sw.time() < 2000:
        output = next(pid)

    ev3.speaker.beep()

    sw100 = StopWatch()
    looptime = StopWatch()
    nloops = 0

    # TODO: jaguilar - Install a pid controller for the target angle. Error is distance from desired
    # wheel rotation.

    while True:
        loopdelay = looptime.time()
        nloops += 1
        looptime.reset()

        output = int(next(pid))
        lleg.dc(output)
        rleg.dc(output)

        if abs(output) < 100:
            sw100.reset()
        elif sw100.time() >= 1000:
            print("motor at max for 1 sec, we crashed!", loopdelay, nloops)
            return
        if abs(gyro.angle()) > 45:
            print("gyro detected at >45 offset, we crashed!", loopdelay, nloops)
            return


if __name__ == "__main__":
    main_loop()
