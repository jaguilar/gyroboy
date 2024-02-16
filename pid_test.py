#!/usr/bin/env pybricks-micropython

import math

try:
    import pytest
except:
    pass
from pid import PID, IntPID, make_log2_ratio

import csv


class WaterHeater:
    j_per_g_c = 4.18
    room_temp = 22  # celcius
    loss_per_delta_c = 0.5 / (30 * 60 * 60)  # Half a degree per hour when at full temp.
    max_power_w = 1125

    def __init__(self, temp_c, mass_kg):
        self.temp_c = temp_c
        self._mass_kg = mass_kg

    def step(self, power_w, dt_s):
        energy = power_w * dt_s
        self.temp_c += energy / (WaterHeater.j_per_g_c * self._mass_kg * 1000)
        self.temp_c -= (
            (self.temp_c - WaterHeater.room_temp) * WaterHeater.loss_per_delta_c * dt_s
        )


def test_pid():
    pid = PID(
        setpoint=54.5,
        kp=25,
        ki=1,
        kd=10,
        min=0,
        max=1125,
        logname="pid.csv",
        logfreq=600,
    )
    wh = WaterHeater(10, 75 * 3.785)
    for _ in range(24 * 360):
        wh.step(pid.update(wh.temp_c, 10), 1)
    print(wh.temp_c)


def test_intpid():

    pid = IntPID(
        setpoint=545,
        kp=250,
        max_expected_abs_err=400,
        integral_time=60,
        derivative_time=6,
        min=0,
        max=11250,
        maxdt=1000,
        logname="intpid.csv",
        logfreq=600,
    )
    wh = WaterHeater(10, 75 * 3.785)
    for _ in range(24 * 36):
        wh.step(pid.update(round(10 * wh.temp_c), 100) / 10, 100)
    print(wh.temp_c)


def scaler_ratio_relerror(f, input_magnitude, maxerr=0.001):
    num, log2_denom = make_log2_ratio(f, input_magnitude, maxerr)
    print(num, log2_denom)
    return math.fabs((float(num) / (1 << log2_denom)) - f) / f


def test_make_int_scaler():
    assert scaler_ratio_relerror(1.5, 1000) < 0.001
    assert scaler_ratio_relerror(1.775, 2415) < 0.001
    assert scaler_ratio_relerror(112414.775, 5) < 0.001
    # When the numbers to be scaled are large, we might lose some precision.
    did_except = False
    try:
        scaler_ratio_relerror(112414.775, 15165, 0.0000001)
    except:
        did_except = True
    assert did_except
    assert scaler_ratio_relerror(112414.775, 15165, 0.0001) < 0.0001


if __name__ == "__main__":
    test_intpid()
