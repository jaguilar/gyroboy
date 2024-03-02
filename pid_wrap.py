#! /usr/bin/env pybricks-micropython

import ffi
import uctypes

libpid = ffi.open("libpid.so")

_ptr_t = "p"
_dbl_t = "d"
_int_t = "i"
_void_t = "v"

_pid_new = libpid.func(
    _ptr_t,
    "jags_pid_new",
    # TODO: jaguilar - this should be a single string.
    _dbl_t  # gain
    + _dbl_t  # integration_time
    + _dbl_t  # derivative_time
    + _int_t  # min_output
    + _int_t  # max_output
    + _int_t  # max_expected_err
    + _ptr_t  # logfile name
    + _int_t  # max_dt
    + _dbl_t  # max_ratio_err
    + _int_t,  # logfreq
)

_pid_update = libpid.func(
    _int_t,
    "jags_pid_update",
    _ptr_t + _int_t + _int_t + _int_t,  # pid  # dt  # setpoint  # measurement
)

_pid_free = libpid.func(_void_t, "jags_pid_free", _ptr_t)  # pid


class PID:
    def __init__(
        self,
        max_expected_abs_err: int,
        min: int,
        max: int,
        gain: float,
        integral_time: float = 0,  # In terms of dt's units.
        derivative_time: float = 0,
        logname=None,
        maxdt=20,
        logfreq=50,
        maxrelerr=0.02,
    ):
        logname = bytearray(b"") if logname is None else bytearray(logname + "\0")
        self._pid = _pid_new(
            gain,
            integral_time,
            derivative_time,
            min,
            max,
            max_expected_abs_err,
            logname,
            maxdt,
            maxrelerr,
            logfreq,
        )
        if not self._pid:
            raise Exception("error creating pid")

    def __del__(self):
        _pid_free(self._pid)

    def update(self, dt, setpoint, measurement):
        return _pid_update(self._pid, dt, setpoint, measurement)


def test_intpid():
    class WaterHeater:
        j_per_g_c = 4.18
        room_temp = 22  # celcius
        loss_per_delta_c = 0.5 / (
            30 * 60 * 60
        )  # Half a degree per hour when at full temp.
        max_power_w = 1125

        def __init__(self, temp_c, mass_kg):
            self.temp_c = temp_c
            self._mass_kg = mass_kg

        def step(self, power_w, dt_s):
            energy = power_w * dt_s
            self.temp_c += energy / (WaterHeater.j_per_g_c * self._mass_kg * 1000)
            self.temp_c -= (
                (self.temp_c - WaterHeater.room_temp)
                * WaterHeater.loss_per_delta_c
                * dt_s
            )

    setpoint = 545
    pid = PID(
        gain=250,
        max_expected_abs_err=400,
        integral_time=60,
        derivative_time=6,
        min=0,
        max=11250,
        maxdt=1000,
        logname=b"intpid.csv",
        logfreq=600,
    )
    wh = WaterHeater(10, 75 * 3.785)
    for _ in range(24 * 36):
        wh.step(
            pid.update(dt=100, setpoint=setpoint, measurement=round(10 * wh.temp_c))
            / 10,
            100,
        )
    print(wh.temp_c)


if __name__ == "__main__":
    test_intpid()
