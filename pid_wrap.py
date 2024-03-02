#! /usr/bin/env pybricks-micropython

import ffi
import uctypes

libpid = ffi.open("libpid.so")

_ptr_t = "p"
_flt_t = "f"
_int_t = "i"
_void_t = "v"

_pid_new = libpid.func(
    _ptr_t,
    "jags_pid_new",
    # TODO: jaguilar - this should be a single string.
    _flt_t  # gain
    + _flt_t  # integration_time
    + _flt_t  # derivative_time
    + _int_t  # min_output
    + _int_t  # max_output
    + _int_t  # max_expected_err
    + _ptr_t  # logfile name
    + _int_t  # max_dt
    + _flt_t  # max_ratio_err
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

if __name__ == "__main__":
    test_intpid()
