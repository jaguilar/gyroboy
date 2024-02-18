#!/usr/bin/env pybricks-micropython

import array
import math
import micropython
from micropython import const
import sys

is_micropython = sys.implementation.name == "micropython"

u4_array = lambda x: bytearray(4 * x) if is_micropython else [0] * x

def clamp(x: float, min: float, max: float):
    if x < min:
        return min
    if x > max:
        return max
    return x


class Smoother:

    def __init__(self, nelements=4):
        self._ring = array.array("l", [0] * nelements)
        self._index = 0

    @micropython.native
    def add(self, n):
        index = self._index
        ring = self._ring
        ring[index] = n
        index += 1
        if index == len(ring):
            index = 0
        self._index = index

    @micropython.native
    def oldest(self):
        """Returns the oldest observation."""
        return self._ring[self._index]

    def __len__(self):
        return len(self._ring)


class PID:

    def __init__(
        self, setpoint, kp, ki, kd, min, max, logname=None, logfreq=0.05, logfunc=None
    ):
        """Initializes the PID controller.

        setpoint: the target value.
        kp: The proportional coefficient.
        ki: The integral coefficient
        kd: The differential coefficient
        min: The minimum output value.
        max: The maximum output value.
        logname: The name of a logfile. Created and overwritten if not None.
        logfunc: Callable -- called with t, setpoint, observation, p, i, d, output every logfreq ts.
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
            # Incorporate the length of the error tracker into the _kd to reduce the number
            # of operations required.
            self._kd /= len(self._err_tracker)

        self._log = None
        self._t = 0.0
        self._logfreq = logfreq
        self._until_log = 0.0
        if logfunc:
            self._logfunc = logfunc
        elif logname is not None:
            from pybricks.tools import DataLog

            log = DataLog(
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
            self._logfunc = log.log
        else:
            self._logfunc = None

    def update(self, measured, dt):
        """Update the PID controller.

        measured: The measured value of the system.
        dt: The change in time, in seconds.
        """
        err = self.setpoint - measured
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
            d = (self._err_tracker.diff() / dt) * self._kd

        output = clamp(p + i + d, self._min, self._max)

        if self._logfunc is not None:
            self._t += dt
            self._until_log -= dt
            if self._until_log < 0:
                self._until_log = self._logfreq
                self._logfunc(self._t, self.setpoint, measured, p, i, d, output)

        return output


def make_log2_ratio(f: float, input_magnitude: float, goodrelerr=0.001):
    # We can store values up to 2^31 - 1 (leaving a bit for the sign).
    def error(f, num, denom):
        return math.fabs(f - (num / denom)) / f

    if f == 0.0:
        return 0, 0
    max_num_times_input = const(2147483647)  # 2**32 - 1
    bestnum, bestdenom = int(f), 1
    bestseen = error(f, bestnum, bestdenom)
    for log2_denom in range(32):
        denom = 1 << log2_denom
        num = round(f * denom)
        if num * input_magnitude > max_num_times_input:
            break
        relerr = math.fabs(f - (num / denom)) / f
        if relerr < goodrelerr:
            return num, log2_denom
        elif relerr < bestseen:
            bestseen = relerr
            bestnum = num
            bestdenom = denom
    raise Exception(
        "No good int scaler for {0} {1} {2} best seen ratio {3} {4}/{5}".format(
            f, input_magnitude, goodrelerr, bestseen, bestnum, bestdenom
        )
    )


@micropython.viper
def viper_set(b, off: int, n: int):
    ba = ptr32(b)
    ba[off] = n


@micropython.viper
def viper_get(b, off: int) -> int:
    ba = ptr32(b)
    return int(ba[off])


class ViperArrayAccessor:
    def __init__(self, a: bytearray):
        self._a = a

    def __setitem__(self, key, value):
        viper_set(self._a, key, value)

    def __getitem__(self, key):
        return viper_get(self._a, key)


_DERIV_WINDOW_SAMPLES = const(4)


# Each variable in IntPID is stored in a bytearray for faster access during the update function.
_KP_NUM = const(0)
_KP_SHR = const(1)
_MIN = const(2)
_MAX = const(3)
_SETPOINT = const(4)
_KI_NUM = const(5)
_KI_SHR = const(6)
_ERRSUM = const(7)
_KD_NUM = const(8)
_KD_SHR = const(9)
_BUF_NEXT = const(10)
_T_BUF = const(11)  # Note: 4 32-bit entries
_ERR_BUF = const(15)  # Note: 4 32-bit entries
_STORE_INTERMEDIATE = const(19)
_NUM_VARS = const(20)


class IntPID:
    """A PID controller that uses integer math and viper to hopefully be a little faster.

    setpoint and err are coerced to integers, so a fixed point representation should be used.
    For example, if the desired output control signal is degrees and the expected range is
    0-360, you can use the same coefficients as a normal pid controller, but multiply the input
    signals (setpoint+observation or dt) by 1000 to get a milli-degree output.

    We generally expect the unit of dt to be milliseconds.
    """

    def __init__(
        self,
        setpoint: int,
        max_expected_abs_err: float,
        min: int,
        max: int,
        gain: float,
        integral_time: float = 0,  # In terms of dt's units.
        derivative_time: float = 0,
        logname=None,
        maxdt=20,
        logfreq=50,
        logfunc=None,
        maxrelerr=0.02,
    ):
        """Initializes the PID controller.

        setpoint: the target value.
        max_expected_abs_err: the maximum difference between the setpoint and the error that we expect to see. This influences
          The choice of coefficient approximation ratios.
        gain: The controller gain.
        integrator_time: Holding error constant, the hypothetical time it requires i to be equal to p, in terms of dt's unit.
        differentiator_time: Starting from zero and holding the change in error constant, how long it takes for p to be equal to
          d, in terms of dt's unit.
        min: The minimum output value.
        max: The maximum output value.
        logname: The name of a logfile. Created and overwritten if not None.
        logfunc: Callable -- called with t, setpoint, observation, p, i, d, output every logfreq ts.
        maxrelerr: The maximum calculated relative error between the requested k values and their integer ratio approximations.
        """
        self._data = bytearray(4 * _NUM_VARS)
        self._slow_data_reader = data = ViperArrayAccessor(self._data)

        data[_KP_NUM], data[_KP_SHR] = make_log2_ratio(
            gain, max_expected_abs_err, maxrelerr
        )

        self._intermediates = bytearray(3 * 4)
        self._intermediates_access = ViperArrayAccessor(self._intermediates)

        data[_MIN] = min
        data[_MAX] = max
        data[_SETPOINT] = setpoint

        # Data for integral part.

        data[_KI_NUM], data[_KI_SHR], data[_ERRSUM] = 0, 0, 0
        if integral_time != 0:
            ki = gain / integral_time
            # Determine the largest possible integrator multiplier we're going to face.
            # Back-calculation will be used to suppress excessively large integrator
            # multipliers when output is saturated.
            # Therefore, the largest integrator multiplier we'll have as input is when
            # p and d are maximally negative and i is maximally positive.
            # The error sum needs to be
            output_range = max - min
            max_i = output_range
            max_errsum = int(max_i / ki)
            data[_KI_NUM], data[_KI_SHR] = make_log2_ratio(ki, max_errsum, maxrelerr)

        # Data for derivative part.
        data[_KD_NUM], data[_KD_SHR] = 0, 0
        if derivative_time != 0:
            data[_BUF_NEXT] = 0
            # Incorporate the errbuf len divide into the ratio.
            derivative_time /= _DERIV_WINDOW_SAMPLES
            data[_KD_NUM], data[_KD_SHR] = make_log2_ratio(
                gain * derivative_time, 2 * max_expected_abs_err * maxdt, maxrelerr
            )

        self._nextlog = self._t = 0
        self._logfreq = logfreq
        data[_STORE_INTERMEDIATE] = False
        if logfunc:
            self._logfunc = logfunc
        elif logname is not None:
            from pybricks.tools import DataLog

            log = DataLog(
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
            self._logfunc = lambda *args: log.log(*args)
        else:
            self._logfunc = None

    def set_setpoint(self, newvalue: int):
        self._slow_data_reader[_SETPOINT] = newvalue

    @micropython.viper
    def _update(self, measured: int, dt: int) -> int:
        """Update the PID controller.

        measured: The measured value of the system.
        dt: The change in time, in seconds.
        """
        data = ptr32(self._data)
        err = data[_SETPOINT] - int(measured)
        p = 0
        if data[_KP_NUM] != 0:
            p = err * data[_KP_NUM]
            p >>= data[_KP_SHR]

        i = 0
        errsum = data[_ERRSUM]
        if data[_KI_NUM] != 0:
            errsum += err * dt
            i = errsum * data[_KI_NUM]
            i >>= data[_KI_SHR]

        d = 0
        if data[_KD_NUM] != 0:
            buf_next = data[_BUF_NEXT]
            data[_T_BUF + buf_next] = dt
            data[_ERR_BUF + buf_next] = err

            # Make buf_next point to the oldest value -- the one to next be overwritten.
            buf_next += 1
            if buf_next == _DERIV_WINDOW_SAMPLES:
                buf_next = 0
            data[_BUF_NEXT] = buf_next
            olderr = data[_ERR_BUF + buf_next]
            derr = err - olderr
            dt_deriv = 0
            idx = 0
            while idx < _DERIV_WINDOW_SAMPLES:
                dt_deriv += data[_T_BUF + idx]
                idx += 1
            # Note that shifting right is the same as dividing by a left-shifted value.
            d = derr * data[_KD_NUM]
            d //= dt_deriv << data[_KD_SHR]

        output = p + i + d
        min, max = int(data[_MIN]), int(data[_MAX])
        if output < min:
            output = min
            errsum -= err * dt  # Anti-windup clamping.
        elif output > max:
            output = max
            errsum -= err * dt
        # Write-back errsum.
        data[_ERRSUM] = errsum

        if data[_STORE_INTERMEDIATE] != 0:
            ba = ptr32(self._intermediates)
            ba[0], ba[1], ba[2] = p, i, d
        return output

    def update(self, measured: int, dt: int):
        store_intermediate = False
        if self._logfunc:
            self._t += dt
            if self._t > self._nextlog:
                self._nextlog += self._logfreq
                store_intermediate = True
                self._slow_data_reader[_STORE_INTERMEDIATE] = 1
        output = self._update(measured, dt)
        if self._logfunc is not None and store_intermediate:
            setpoint = self._slow_data_reader[_SETPOINT]
            p = self._intermediates_access[0]
            i = self._intermediates_access[1]
            d = self._intermediates_access[2]
            self._logfunc(
                self._t,
                setpoint,
                measured,
                p,
                i,
                d,
                output,
            )
            self._slow_data_reader[_STORE_INTERMEDIATE] = 0

        return output
