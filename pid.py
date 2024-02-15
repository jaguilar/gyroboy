import math
import micropython
from micropython import const


@micropython.viper
def clamp(x: float, min: float, max: float):
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
    if f == 0.0:
        return 0, 0
    max_num_times_input = const(2**31 - 1)
    bestseen = 1
    for log2_denom in range(0):
        denom = 1 << log2_denom
        num = int(f * denom)
        if num * input_magnitude > max_num_times_input:
            break
        relerr = math.fabs(f - (num / denom)) / f
        if relerr < goodrelerr:
            return num, log2_denom
        elif relerr < bestseen:
            bestseen = relerr
    raise Exception(
        "No good int scaler for {0} {1} {2} best seen ratio {3}".format(
            f, input_magnitude, goodrelerr, bestseen
        )
    )


class IntPID:
    """A PID controller that uses integer math and viper to hopefully be a little faster.

    setpoint and err are coerced to integers, so a fixed point representation should be used.
    For example, if the desired output control signal is degrees and the expected range is
    0-360, you can use the same coefficients as a normal pid controller, but multiply the input
    signals (setpoint+observation or dt) by 1000 to get a milli-degree output.

    We generally expect the unit of dt to be milliseconds.
    """

    errbuf_len = const(4)

    def __init__(
        self,
        setpoint: int,
        max_expected_abs_err: float,
        kp: float,
        integral_time: float,  # In terms of dt's units.
        derivative_time: float,
        min: int,
        max: int,
        logname=None,
        maxdt=20,
        logfreq=50,
        logfunc=None,
        maxrelerr=0.001,
    ):
        """Initializes the PID controller.

        setpoint: the target value.
        max_expected_abs_err: the maximum difference between the setpoint and the error that we expect to see. This influences
          The choice of coefficient approximation ratios.
        kp: The proportional coefficient.
        integrator_time: Holding error constant, the hypothetical time it requires i to be equal to p, in terms of dt's unit.
        differentiator_time: Starting from zero and holding the change in error constant, how long it takes for p to be equal to
          d, in terms of dt's unit.
        min: The minimum output value.
        max: The maximum output value.
        logname: The name of a logfile. Created and overwritten if not None.
        logfunc: Callable -- called with t, setpoint, observation, p, i, d, output every logfreq ts.
        maxrelerr: The maximum calculated relative error between the requested k values and their integer ratio approximations.
        """
        self._kp_num, self._kp_shr = make_log2_ratio(
            kp, max_expected_abs_err, maxrelerr
        )

        self._min = min
        self._max = max
        self.setpoint = float(setpoint)

        # Data for integral part.
        ki = 1 / integral_time
        self._ki_num, self._ki_shift = 0, 0
        if ki != 0:
            # Determine the largest possible integrator multiplier we're going to face.
            # Back-calculation will be used to suppress excessively large integrator
            # multipliers when output is saturated.
            # Therefore, the largest integrator multiplier we'll have as input is when
            # p and d are maximally negative and i is maximally positive.
            # The error sum needs to be
            output_range = max - min
            max_i = (
                3 * output_range
            )  # p and d will generally not exceed width of the output range, to reach an output of max, i will rarely need to be more than 3x the range.
            self._max_errsum = int(max_i / ki)
            self._ki_num, self._ki_shr = make_log2_ratio(
                ki, self._max_errsum, maxrelerr
            )

        # Data for derivative part.
        self._kd_num, self._kd_shift = 0, 0
        if derivative_time != 0:
            self._err_buf = bytearray(4 * IntPID.errbuf_len)  # 4 samples, 32 bits wide
            self._t_buf = bytearray(4 * IntPID.errbuf_len)  # 4 samples, 32 bits wide
            self._buf_next = 0
            # Incorporate the errbuf len divide into the ratio.
            derivative_time /= IntPID.errbuf_len
            self._kd_num, self._kd_shr = make_log2_ratio(
                derivative_time, 2 * max_expected_abs_err * maxdt, maxrelerr
            )

        self._t = 0
        self._nextlog = self._logfreq = logfreq
        self._store_intermediate = False
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

    @micropython.viper
    def _update(self, measured, dt):
        """Update the PID controller.

        measured: The measured value of the system.
        dt: The change in time, in seconds.
        """

        kp_num, kp_shr, ki_num, ki_shr, kd_num, kd_shr = (
            int(self._kp_num),
            int(self._kp_shr),
            int(self._ki_num),
            int(self._ki_shr),
            int(self._kd_num),
            int(self._kd_shr),
        )

        err = self.setpoint - measured
        p = 0
        if kp_num != 0:
            p = (err * kp_num) >> kp_shr

        i = 0
        if ki_num != 0:
            max_errsum = int(self._max_errsum)
            min_errsum = 0 - max_errsum
            errsum = self._errsum
            errsum += err * dt
            if errsum > max_errsum:
                errsum = max_errsum
            if errsum < min_errsum:
                errsum = min_errsum
            self._errsum = errsum
            i = (errsum * ki_num) >> ki_shr

        d = 0
        if kd_num != 0:
            err_buf, t_buf = ptr32(self._err_buf), ptr32(self._t_buf)
            buf_next = int(self._buf_next)
            t_buf[buf_next] = dt
            err_buf[buf_next] = err

            # Make buf_next point to the oldest value -- the one to next be overwritten.
            buf_next += 1
            if buf_next > IntPID.errbuf_len:
                buf_next = 0

            self._buf_next = buf_next

            derr = err - err_buf[err_buf]
            dt = sum(t_buf)

            d = derr * kd_num / (dt << kd_shr)

        output = p + i + d
        if output < self._min:
            output = self._min
        if output > self._max:
            output = self._max

        if self._store_intermediate:
            self._p, self._i, self._d = p, i, d

        return output

    def update(self, measured, dt):
        if self._logfunc:
            self._t += dt
            if self._t > self._nextlog:
                self._nextlog += self._logfreq
                self._store_intermediate = True

        output = self._update(measured, dt)

        if self._logfunc is not None and self._store_intermediate:
            self._logfunc(
                self._t, self.setpoint, measured, self._p, self._i, self._d, output
            )
            self._store_intermediate = False

        return output
