#!/usr/bin/env pybricks-micropython

import micropython
import uctypes


from pybricks.tools import StopWatch


def clampfloat(x: float, min: float, max: float) -> float:
    if x < min:
        return min
    if x > max:
        return max
    return x


class Container:
    def __init__(self, a: float, b: float, c: float, d: float):
        self.a = a
        self.b = b
        self.c = c
        self.d = d


class IntContainer:
    def __init__(self, a: int, b: int, c: int, d: int):
        self.a = a
        self.b = b
        self.c = c
        self.d = d


def addfloatsthenclamp(c: Container) -> float:
    return clampfloat(c.a + c.b + c.c + c.d, -1.5, 5.5)


descriptor = {
    "a": (0 | uctypes.INT32),
    "b": (4 | uctypes.INT32),
    "c": (8 | uctypes.INT32),
    "d": (12 | uctypes.INT32),
}


@micropython.viper
def clampint(x: int, min: int, max: int) -> int:
    if x < min:
        return min
    if x > max:
        return max
    return x


@micropython.viper
def addintsthenclamp(s):
    return clampint(s.a + s.b + s.c + s.d, 1, 20)


@micropython.viper
def addintsbytearraythenclamp(s) -> int:
    sp = ptr32(s)
    sum = int(sp[0]) + int(sp[1]) + int(sp[2]) + int(sp[3])
    return int(clampint(sum, 1, 20))


@micropython.viper
def addintsviper(s) -> int:
    a = int(s.a)
    b = int(s.b)
    c = int(s.c)
    d = int(s.d)
    sum = a + b + c + d
    return int(clampint(sum, 1, 20))


@micropython.viper
def get(s, i: int) -> int:
    sp = ptr32(s)
    return int(sp[i])


if __name__ == "__main__":
    sw = StopWatch()
    if False:
        cs = (Container(1.2, 2.4, 5.2, 1.2), Container(0.1, 0.2, 0.2, 0.3))
        for _ in range(10000):
            for c in cs:
                addfloatsthenclamp(c)
        print(sw.time())

    ba = bytearray(uctypes.sizeof(descriptor))
    s = uctypes.struct(uctypes.addressof(ba), descriptor)
    s.a, s.b, s.c, s.d = 1, 2, 3, 4

    sw.reset()
    for _ in range(20000):
        addintsthenclamp(s)
    print(sw.time())

    print(ba)
    sw.reset()
    print(get(ba, 0))
    print(get(ba, 1))
    print(get(ba, 2))
    print(get(ba, 3))
    for _ in range(20000):
        addintsbytearraythenclamp(ba)
    print(sw.time())

    sw.reset()
    c = IntContainer(1, 2, 3, 4)
    for _ in range(20000):
        addintsviper(c)
    print(sw.time())
