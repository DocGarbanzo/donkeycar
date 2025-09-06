"""
Simple console plotting/logging parts.

This module provides a generic Plotter part that prints values
from the vehicle memory to stdout for quick calibration and debugging.
"""

from typing import Tuple


class Plotter:
    """Minimal console plotter.

    Usage:
        p = Plotter('angle', 'throttle')
        p.run(0.4, 0.5)  -> "angle = +0.400, throttle = +0.500"
        p.run(-0.3)      -> "angle = -0.300"
        p.run(1.1, 2.2, 3.3) -> "angle = +1.100, throttle = +2.200"
    """

    def __init__(self, *names: str):
        self.names: Tuple[str, ...] = tuple(names)
        self._fmt: str = "+4.3f"

    def addFormat(self, fmt: str):
        self._fmt = fmt
        return self

    def run(self, *values):
        count = min(len(values), len(self.names))
        if count == 0:
            return
        out = []
        for i in range(count):
            name = self.names[i]
            val = values[i]
            try:
                sval = format(float(val), self._fmt)
            except Exception:
                sval = str(val)
            out.append(f"{name} = {sval}")
        print(", ".join(out))
