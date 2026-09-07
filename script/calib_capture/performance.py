"""Constant-memory timing counters shared by capture and image workers."""
from __future__ import annotations

import sys
import threading


class Timings:
    def __init__(self) -> None:
        self._values: dict[str, list[int]] = {}
        self._lock = threading.Lock()

    def add(self, stage: str, elapsed_ns: int) -> None:
        with self._lock:
            value = self._values.setdefault(stage, [0, 0, 0])
            value[0] += 1
            value[1] += elapsed_ns
            value[2] = max(value[2], elapsed_ns)

    def summary(self) -> dict:
        with self._lock:
            return {name: {"count": n, "mean_ms": total / n / 1e6,
                           "max_ms": maximum / 1e6, "total_s": total / 1e9}
                    for name, (n, total, maximum) in self._values.items()}


def peak_rss_bytes() -> int | None:
    try:
        import resource
    except ImportError:
        return None
    value = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return int(value if sys.platform == "darwin" else value * 1024)
