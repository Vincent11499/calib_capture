"""Bounded image scheduling with one ordered pair-commit coordinator."""
from __future__ import annotations

import queue
import threading
import time
from collections import deque
from dataclasses import dataclass

from .axera import GrayPair
from .dataset import DatasetWriter, PendingPair
from .performance import Timings


class BufferPool:
    """Lazy allocation: 5 Hz sessions normally reuse just one or two slots."""
    def __init__(self, image_bytes: int, capacity: int) -> None:
        self.image_bytes = image_bytes
        self.capacity = capacity
        self._free: list[tuple[bytearray, bytearray]] = []
        self._busy: set[int] = set()
        self.allocated = 0
        self.high_water = 0
        self._lock = threading.Lock()

    def acquire(self) -> tuple[bytearray, bytearray] | None:
        with self._lock:
            if self._free:
                slot = self._free.pop()
            elif self.allocated < self.capacity:
                slot = (bytearray(self.image_bytes), bytearray(self.image_bytes))
                self.allocated += 1
            else:
                return None
            self._busy.add(id(slot))
            self.high_water = max(self.high_water, len(self._busy))
            return slot

    def release(self, slot: tuple[bytearray, bytearray]) -> None:
        with self._lock:
            if id(slot) not in self._busy:
                raise RuntimeError("buffer slot released twice or belongs to another pool")
            self._busy.remove(id(slot))
            self._free.append(slot)

    def summary(self) -> dict:
        with self._lock:
            return {"capacity_pairs": self.capacity, "allocated_pairs": self.allocated,
                    "allocated_bytes": self.allocated * 2 * self.image_bytes,
                    "in_use_pairs": len(self._busy), "high_water_pairs": self.high_water}


@dataclass
class _Work:
    pair: GrayPair
    slot: tuple[bytearray, bytearray] | None
    submitted_ns: int


class PairDiskWriter:
    def __init__(self, dataset: DatasetWriter, capacity: int) -> None:
        self.dataset = dataset
        self.max_active = (dataset.config.image_workers + 1) // 2
        self.queue: queue.Queue[_Work | None] = queue.Queue(maxsize=capacity)
        self.pool = BufferPool(dataset.config.width * dataset.config.height,
                               capacity + self.max_active + 1)
        self.error: BaseException | None = None
        self.written = 0
        self.queue_high_water = 0
        self.active_high_water = 0
        self.drain_s = 0.0
        self.timings = Timings()
        self._finish_started = False
        self._thread = threading.Thread(target=self._run, name="pair-commit", daemon=True)
        self._thread.start()

    def reserve(self) -> tuple[bytearray, bytearray] | None:
        self.raise_if_failed()
        return None if self.full() else self.pool.acquire()

    def full(self) -> bool:
        return self.queue.full()

    def submit(self, pair: GrayPair, slot: tuple[bytearray, bytearray] | None = None) -> bool:
        """A successful submission transfers slot ownership to the coordinator."""
        self.raise_if_failed()
        if self._finish_started:
            raise RuntimeError("image writer is stopping")
        try:
            self.queue.put_nowait(_Work(pair, slot, time.perf_counter_ns()))
            self.queue_high_water = max(self.queue_high_water, self.queue.qsize())
            return True
        except queue.Full:
            return False

    def _release(self, work: _Work) -> None:
        if work.slot is not None:
            self.pool.release(work.slot)
        self.queue.task_done()

    def _complete(self, job: PendingPair, work: _Work) -> None:
        try:
            if self.error is None:
                self.dataset.finish_pair(job)
                self.written += 1
            else:
                self.dataset.cancel_pair(job)
        except BaseException as exc:
            self.error = self.error or exc
        finally:
            self._release(work)

    def _run(self) -> None:
        pending: deque[tuple[PendingPair, _Work]] = deque()
        stopping = False
        while not stopping or pending:
            if pending and (stopping or len(pending) >= self.max_active or pending[0][0].done()):
                self._complete(*pending.popleft())
                continue
            try:
                work = self.queue.get(timeout=0.01 if pending else 0.1)
            except queue.Empty:
                continue
            if work is None:
                self.queue.task_done()
                stopping = True
                continue
            if self.error is not None:
                self._release(work)
                continue
            try:
                self.timings.add("queue_wait", time.perf_counter_ns() - work.submitted_ns)
                pending.append((self.dataset.start_pair(work.pair), work))
                self.active_high_water = max(self.active_high_water, len(pending))
            except BaseException as exc:
                self.error = self.error or exc
                self._release(work)

    def raise_if_failed(self) -> None:
        if self.error is not None:
            raise RuntimeError(f"image writer failed: {self.error}") from self.error
        self.dataset.raise_if_failed()

    def finish(self) -> None:
        start = time.perf_counter()
        if not self._finish_started:
            self._finish_started = True
            self.queue.put(None)
        self._thread.join()
        self.drain_s = max(self.drain_s, time.perf_counter() - start)
        self.raise_if_failed()

    def summary(self) -> dict:
        return {"queue_capacity_pairs": self.queue.maxsize,
                "queue_high_water_pairs": self.queue_high_water,
                "max_active_pairs": self.max_active,
                "active_high_water_pairs": self.active_high_water,
                "buffer_pool": self.pool.summary(), "drain_s": self.drain_s,
                "timings": self.timings.summary()}
