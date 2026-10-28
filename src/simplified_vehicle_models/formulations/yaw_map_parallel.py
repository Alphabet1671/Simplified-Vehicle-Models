"""Bounded spawn-based direct yaw-cell evaluation."""
from __future__ import annotations

import multiprocessing as mp
import os
import time
import threading
import traceback
from multiprocessing.connection import wait

from simplified_vehicle_models.formulations.yaw_map_domain import ProbeResult, ProbeStatus


def _evaluate_cell(vehicle, limits, reduced, point):
    if reduced:
        from simplified_vehicle_models.formulations.yaw_map_reduced import direct_reduced_cell
        return direct_reduced_cell(vehicle, *point, limits)
    from simplified_vehicle_models.formulations.yaw_capability import direct_yaw_cell
    return direct_yaw_cell(vehicle, *point, limits)


def _worker_main(connection, parent_sentinel, vehicle, limits, reduced, worker):
    """Serve bounded batches; parent death interrupts even a blocked solver."""
    for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
        os.environ[name] = '1'
    def watch_parent():
        try:
            parent_sentinel.recv_bytes()
        except (EOFError, OSError):
            os._exit(0)
        # A graceful message is used only by an explicit shutdown path; the
        # current owner always closes the pipe, so EOF also means stop now.
        os._exit(0)
    watcher = threading.Thread(target=watch_parent, daemon=True)
    watcher.start()
    try:
        while True:
            request = connection.recv()
            if request is None:
                return
            request_id, points = request
            try:
                values = [worker(vehicle, limits, reduced, point) for point in points]
                connection.send((request_id, True, values))
            except BaseException as error:
                connection.send((request_id, False, (type(error).__module__, type(error).__name__, str(error),
                                                       traceback.format_exc())))
    except (EOFError, OSError):
        return
    finally:
        connection.close()


class ParallelYawEvaluator:
    """Physical evaluator API with ordered bounded batches and parent-owned cache.

    ``workers=1`` uses the serial reference path. Spawn workers receive one
    bounded batch at a time over dedicated pipes; the parent owns all processes.
    """
    def __init__(self, vehicle, tv_limits=None, *, workers=1, time_budget_seconds=None,
                 use_reduced=False, batch_size=None, _worker_function=None):
        if not isinstance(workers, int) or isinstance(workers, bool) or workers < 1:
            raise ValueError('workers must be a positive integer')
        if batch_size is None:
            batch_size = max(1, workers * 2)
        if not isinstance(batch_size, int) or isinstance(batch_size, bool) or batch_size < 1:
            raise ValueError('batch_size must be a positive integer')
        from simplified_vehicle_models.formulations.yaw_map_physical import PhysicalYawEvaluator
        self.vehicle, self.tv_limits = vehicle, tv_limits
        self.workers, self.batch_size = workers, batch_size
        self.use_reduced = use_reduced
        self._worker_function = _worker_function
        self._serial = PhysicalYawEvaluator(vehicle, tv_limits, time_budget_seconds=time_budget_seconds,
                                            use_reduced=use_reduced)
        self.deadline = self._serial.deadline
        self.cells = self._serial.cells
        self._pool = None  # Compatibility-visible tuple of owned Process objects.
        self._connections = []
        self._sentinels = []
        self._closed = False
        self._request_id = 0
        self._pool_lock = threading.RLock()
        self._teardown_lock = threading.Lock()
        self._dispatch_lock = threading.Lock()
        self._close_event = threading.Event()

    def set_deadline(self, deadline):
        """Set or tighten an absolute monotonic deadline (never extend it)."""
        self.deadline = deadline if self.deadline is None else min(self.deadline, deadline)
        self._serial.deadline = self.deadline

    def __enter__(self):
        with self._pool_lock:
            if self._closed:
                raise RuntimeError('parallel evaluator is closed')
            return self._start_pool()

    def _start_pool(self):
        if self._closed:
            raise RuntimeError('parallel evaluator is closed')
        if self.workers > 1 and self._pool is None:
            context = mp.get_context('spawn')
            child_sentinel, parent_sentinel = context.Pipe(duplex=False)
            processes, connections = [], []
            try:
                for _ in range(self.workers):
                    parent_conn, child_conn = context.Pipe(duplex=True)
                    process = context.Process(target=_worker_main,
                        args=(child_conn, child_sentinel, self.vehicle, self.tv_limits,
                              self.use_reduced, self._worker_function or _evaluate_cell))
                    process.start()
                    child_conn.close()
                    processes.append(process)
                    connections.append(parent_conn)
                child_sentinel.close()
            except BaseException:
                child_sentinel.close()
                parent_sentinel.close()
                for connection in connections: connection.close()
                self._stop_processes(processes)
                raise
            self._pool = processes
            self._connections = connections
            self._sentinels = [parent_sentinel]
            self._child_sentinel = child_sentinel
        return self

    @staticmethod
    def _stop_processes(processes):
        for process in processes:
            if process.is_alive():
                process.terminate()
        for process in processes:
            if process.pid is not None:
                process.join(timeout=0.25)
        for process in processes:
            if process.is_alive():
                process.kill()
        for process in processes:
            if process.pid is not None:
                process.join(timeout=1.0)
        alive = [process.pid for process in processes if process.is_alive()]
        if alive:
            raise RuntimeError(f'failed to reap yaw-map workers: {alive}')

    def _terminate_workers(self):
        self.close()

    def _dispatch(self, points):
        from simplified_vehicle_models.formulations.yaw_capability import DirectYawCell
        with self._dispatch_lock:
            with self._pool_lock:
                if self._closed:
                    raise RuntimeError('parallel evaluator is closed')
                if self._pool is None:
                    self._start_pool()
                processes = self._pool
                connections = list(self._connections)
                self._request_id += 1
                request_id = self._request_id
            groups = [[] for _ in processes]
            for index, point in enumerate(points):
                groups[index % len(processes)].append((index, point))
            results = [None] * len(points)
            active = []
            try:
                for index, group in enumerate(groups):
                    if not group:
                        continue
                    connection = connections[index]
                    connection.send((request_id, [point for _, point in group]))
                    active.append((index, group, connection))
                pending = {conn: (index, group) for index, group, conn in active}
                while pending:
                    if self._close_event.is_set():
                        raise RuntimeError('parallel evaluator is closed')
                    remaining = None if self.deadline is None else max(0., self.deadline - time.monotonic())
                    wait_time = .1 if remaining is None else min(.1, remaining)
                    ready = wait(list(pending), timeout=wait_time)
                    for conn in ready:
                        worker_index, group = pending.pop(conn)
                        try:
                            response_id, ok, payload = conn.recv()
                        except (EOFError, OSError) as error:
                            if self._close_event.is_set():
                                raise RuntimeError('parallel evaluator is closed') from error
                            process = processes[worker_index]
                            self.close()
                            raise RuntimeError(f'yaw-map worker {worker_index} exited without a result (exitcode={process.exitcode})') from error
                        if response_id != request_id:
                            self.close()
                            raise RuntimeError('yaw-map worker returned an out-of-order batch')
                        if not ok:
                            module, name, message, formatted = payload
                            self.close()
                            if module == 'builtins':
                                import builtins
                                error_type = getattr(builtins, name, RuntimeError)
                                if isinstance(error_type, type) and issubclass(error_type, BaseException):
                                    raise error_type(message)
                            raise RuntimeError(f'yaw-map worker raised {module}.{name}: {message}\\n{formatted}')
                        if len(payload) != len(group):
                            self.close()
                            raise RuntimeError('yaw-map worker returned an incorrect result count')
                        for (position, _point), value in zip(group, payload):
                            results[position] = value
                    if pending and self.deadline is not None and time.monotonic() >= self.deadline:
                        self.close()
                        detail = 'physical map build time budget exhausted during worker batch'
                        return [value if value is not None else DirectYawCell(ProbeStatus.UNRESOLVED, detail=detail)
                                for value in results]
                return results
            except (BrokenPipeError, EOFError, OSError) as error:
                if not self._close_event.is_set():
                    self.close()
                raise RuntimeError('yaw-map worker communication failed') from error

    def close(self):
        self._close_event.set()
        with self._pool_lock, self._teardown_lock:
            self._closed = True
            processes = self._pool
            if processes is None:
                return
            for connection in self._connections:
                try: connection.close()
                except OSError: pass
            for sentinel in self._sentinels:
                try: sentinel.close()
                except OSError: pass
            self._connections = []
            self._sentinels = []
            self._pool = None
            self._stop_processes(processes)

    def __exit__(self, *_exc):
        self.close()

    def cell(self, speed, ax, ay):
        return self.cells_many(((speed, ax, ay),))[0]

    def cells_many(self, coordinates):
        from simplified_vehicle_models.formulations.yaw_capability import DirectYawCell
        if self._closed:
            raise RuntimeError('parallel evaluator is closed')
        points = [tuple(map(float, p)) for p in coordinates]
        keys = list(points)
        missing = list(dict.fromkeys(k for k in keys if k not in self.cells))
        if self.deadline is not None and time.monotonic() >= self.deadline:
            result = [self.cells.get(k) or DirectYawCell(ProbeStatus.UNRESOLVED,
                      detail='physical map build time budget exhausted') for k in keys]
            self.cells.update({key: cell for key, cell in zip(keys, result) if key not in self.cells})
            return result
        for offset in range(0, len(missing), self.batch_size):
            chunk = missing[offset:offset + self.batch_size]
            if self.workers == 1:
                worker = self._worker_function or _evaluate_cell
                result = [worker(self.vehicle, self.tv_limits, self.use_reduced, p) for p in chunk]
            else:
                if self._closed:
                    raise RuntimeError('parallel evaluator is closed')
                result = self._dispatch(chunk)
            self.cells.update(zip(chunk, result))
            if self.deadline is not None and time.monotonic() >= self.deadline:
                for key in missing[offset + len(chunk):]:
                    self.cells.setdefault(key, DirectYawCell(ProbeStatus.UNRESOLVED,
                        detail='physical map build time budget exhausted before worker batch'))
                break
        return [self.cells[k] for k in keys]

    def probe_many(self, coordinates):
        return [ProbeResult(c.status, c.detail) for c in self.cells_many(coordinates)]

    def __call__(self, speed, ax, ay):
        c = self.cell(speed, ax, ay)
        return ProbeResult(c.status, c.detail)

    def build(self, vehicle, grid, tv_limits=None, *, yaw_inertia):
        if vehicle is not self.vehicle or tv_limits != self.tv_limits:
            raise ValueError('physical cache inputs differ from builder inputs')
        import numpy as np
        from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap, YawCapabilityNumericalError
        shape = (len(grid.speed), len(grid.longitudinal_acceleration), len(grid.lateral_acceleration))
        into, out = np.zeros(shape), np.zeros(shape); mask = np.zeros(shape, bool)
        chunk = []
        start_index = 0
        def assemble(cells, base):
            for offset, cell in enumerate(cells):
                index = base + offset
                i, rem = divmod(index, shape[1]*shape[2]); j, k = divmod(rem, shape[2])
                if cell.status is ProbeStatus.UNRESOLVED:
                    raise YawCapabilityNumericalError(cell.detail)
                if cell.status is ProbeStatus.FEASIBLE:
                    mask[i,j,k] = True; into[i,j,k], out[i,j,k] = cell.into_turn, cell.out_of_turn
        for v in grid.speed:
            for ax in grid.longitudinal_acceleration:
                for ay in grid.lateral_acceleration:
                    chunk.append((float(v), float(ax), float(ay)))
                    if len(chunk) >= self.batch_size:
                        assemble(self.cells_many(chunk), start_index)
                        start_index += len(chunk); chunk = []
        if chunk:
            assemble(self.cells_many(chunk), start_index)
        from simplified_vehicle_models.formulations.point_mass_capability import capability_hash
        kind = 'physical_no_tv_v1' if tv_limits is None else 'physical_tv_v1'
        ident = capability_hash((vehicle, grid, tv_limits, yaw_inertia, 'physical'))
        return YawCapabilityMap(grid.speed, grid.longitudinal_acceleration, grid.lateral_acceleration,
                                into, out, mask, yaw_inertia, kind, ident)
