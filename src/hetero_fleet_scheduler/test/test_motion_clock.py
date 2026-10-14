"""A slow or paused simulator must not burn recovery windows in wall time."""
import threading
import time
from types import SimpleNamespace
from test_standoff import make_manager, place, FakeFuture
from hetero_fleet_scheduler.task_model import Pose2D


def test_paused_clock_waits_then_times_out_only_after_simulation_window():
    manager = make_manager()
    place(manager, (0.,0.,0.))
    clock = [20.]
    manager._now = lambda: clock[0]
    manager.stall_timeout_sec = .2
    future, result = FakeFuture(), {}
    worker = threading.Thread(target=lambda: result.update(value=manager._wait_for_result(
        manager.robots['r1'], Pose2D(10.,0.,0.), future)))
    worker.start()
    try:
        time.sleep(.4)  # exceeds the timeout in WALL time with no simulation progress
        assert worker.is_alive() and not result
        clock[0] = 20.3
        worker.join(1.)
        assert not worker.is_alive()
        assert result['value'] == (None, True, 10.)
    finally:
        future.set_result(SimpleNamespace(status=4))
        worker.join(1.)


def test_clock_reset_starts_a_fresh_window():
    manager = make_manager()
    place(manager, (0.,0.,0.))
    clock = [100.]
    manager._now = lambda: clock[0]
    manager.stall_timeout_sec = .2
    future, result = FakeFuture(), {}
    worker = threading.Thread(target=lambda: result.update(value=manager._wait_for_result(
        manager.robots['r1'], Pose2D(10.,0.,0.), future)))
    worker.start()
    try:
        time.sleep(.15)
        clock[0] = 0.
        time.sleep(.25)
        assert worker.is_alive()
        clock[0] = .1
        future.set_result(SimpleNamespace(status=4))
        worker.join(1.)
        assert result['value'][0] == 4 and not result['value'][1]
    finally:
        future.set_result(SimpleNamespace(status=4))
        worker.join(1.)
