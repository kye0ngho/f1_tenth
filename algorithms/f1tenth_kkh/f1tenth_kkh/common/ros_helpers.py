"""Small rclpy-adjacent helpers shared by all f1tenth_kkh controllers.

Ported from LinearMpcNode.age_seconds / warn_throttled in
algorithms/control/control/linear_mpc_node.py.
"""


def age_seconds(clock, stamp) -> float:
    if stamp is None:
        return float('inf')
    return (clock.now() - stamp).nanoseconds * 1e-9


class WarnThrottle:
    """Logs a warning, deduped by message text within a period_s window."""

    def __init__(self, logger, period_s: float = 2.0):
        self._logger = logger
        self._period_s = period_s
        self._last_message = None
        self._last_time = None

    def warn(self, clock, message: str):
        now = clock.now()
        if (message != self._last_message
                or self._last_time is None
                or (now - self._last_time).nanoseconds
                > self._period_s * 1e9):
            self._logger.warn(message)
            self._last_message = message
            self._last_time = now
