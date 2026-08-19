#!/usr/bin/env python3
"""Focused, non-ROS-graph unit test for the 2026-08-19 fix to
mpcc_node.py's _active_speed_cap() ramp-hold, which previously only ever
armed once at node startup (self._last_speed_cap_update reset to None only
in __init__) instead of on every GLOBAL -> LOCAL_AVOIDANCE_* transition.

Instantiates the real node (rclpy.init() only, no simulator/topics/solver
needed -- replan_state_callback and _active_speed_cap don't touch the QP)
and drives replan_state_callback()/_active_speed_cap() directly with a fake
clock so ticks advance by a controlled dt instead of real time.

Run inside the ROS2 environment:
    python3 scripts/focused_test_speed_cap_reset.py
"""
import sys
from pathlib import Path

import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from std_msgs.msg import String

sys.path.insert(
    0, str(Path(__file__).resolve().parent.parent
           / 'algorithms' / 'f1tenth_kkh'))

from f1tenth_kkh.mpcc_node import MpccNode  # noqa: E402


class FakeClock:
    def __init__(self, dt_s):
        self.dt = dt_s
        self._now = Time(seconds=1000)

    def tick(self):
        self._now = self._now + Duration(seconds=self.dt)

    def now(self):
        return self._now


def run():
    rclpy.init()
    node = MpccNode()

    dt = 0.1  # matches default control 'dt' param
    fake_clock = FakeClock(dt)
    node.get_clock = lambda: fake_clock

    # Node defaults (max_speed=0.80, avoidance_speed_cap=1.80) don't
    # actually constrain avoidance below max_speed, so a ramp would never
    # be visible. Use the values from mpcc_params_replan_3mps_gentle_sim.yaml
    # (the config this fix is meant for) so the ramp is observable.
    node.max_speed = 3.0
    node.avoidance_speed_cap = 1.20
    node.blocked_speed_cap = 0.0

    failures = []

    def send(state):
        fake_clock.tick()
        node.replan_state_callback(String(data=state))

    def cap_after(n_ticks, decel_per_tick=None):
        caps = []
        for _ in range(n_ticks):
            fake_clock.tick()
            caps.append(node._active_speed_cap())
        return caps

    # --- Test 1: first-ever GLOBAL -> LOCAL_AVOIDANCE_LEFT (boot case,
    # already worked before this fix -- non-regression check). ---
    node.replan_state = 'GLOBAL'
    node._ramped_speed_cap = node.max_speed
    node._last_speed_cap_update = None
    send('LOCAL_AVOIDANCE_LEFT')
    caps = cap_after(3)
    print(f'Test 1 (first-ever entry): caps over 3 ticks = {caps}')
    if caps[0] != node.max_speed:
        failures.append(
            'Test 1 FAILED: first tick after first-ever avoidance entry '
            'should hold the current cap (%.2f), got %.2f'
            % (node.max_speed, caps[0]))
    if not (caps[1] < caps[0]):
        failures.append(
            'Test 1 FAILED: cap should start ramping down on the tick '
            'after the held one')

    # --- Test 2: THE BUG -- a *second* GLOBAL -> LOCAL_AVOIDANCE_LEFT
    # transition later in the same run must also hold-then-ramp, not skip
    # straight to ramping. ---
    node.replan_state = 'GLOBAL'
    node._ramped_speed_cap = node.max_speed
    send('LOCAL_AVOIDANCE_LEFT')
    cap_after(5)  # let it ramp down and settle during episode 1
    send('GLOBAL')
    cap_after(5)  # back to global, cap ramps back up
    cap_before_episode2 = node._ramped_speed_cap
    send('LOCAL_AVOIDANCE_LEFT')  # episode 2 entry -- the bug scenario
    caps2 = cap_after(3)
    print(f'Test 2 (second episode entry): cap_before={cap_before_episode2:.3f}, '
          f'caps over 3 ticks = {[round(c, 3) for c in caps2]}')
    if caps2[0] != cap_before_episode2:
        failures.append(
            'Test 2 FAILED (the historical bug): second avoidance episode '
            'entry did not hold the cap on its first tick -- got %.3f, '
            'expected the pre-transition cap %.3f unchanged'
            % (caps2[0], cap_before_episode2))

    # --- Test 3: GLOBAL -> BLOCKED -> GLOBAL -> LOCAL_AVOIDANCE_LEFT --
    # the reset must still arm correctly despite the intervening BLOCKED
    # excursion (BLOCKED itself has its own instant-drop, no ramp). ---
    node.replan_state = 'GLOBAL'
    node._ramped_speed_cap = node.max_speed
    send('BLOCKED')
    blocked_caps = cap_after(2)
    if blocked_caps[0] != node.blocked_speed_cap:
        failures.append(
            'Test 3 FAILED: BLOCKED should drop the cap instantly, got %.3f'
            % blocked_caps[0])
    send('GLOBAL')
    cap_after(5)
    cap_before_episode3 = node._ramped_speed_cap
    send('LOCAL_AVOIDANCE_LEFT')
    caps3 = cap_after(3)
    print(f'Test 3 (entry after BLOCKED excursion): '
          f'cap_before={cap_before_episode3:.3f}, '
          f'caps over 3 ticks = {[round(c, 3) for c in caps3]}')
    if caps3[0] != cap_before_episode3:
        failures.append(
            'Test 3 FAILED: avoidance entry immediately after a BLOCKED '
            'excursion did not hold the cap on its first tick')

    rclpy.shutdown()

    if failures:
        print('\n'.join(['', 'FAILURES:'] + failures))
        return 1
    print('\nAll focused speed-cap-reset checks passed.')
    return 0


if __name__ == '__main__':
    sys.exit(run())
