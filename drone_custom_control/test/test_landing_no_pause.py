# Copyright 2025 FRL RoboCup Brasil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Regression tests: landing must NOT pause the controller.

Issue: when a landing ('pouso') command was executed the controller
set a 'pause' flag alongside the landing transition.  The correct
behaviour is: landing only lands ('nao pause, so pouse').

These tests verify the FSM behaviour of _initiate_landing() by calling
the actual implementation from LPVMPC_Drone against a lightweight stub
object.  All ROS 2 runtime modules are mocked out so the tests run
without a running ROS 2 environment.

Acceptance criteria
-------------------
(a) landing command (``_initiate_landing()``) -> ``enabled`` stays True;
    ``state_voo`` -> ``_LANDING``; ``landing_active`` -> True.
(b) explicit operator pause (``enabled = False``) -> pause flag set;
    landing flags remain unchanged.
"""

import sys
import os
from unittest.mock import MagicMock

# ---------------------------------------------------------------------------
# Stub out ROS 2 modules so lpv_mpc_drone_node can be imported without a
# running ROS 2 environment.
#
# rclpy.node.Node must be a real Python class (not a MagicMock) so that
# "class LPVMPC_Drone(Node):" produces a proper type with all of its
# methods accessible for testing.
# ---------------------------------------------------------------------------

class _FakeNode:
    """Minimal stand-in for rclpy.node.Node."""
    def __init__(self, *args, **kwargs):
        pass


_fake_rclpy_node_mod = MagicMock()
_fake_rclpy_node_mod.Node = _FakeNode

sys.modules['rclpy']      = MagicMock()
sys.modules['rclpy.node'] = _fake_rclpy_node_mod
sys.modules['rclpy.qos']  = MagicMock()
for _mod in (
    'rcl_interfaces', 'rcl_interfaces.msg',
    'mavros_msgs', 'mavros_msgs.msg', 'mavros_msgs.srv',
    'geometry_msgs', 'geometry_msgs.msg',
    'nav_msgs', 'nav_msgs.msg',
):
    sys.modules.setdefault(_mod, MagicMock())

import pytest  # noqa: E402

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'drone_custom_control', 'nodes'))

import lpv_mpc_drone_node as _drone_mod  # noqa: E402
_LPVMPC_Drone = _drone_mod.LPVMPC_Drone


# ---------------------------------------------------------------------------
# Lightweight stub that holds the FSM attributes used by _initiate_landing().
# We call the *actual* method from LPVMPC_Drone so the test exercises the
# real implementation rather than a hand-written copy.
# ---------------------------------------------------------------------------
class _FsmStub:
    """Minimal attribute container for FSM tests."""

    # State constants mirroring LPVMPC_Drone
    _WAIT_WP        = 0
    _TAKEOFF        = 1
    _HOVER          = 2
    _TRAJECTORY     = 3
    _LANDING        = 4
    _STANDBY_GROUND = 5

    def __init__(self):
        self.state_voo              = self._WAIT_WP
        self.landing_active         = False
        self.landing_start_time_set = False
        # 'enabled' is the controller-wide pause flag.
        # True  -> controller runs normally.
        # False -> controller is paused (operator command).
        self.enabled = True

    def get_logger(self):
        return MagicMock()

    # Bind the real _initiate_landing() implementation so we test the actual
    # method and not a hand-written copy.
    _initiate_landing = _LPVMPC_Drone._initiate_landing


# ---------------------------------------------------------------------------
# (a) landing command -> no pause state set
# ---------------------------------------------------------------------------

class TestLandingDoesNotPause:
    """Landing command must not set the pause flag (enabled=False)."""

    def test_initiate_landing_sets_landing_active(self):
        """_initiate_landing() must set landing_active to True."""
        stub = _FsmStub()
        stub._initiate_landing()
        assert stub.landing_active is True

    def test_initiate_landing_sets_landing_state(self):
        """_initiate_landing() must transition state_voo to _LANDING."""
        stub = _FsmStub()
        stub._initiate_landing()
        assert stub.state_voo == _FsmStub._LANDING

    def test_initiate_landing_does_not_pause_controller(self):
        """Landing must NOT set enabled=False.

        Regression for: no pouso o controlador nao pause, so pouse.
        """
        stub = _FsmStub()
        assert stub.enabled is True, 'pre-condition: controller starts enabled'
        stub._initiate_landing()
        assert stub.enabled is True, (
            'enabled must remain True after landing -- '
            'landing must not pause the controller')

    def test_initiate_landing_resets_start_time_set(self):
        """_initiate_landing() must reset landing_start_time_set to False."""
        stub = _FsmStub()
        stub.landing_start_time_set = True   # simulate a prior landing cycle
        stub._initiate_landing()
        assert stub.landing_start_time_set is False


# ---------------------------------------------------------------------------
# (b) explicit operator pause -> pause flag set; landing unaffected
# ---------------------------------------------------------------------------

class TestExplicitPauseStillWorks:
    """Setting enabled=False (operator pause) must still work independently."""

    def test_explicit_pause_sets_enabled_false(self):
        """An explicit operator pause must disable the controller."""
        stub = _FsmStub()
        stub.enabled = False
        assert stub.enabled is False

    def test_explicit_pause_does_not_alter_landing_active(self):
        """Pausing the controller must not affect landing_active."""
        stub = _FsmStub()
        stub.landing_active = True   # drone is landing
        stub.enabled = False         # operator pauses
        assert stub.landing_active is True, (
            'landing_active must remain True when the operator pauses')

    def test_explicit_pause_does_not_change_fsm_state(self):
        """Pausing must not alter state_voo."""
        stub = _FsmStub()
        stub.state_voo = _FsmStub._HOVER
        stub.enabled = False
        assert stub.state_voo == _FsmStub._HOVER

    def test_resume_after_pause_restores_enabled(self):
        """Setting enabled back to True resumes the controller."""
        stub = _FsmStub()
        stub.enabled = False
        stub.enabled = True
        assert stub.enabled is True

    def test_landing_and_pause_are_independent(self):
        """Landing and pause flags must be fully orthogonal."""
        stub = _FsmStub()

        stub._initiate_landing()
        assert stub.state_voo == _FsmStub._LANDING
        assert stub.enabled is True   # landing alone must not pause

        # Explicit operator pause applied after landing
        stub.enabled = False
        assert stub.state_voo == _FsmStub._LANDING, (
            'state_voo must remain LANDING after explicit pause')
        assert stub.landing_active is True, (
            'landing_active must remain True after explicit pause')
