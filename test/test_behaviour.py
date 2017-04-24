import sys
import os
import rospkg
import unittest

rospack = rospkg.RosPack()
mission_control_path = rospack.get_path('mission_control')

sys.path.append("%s/src" % mission_control_path)

import behaviour
from mission_control_utils_cache import Cache

class TestBehaviour(unittest.TestCase):

    beha = None

    def setUp(self):
        self.beha = behaviour.Behaviour()

    def tearDown(self):
        del self.beha

    def test_set_priority(self):
        self.beha.set_priority(3)

        self.assertTrue(self.beha._priority == 3)

    def test_set_priority_range(self):
        self.beha.set_priority(0)

        self.assertFalse(self.beha._priority == 0)
        self.assertTrue(self.beha._priority == 1)

    def test_set_debug_level(self):
        self.beha.set_debug_level(2)

        self.assertTrue(self.beha._debug_level == 2)
        self.assertTrue(Cache.debug_level == 2)

    def test_set_debug_level_range(self):
        self.beha.set_debug_level(6)

        self.assertTrue(self.beha._debug_level == 3)
        self.assertTrue(Cache.debug_level == 3)

        self.beha.set_debug_level(0)

        self.assertTrue(self.beha._debug_level == 1)
        self.assertTrue(Cache.debug_level == 1)

    def test_set_executable(self):
        file_path = os.getcwd() + "/test/statemachine.py"
        success = self.beha.set_executable(file_path)

        self.assertTrue(success)

    def test_set_executable_statemachine_not_found(self):
        file_path = os.getcwd() + "/test/not_statemachine.py"
        success = self.beha.set_executable(file_path)

        self.assertFalse(success)

    def test_set_active(self):
        self.beha.set_active("False and True")

        self.assertEquals(self.beha._active_str, "False and True")

    def test_set_active_empty(self):

        self.beha.set_active("")

        self.assertEquals(self.beha._active_str, "False")

    def test_is_active_false(self):

        self.beha.set_active("False and True")
        active = self.beha.is_active()

        self.assertFalse(active)

    def test_is_active_true(self):

        self.beha.set_active("False or True")
        active = self.beha.is_active()

        self.assertTrue(active)
