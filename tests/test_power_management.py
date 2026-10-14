"""Tests for the Windows keep-awake execution-state request."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch


MODULE_PATH = Path(__file__).resolve().parents[1] / "Python_GUI/services/power_management.py"
SPEC = importlib.util.spec_from_file_location("power_management", MODULE_PATH)
power_management = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(power_management)


class FakeWin32Function:
    def __init__(self):
        self.calls = []
        self.argtypes = None
        self.restype = None

    def __call__(self, flags):
        self.calls.append(flags)
        return 1


class FakeKernel32:
    def __init__(self):
        self.SetThreadExecutionState = FakeWin32Function()


class PowerManagementTests(unittest.TestCase):
    def test_windows_awake_request_holds_display_and_system_then_restores(self):
        kernel32 = FakeKernel32()
        with patch.object(power_management.sys, "platform", "win32"):
            with patch.object(
                power_management.ctypes, "WinDLL", return_value=kernel32, create=True
            ):
                restore = power_management.enable_awake_mode()
                restore()

        self.assertEqual(
            kernel32.SetThreadExecutionState.calls,
            [
                power_management.ES_CONTINUOUS
                | power_management.ES_SYSTEM_REQUIRED
                | power_management.ES_DISPLAY_REQUIRED,
                power_management.ES_CONTINUOUS,
            ],
        )

    def test_non_windows_platform_does_not_load_windows_api(self):
        with patch.object(power_management.sys, "platform", "linux"):
            with patch.object(
                power_management.ctypes,
                "WinDLL",
                side_effect=AssertionError("Windows API must not load"),
                create=True,
            ):
                self.assertIsNone(power_management.enable_awake_mode())


if __name__ == "__main__":
    unittest.main()
