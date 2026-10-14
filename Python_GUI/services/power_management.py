"""Request that Windows keep the display and system awake while the GUI runs."""
import ctypes
import sys
from ctypes import wintypes


ES_CONTINUOUS = 0x80000000
ES_SYSTEM_REQUIRED = 0x00000001
ES_DISPLAY_REQUIRED = 0x00000002


def enable_awake_mode():
    """Enable Windows keep-awake flags and return a callback that clears them.

    Returns None on non-Windows platforms. The caller must invoke the returned
    callback on the same thread when the keep-awake request should end.
    """
    if sys.platform != "win32":
        return None

    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    set_thread_execution_state = kernel32.SetThreadExecutionState
    set_thread_execution_state.argtypes = [wintypes.DWORD]
    set_thread_execution_state.restype = wintypes.DWORD

    active_flags = ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED
    if not set_thread_execution_state(active_flags):
        raise ctypes.WinError(ctypes.get_last_error())

    def restore_default_power_policy():
        if not set_thread_execution_state(ES_CONTINUOUS):
            raise ctypes.WinError(ctypes.get_last_error())

    return restore_default_power_policy
