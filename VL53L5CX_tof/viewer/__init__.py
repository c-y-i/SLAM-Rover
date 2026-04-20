"""Compatibility wrapper for the shared workspace viewer package."""

from pathlib import Path
import sys


_SHARED_ROOT = Path(__file__).resolve().parents[2] / "python_viewers"
if str(_SHARED_ROOT) not in sys.path:
    sys.path.insert(0, str(_SHARED_ROOT))

from sensor_viewers.vl53l5cx_viewer import __version__, main  # noqa: E402,F401
