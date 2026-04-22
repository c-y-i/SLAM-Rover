#!/usr/bin/env python3
"""Compatibility wrapper for moved rover replay tool."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from py_scripts.rover_tools.replay import main


if __name__ == "__main__":
    main()
