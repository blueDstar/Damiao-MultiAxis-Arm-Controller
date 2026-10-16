"""Launch with the installed Python, reusing local CAN packages when needed."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def prepare_local_packages() -> None:
    if any(importlib.util.find_spec(name) is None for name in ("can", "serial", "dotenv")):
        local_packages = Path(__file__).resolve().parent / ".venv" / "Lib" / "site-packages"
        if local_packages.is_dir():
            sys.path.insert(0, str(local_packages))


if __name__ == "__main__":
    prepare_local_packages()
    from multi_motor.gui import main

    main()
