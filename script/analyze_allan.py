"""Standalone entry point, equivalent to calibration_data.py allan."""
import sys
from calib_capture.cli import main

if __name__ == "__main__":
    raise SystemExit(main(["allan", *sys.argv[1:]]))
