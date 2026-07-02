#!/usr/bin/env python3
"""Rift CV1 Control Center — entry point.

No arguments → GTK GUI.  With arguments → CLI (see `--help`):
  status | fix-usb | test | export | devices | info | switch-runtime | room
"""
import sys

from riftcv1.main import main

if __name__ == "__main__":
    sys.exit(main())
