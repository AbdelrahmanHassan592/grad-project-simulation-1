#!/usr/bin/env python3
from pathlib import Path
import sys
# Also support direct execution before building the workspace.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from my_car_description.world_check import main
if __name__ == '__main__': main()
