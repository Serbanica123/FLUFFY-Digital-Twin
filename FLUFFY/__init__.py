import sys
import os

__all__ = ["Builder", "actuators", "sensors", "conveyors", "Fluffy"]

# Get the parent directory (project root)
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__)))

# Add the parent directory to sys.path if not already present
if parent_dir not in sys.path:
    sys.path.append(parent_dir)
