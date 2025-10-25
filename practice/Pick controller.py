import omni
from omni.isaac.dynamic_control import _dynamic_control
import numpy as np
import time

# Acquire the Dynamic Control interface
dc = _dynamic_control.acquire_dynamic_control_interface()

# Retrieve the UR5 articulation (robot arm) from the simulation using its path.
articulation = dc.get_articulation("/UR5")

# Wake up the articulation
dc.wake_up_articulation(articulation)

# Define the target joint angles for pick and place positions.
pick_position = np.array([-1.703, 1.739, 0.595, -2.277, 2.217, -1.661], dtype=np.float32)

# Function to move UR5 to a specific joint position
def move_to_position(target_position):
    print(f"Beweeg naar joint posities: {target_position}")
    # Apply the joint position targets
    dc.set_articulation_dof_position_targets(articulation, target_position)
    
    # Wait for the robot to reach the target position
    # We assume the robot will reach the position in this fixed time interval.

# Start the simulation
omni.timeline.get_timeline_interface().play()  # Start de simulatie

# Beweeg naar de pick positie
move_to_position(pick_position)

# Beweeg naar de place positie

print("Beweging voltooid.")
