import time
import carb
from omni.isaac.core import World
import builtins
import sys
sys.path.append("/home/mechatronica/.local/share/ov/pkg/isaac-sim-4.2.0/FLUFFY/FLUFFYBuilder")
from Builder import *

# This constant needs to be False for the physics to be initialised. It is not False by default when not making a
# SimulationApp instance.
builtins.ISAAC_LAUNCHED_FROM_TERMINAL = False

# this piece of code demonstrates how to initialize the builder.
# ALWAYS KEEP THE LANES IN THIS SPECIFIC ORDER!
# if the lanes are not in this order the builder won't work
vision_builder = VisionLaneBuilder()
repair_builder = RepairLaneBuilder()
assembly_builder = AssemblyLaneBuilder()
dir = Director()
vision_lane = dir.construct(vision_builder)
repair_lane = dir.construct(repair_builder)
assembly_lane = dir.construct(assembly_builder)

#always reset the builder after constructing
dir.reset(Lane())

#This delay is used troughout the code to time simulation events without blocking the simulation loop.
previous_time=0
def non_blocking_delay(interval):

    current_time = time.time()  
    global previous_time
    if current_time - previous_time >= interval:
        previous_time = current_time  
        return True  
    return False 

# initialise the world.
world = World()
carb.log_info("Starting simulation...")  # Log at WARNING level

#set initial speed of a conveyor
speed=0.5
assembly_lane.conveyorControllers["Chain_ConveyorBeltGraph_02"].setVelocity(speed)

# main simulation loop. now set to 5000 frames but it could be a while loop too.
for i in range(5000):
    
    # always use the non-blocking delay as the condition of an IF statement.
    if non_blocking_delay(3): 

        # set the speed variable to the negative of the previous speed.  
        speed=-speed
        carb.log_info("Changed Direction")

        # set the speed of the conveyor to the updated speed variable
        assembly_lane.conveyorControllers["Chain_ConveyorBeltGraph_02"].setVelocity(speed)

    # whenever the sensor is detected, print to the Isaac Sim console.
    # NOTE: this statement should be OUTSIDE the non_blocking_delay function or it will only work once per delay time 
    # (3 seconds in this case).
    if assembly_lane.sensors.getSensorOutput(cutoffValue=0.03, sensorID="1"):
        carb.log_info("Sensor Detected")
    else:
        carb.log_info("_")
    
    # step the simulation forward by one frame
    world.step()

carb.log_warn("Done")  # Log before closing
