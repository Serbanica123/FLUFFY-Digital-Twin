import time
import carb
from omni.isaac.core import World
import builtins
import sys

sys.path.append(
    "/home/mechatronica/.local/share/ov/pkg/isaac-sim-4.2.0/FLUFFY/FLUFFYBuilder"
)
from Fluffy import *

# This constant needs to be False for the physics to be initialised. It is not False by default when not making a
# SimulationApp instance.
builtins.ISAAC_LAUNCHED_FROM_TERMINAL = False

# This delay is used troughout the code to time simulation events without blocking the simulation loop.
previous_times = {}


def non_blocking_delay(identifier, interval):
    current_time = time.time()

    # Retrieve the previous_time for this identifier, defaulting to 0 if not present
    if identifier not in previous_times:
        previous_times[identifier] = current_time

    if current_time - previous_times[identifier] >= interval:
        previous_times[identifier] = current_time
        return True
    return False


class StateMachine:
    def __init__(self):
        self.state = "detecting_assembly"  # Initial state
        self.state_functions = {
            "assembly_sensor_detected": on_assembly_sensor_detected,
            "assembly_transfer": on_assembly_transfer,
            "lower_actuator_assembly": on_lower_actuators_assembly,
            "detecting_assembly": on_detecting_assembly,
            "detecting_vision": on_detecting_vision,
            "vision_sensor_detected": on_vision_sensor_detected,
            "vision_transfer": on_vision_transfer,
            "lower_actuator_vision": on_lower_actuators_vision,
        }
        self.transitions = {
            "detecting_assembly": "assembly_sensor_detected",
            "assembly_sensor_detected": "assembly_transfer",
            "assembly_transfer": "lower_actuator_assembly",
            "lower_actuator_assembly": "detecting_vision",
            "detecting_vision": "vision_sensor_detected",
            "vision_sensor_detected": "vision_transfer",
            "vision_transfer": "lower_actuator_vision",
            "lower_actuator_vision": "detecting_assembly",
        }

    def transition(self):
        self.state = self.transitions[self.state]

    def call_state_function(self):
        state_complete = self.state_functions[self.state]()
        if state_complete:
            self.transition()

    def get_current_state(self):
        return self.state


fluffy = FLUFFY(
    False,
    0,
    "/home/mechatronica/.local/share/ov/pkg/isaac-sim-4.2.0/FLUFFY/FLUFFYBuilder/configFLUFFY.json",
)


# State-specific functions
def on_assembly_sensor_detected():
    if non_blocking_delay("seq_1_delay_0", 2):
        fluffy.actuatorController["assembly_lane"].moveJoint(
            "HQ2_Actuator_03", position=1
        )
        fluffy.actuatorController["vision_lane"].moveJoint("HQ2_Actuator", position=1)
        fluffy.actuatorController["vision_lane"].moveJoint(
            "Side_Actuator_03", position=1
        )
        carb.log_info("Assembly Actuators Up")
        return True
    else:
        return False


def on_assembly_transfer():
    if non_blocking_delay("seq_1_delay_1", 1):
        fluffy.conveyorControllers["vision_lane"][
            "Belt_ConveyorBeltGraph_01"
        ].setVelocity(belt_speed)
        fluffy.conveyorControllers["assembly_lane"][
            "Belt_ConveyorBeltGraph_06"
        ].setVelocity(belt_speed)
        carb.log_info("Assembly Belts On")
        return True
    else:
        return False


def on_lower_actuators_assembly():
    if non_blocking_delay("seq_1_delay_2", 7):
        fluffy.conveyorControllers["vision_lane"][
            "Belt_ConveyorBeltGraph_01"
        ].setVelocity(0.0)
        fluffy.conveyorControllers["assembly_lane"][
            "Belt_ConveyorBeltGraph_06"
        ].setVelocity(0.0)
        fluffy.actuatorController["assembly_lane"].moveJoint(
            "HQ2_Actuator_03", position=-1
        )
        fluffy.actuatorController["vision_lane"].moveJoint("HQ2_Actuator", position=-1)
        carb.log_info("Assembly Actuators Down")
        return True
    else:
        return False


def on_detecting_assembly():
    if fluffy.Sensors["assembly_lane"].getSensorOutput(cutoffValue=0.03, sensorID="0"):
        return True
    else:
        return False


def on_detecting_vision():
    if fluffy.Sensors["vision_lane"].getSensorOutput(cutoffValue=0.03, sensorID="3"):
        return True
    else:
        return False


def on_vision_sensor_detected():
    if non_blocking_delay("seq_2_delay_0", 4.45):

        fluffy.actuatorController["assembly_lane"].moveJoint(
            "HQ2_Actuator_04", position=1
        )
        fluffy.actuatorController["vision_lane"].moveJoint(
            "HQ3_Actuator_Top", position=1
        )
        fluffy.actuatorController["vision_lane"].moveJoint(
            "HQ3_Actuator_Bottom", position=1
        )
        fluffy.actuatorController["vision_lane"].moveJoint(
            "Side_Actuator_02", position=1
        )
        carb.log_info("Vision Actuators Up")
        return True
    else:
        return False


def on_vision_transfer():
    if non_blocking_delay("seq_2_delay_1", 1.5):
        fluffy.conveyorControllers["vision_lane"]["Belt_ConveyorBeltGraph"].setVelocity(
            belt_speed
        )
        fluffy.conveyorControllers["assembly_lane"][
            "Belt_ConveyorBeltGraph_07"
        ].setVelocity(belt_speed)
        carb.log_info("Vision Belts On")
        return True
    else:
        return False


def on_lower_actuators_vision():
    if non_blocking_delay("seq_2_delay_2", 8):
        fluffy.conveyorControllers["vision_lane"]["Belt_ConveyorBeltGraph"].setVelocity(
            0.0
        )
        fluffy.conveyorControllers["assembly_lane"][
            "Belt_ConveyorBeltGraph_07"
        ].setVelocity(0.0)
        fluffy.actuatorController["assembly_lane"].moveJoint(
            "HQ2_Actuator_04", position=-1
        )
        fluffy.actuatorController["vision_lane"].moveJoint(
            "HQ3_Actuator_Top", position=-1
        )
        fluffy.actuatorController["vision_lane"].moveJoint(
            "HQ3_Actuator_Bottom", position=-1
        )
        fluffy.actuatorController["vision_lane"].moveJoint(
            "Side_Actuator_02", position=-1
        )
        carb.log_info("Vision Actuators Down")
        previous_times.clear()
        return True
    else:
        return False


# initialise the world.
world = World()
carb.log_info("Starting simulation...")  # Log at INFO level
fluffy.conveyorControllers["assembly_lane"]
# set initial position of actuators and disable all conveyors.
for controller in fluffy.conveyorControllers["assembly_lane"].values():
    controller.setVelocity(0.0)

for controller in fluffy.conveyorControllers["repair_lane"].values():
    controller.setVelocity(0.0)

for controller in fluffy.conveyorControllers["vision_lane"].values():
    controller.setVelocity(0.0)

fluffy.actuatorController["assembly_lane"].resetJointStates()
fluffy.actuatorController["vision_lane"].resetJointStates()
fluffy.actuatorController["repair_lane"].resetJointStates()

# set initial speed of chain conveyors
chain_speed = 0.3
belt_speed = 0.1
fluffy.conveyorControllers["assembly_lane"]["Chain_ConveyorBeltGraph_02"].setVelocity(
    chain_speed
)
fluffy.conveyorControllers["vision_lane"]["Chain_ConveyorBeltGraph"].setVelocity(
    chain_speed
)

# set speed for side chains (in the real system these don't move but for the simulation it is needed to transfer
# carriers between lanes succesfully). these can always be left on.
fluffy.conveyorControllers["repair_lane"]["Side_Chain_ConveyorBeltGraph"].setVelocity(
    belt_speed
)
fluffy.conveyorControllers["repair_lane"][
    "Side_Chain_ConveyorBeltGraph_01"
].setVelocity(belt_speed)
fluffy.conveyorControllers["assembly_lane"][
    "Side_Chain_ConveyorBeltGraph_02"
].setVelocity(belt_speed)
fluffy.conveyorControllers["assembly_lane"][
    "Side_Chain_ConveyorBeltGraph_03"
].setVelocity(belt_speed)

state_machine = StateMachine()

# main simulation loop.
while True:

    if not world.is_playing():
        carb.log_info("ending simulation loop")
        break
    state_machine.call_state_function()
    world.step()

carb.log_info("Done")  # Log before closing
