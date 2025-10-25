from omni.isaac.core import World
from FLUFFY.FLUFFY.actuators_v2 import customActuatorController
from omni.isaac.core.articulations import ArticulationView


world = World()

# Example articulation view (replace with your actual articulation view)
articulation_view = ArticulationView(prim_paths_expr="/World/FLUFFY/FLUFFY/HQ_Piston")

# Joint names to control
joint_names = ["HQ_Piston_01"]

# Create the controller
Actuators = customActuatorController(articulation_view, joint_names)

# Move Joint1 to position 1 meters
Actuators.resetJointStates()
Actuators.moveJoint("HQ_Piston_01", position=1.0)

"""
# Batch control for multiple joints
joint_targets = {
    "HQ_Piston": {"position": 0},
    "HQ_Belt_01": {"position": 0}
}
   
Actuators.moveJoints(joint_targets)
"""

#world.add_physics_callback("moveActuators", 0.016666666667)

#moveActuators()

# Get the state of Joint1
joint1_state = customActuatorController.getJointState("HQ_Piston")
print("HQ_Piston State: ", joint1_state)


print(articulation_view.num_dof)