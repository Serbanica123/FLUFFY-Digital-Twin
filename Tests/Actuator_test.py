from FLUFFY.actuators import *

lane_joints = ['ChainConstraint', 'Stopper_Actuator', 'Stopper_Actuator_01', 'HQ3_Actuator_Bottom', 'Side_Actuator',
           'Stopper_Actuator_02', 'HQ2_Actuator', 'Side_Actuator_01', 'HQ3_Actuator_Bottom_01', 'Stopper_Actuator_03', 
           'Side_Actuator_02', 'HQ3_Actuator_Bottom_02', 'Side_Actuator_03', 'Vision_Actuator_Front', 
           'Vision_Actuator_Rear', 'HQ3_Actuator_Top', 'BeltConstraint_01', 'HQ3_Actuator_Top_01', 
           'HQ3_Actuator_Top_02', 'BeltConstraint', 'BeltConstraint_02', 'BeltConstraint_03']

# Example usage
controller = CustomActuatorController(
    articulation_base_path="/World/FLUFFY/FLUFFY",
    articulation_joints=lane_joints,
    debugMode=False
)

#controller.moveJoint("HQ2_Actuator", position=1)
#controller.moveJoint("HQ3_Actuator_Bottom_01", position=1)
#controller.moveJoint("Vision_Actuator_Rear", position=-1)
#print(controller.getAllJointStates())

controller.moveJoints({
    "HQ2_Actuator": {"position": 1.0},
    "HQ3_Actuator_Bottom_01": {"position": 1.0},
    "Vision_Actuator_Rear": {"position": 1.0}
})