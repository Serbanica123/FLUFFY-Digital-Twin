from omni.isaac.core.utils.types import ArticulationActions
import numpy as np
from omni.isaac.core.articulations import ArticulationView

class CustomActuatorController:
    """
    A class to control and manage actuators of articulated robots.
    
    This class provides methods to reset joint states, move individual or multiple joints,
    and retrieve joint states (position, velocity, and effort). It supports debug mode to print
    internal states during execution for easier debugging.
    
    Attributes:
        articulation_base_path (str): The base path to the articulation in the simulation world.
        joint_names (list): A list of joint names to control.
        debugMode (bool): A flag to enable/disable debug mode.
        applied_actions (dict): A dictionary storing the actions applied to joints.
        articulation_view (ArticulationView): An instance of ArticulationView that provides 
                                              access to the articulation's joints and their states.
        total_dofs (int): The total number of degrees of freedom in the articulation.
    """
    def __init__(self, articulation_base_path, articulation_joints, debugMode=False):
        self._articulation_base_path = articulation_base_path
        self.joint_names = articulation_joints
        self.debugMode = debugMode

        self.applied_actions = {}  # Initialize an empty dictionary to store applied actions

        # Initialize a single articulation view that covers all joints
        self.articulation_view = ArticulationView(f"{self._articulation_base_path}/{self.joint_names[0]}")
        self.articulation_view.initialize()

        # Get the degrees of freedom (DOFs) from the articulation view
        self.total_dofs = self.articulation_view.num_dof

        if self.debugMode:
            print(f"Articulation view initialized with {self.total_dofs} DOFs.")
            print(f"Joints: {self.articulation_view.dof_names}")

    def _initialize(self):
        """
        Initializes the articulation view if it is not already initialized.
        This method ensures that the articulation view is set up before interacting with it.

        this method is used internally and is not meant to be used outside the class
        """
        if not self.articulation_view.initialized:
            self.articulation_view.initialize()

    def resetJointStates(self):
        """
        Resets the joint states to default values.
        
        This method sets all joint positions to -1.0, velocities to 0.0, and efforts to 0.0.
        It then applies these reset values to the articulation.

        Example usage:
        ```python
            CustomActuatorController.resetJointStates()
        ```
        """
        self._initialize()

        default_pos = -1.0
        default_vel = 0.0
        default_eff = 0.0

        joint_positions = [default_pos] * self.total_dofs
        joint_velocities = [default_vel] * self.total_dofs
        joint_efforts = [default_eff] * self.total_dofs

        action = ArticulationActions(
            joint_positions=np.array(joint_positions, dtype=np.float32),
            joint_velocities=np.array(joint_velocities, dtype=np.float32),
            joint_efforts=np.array(joint_efforts, dtype=np.float32),
            joint_indices=None,
        )
        
        for i in range(self.total_dofs):
            self.applied_actions[i] = {
                'position': default_pos,
                'velocity': default_vel,
                'effort': default_eff,
        }

        self.articulation_view.apply_action(action)

    def moveJoint(self, joint_name, position=None, velocity=None, effort=None):
        """
        Move one specific joint to a desired position, velocity, and/or effort.
        
        Args:
            joint_name (str): The name of the joint to move.
            position (float, optional): The target position for the joint.
            velocity (float, optional): The target velocity for the joint.
            effort (float, optional): The target effort for the joint.
        
        Raises:
            ValueError: If the specified joint_name does not exist in the articulation.
        
        Example usage:
        ```python
            CustomActuatorController.moveJoint("joint_1", position=1.0, velocity=0.5)
        ```
        """
        self._initialize()

        if joint_name not in self.articulation_view.dof_names:
            raise ValueError(f"Joint '{joint_name}' not found in the articulation view.")

        # Initialize the arrays to hold the joint positions, velocities, and efforts
        joint_positions = [-1.0] * self.total_dofs
        joint_velocities = [0.0] * self.total_dofs
        joint_efforts = [0.0] * self.total_dofs

        # Track the applied actions and position updates
        for i, joint in enumerate(self.joint_names):
            # Find the DOF index corresponding to the joint in the articulation view
            dof_index = self.articulation_view.dof_names.index(joint)

            # If the joint has an applied action, use the position from the stored action
            if joint in self.applied_actions:
                joint_positions[dof_index] = self.applied_actions[joint]['position']
            else:
                # If no action has been applied to the joint, leave the position as -1
                pass

        if self.debugMode:
            print(f"current joint position states are: {joint_positions} with length {len(joint_positions)}")
            

        # Find the index of the given joint name in the articulation view
        dof_index = self.articulation_view.dof_names.index(joint_name)

        # If specified, update the joint position, velocity, and effort for the matching DOF
        if position is not None:
            joint_positions[dof_index] = position
        if velocity is not None:
            joint_velocities[dof_index] = velocity
        if effort is not None:
            joint_efforts[dof_index] = effort

        # Apply the action to the articulation
        action = ArticulationActions(
            joint_positions=np.array(joint_positions, dtype=np.float32),
            joint_velocities=np.array(joint_velocities, dtype=np.float32),
            joint_efforts=np.array(joint_efforts, dtype=np.float32),
            joint_indices=None,  # Set joint_indices as None to apply it to all joints
        )

        # Save the applied action for the joint
        self.applied_actions[joint_name] = {
            'position': position if position is not None else joint_positions[dof_index],
            'velocity': velocity if velocity is not None else joint_velocities[dof_index],
            'effort': effort if effort is not None else joint_efforts[dof_index],
        }

        self.articulation_view.apply_action(action)

    def moveJoints(self, joint_targets):
        """
        Move multiple joints to their desired positions, velocities, and/or efforts.
        
        Args:
            joint_targets (dict): A dictionary where the keys are joint names and the values are 
                                          dictionaries containing the target position, velocity, and/or effort.

        Example usage:
        ```python
            CustomActuatorController.moveJoints({
                "joint_1": {"position": 1.0, "velocity": 0.5},
                "joint_2": {"position": -0.5}
            })
        ```
        """
        self._initialize()

        # Initialize the arrays to hold the joint positions, velocities, and efforts
        joint_positions = [-1.0] * self.total_dofs
        joint_velocities = [0.0] * self.total_dofs
        joint_efforts = [0.0] * self.total_dofs

        # Track the applied actions and position updates
        for i, joint in enumerate(self.joint_names):
            # Find the DOF index corresponding to the joint in the articulation view
            dof_index = self.articulation_view.dof_names.index(joint)

            # If the joint has an applied action, use the position from the stored action
            if joint in self.applied_actions:
                joint_positions[dof_index] = self.applied_actions[joint]['position']
            else:
                # If no action has been applied to the joint, leave the position as -1
                pass

        if self.debugMode:
            print(f"current joint position states are: {joint_positions} with length {len(joint_positions)}")

        for joint_name, targets in joint_targets.items():
            if joint_name not in self.articulation_view.dof_names:
                raise ValueError(f"Joint '{joint_name}' not found in the articulation view.")
            
            # Find the index of the given joint name in the articulation view
            dof_index = self.articulation_view.dof_names.index(joint_name)

            # If specified, update the joint position, velocity, and effort for the matching DOF
            if "position" in targets:
                joint_positions[dof_index] = targets["position"]
            if "velocity" in targets:
                joint_velocities[dof_index] = targets["velocity"]
            if "effort" in targets:
                joint_efforts[dof_index] = targets["effort"]

            # Save the applied action for the joint in self.applied_actions
            self.applied_actions[joint_name] = {
                'position': joint_positions[dof_index],
                'velocity': joint_velocities[dof_index],
                'effort': joint_efforts[dof_index]
            }

        # Apply the action to the articulation
        action = ArticulationActions(
            joint_positions=np.array(joint_positions, dtype=np.float32),
            joint_velocities=np.array(joint_velocities, dtype=np.float32),
            joint_efforts=np.array(joint_efforts, dtype=np.float32),
            joint_indices=None,  # Set joint_indices as None to apply it to all joints
        )
        
        self.articulation_view.apply_action(action)

    def getJointState(self, joint_name):
        """
        Retrieve the current state (position, velocity, and effort) of a specific joint.
        
        Args:
            joint_name (str): The name of the joint to retrieve the state for.
        
        Returns:
            dict: A dictionary containing the joint's position, velocity, and effort.

        Example usage:
        ```python
            state = CustomActuatorController.getJointState("joint_1")
            print(state)
        ```
        """
        # Retrieve all joint states first
        jointStates = self.getAllJointStates()

        # Initialize an empty result dictionary to store the specific joint state
        joint_state = {}

        # Loop through the available joint states and filter for the given joint_name
        for name, state in jointStates.items():
            if name == joint_name:
                # Check if position, velocity, and effort are arrays or scalars
                joint_state = {
                    'position': state['position'][0] if isinstance(state['position'], (list, np.ndarray)) else state['position'],
                    'velocity': state['velocity'][0] if isinstance(state['velocity'], (list, np.ndarray)) else state['velocity'],
                    'effort': state['effort'][0] if isinstance(state['effort'], (list, np.ndarray)) else state['effort']
                }
                break  # Once the joint is found, exit the loop

        # Return the joint state for the requested joint_name
        return joint_state

    def getAllJointStates(self):
        """
        Retrieve the current states (position, velocity, and effort) of all joints.
        
        Returns:
            dict: A dictionary where keys are joint names and values are dictionaries containing 
                  the joint's position, velocity, and effort.

        Example usage:
        ```python
            all_states = CustomActuatorController.getAllJointStates()
            print(all_states)
        ```
        """
        self._initialize()

        all_joint_states = {}

        # Get joint states for all DOFs
        joint_positions = self.articulation_view.get_joint_positions()
        joint_velocities = self.articulation_view.get_joint_velocities()
        joint_efforts = self.articulation_view.get_measured_joint_efforts()

        if self.debugMode:
            # Debugging: Log the names of all DOFs
            print(f"DOF names: {self.articulation_view.dof_names}")

        for i, joint_name in enumerate(self.articulation_view.dof_names):
            joint_state = {
                "position": joint_positions[0, i].item(),
                "velocity": joint_velocities[0, i].item(),
                "effort": joint_efforts[0, i].item(),
            }
            all_joint_states[joint_name] = joint_state

        return all_joint_states

