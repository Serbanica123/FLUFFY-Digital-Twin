import numpy as np
import os
import carb
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
from omni.isaac.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from omni.isaac.motion_generation import interface_config_loader
import time

class UR5KinematicsExample():
    
    def __init__(self):
        self._kinematics_solver = None
        self._articulation_kinematics_solver = None
        self._articulation = None
        self._target_position = np.array([0.5, 0, 0.5])  # Default target position
        self._target_orientation = euler_angles_to_quats([0, np.pi, 0])  # Default target orientation (rotation)

    def load_example_assets(self):
        # Retrieve the existing current stage
        stage = get_current_stage()

        # Try to get the existing Articulation (UR5 robot)
        prim = stage.GetPrimAtPath("/World/ur5")  # Modify the path if needed to match your existing robot

        # Ensure that we found the Articulation
        if not prim:
            raise ValueError("Articulation (UR5) not found in the stage.")

        # Create an Articulation object from the string path of the Prim
        self._articulation = Articulation("/World/ur5")  # Use the string path directly

        # Ensure the Articulation is fully initialized (if needed)
        if not self._articulation.is_valid():
            raise ValueError("Articulation (UR5) is not properly initialized.")

        # Return the articulation for further use
        return self._articulation

    def setup(self):
        # Load the UR5-specific kinematics configuration files
        mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")
        kinematics_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")

        # Paths to the robot description and URDF for the UR5 robot
        self._kinematics_solver = LulaKinematicsSolver(
            robot_description_path=kinematics_config_dir + "/universal_robots/ur5/rmpflow/ur5_robot_description.yaml", 
            urdf_path=kinematics_config_dir + "/universal_robots/ur5/ur5.urdf"
        )

        # Print valid frame names for IK calculation
        print("Valid frame names for kinematics calculation:", self._kinematics_solver.get_all_frame_names())

        # Set the end-effector name for the UR5 robot (e.g., 'tool0' or the equivalent frame)
        end_effector_name = "tool0"  # Update to match UR5's end effector frame

        # Create the articulation kinematics solver
        self._articulation_kinematics_solver = ArticulationKinematicsSolver(
            self._articulation, self._kinematics_solver, end_effector_name
        )

        # Wait until the articulation and view are initialized (adding delay if needed)
        self._initialize_articulation_view()

    def set_target(self, position, orientation):
        """
        Set the target position and orientation manually.

        Parameters:
        - position (np.array): A 3D array representing the target position in world coordinates.
        - orientation (np.array): A quaternion representing the target orientation.
        """
        self._target_position = np.array(position)
        self._target_orientation = np.array(orientation)

    def _initialize_articulation_view(self):
        """
        Ensures that the ArticulationView is initialized before using it for IK calculations.
        """
        attempt_count = 0
        max_attempts = 10
        while attempt_count < max_attempts:
            # Ensure the articulation and view are properly initialized
            if self._articulation.is_valid():
                try:
                    # If the articulation view can be accessed successfully, break the loop
                    self._articulation.get_articulation_view()
                    print("ArticulationView successfully initialized.")
                    return
                except Exception as e:
                    carb.log_warn(f"Attempt {attempt_count+1}: ArticulationView not initialized yet.")
            time.sleep(0.5)  # Wait a bit before retrying
            attempt_count += 1
        
        raise ValueError("Failed to initialize ArticulationView after multiple attempts.")

    def update(self, step: float):
        # Ensure that the Articulation is fully initialized before proceeding
        if not self._articulation.is_valid():
            carb.log_warn("Articulation is not properly initialized.")
            return

        # Track any movements of the robot base
        robot_base_translation, robot_base_orientation = self._articulation.get_world_pose()

        # Set the robot base pose for the kinematics solver
        self._kinematics_solver.set_robot_base_pose(robot_base_translation, robot_base_orientation)

        # Compute the inverse kinematics solution for the given target
        action, success = self._articulation_kinematics_solver.compute_inverse_kinematics(
            self._target_position, self._target_orientation
        )

        if success:
            # Ensure that the articulation is applied to the action
            self._articulation.set_joint_positions(action)  # Directly setting joint positions
        else:
            carb.log_warn("IK did not converge to a solution. No action is being taken.")

    def reset(self):
        # Kinematics solver is stateless, so nothing specific is needed here
        pass

# Example usage
ur5_kinematics_example = UR5KinematicsExample()

# Load the UR5 assets (robot) from the world (stage)
ur5_kinematics_example.load_example_assets()

# Set up the kinematics solver
ur5_kinematics_example.setup()

# Define a new target position and orientation
new_target_position = [0.7, 0, 0.6]  # Example position (x, y, z)
new_target_orientation = euler_angles_to_quats([0, np.pi / 2, 0])  # Example orientation (roll, pitch, yaw)

# Set the new target manually
ur5_kinematics_example.set_target(new_target_position, new_target_orientation)

# Update loop (typically called in the simulation's step/update cycle)
# You would call this method during the simulation's step/update cycle.
ur5_kinematics_example.update(step=0.01)
