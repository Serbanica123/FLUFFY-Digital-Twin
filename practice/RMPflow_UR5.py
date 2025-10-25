from omni.isaac.motion_generation.interface_config_loader import (
    get_supported_robot_policy_pairs,
    load_supported_motion_policy_config,
)

class FrankaRmpFlowExample():
    def __init__(self):
        self._rmpflow = None
        self._articulation_rmpflow = None

        self._articulation = None
        self._target = None

    def load_example_assets(self):
        # Add the Franka and target to the stage
        # The position in which things are loaded is also the position in which they

        robot_prim_path = "/panda"
        path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/UR5.usd"

        add_reference_to_stage(path_to_robot_usd, robot_prim_path)
        self._articulation = Articulation(robot_prim_path)

        add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", "/World/target")
        self._target = XFormPrim("/World/target", scale=[.04,.04,.04])

        self._obstacle = FixedCuboid("/World/obstacle",size=.05,position=np.array([0.4, 0.0, 0.65]),color=np.array([0.,0.,1.]))

        # Return assets that were added to the stage so that they can be registered with the core.World
        return self._articulation, self._target, self._obstacle

    def setup(self):
        # Loading RMPflow can be done quickly for supported robots
        print("Supported Robots with a Provided RMPflow Config:", list(get_supported_robot_policy_pairs().keys()))
        rmp_config = load_supported_motion_policy_config("UR5","RMPflow")

        #Initialize an RmpFlow object
        self._rmpflow = RmpFlow(**rmp_config)
        self._rmpflow.add_obstacle(self._obstacle)

        #Use the ArticulationMotionPolicy wrapper object to connect rmpflow to the Franka robot articulation.
        self._articulation_rmpflow = ArticulationMotionPolicy(self._articulation,self._rmpflow)

        self._target.set_world_pose(np.array([.5,0,.7]),euler_angles_to_quats([0,np.pi,0]))

    def update(self, step: float):
        # Step is the time elapsed on this frame
        target_position, target_orientation = self._target.get_world_pose()

        self._rmpflow.set_end_effector_target(
            target_position, target_orientation
        )

        # Track any movements of the cube obstacle
        self._rmpflow.update_world()

        #Track any movements of the robot base
        robot_base_translation,robot_base_orientation = self._articulation.get_world_pose()
        self._rmpflow.set_robot_base_pose(robot_base_translation,robot_base_orientation)

        action = self._articulation_rmpflow.get_next_articulation_action(step)
        self._articulation.apply_action(action)

    def reset(self):
        # Rmpflow is stateless unless it is explicitly told not to be

        self._target.set_world_pose(np.array([.5,0,.7]),euler_angles_to_quats([0,np.pi,0]))