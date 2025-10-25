import carb
import logging

# Set up Python's standard logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)  # Set the logging level to capture INFO, WARNING, and ERROR
ch = logging.StreamHandler()  # Stream handler to output logs to the console
ch.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
ch.setFormatter(formatter)
logger.addHandler(ch)

class CreateNavMeshVolumeCommand(omni.kit.commands.Command):
    """Creates a navigation mesh volume and generates a NavMesh inside it."""
    def __init__(
        self, parent_prim_path: Sdf.Path = Sdf.Path.emptyPath, layer: Sdf.Layer = None, usd_context_name: str = ""
    ):
        self._usd_undo = None
        self._parent_prim_path = parent_prim_path
        self._layer = layer
        self._usd_context = omni.usd.get_context(usd_context_name)
        self._selection = self._usd_context.get_selection()

    def do(self):
        stage = self._usd_context.get_stage()
        if self._layer is None:
            self._layer = stage.GetEditTarget().GetLayer()
        self._usd_undo = UsdLayerUndo(self._layer)
        self._prim_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, f"/{NAVMESH_VOLUME_NAME}", True))
        self._usd_undo.reserve(self._prim_path)
        NavSchema.NavMeshVolume.Define(stage, self._prim_path)
        omni.kit.commands.execute(
            "ApplyNavMeshAPICommand", prim_path=self._prim_path, api=NavSchema.NavMeshAreaAPI
        )

        # Log information about the NavMesh volume creation
        carb.log_info(f"Created NavMesh volume at {self._prim_path}")

        # the unit of the defined extent is meter, should convert to the target unit
        halfExtent = HALF_EXTENT / UsdGeom.GetStageMetersPerUnit(stage)

        # set the default boundable extent
        prim = stage.GetPrimAtPath(self._prim_path)
        boundable = UsdGeom.Boundable(prim)
        extentAttr = boundable.GetExtentAttr()
        if extentAttr:
            extentAttr.Set([(-halfExtent, -halfExtent, -halfExtent), (halfExtent, halfExtent, halfExtent)])

        self._selection.set_prim_path_selected(self._prim_path.pathString, True, True, True, True)

        inav = nav.acquire_interface()

        # Check if the NavMesh interface was successfully acquired
        if inav is None:
            carb.log_error("Failed to acquire the NavMesh interface.")
            logger.error("Failed to acquire the NavMesh interface.")
            return
        else:
            carb.log_info("NavMesh interface acquired.")
            logger.info("NavMesh interface acquired.")

        # List available methods to see if generate_navmesh() exists
        carb.log_info(f"NavMesh interface methods: {dir(inav)}")
        logger.info(f"NavMesh interface methods: {dir(inav)}")

        volume_count = inav.get_navmesh_volume_count()
        if volume_count == 0:
            world_bound = self._usd_context.compute_path_world_bounding_box(Sdf.Path.absoluteRootPath.pathString)
            world_range = Gf.Range3d(Gf.Vec3d(*world_bound[0]), Gf.Vec3d(*world_bound[1]))

            mid_point = world_range.GetMidpoint()
            dimension = world_range.GetSize()
        else:
            mid_point = Gf.Vec3d(0, 0, 0)
            dimension = Gf.Vec3d(MIN_SIZE, MIN_SIZE, MIN_SIZE)

        # Prevent creating a bound that's too small
        for i in range(3):
            if dimension[i] < MIN_SIZE:
                dimension[i] = MIN_SIZE
            dimension[i] += 50  # Padding for border

        translate = Gf.Matrix4d(1.0)
        translate.SetTranslate(mid_point)
        scale = Gf.Matrix4d(1.0)
        scale.SetScale(dimension)
        xform = scale * translate

        parent_world_xform = Gf.Matrix4d(
            *self._usd_context.compute_path_world_transform(self._prim_path.GetParentPath().pathString)
        )

        # Counter ancestors transform
        xform *= parent_world_xform.GetInverse()

        omni.kit.commands.execute("TransformPrim", path=self._prim_path, new_transform_matrix=xform)

        # Debug: Trigger NavMesh generation and check results
        self._generate_navmesh()

    def _generate_navmesh(self):
        """Trigger the generation of the NavMesh within the volume."""
        inav = nav.acquire_interface()
        
        if inav is None:
            carb.log_error("No valid NavMesh interface available.")
            logger.error("No valid NavMesh interface available.")
            return

        # Check if generate_navmesh() exists
        if hasattr(inav, 'generate_navmesh'):
            carb.log_info("Generating NavMesh...")
            logger.info("Generating NavMesh...")
            inav.generate_navmesh()  # This triggers the actual generation of the NavMesh
            carb.log_info("NavMesh generation triggered.")
            logger.info("NavMesh generation triggered.")
        else:
            carb.log_error("NavMesh interface does not have the method 'generate_navmesh'.")
            logger.error("NavMesh interface does not have the method 'generate_navmesh'.")

    def undo(self):
        if self._usd_undo is not None:
            self._usd_undo.undo()
# Import the class and necessary modules
from pxr import Sdf

# Define the parent path for the NavMesh volume, you can use the default root path if needed
parent_prim_path = Sdf.Path('/World')  # Example: Using the root path for the parent
layer = None  # You can specify a layer if necessary, otherwise leave as None

# Instantiate the command
command = CreateNavMeshVolumeCommand(parent_prim_path=parent_prim_path, layer=layer)

# Trigger the command to create the NavMesh volume
command.do()

# Optionally, if you want to undo the action, you can call:
# command.undo()
