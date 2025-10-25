# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from __future__ import annotations

from typing import Any, Dict

import omni.anim.navigation.core as nav
import omni.kit.commands
import omni.usd
from omni.kit.usd_undo import UsdLayerUndo
from pxr import Gf, Sdf, Usd, UsdGeom
import NavSchema

NAVMESH_VOLUME_NAME = "NavMeshVolume"

HALF_EXTENT = 0.5 #meter
MIN_SIZE = 400


def get_stage_default_prim_path(stage):
    if stage.HasDefaultPrim():
        return stage.GetDefaultPrim().GetPath()
    else:
        return Sdf.Path.absoluteRootPath


def refresh_property_window():
    try:
        import omni.kit.window.property as p
        p.get_window().request_rebuild()
    except ImportError:
        pass


class CreateNavMeshVolumeCommand(omni.kit.commands.Command):
    """Creates a navigation mesh volume.
    Args:
        parent_prim_path: The parent prim path in the stage where to add the navigation mesh volume.
        layer: The layer to create the navigation mesh volume.
    """

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
        volume_count = inav.get_navmesh_volume_count()
        if volume_count == 0:
            # query the bounding box of all items on the stage, so the NavMeshVolume by default captures all of them
            # but only do it for the first NavMeshVolume in the scene
            world_bound = self._usd_context.compute_path_world_bounding_box(Sdf.Path.absoluteRootPath.pathString)
            world_range = Gf.Range3d(Gf.Vec3d(*world_bound[0]), Gf.Vec3d(*world_bound[1]))

            mid_point = world_range.GetMidpoint()
            dimension = world_range.GetSize()
        else:
            mid_point = Gf.Vec3d(0, 0, 0)
            dimension = Gf.Vec3d(MIN_SIZE, MIN_SIZE, MIN_SIZE)

        # preventing creating a bound that's too small
        for i in range(3):
            if dimension[i] < MIN_SIZE:
                dimension[i] = MIN_SIZE

            # add a padding to the border
            dimension[i] += 50

        translate = Gf.Matrix4d(1.0)
        translate.SetTranslate(mid_point)
        scale = Gf.Matrix4d(1.0)
        scale.SetScale(dimension)
        xform = scale * translate

        parent_world_xform = Gf.Matrix4d(
            *self._usd_context.compute_path_world_transform(self._prim_path.GetParentPath().pathString)
        )

        # counter ancestors transform
        xform *= parent_world_xform.GetInverse()

        omni.kit.commands.execute("TransformPrim", path=self._prim_path, new_transform_matrix=xform)

    def undo(self):
        if self._usd_undo is not None:
            self._usd_undo.undo()


class ApplyNavMeshAPICommand(omni.kit.commands.Command):
    """Applies an API schema to a prim
    Args:
        prim_path: The prim path in the stage on which to add the API schema
        api: The api schema to add
        usd_context_name: The name of the usd context for this command
    """

    def __init__(self, prim_path: Sdf.Path, api: Usd.APISchemaBase, usd_context_name: str = ""):
        self._api = api
        self._prim_path = prim_path
        self._usd_context_name = usd_context_name
        self._usd_context = omni.usd.get_context(self._usd_context_name)
        self._stage = self._usd_context.get_stage()

    def do(self):
        prim = self._stage.GetPrimAtPath(self._prim_path)
        prim.ApplyAPI(self._api)
        refresh_property_window()

    def undo(self):
        prim = self._stage.GetPrimAtPath(self._prim_path)
        prim.RemoveAPI(self._api)
        refresh_property_window()


class RemoveNavMeshAPICommand(omni.kit.commands.Command):
    """Removes an API schema from a prim
    Args:
        prim_path: The prim path in the stage on which to remove the API schema
        api: The api schema to remove
        usd_context_name: The name of the usd context for this command
    """
    def __init__(self, prim_path: Sdf.Path, api: Usd.APISchemaBase, usd_context_name: str = ""):
        self._api = api
        self._prim_path = prim_path
        self._usd_context_name = usd_context_name
        self._usd_context = omni.usd.get_context(self._usd_context_name)
        self._stage = self._usd_context.get_stage()

    def do(self):
        prim = self._stage.GetPrimAtPath(self._prim_path)
        prim.RemoveAPI(self._api)
        refresh_property_window()

    def undo(self):
        prim = self._stage.GetPrimAtPath(self._prim_path)
        prim.ApplyAPI(self._api)
        refresh_property_window()


omni.kit.commands.register_all_commands_in_module(__name__)
