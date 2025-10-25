# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

PERSISTENT_SETTINGS_PREFIX = "/persistent"


class NavMeshSettings:
    DEFAULT_EXCLUDE_RIGID_BODIES_PATH = (
        f"{PERSISTENT_SETTINGS_PREFIX}/exts/omni.anim.navigation.core/navMesh/config/excludeRigidBodies"
    )
    DEFAULT_EXCLUDE_INTERACTIVE_OBJECTS_PATH = (
        f"{PERSISTENT_SETTINGS_PREFIX}/exts/omni.anim.navigation.core/navMesh/config/excludeInteractiveObjects"
    )

    # ======= Defaults Setting Paths ===========

    # bake settings
    DEFAULT_AGENT_HEIGHT_SETTING_PATH = f"{PERSISTENT_SETTINGS_PREFIX}/exts/omni.anim.navigation.core/navMesh/config/agentHeight"
    DEFAULT_AGENT_RADIUS_SETTING_PATH = f"{PERSISTENT_SETTINGS_PREFIX}/exts/omni.anim.navigation.core/navMesh/config/agentRadius"
    DEFAULT_AGENT_MAX_STEP_HEIGHT_SETTING_PATH = f"{PERSISTENT_SETTINGS_PREFIX}/exts/omni.anim.navigation.core/navMesh/config/agentMaxStepHeight"
    DEFAULT_AGENT_MAX_FLOOR_SLOPE_SETTING_PATH = f"{PERSISTENT_SETTINGS_PREFIX}/exts/omni.anim.navigation.core/navMesh/config/agentMaxFloorSlope"
    DEFAULT_AUTO_REBAKE_SETTING_PATH = f"{PERSISTENT_SETTINGS_PREFIX}/exts/omni.anim.navigation.core/navMesh/config/autoRebakeOnChanges"
    DEFAULT_AUTO_REBAKE_DELAY_SETTING_PATH = f"{PERSISTENT_SETTINGS_PREFIX}/exts/omni.anim.navigation.core/navMesh/config/autoRebakeDelaySeconds"

    # visualization
    DEFAULT_VIZ_GEOM_ENABLE_SETTING_PATH = f"{PERSISTENT_SETTINGS_PREFIX}/exts/omni.anim.navigation.core/navMesh/config/vizGeomEnable"
    DEFAULT_VIZ_SURFACE_ENABLE_SETTING_PATH = f"{PERSISTENT_SETTINGS_PREFIX}/exts/omni.anim.navigation.core/navMesh/config/vizSurfaceEnable"
    DEFAULT_VIZ_OUTLINE_ENABLE_SETTING_PATH = f"{PERSISTENT_SETTINGS_PREFIX}/exts/omni.anim.navigation.core/navMesh/config/vizOutlineEnable"

    # ======= Setting Paths ===========
    VIEW_NAVMESH_SETTING_PATH = f"{PERSISTENT_SETTINGS_PREFIX}/exts/omni.anim.navigation.core/navMesh/viewNavMesh"

    CACHE_ENABLED_SETTING_PATH = "/exts/omni.anim.navigation.core/navMesh/cache/enabled"
    EXCLUDE_RIGID_BODIES_PATH = "/exts/omni.anim.navigation.core/navMesh/config/excludeRigidBodies"
    EXCLUDE_INTERACTIVE_OBJECTS_PATH = "/exts/omni.anim.navigation.core/navMesh/config/excludeInteractiveObjects"
    AUTO_REBAKE_SETTING_PATH = "/exts/omni.anim.navigation.core/navMesh/config/autoRebakeOnChanges"
    AUTO_REBAKE_DELAY_SETTING_PATH = "/exts/omni.anim.navigation.core/navMesh/config/autoRebakeDelaySeconds"
    AGENT_HEIGHT_SETTING_PATH = "/exts/omni.anim.navigation.core/navMesh/config/agentHeight"
    AGENT_RADIUS_SETTING_PATH = "/exts/omni.anim.navigation.core/navMesh/config/agentRadius"
    AGENT_MAX_STEP_HEIGHT_SETTING_PATH = "/exts/omni.anim.navigation.core/navMesh/config/agentMaxStepHeight"
    AGENT_MAX_FLOOR_SLOPE_SETTING_PATH = "/exts/omni.anim.navigation.core/navMesh/config/agentMaxFloorSlope"
    # visualization
    VIZ_GEOM_ENABLE_SETTING_PATH = "/exts/omni.anim.navigation.core/navMesh/config/vizGeomEnable"
    VIZ_SURFACE_ENABLE_SETTING_PATH = "/exts/omni.anim.navigation.core/navMesh/config/vizSurfaceEnable"
    VIZ_OUTLINE_ENABLE_SETTING_PATH = "/exts/omni.anim.navigation.core/navMesh/config/vizOutlineEnable"
