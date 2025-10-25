import json
import re
import os
from actuators import *
from sensors import *
from conveyors import *
from Builder import *


class FLUFFY:
    """
    A class representing the FLUFFY system, responsible for managing lanes, actuators, conveyors, and sensors.

    Attributes:
    - path (str): The path associated with the FLUFFY system.
    - fluffyDict (dict): A dictionary holding the system's configuration.
    - id (int): The ID of the FLUFFY system instance.
    - configPath (str): The path to the configuration JSON file.
    - actuatorController (dict): A dictionary mapping actuator names to custom actuator controllers.
    - conveyorControllers (dict): A dictionary of dictionaries for conveyor controllers, organized by lane.
    - Sensors (dict): A dictionary of Sensors, keyed by lane names.\n
    Example Usage:
    ```python
    #Writing to the configuration file:
        FLUFFY(write=True, id=0, configPath="/home/mechatronica/.local/share/ov/pkg/isaac-sim-4.2.0/FLUFFY/FLUFFYBuilder/configFLUFFY.json").writeConfig()\n
    #Reading Fluffy system:
        fluffy=FLUFFY(False,0,"/home/mechatronica/.local/share/ov/pkg/isaac-sim-4.2.0/FLUFFY/FLUFFYBuilder/configFLUFFY.json")\n
    #Using the controllers, check config file for correct controller paths
        fluffy.actuatorController["assembly_lane"].moveJoint("HQ2_Actuator_03", position=1)
        fluffy.conveyorControllers["vision_lane"]["Belt_ConveyorBeltGraph_01"].setVelocity(0.0)
        fluffy.Sensors["vision_lane"].getSensorOutput(cutoffValue=0.03, sensorID="3"):\n

    ```
    """

    def __init__(
        self,
        write: bool = False,
        id: int = 0,
        configPath: str = os.path.abspath(os.path.join(os.path.dirname(__file__), "configFLUFFY.json"))
    ):
        """
        Initializes a FLUFFY instance.

        This method sets up the FLUFFY system either by creating a new configuration or reading from an
        existing one, depending on the 'write' parameter.

        Args:
            write (bool): Whether to create a new configuration or read an existing one.
            id (int): The ID of the FLUFFY instance.
            configPath (str): Path to the configuration file.
        """
        self.path = ""
        self.fluffyDict = {}
        self.id = 0
        self.configPath = configPath
        self.actuatorController: dict[str, CustomActuatorController] = {}
        self.conveyorControllers: dict[str, dict[str, conveyor]] = {}
        self.Sensors: dict[str, Sensors] = {}

        if write:
            # Construct new lane configurations for vision, repair, and assembly lanes
            vision_builder = VisionLaneBuilder()
            repair_builder = RepairLaneBuilder()
            assembly_builder = AssemblyLaneBuilder()

            # Director is responsible for constructing and getting the full configuration
            dir = Director(id)
            vision_lane = dir.construct(vision_builder)
            repair_lane = dir.construct(repair_builder)
            assembly_lane = dir.construct(assembly_builder)
            self.path = dir.getPath()

            # Store the configurations in a dictionary
            self.fluffyDict = {
                "id": self.id,
                "path": self.path,
                "vision_lane": vision_lane.getLaneDict(),
                "repair_lane": repair_lane.getLaneDict(),
                "assembly_lane": assembly_lane.getLaneDict(),
            }
            # Reset lane system
            dir.reset(Lane())
        else:
            # Read existing configuration from file
            self.fluffyDict = self.readConfig()
            lane_exp = "_lane"
            self.path = self.fluffyDict["path"]
            self.id = self.fluffyDict["id"]
            pattern = r"FLUFFY/([^/]+)/ConveyorNode"

            # Parse the configuration and instantiate corresponding objects
            for key in self.fluffyDict.keys():
                if bool(re.search(lane_exp, key)):
                    for lane_key in self.fluffyDict[key].keys():
                        if lane_key == "Sensors":
                            self.Sensors[key] = Sensors(self.fluffyDict[key][lane_key])
                        elif lane_key == "Actuators":
                            self.actuatorController[key] = CustomActuatorController(
                                self.path, self.fluffyDict[key][lane_key]
                            )
                        elif lane_key == "Conveyors":
                            conveyorLaneControllers: dict[str, conveyor] = {}
                            for conveyor_prim in self.fluffyDict[key][lane_key]:
                                match = re.search(pattern, conveyor_prim)
                                if match:
                                    conveyorLaneControllers[match.group(1)] = conveyor(
                                        conveyor_prim
                                    )
                            self.conveyorControllers[key] = conveyorLaneControllers

    def writeConfig(self):
        """
        Writes the current FLUFFY configuration to a JSON file.

        This method serializes the current configuration dictionary into a JSON file at the path
        specified by 'configPath'. This is typically used to save a newly created configuration.
        """
        with open(self.configPath, "w") as file:
            json.dump(self.fluffyDict, file, indent=4)

    def deleteConfig(self):
        """
        Deletes the existing configuration by clearing the file contents.

        This method effectively deletes the configuration by emptying the contents of the configuration file.
        """
        open(self.configPath, "w").close()

    def readConfig(self) -> dict:
        """
        Reads the FLUFFY configuration from a JSON file.

        This method loads and parses the configuration JSON file at the path specified by 'configPath'.

        Returns:
            dict: The configuration data loaded from the file.
        """
        with open(self.configPath, "r") as file:
            data = file.read()
            dictionary = json.loads(data)
        return dictionary

    def getLanes(self):
        """
        Retrieves the lane configurations.

        This method returns the configurations for the vision lane, repair lane, and assembly lane
        as a tuple.

        Returns:
            tuple: A tuple containing the vision lane, repair lane, and assembly lane configurations.
        """
        return (self.vision_lane, self.repair_lane, self.assembly_lane)


if __name__ == "__main__":
    """
    Main execution block to create and write FLUFFY configuration.

    This block is responsible for creating an instance of the FLUFFY class and writing its
    configuration to a file if the script is run as the main module.
    """
    FLUFFY(True, 0).writeConfig()
