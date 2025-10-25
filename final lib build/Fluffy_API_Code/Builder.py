class Lane:
    """
    Represents a lane in an industrial system that includes sensors, actuators,
    conveyors, and vision systems. Tracks the number of each component type and
    allows for initialization and management of these components.
    """

    numSystems = {
        "Sensor": 0,
        "Chain": 0,
        "Belt": 0,
        "Side Chain": 0,
        "Camera": 0,
        "EQ Actuator": 0,
        "Side Actuator": 0,
        "Camera Actuator": 0,
        "Stopper Actuator": 0,
        "HQ3 Actuator": 0,
        "HQ2 Actuator": 0,
        "Chain Constraint": 0,
        "Belt Constraint": 0,
    }

    def __init__(self):
        """
        Initializes an empty Lane with default paths and empty lists
        for various system components.
        """
        self.path = ""
        self.visionSystemPrims = []
        self.actuatorPrims = []
        self.conveyorPrims = []
        self.sensorPrims = []
        self.sensorDict = {}
        self.conveyorControllers = {}

    def reset(self):
        """
        Resets the count of all system components to zero.
        """
        for sys in self.numSystems.keys():
            self.numSystems[sys] = 0

    def checkIndex(self, index):
        """
        Formats an index into a standardized string format.

        Args:
            index (int): The index to format.

        Returns:
            str: Formatted index string.
        """
        if index == 0:
            return ""
        elif index > 9:
            return "_" + str(index)
        else:
            return str("_0" + str(index))

    def setSensors(self, numSensors):
        """
        Adds a specified number of sensors to the lane.

        Args:
            numSensors (int): The number of sensors to add.
        """
        for i in range(numSensors):
            sensor = (
                self.path
                + "/LightBeam_Sensor"
                + self.checkIndex(i + Lane.numSystems["Sensor"])
            )
            self.sensorPrims.append(sensor)
            self.sensorDict[str(i)] = sensor

        Lane.numSystems["Sensor"] += numSensors

    def setConveyors(self, numChainConveyors=0, numSideConveyors=0, numBeltConveyors=0):
        """
        Adds conveyors to the lane based on the specified numbers for different types.

        Args:
            numChainConveyors (int): Number of chain conveyors to add.
            numSideConveyors (int): Number of side conveyors to add.
            numBeltConveyors (int): Number of belt conveyors to add.
        """
        for i in range(numChainConveyors):
            conveyorPrim = (
                self.path
                + "/Chain_ConveyorBeltGraph"
                + self.checkIndex(i + Lane.numSystems["Chain"])
                + "/ConveyorNode"
            )
            chainConstraint = "ChainConstraint" + self.checkIndex(
                i + Lane.numSystems["Chain Constraint"]
            )
            self.conveyorPrims.append(conveyorPrim)
            self.actuatorPrims.append(chainConstraint)

        Lane.numSystems["Chain"] += numChainConveyors
        Lane.numSystems["Chain Constraint"] += numChainConveyors

        for i in range(numBeltConveyors):
            conveyorPrim = (
                self.path
                + "/Belt_ConveyorBeltGraph"
                + self.checkIndex(i + Lane.numSystems["Belt"])
                + "/ConveyorNode"
            )
            beltConstraint = "BeltConstraint" + self.checkIndex(
                i + Lane.numSystems["Belt Constraint"]
            )
            self.conveyorPrims.append(conveyorPrim)
            self.actuatorPrims.append(beltConstraint)

        Lane.numSystems["Belt"] += numBeltConveyors
        Lane.numSystems["Belt Constraint"] += numBeltConveyors

        for i in range(numSideConveyors):
            conveyorPrim = (
                self.path
                + "/Side_Chain_ConveyorBeltGraph"
                + self.checkIndex(i + Lane.numSystems["Side Chain"])
                + "/ConveyorNode"
            )
            chainConstraint = "ChainConstraint" + self.checkIndex(
                i + Lane.numSystems["Chain Constraint"]
            )
            self.actuatorPrims.append(chainConstraint)
            self.conveyorPrims.append(conveyorPrim)

        Lane.numSystems["Side Chain"] += numSideConveyors
        Lane.numSystems["Chain Constraint"] += numSideConveyors

    def setVisionSystem(self, numCameras):
        """
        Adds cameras to the vision system.

        Args:
            numCameras (int): Number of cameras to add.
        """
        for i in range(numCameras):
            camera = self.path + "/" + self.checkIndex(i + Lane.numSystems["Camera"])
            self.visionSystemPrims.append(camera)
        Lane.numSystems["Camera"] += numCameras


def setActuators(
    self,
    numSmallActuators=0,
    numCameraActuators=0,
    numSideStoppers=0,
    numStoppers=0,
    numBigBelt=0,
    numSmallBelt=0,
):
    """
    Sets the actuators for different systems based on the provided counts.

    This function dynamically creates and appends actuator names to the `actuatorPrims`
    list based on the number of actuators specified for each category. The function updates
    the system counts for each actuator type in the `Lane.numSystems` dictionary.

    Parameters:
    - numSmallActuators (int): The number of small actuators to create.
    - numCameraActuators (int): The number of camera actuators to create.
    - numSideStoppers (int): The number of side stoppers to create.
    - numStoppers (int): The number of stopper actuators to create.
    - numBigBelt (int): The number of big belt actuators to create.
    - numSmallBelt (int): The number of small belt actuators to create.
    """
    # Create small actuators and append to the actuatorPrims list
    for i in range(numSmallActuators):
        actuator = "EQ_Actuator" + self.checkIndex(i + Lane.numSystems["EQ Actuator"])
        self.actuatorPrims.append(actuator)

    Lane.numSystems["EQ Actuator"] += numSmallActuators

    # Create side stoppers and append to the actuatorPrims list
    for i in range(numSideStoppers):
        actuator = "Side_Actuator" + self.checkIndex(
            i + Lane.numSystems["Side Actuator"]
        )
        self.actuatorPrims.append(actuator)

    Lane.numSystems["Side Actuator"] += numSideStoppers

    # Create camera actuators (front and rear) and append to the actuatorPrims list
    for i in range(numCameraActuators):
        actuator1 = "Vision_Actuator_Front" + self.checkIndex(
            i + Lane.numSystems["Camera Actuator"]
        )
        actuator2 = "Vision_Actuator_Rear" + self.checkIndex(
            i + Lane.numSystems["Camera Actuator"]
        )

        self.actuatorPrims.append(actuator1)
        self.actuatorPrims.append(actuator2)

    Lane.numSystems["Camera Actuator"] += numCameraActuators

    # Create stopper actuators and append to the actuatorPrims list
    for i in range(numStoppers):
        actuator = "Stopper_Actuator" + self.checkIndex(
            i + Lane.numSystems["Stopper Actuator"]
        )
        self.actuatorPrims.append(actuator)

    Lane.numSystems["Stopper Actuator"] += numStoppers

    # Create big belt actuators (top and bottom) and append to the actuatorPrims list
    for i in range(numBigBelt):
        actuator1 = "HQ3_Actuator_Top" + self.checkIndex(
            i + Lane.numSystems["HQ3 Actuator"]
        )
        actuator2 = "HQ3_Actuator_Bottom" + self.checkIndex(
            i + Lane.numSystems["HQ3 Actuator"]
        )

        self.actuatorPrims.append(actuator1)
        self.actuatorPrims.append(actuator2)

    Lane.numSystems["HQ3 Actuator"] += numBigBelt

    # Create small belt actuators and append to the actuatorPrims list
    for i in range(numSmallBelt):
        actuator = "HQ2_Actuator" + self.checkIndex(i + Lane.numSystems["HQ2 Actuator"])
        self.actuatorPrims.append(actuator)

    Lane.numSystems["HQ2 Actuator"] += numSmallBelt


def getComponentNames(self, type):
    """
    Prints the names of components based on the specified type.

    This method prints the names of the components stored in `actuatorPrims`,
    `sensorPrims`, or `conveyorPrims` based on the provided `type` argument.

    Parameters:
    - type (str): The type of components to print ("actuators", "sensors", or "conveyors").
    """
    if type == "actuators":
        print("Actuators: ")
        print(self.actuatorPrims)
    elif type == "sensors":
        print("Sensors: ")
        print(self.sensorPrims)
    elif type == "conveyors":
        print("Conveyors: ")
        print(self.conveyorPrims)
    else:
        print("Component type not specified.")


def setPath(self, path):
    """
    Sets the path for the current object.

    This function assigns the given `path` value to the instance's `path` attribute.

    Parameters:
    - path (str): The path to set for the current object.

    Returns:
    - str: The current path.
    """
    self.path = path
    return self.path


def getLaneDict(self):
    """
    Returns a dictionary containing all component names.

    This function returns a dictionary with keys representing component types
    ("Sensors", "Actuators", "Conveyors") and the corresponding component lists
    (`sensorDict`, `actuatorPrims`, `conveyorPrims`).

    Returns:
    - dict: A dictionary containing component types as keys and their associated components.
    """
    laneDic = {
        "Sensors": self.sensorDict,
        "Actuators": self.actuatorPrims,
        "Conveyors": self.conveyorPrims,
    }
    return laneDic


def display(self):
    """
    Displays the current components of the object.

    This function prints the current sensors, actuators, vision system components,
    and conveyors to the console.
    """
    print("Sensors: ")
    print(self.sensorDict, end="\n\n")
    print("Actuators: ")
    print(self.actuatorPrims, end="\n\n")
    print("Vision: ")
    print(self.visionSystemPrims, end="\n\n")
    print("Conveyors: ")
    print(self.conveyorPrims, end="\n\n")
    print("")


class Builder:
    """
    A base builder class that defines the interface for constructing lanes.

    The Builder class provides methods for building the different components of a lane.
    Derived classes should implement these methods to create specific types of lanes.

    Attributes:
    - _lane (Lane): An instance of the Lane class used for building components.
    """

    def __init__(self):
        """
        Initializes the Builder with a new Lane instance.
        """
        self._lane = Lane()

    def buildConveyors(self):
        """
        Builds conveyors for the lane.

        This method should be overridden by a subclass to implement the specific
        logic for building conveyors. In the base Builder class, it is a placeholder
        that returns 0.

        Returns:
        - int: Placeholder return value (0).
        """
        return 0

    def buildSensors(self):
        """
        Builds sensors for the lane.

        This method should be overridden by a subclass to implement the specific
        logic for building sensors. In the base Builder class, it is a placeholder
        that returns 0.

        Returns:
        - int: Placeholder return value (0).
        """
        return 0

    def buildVisionSystem(self):
        """
        Builds the vision system for the lane.

        This method should be overridden by a subclass to implement the specific
        logic for building the vision system. In the base Builder class, it is a placeholder
        that returns 0.

        Returns:
        - int: Placeholder return value (0).
        """
        return 0

    def buildActuators(self):
        """
        Builds actuators for the lane.

        This method should be overridden by a subclass to implement the specific
        logic for building actuators. In the base Builder class, it is a placeholder
        that returns 0.

        Returns:
        - int: Placeholder return value (0).
        """
        return 0

    def buildPath(self, id=0):
        """
        Builds the path for the lane.

        Constructs a path based on the provided ID and updates the lane instance.

        Parameters:
        - id (int): The ID to append to the path string.

        Returns:
        - str: The generated path.
        """
        return self._lane.setPath(
            "/World/FLUFFY" + self._lane.checkIndex(id) + "/FLUFFY"
        )

    def getResult(self):
        """
        Returns the built Lane instance.

        Returns:
        - Lane: A new instance of the Lane class (empty lane).
        """
        return Lane()


class RepairLaneBuilder(Builder):
    """
    A builder for constructing a repair lane.

    Inherits from the Builder class and implements the methods for building
    a lane that includes conveyors, sensors, actuators, and vision systems
    specific to a repair lane.
    """

    def __init__(self):
        """
        Initializes the RepairLaneBuilder with a new Lane instance.
        """
        self._lane = Lane()

    def buildConveyors(self):
        """
        Builds conveyors specific to the repair lane.

        Sets the type and number of conveyors for the repair lane.
        """
        self._lane.setConveyors(
            numChainConveyors=1, numSideConveyors=2, numBeltConveyors=2
        )

    def buildActuators(self):
        """
        Builds actuators specific to the repair lane.

        Sets the type and number of actuators for the repair lane.
        """
        self._lane.setActuators(numSmallActuators=2, numSmallBelt=2, numStoppers=2)

    def buildSensors(self):
        """
        Builds sensors specific to the repair lane.

        Sets the number of sensors for the repair lane.
        """
        self._lane.setSensors(numSensors=5)

    def buildVisionSystem(self):
        """
        Builds the vision system for the repair lane.

        Sets the number of cameras for the repair lane's vision system.
        """
        self._lane.setVisionSystem(numCameras=0)

    def getLane(self):
        """
        Returns the constructed repair lane.

        Returns:
        - Lane: The fully constructed repair lane instance.
        """
        return self._lane


class AssemblyLaneBuilder(Builder):
    """
    A builder for constructing an assembly lane.

    Inherits from the Builder class and implements the methods for building
    a lane that includes conveyors, sensors, actuators, and vision systems
    specific to an assembly lane.
    """

    def __init__(self):
        """
        Initializes the AssemblyLaneBuilder with a new Lane instance.
        """
        self._lane = Lane()

    def buildConveyors(self):
        """
        Builds conveyors specific to the assembly lane.

        Sets the type and number of conveyors for the assembly lane.
        """
        self._lane.setConveyors(
            numChainConveyors=1, numSideConveyors=2, numBeltConveyors=2
        )

    def buildActuators(self):
        """
        Builds actuators specific to the assembly lane.

        Sets the type and number of actuators for the assembly lane.
        """
        self._lane.setActuators(numSmallActuators=4, numSmallBelt=2, numStoppers=5)

    def buildSensors(self):
        """
        Builds sensors specific to the assembly lane.

        Sets the number of sensors for the assembly lane.
        """
        self._lane.setSensors(numSensors=8)

    def buildVisionSystem(self):
        """
        Builds the vision system for the assembly lane.

        Sets the number of cameras for the assembly lane's vision system.
        """
        self._lane.setVisionSystem(numCameras=0)

    def getLane(self):
        """
        Returns the constructed assembly lane.

        Returns:
        - Lane: The fully constructed assembly lane instance.
        """
        return self._lane


class VisionLaneBuilder(Builder):
    """
    A builder for constructing a vision lane.

    Inherits from the Builder class and implements the methods for building
    a lane that includes conveyors, sensors, actuators, and vision systems
    specific to a vision lane.
    """

    def __init__(self):
        """
        Initializes the VisionLaneBuilder with a new Lane instance.
        """
        self._lane = Lane()

    def buildConveyors(self):
        """
        Builds conveyors specific to the vision lane.

        Sets the type and number of conveyors for the vision lane.
        """
        self._lane.setConveyors(numChainConveyors=1, numBeltConveyors=4)

    def buildActuators(self):
        """
        Builds actuators specific to the vision lane.

        Sets the type and number of actuators for the vision lane.
        """
        self._lane.setActuators(
            numSmallBelt=1,
            numStoppers=4,
            numSideStoppers=4,
            numBigBelt=3,
            numCameraActuators=1,
        )

    def buildSensors(self):
        """
        Builds sensors specific to the vision lane.

        Sets the number of sensors for the vision lane.
        """
        self._lane.setSensors(numSensors=8)

    def buildVisionSystem(self):
        """
        Builds the vision system for the vision lane.

        Sets the number of cameras for the vision lane's vision system.
        """
        self._lane.setVisionSystem(numCameras=1)

    def getLane(self):
        """
        Returns the constructed vision lane.

        Returns:
        - Lane: The fully constructed vision lane instance.
        """
        return self._lane


class Director:
    """
    A director class that orchestrates the construction of a lane.

    The Director class is responsible for guiding the construction process
    using a specific builder and ensuring the components are built in the
    correct order.

    Attributes:
    - id (int): An identifier for the lane being built.\n

    """

    def __init__(self, id):
        """
            Initializes the Director with a specific lane ID.

            Parameters:
            - id (int): The ID to assign to the lane being constructed.\n
        Usage Example:
            Example usage:
            ```python
                vision_builder = VisionLaneBuilder()
                repair_builder = RepairLaneBuilder()
                assembly_builder = AssemblyLaneBuilder()

                dir = Director(id)
                vision_lane = dir.construct(vision_builder)
                repair_lane = dir.construct(repair_builder)
                assembly_lane = dir.construct(assembly_builder)
                self.path = dir.getPath()
                self.fluffyDict = {
                    "id": self.id,
                    "path": self.path,
                    "vision_lane": vision_lane.getLaneDict(),
                    "repair_lane": repair_lane.getLaneDict(),
                    "assembly_lane": assembly_lane.getLaneDict(),
                }
                dir.reset(Lane())
            ```
        """
        self.id = id

    def construct(self, builder):
        """
        Constructs the lane using the specified builder.

        This method guides the builder through the construction process and
        returns the fully constructed lane.

        Parameters:
        - builder (Builder): The builder object used to construct the lane.

        Returns:
        - Lane: The constructed lane instance.
        """
        self.path = builder.buildPath(self.id)
        builder.buildConveyors()
        builder.buildActuators()
        builder.buildSensors()
        builder.buildVisionSystem()
        return builder.getLane()

    def reset(self, lane):
        """
        Resets the specified lane.

        This method invokes the reset method of the lane to reinitialize it.

        Parameters:
        - lane (Lane): The lane to reset.
        """
        lane.reset()

    def getPath(self):
        """
        Returns the path of the current lane being built.

        Returns:
        - str: The current lane path.
        """
        return self.path
