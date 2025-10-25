import omni.graph.core as og
import numpy as np

class conveyor():
    """
    A class to control a conveyor belt using OmniGraph attributes.
    
    Attributes:
        conveyorPrim (str): The base path of the conveyor in OmniGraph.
        debugMode (bool): Enables debug messages when set to True.
    
    Example Usage:
    ```python
        conveyor = Conveyor("/World/ConveyorGraph", debugMode=True)
    ```
    """

    def __init__(self, conveyorPrim= "" ,debugMode= False) -> None:

        self.debugMode = debugMode
        self.conveyorPrim = conveyorPrim
                
        self.velocityPrimPath = self.conveyorPrim + "/inputs:velocity" 
        self.directionPrimPath = self.conveyorPrim + "/inputs:direction" 
        self.enablePrimPath = self.conveyorPrim + "/inputs:enabled"
        self.animateDirectionPrimPath = self.conveyorPrim + "/inputs:animateDirection"
        self.animateTexturePrimPath = self.conveyorPrim+ "/inputs:animateTexture"
        #self.enabledState = None
        #self.animateTextureState = None

    def getVelocity(self) -> float: #verified
        """
        Retrieves the current velocity of the conveyor belt.
        
        Returns:
            float: The current velocity of the conveyor.
        
        Example Usage:
        ```python
            velocity = conveyor.getVelocity()
            print(velocity)
        ```
        """
        # get existing value from an attribute
        self.current_velocity = og.Controller.attribute(self.velocityPrimPath).get()
        if self.debugMode:
            #attributeType = og.Controller.attribute(self.velocityPrimPath).attribute_type()
            print("Current velocity: ", self.current_velocity)
            #print("and needed attribute type is: ", attributeType)
        return np.round(self.current_velocity, 4)

    def getDirection(self) -> list[float]:  #verified
        """
        Retrieves the current direction of the conveyor belt.
        
        Returns:
            list[float]: The current direction vector of the conveyor.
            this list represents the [X, Y, Z] components of the direction in isaac sim.

        Example Usage:
        ```python
            direction = conveyor.getDirection()
        ```
        """
        # get existing value from an attribute
        self.current_direction = og.Controller.attribute(self.directionPrimPath).get()
        if self.debugMode:
            #attributeType = og.Controller.attribute(self.directionPrimPath).attribute_type()
            print("Current direction: ", self.current_direction)
            #print("and attribute type is: ", attributeType)
        return self.current_direction

    def setVelocity(self, velocity: float) -> None: #verified
        """
        Sets the velocity of the conveyor belt.
        
        Args:
            velocity (float): The velocity to set for the conveyor.
        
        Example Usage:
        ```python
            conveyor.setVelocity(0.5)
        ```
        """
        # set new value
        og.Controller.attribute(self.velocityPrimPath).set(velocity)
        if self.debugMode:
            print("Succesfully set conveyer velocity to: ", velocity)
    
    def setDirection(self, direction: np.array) -> None: #verified
        """
        Sets the direction of the conveyor belt.
        
        Args:
            direction (np.array): A 2D vector specifying the direction.

        Example Usage:
        ```python
            conveyor.setDirection(np.array([1.0, 0.0, 0.0]))
        ```
        """
        self.animateDirection = np.array([direction[0], direction[1]])
        # set new values for conveyor direction and animate direction
        og.Controller.attribute(self.directionPrimPath).set(direction)
        og.Controller.attribute(self.animateDirectionPrimPath).set(self.animateDirection)
        if self.debugMode:
            print("Succesfully set conveyer direction to: ", direction)
            print("Succesfully set animation direction to: ", self.animateDirection)
    
    def enableBelt(self) -> bool: # Verified
        """
        Enables the conveyor belt. this method does NOT change conveyor speed, it only turns the conveyor on
        
        Returns:
            bool: True if the belt was enabled, False otherwise.

        Example Usage:
        ```python
            conveyor.enableBelt()
        ```
        """
        self.enabledState = og.Controller.attribute(self.enablePrimPath).get()
        if not self.enabledState:
            og.Controller.attribute(self.enablePrimPath,).set(True)
            if self.debugMode:
                print(f"conveyor: '{self.conveyorSelect}' is now enabled")
        else:
            if self.debugMode:
                print(f"conveyor: '{self.conveyorSelect}' was already enabled")
        return True
    
    def enableAnimateTexture(self) -> bool: # Verified 
        """
        Enables the texture animation of the conveyor. in order for this to work, the conveyor needs a texture to be applied.
        
        Returns:
            bool: True if animation was enabled, False otherwise.
        
        Example Usage:
        ```python
            conveyor.enableAnimateTexture()
        ```
        """
        self.animateTextureState = og.Controller.attribute(self.animateTexturePrimPath).get()
        if not self.animateTextureState:
            og.Controller.attribute(self.animateTexturePrimPath).set(True)
            if self.debugMode:
                print(f"animation of conveyor: '{self.conveyorSelect}' is now enabled")
        else:
            if self.debugMode:
                print(f"animation of conveyor: '{self.conveyorSelect}' was already enabled")
        return True

    def disableBelt(self) -> bool: # Verified
        """
        Disables the conveyor belt.  this method does NOT change conveyor speed, it only turns the conveyor off
        
        Returns:
            bool: False when the belt is disabled.
        
        Example Usage:
        ```python
            conveyor.disableBelt()
        ```
        """
        self.enabledState = og.Controller.attribute(self.enablePrimPath).get()
        if self.enabledState:
            og.Controller.attribute(self.enablePrimPath).set(False)
            if self.debugMode:
                print(f"conveyor: '{self.conveyorSelect}' is now disabled")
        else:
            if self.debugMode:
                print(f"conveyor: '{self.conveyorSelect}' was already disabled")
        return False

    def disableAnimateTexture(self) -> bool: # Verified
        """
        Disables the texture animation of the conveyor.  in order for this to work, the conveyor needs a texture to be applied.
        
        Returns:
            bool: False when the animation is disabled.
        
        Example Usage:
        ```python
            conveyor.disableAnimateTexture()
        ```
        """
        self.animateTextureState = og.Controller.attribute(self.animateTexturePrimPath).get()
        if self.animateTextureState:
            og.Controller.attribute(self.animateTexturePrimPath).set(False)
            if self.debugMode:
                print(f"animation of conveyor: {self.conveyorSelect}' is now disabled")
        else:
            if self.debugMode:
                print(f"animation of conveyor: '{self.conveyorSelect}' was already disabled")
        return False

# the toggle function was disabled because it will likely not work until omni.graph.core is updated
# this likely has to do with states sometimes not being recognised by the reading of omnigraphs
"""
    def toggleBelt(self) -> bool: #returns the new state of conveyers
        self.enabledState = og.Controller.attribute(self.enablePrimPath).get()
        og.Controller.attribute(self.enablePrimPath).set(not self.enabledState)
        if self.debugMode:
            print(f"enabled state after toggle is: {self.enabledState}")
        return self.enabledState
"""
