import omni.graph.core as og
import numpy as np

class conveyers():

    def __init__(self, debugMode= False) -> None:
        self.debugMode = debugMode
        self.velocityPrimPath = "/action_graph/print.inputs:text" #change this to propper prim path
        self.directionPrimPath = "/action_graph/print.inputs:text" #change this to propper prim path
        self.enablePrimPath = "/action_graph/print.inputs:text" #change this to propper prim path

    def get_conveyer_velocity(self) -> float:
        # get existing value from an attribute
        current_velocity = og.Controller.attribute(self.velocityPrimPath).get()
        if self.debugMode:
            attributeType = og.Controller.attribute(self.directionPrimPath).attribute_type()
            print("Current velocity: ", current_velocity)
            print("and needed attribute type is: ", attributeType)
        return current_velocity

    def get_conveyer_direction(self):
        # get existing value from an attribute
        current_direction = og.Controller.attribute(self.directionPrimPath).get()
        if self.debugMode:
            attributeType = og.Controller.attribute(self.directionPrimPath).attribute_type()
            print("Current velocity: ", current_direction)
            print("and attribute type is: ", attributeType)
        return current_direction

    def set_conveyer_velocity(self, velocity) -> None:
        # set new value
        og.Controller.attribute(self.velocityPrimPath).set(velocity)
        if self.debugMode:
            print("Succesfully set conveyer velocity to: ", velocity)
    
    def set_conveyer_direction(self, direction) -> None:
        # set new value
        og.Controller.attribute(self.directionPrimPath).set(direction)
        if self.debugMode:
            print("Succesfully set conveyer direction to: ", direction)
    
    def toggle_conveyer(self) -> bool: #returns the new state of conveyers
        current_direction = og.Controller.attribute(self.enablePrimPath).get()
        if current_direction is True:
            og.Controller.attribute(self.enablePrimPath).set(False)
            return False
        else:
            og.Controller.attribute(self.enablePrimPath).set(True)
            return True