from omni.isaac.manipulators import PickPlaceController
from omni.isaac.ur5 import UR5Controller  # Ingebouwde UR5 controller in Isaac Sim
from omni.isaac.manipulators.grippers.gripper import Gripper
import numpy as np

# Maak een UR5 controller aan voor de UR5 robot
ur5_controller = UR5Controller(name="ur5_controller")

# Maak een gripper aan (als je een gripper hebt toegevoegd aan de robot)
gripper = Gripper(name="gripper")

# Maak een PickPlaceController aan
pick_place = PickPlaceController(
    name="pick_place",
    cspace_controller=ur5_controller,
    gripper=gripper,
)

# Voorbeeld van het gebruik van de PickPlaceController
picking_position = np.array([0.5, 0.0, 0.1])  # Locatie van het object om op te pakken
placing_position = np.array([0.6, 0.1, 0.1])  # Locatie waar het object neergezet moet worden
current_joint_positions = np.array([0, 0, 0, 0, 0, 0])  # Huidige gewrichtsposities van de UR5

# Voer de forward stap uit
action = pick_place.forward(
    picking_position=picking_position,
    placing_position=placing_position,
    current_joint_positions=current_joint_positions
)

# Controleer of de operatie voltooid is
if pick_place.is_done():
    print("Pick and place operation is complete.")
