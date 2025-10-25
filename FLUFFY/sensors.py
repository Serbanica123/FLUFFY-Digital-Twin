import numpy as np
from omni.isaac.sensor import _sensor


class Sensors:  
    """
    A class to control all lightbeam-sensors within the FLUFFY system.

    this class provides methods to read processed distances from the sensors and calculate sensor outputs
    based on a cutoff value.

    Args:
        sensorDict (dict[str, str]): A dictionary where keys are sensor IDs and values are prim paths.
    
    Example Usage:
    ```python
        sensorDict = {"A12": "/World/LightBeam_Sensor",
                      "B32": "/World/LightBeam_Sensor_01"}

        sensors = Sensors(sensorDict)
    ```
    """

#whole class is tested and validated. the direct distance from a lightbeam sensor is now calculated from the raw data

    def __init__(self, sensorDict: dict[str, str]):
        self.sensorDict = sensorDict

        # remove this after testing
        self.lightbeamSensorDict = {sensorID: primPath for sensorID, primPath in self.sensorDict.items()}

        #raise warning if a non-lightbeam sensor was used as input
        if len(self.lightbeamSensorDict) < len(self.sensorDict):
            raise UserWarning("one or more items in sensorDict was not a lightbeam sensor and will not be used")
        
        # Get the lightbeam sensor interface
        self._ls = _sensor.acquire_lightbeam_sensor_interface()

    @staticmethod
    def _calculate_combined_value(values: list[float]) -> float:
        """
        Calculate a single value based on how many directions have values > 0.
        - For 1 value: return as-is.
        - For 2 values: apply Pythagorean theorem.
        - For 3 values: apply Pythagorean theorem iteratively.

        Args:
            values (list[float]): A list of sensor values.

        Returns:
            float: The combined value.
        
        this method is used internally in the class and is not supposed to be used outside of it.
        """
        
        positive_values = values[values > 0]

        if len(positive_values) == 1:
            return positive_values[0]
        elif len(positive_values) == 2:
            return np.sqrt(positive_values[0]**2 + positive_values[1]**2)
        elif len(positive_values) == 3:
            first = np.sqrt(positive_values[0]**2 + positive_values[1]**2)
            return np.sqrt(first**2 + positive_values[2]**2)
        else:
            return 0.0  # Default when no positive values exist

    def getProcessedSensorData(self, sensorID: str) -> float:
        """
        Get the processed lightbeam sensor data for a single sensor.

        Args:
            sensorID (str): The ID of the sensor.

        Returns:
            float: The processed sensor data.
        
        Example Usage:
        ```python
            data = sensors.getProcessedSensorData("sensor_1")
        ```
        """
        if sensorID in self.lightbeamSensorDict:
            primPath = self.lightbeamSensorDict[sensorID]
            raw_data = self._ls.get_hit_pos_data(primPath)  # Assuming raw_data is a list of directional values
            return np.round(self._calculate_combined_value(raw_data), 4)
        else:
            raise ValueError(f"Sensor ID {sensorID} does not exist")

    def getAllProcessedSensorData(self) -> list[float]:
        """
        Get the processed data for all lightbeam sensors.

        Returns:
            list[float]: A list of processed sensor data.

        Example Usage:
        ```python
            all_data = sensors.getAllProcessedSensorData()
            print(all_data)
        ```
        """
        return [
            np.round(self._calculate_combined_value(self._ls.get_hit_pos_data(primPath)), 4)
            for primPath in self.lightbeamSensorDict.values()
        ]

    def getSensorOutput(self, cutoffValue: float, sensorID: str) -> bool:
        """
        Get the output (True/False) based on processed data for a single sensor.

        Args:
            cutoffValue (float): The threshold value for the sensor.
            sensorID (str): The ID of the sensor.

        Returns:
            bool: The output of the sensor.

        Example Usage:
        ```python
            output = sensors.getSensorOutput(0.03, "LightBeam_Sensor_01")
        ```
        """
        processed_data = self.getProcessedSensorData(sensorID)
        return processed_data < cutoffValue

    def getAllSensorOutputs(self, cutoffValue: float) -> list[bool]:
        """
        Get the outputs (True/False) based on processed data for all sensors.

        Args:
            cutoffValue (float): The threshold value for the sensors.

        Returns:
            list[bool]: A list of outputs for each sensor.
        
        Example Usage:
        ```python
            all_outputs = sensors.getAllSensorOutputs(2.5)
            print(all_outputs)
        ```
        """
        return [
            self.getProcessedSensorData(sensorID) < cutoffValue
            for sensorID in self.lightbeamSensorDict
        ]
