from FLUFFY.sensors import *

# Define your sensor dictionary with sensor IDs and their corresponding prim paths
sensorPrimPaths = {
    "A12": "/World/LightBeam_Sensor",
    "B32": "/World/LightBeam_Sensor_01",
}

# Initialize the Sensors class
sensors = Sensors(sensorDict=sensorPrimPaths)

# Use the methods as before
raw_data_A12 = sensors.getProcessedSensorData("A12")
all_raw_data = sensors.getAllProcessedSensorData()
output_A12 = sensors.getSensorOutput(cutoffValue=0.5, sensorID="A12")
all_outputs = sensors.getAllSensorOutputs(cutoffValue=0.5)

print(f"raw data A12 = {raw_data_A12}")
print(f"all raw data = {all_raw_data}")
print(f"output A12 = {output_A12}")
print(f"all outputs = {all_outputs}")