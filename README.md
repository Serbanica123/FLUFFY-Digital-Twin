# FLUFFY Simulation API
 This repository contains the code to control the Omniverse simulation of FLUFFY

# installing dependencies
use command: pip install isaacsim

# GitHub setup
- FLUFFY contains only the source code of the project
- Tests contains a few tests for different classes
- final lib build contains all files needed to build the .whl file
- demo code contains a few demos of the library
- robotarm contains a few tests with robotarms (not implemented yet)

# installing the library on other devices
30/01/2025 
ROBOSAPIENS – FLUFFY DIGITAL TWIN 
Version 0.1 

Introduction 

This document provides a step-by-step guide on how to install a Python Wheel (.whl) file in a custom directory within Isaac Sim, ensuring it does not get installed in the default Python environment of VS Code or the system. 

1. Prepare the. whl File 

Before starting, ensure you have the Python Wheel file (.whl) you want to install. You should have received a file named similar to: 

Fluffy_API_Code_S7-0.1.0-py3-none-any 

Save this file in an accessible location, such as: 

C:\Users\student\Downloads\my_api2-0.1.0-py3-none-any.whl 

2. Install the Wheel  
    Open PowerShell in VS Code. 
    Run the following command to install the package in the Isaac Sim directory: 
Note! Path can differentiate according to manually made directory  


Windows: 
pip install --no-user "C:\path\to\file\my_api2-0.1.0-py3-none-any.whl" 

 

ubuntu: 
pip install --no-user "/path/to/file/my_api2-0.1.0-py3-none-any.whl" 

 
If the installation is successful, you will see output similar to: 
Successfully installed my_api2-0.1.0 

If you encounter any errors, ensure you typed the command correctly and that the file paths are valid. 


4. Modify sys.path in Your Python Script 

Since the package is installed in a custom location, you must explicitly tell Python where to find it. 
Add the following line at the beginning of your Python script: 

import sys 
sys.path.append("location of wheel installation (inside isaac sim folder)") 

import my_api2 
print("my_api2 is correctly installed and imported!") 

​ 

 
