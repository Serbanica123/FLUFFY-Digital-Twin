import os
from setuptools import setup, find_packages

setup(
    name="Fluffy_API_Code_S7",
    version="0.1.0",
    author="Fluffy_S7_group",
    description="Controlling Fluffy API, VS Code, and Isaac Sim",
    packages=find_packages(),
)

print("NOTE: This package requires the 'Isaac Sim VS Code Edition' extension.")
print("Please install it manually from the VS Code Marketplace.")
