# README File

Last updated: 2024 - 08 - 01

Created by: Jesse Poland


## Introduction

This README file is created to explain the folder structure and content for running and analysing OpenTrafficSim (OTS) models from Python. Python is used to utilize the extensive exploration tools of the EMA_Workbench and use other Python packages for analysing output data and creating visualisations.

# Setting up the environment
The environment can be created using the requirements.txt file, which contains all Python package dependencies to run all code within the project. To install these packages in PyCharm, double click the requirements.txt file.

# Running OTS models
Python scripts containing the OTS models are saved in the ots_models folder. An example of how to define an OTS model in python is present in ots_vehicle_automation_model.py.

The OTS model script (ots_vehicle_automation_model.py) communicates with java_manager.py to compile the Java project and run the Java class of the simulation model.

The OTS models can run with or without visualisations:
- With visualisations: headless = false in the OTS model script (ots_vehicle_automation_model.py)
- Without visualisations: headless = true in the OTS model script (ots_vehicle_automation_model.py)

Within this project, EMA_Workbench is used to run OTS models by setting their parameters (levers, constants, and uncertainties) and store/analyse output parameters. Examples can be found in open_exploration/open_exploration_runs.py and experiment_runs/policy_runs.py.

## Folder structure
```
├── `README.md`                             <- this README file
│
├── `.old`                                  <- old code
│   ├──
│
├── experiment_runs                         <- Scripts to run the OTS model with specific parameters for experiments
│   └── `policy_runs.py`
│
├── notebooks                               <- Jupyter notebooks
│   ├──
│
├── open_exploration                        <- Scripts to explore and analyse the OTS model
│   ├──
│
├── ots_models                              <- Scripts containing OTS models and the Java manager
│   ├── `java_manager.py`
│   └── `ots_vehicle_automation_model.py`
│                         
├── test                                    <- Scripts to test individual code
│   ├──
│
├── `requirements.txt`                      <- File with the Python package dependencies to install in your environment
│
```

