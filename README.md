# Introduction to Robotics Module

This repository contains various resources and code for an "Introduction to Robotics" module, focusing on kinematics, robotic vision, and trajectory planning. Much of this comprised coursework submitted during my degree, and therefore may be a useful reference for those studying robotics modules.

## Notebooks

### Overview
The notebooks are valuable resources for understanding symbolic derivations in robotics, particularly forward kinematics and other concepts critical to robotic movement and control.

#### Contents
- `FrameTransformations.ipynb`: Demonstrations of frame transformations in robotic systems.
- `MobileRobotDerivations.ipynb`: Explorations of mobile robot motion and control.
- `RoboticVisionDerivations.ipynb`: Insights into robotic vision algorithms and techniques.
- `SerialManipulatorDerivations.ipynb`: Derivations related to serial manipulator dynamics and kinematics.
- `TrajectoryPlanning.ipynb`: Techniques for planning and executing robotic trajectories.

## Core Classes

### Overview
The core classes are the backbone of the repository, providing essential functionalities for robot kinematics and manipulations.

#### `Robot.py`
This class is the primary interface for defining and controlling a robot. It includes functions for setting up robot parameters, calculating kinematics, and managing movements.

#### `DenavitHartenberg.py`
A foundational class for applying the Denavit-Hartenberg convention, a standard method to represent the kinematic chains of robots.

#### `FrameTransformation.py`
Handles the transformations between different frames of reference in robotic systems, crucial for accurate position and orientation calculations.

#### `GeneralManipulator.py`
A versatile class designed for handling various manipulations with robotic arms, including kinematics and trajectory planning.

## Courseworks, Exam Code, and Tutorial Sheets

### Overview
This section includes coursework, exam solutions, and tutorial sheets from the "Introduction to Robotics" module. They are included for reference only.

#### Contents
- Coursework: Assignments covering key topics in robotics.
- Exam Code: Solutions to exam problems and theoretical exercises.
- Tutorial Sheets: Practice problems and tutorials for hands-on learning.

## Setup and Dependencies

To set up this project and run the notebooks and scripts, ensure you have the following dependencies installed:

- Python 3.x
- Jupyter Notebooks (for running `.ipynb` files)
- Necessary Python libraries (e.g., NumPy, Matplotlib, etc.) - refer to `requirements.txt` for a complete list.

Clone the repository and install the required libraries using:

```bash
git clone https://github.com/ITregear/introduction-to-robotics-module
cd introduction-to-robotics-module
pip install -r requirements.txt
```

## Contributions

Contributions to this project are welcome! If you have suggestions, bug reports, or enhancements, please feel free to submit a pull request or open an issue.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
