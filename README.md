# Hypatia #11391 Python Codebase

## Overview

This is the robot code for Hypatia 11391's robot. The original code was Java, but was rewritten in Python in 2026. The programming paradigm used is OOP (Object Oriented Programming) and Command-Based Programming.

For more information, check out our [website](https://sites.google.com/visst.ca/hypatiafrc/home).

## Environment Setup

Before cloning the repo, make sure you have your favorite IDE (Integrated Development Environment), [Python](https://www.python.org/downloads), and [Git](https://git-scm.com/install) installed on your computer. Hypatia#11391 officially recommends [VS Code](https://code.visualstudio.com) but any IDE should work. Once all of the above is installed, clone the repo with `git clone https://github.com/Hypatia11391/BIOCORE_2027_off_season.git`. Once downloaded, setup a virtual environment with `python -m venv .venv` and install the robotpy package using pip `py -3 -m pip install robotpy`. Finally, run `py -3 -m robotpy sync`. You should now be able to run the code on your machine.

### 1. Install your favorite IDE
  Any IDE works but Hypatia#11391 officially recommends [VS Code](https://code.visualstudio.com)

### 2. Install Python
  Go to the official [Python](https://www.python.org/downloads) website and follow the install instructions for your platform

### 3. Install Git
  Go to the official [Git](https://git-scm.com/install) website and follow the install instructions for your platform. Make sure it is installed by running `git --version` in your terminal.


### 4. Clone the main repo
  In your terminal or powershell run `git clone https://github.com/Hypatia11391/BIOCORE_2027_off_season.git` in the folder where you want the code to be

### 5. Setup a python virtual environment and install required packages

- #### Windows:
  1. run `py -3 -m venv .venv`
  2. run `source .venv/Scripts/activate`
  3. run `py -3 -m pip install robotpy`
  4. run `py -3 -m robotpy sync`

- #### MacOS/Linux:
  1. run `python3 -m venv .venv`
  2. run `source .venv/bin/activate`
  3. run `python3 -m pip install robotpy`
  4. run `python3 -m robotpy sync`


### 6. How to test code on your machine and deploy it to the robot
- #### Windows
- To test code you can run `py -3 -m robotpy sim`
- To deploy code you can run `py -3 -m robotpy deploy`

- #### MacOS/Linux
- To test code you can run `python3 -m robotpy sim`
- To deploy code you can run `python3 -m robotpy deploy`

### 7. Project layout and structure
All our code is written in the `src/` folder in the root of the project. `src/` contains `commands/`, `navx/`, and `subsystems/` each containing specific parts of the robot code. The directory names should be self explanatory. In the root directory there is also the `robot.py` file, which is the entry point for the program. Everything in the `deploy/` directory gets sent to the robot along with the rest of the code when `python -m robotpy deploy` is ran. the `tests/` directory will hopefully contain something by the time you're reading this. The `pyproject.toml` lists required libraries, versions, etc.

## Contributing

### Naming guidelines
All variables and functions in this project follow the snake case naming convention where variables and functions are written as: `this_is_a_variable`. Please familiarize and use this naming convention. When naming your own variables or functions please use a name that describes it or otherwise makes it clear to others why it is there. Example: If you have a robot speed variable don't name it something like `rs`, instead do something like `robot_speed`.

### Git branch naming
If you are working on a branch by yourself, use your first name and then a dash (-) or slash (/) and then what you are working on. For example I am using `sasha-update-readme` for writing this document. As long as its clear to others who is working on the branch and what they're working on its probably fine. 

### How to use branches
Never and I mean NEVER push code to the main branch under any circumstances. Any change, no matter how small deserves its own branch which will then be merged into main via a pull request. 


### 8 Resources

If you want to learn more about programming and FRC check out the following resources:
- [WPILib Docs](https://docs.wpilib.org/en/stable/docs/software/python/index.html)
- [Python Docs](https://docs.python.org/3)
- [Python Learning Courses](https://www.codecademy.com/catalog/language/python)
- [FRC Website](https://firstroboticscanada.org/frc)
- [Hypatia#11391 Website](https://sites.google.com/visst.ca/hypatiafrc/home)
