# Team 7445 FRC 2019 C++ Robot Code

## Setup

Development was done completely inside of JetBrains CLion. The makefile is already generated, so do NOT run the gradle command for CLion. Instead, run the gradle task `build` to grab the necessary libraries then `File > Reload CMake Project` to set everything up.

## Structure

Robot class is where everything comes together. It reads all of the inputs and packages it into a Command class, which is interpreted by each subsystem.

### lib

Contains all of the classes that can be used independent of any specific robot. This includes:

* Routines
    * Routine Manager
    * Parallel Base
    * Sequential Base
    * Wait Base
* Limelight
* Logger
* Subsystem

### routine

Has all the routines that are specific to the 2019 robot.

### subsystem

Contains each subsystem that are specific to the 2019 robot. This includes:

* Ball Intake
* Drive
* Elevator
* Flipper
* Hatch Intake
* Outrigger