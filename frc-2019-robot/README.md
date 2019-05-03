# Team 7445 FRC 2019 C++ Robot Code

## Setup

Development was done completely inside of JetBrains CLion. The makefile is already generated, so do NOT run the gradle command for CLion. Instead, run the gradle task `build` to grab the necessary libraries then `File > Reload CMake Project` to set everything up.

## Structure

The Robot class is where everything comes together. It reads all of the inputs and packages it into a Command struct, which is interpreted by each subsystem. It also handles adding routines to the routine manager based on the operator's input. Each subsystem has control over their physical system's controllers. It can either be locked or unlocked, when unlocked the operator directly controls the subsystem, when locked usually a routine is controlling it. A controllable subsystem has different subsystem controllers which alter the behavior for certain use cases. The subsystem will forward the Command struct to whatever controller currently has control, where outputs to the subsystems physical systems are determined.

### lib

Contains all of the classes that can be used independent of any specific robot. This includes:

* Routines
    * Routine Manager
    * Parallel Base
    * Sequential Base
    * Wait Base
    * Subsystem Routine
* Limelight
* Logger
* Subsystem
* Controllable Subsystem
* Subsystem Controller

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