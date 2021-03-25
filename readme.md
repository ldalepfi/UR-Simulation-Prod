# Universal Robot Simulation
This repository contains tools for simulation and path visualisation of the Universal Robot 10. The program is supplied with a print area coordinates for a stack of boxes which it then traverses and prints over.


## Robot Program
[urpgrogams](urprograms) \
Contains directories for pre-production (simulation) and production (physical) Universal Robot Programs (URP). 

####Error Enumerations
-0: Gantry not locked \
-1: Gantry double locked \
-10: Point out of x bounds \
-20x:


## Portmarking Simulation
This  leverages the universal robot RTDE (real-time data exchange) python bindings to control the IO and take physical encoder readings. This provides a way to test the programming without the physical robots or a PLC. 

[portmark.py](portmark.py) runs the simulation\
[portmark.xml](portmark.xml) details the IO mappings

#### Operating
Before running the portmark.py simulation
1. Select the appropriate stack formation type from the enumeration provided at the top of the script.
2. Select the operating host address and port {physical=12.10.11.21:30004, simulated=ursim:30004}. The script must properly terminate before re-running to release the socket.
3. Ensure that the robot is powered on:\
    PHYS) Configure PC to static ip in the same network. Connect ethernet cable between PC and robot. Load PreProd URP, power on.\
    SIM) Runrrr the [emulator](https://www.universal-robots.com/download/software-cb-series/simulator-non-linux/offline-simulator-cb-series-non-linux-ursim-3150/). Load PreProd URP, power on.
   
## Portmarking 3D Visualisation
This script uses joint angles with forward kinematics from either simulation or a physical run to visualise the path the robot takes, and IO readings to see where printing has occured. This is  useful for debugging the URP and optimising the path. Displayed speed readings may be used to verify that the velocity is constant during the print cycle.

[robo_plotting.py](robo_plotting.py) runs the visualisation graph \
[data.csv](data.csv) Data extracted from the most recently finished program portmark.py run.

#### Markups
- (-, blue) Robot
- (x, blue) Robot joints
- (--, yellow) Path followed
- (*, magenta) The centre of print areas (SIDE A) 
- (*, magenta) The centre of print areas (SIDE B) 
- (o, red) HI print bits
 

