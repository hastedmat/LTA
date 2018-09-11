% Local Tangent Based A* Path Planning Algorithm
% Robotics Machine Intelligence (ROMI) Lab
% September 10, 2018 


Matlab Code for Local Tangent Based A* (LTA*)
========================================
This code provides basic implementation of LTA* in 2D maps with different environments. 
Maps used here are all binary maps (occupancy grids).
This source code contains two main files that can be used to run LTA* in matlab.

"AutoLTASTAR.m":
-------------
This file can be used to implement LTA* in any environment for path planning with pre defined start and goal location.

It has two vaiables named as "source" and "goal" containing cartesian points of start and goal location.

After planning initial path it will ask for user permission ("press and Key") for motion of robot on the planned path and will start moving on it. after every ten steps it will ask for permission again.

While motion on the planned path it also displays local tangents drawn from current location to obstacles. To view these points uncomment line 200 in "AutoLTASTAR.m"


"LTASTAR.m":
----------
"LTASTART.m" has graphical user interface thats lets user decide initial and final location on the map.

To select initial and final point,fisrt select initial position (start point) of robot by left click at any unoccupied area of map. After that select goal location (end point) by right click at any unoccupied location on the map.

Like "AutoLTASTAR.m" after initial path planning it will start moving on planned path after user permission ("press any key").

LTASTAR.m provides flexibility to change final (goal) locatoion of robot in the environment at the run time. to do so it will ask user for updated  location of goal in the environment which can be input using left mouse click.

While motion on the planned path it also displays local tangents drawn from current location to obstacles. To view these points uncomment line 200 in "LTASTAR.m"

Maps:
-----
To implement these source codes binary maps are also included in the package containing different obstacle environments from simple rectangles to complex mazes and obstacles.

Experimental Results:
--------------------
This package also contains experimental videos of LTA* implementation in real environments.



