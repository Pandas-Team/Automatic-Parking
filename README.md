# Auto park repo for Rahneshan competitions
This repository contains an auto park system including path planning, path tracking, and double parking in a designed envroinment.

## Envroinment
Our first step to develop an auto park system was to design and develop an environment capable of giving visual render using ```OpenCV``` library. Environment is implemented in ```environment.py``` as a class and recieves obstacles at the beginning. Agent can be placed using ```env.render(x,y,angle)```.

A sample of environment is displayed bellow.
![developed environment](extra/env.png)
## Path planning

#### A* algorithm
Agent will find a path from start to its goal using A*. This implementation of A* from [PythonRobotics](https://pythonrobotics.readthedocs.io/en/latest/modules/path_planning.html), considers parameters like obstacles and robot radius.

#### interpolating path with spline

## Path tracking

## Double parking
