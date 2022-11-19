# Automatic Parallel Parking: Path Planning, Path Tracking & Control

This repository contains a python implementation of an automatic parallel parking system in a virtual environment that includes path planning, path tracking, and parallel parking.
 The agent navigates its route through the environment and is directed to the assigned park location by the MPC controller. You can find the article on [Towards Data Science](https://towardsdatascience.com/automatic-parallel-parking-system-including-path-planning-path-tracking-and-parallel-parking-in-a-ece780b2e8e0).

## Envroinment
The first step to develop an auto park system is to design and develop an environment capable of giving visual render using ```OpenCV``` library.
The environment is implemented in ```environment.py``` as a class and receives obstacles at the beginning ```env = Environment(obs)```.
The agent can be placed using ```env.render(x,y,angle)```.
A sample of the environment is displayed below. You can choose the parking spot from 1 to 24.

![developed environment](https://user-images.githubusercontent.com/56114938/127310550-745d7123-f02f-48ae-96a7-9f82089b9fd9.JPG)

## Path Planning
#### A* Algorithm
The agent will find a path from start to its goal using A*. 
This implementation of A* from [PythonRobotics](https://pythonrobotics.readthedocs.io/en/latest/modules/path_planning.html) considers parameters like obstacles and robot radius.

#### Interpolating Path With B-spline
After finding a path in a discrete 100\*100 space, the path is smoothed and scaled to 1000\*1000 space of environment using b-spline.
The result is a set of points to guide our agent!

## Path Tracking
**The kinematic model** of the car is:
```math
\left\{\begin{matrix}
\dot{x} = v . cos(ψ)\\
\dot{y} = v . sin(ψ)\\
\dot{v} = a\\
\dot{ψ} = v . tan(δ)/L
\end{matrix}\right.
```
```a: acceleration, δ: steering angle, ψ: yaw angle, L: wheelbase, x: x-position, y: y-position, v: velocity```

**The state vector** is:
```math
z=[x,y,v,ψ]
```
```x: x-position, y: y-position, v: velocity, ψ: yaw angle```

**The input vector** is:
```math
u=[a,δ]
```
```a: acceleration, δ: steering angle```

#### Control
The MPC controller controls vehicle speed and steering based on the model and the car is directed through the path. There is an option for using the linearized model for MPC. In this case MPC linearizes the kinematic model around the operating point and then does the optimization.

## Parallel Parking
This part consists of 4 rules that the agent must choose according to parking position. 
At first, the agent will find a path to park position then it will compute the arriving angle. 
Based on the arriving angle, the agent chooses a coordinate as ensure1. 
After that, the parking path is planned from ensure1 to ensure2 using 2 circle equations as mentioned below. 
MPC controls the agent and car parks in ensure2 coordinate.

![automatic_parking_process](https://user-images.githubusercontent.com/56114938/128083454-60f8ba82-00a8-43a2-b8ad-8d4ad09cc762.gif)

## Inference
Run the code using this command:
```
$ python main_autopark.py --x_start 0 --y_start 90 --psi_start 0 --parking 7
```
Cite using [![DOI](https://zenodo.org/badge/328711550.svg)](https://zenodo.org/badge/latestdoi/328711550)

## About Us
We as Team Pandas won 1st place in the National Rahneshan competition 2020-2021 for autonomous vehicles. This contest has been one of the most competitive and challenging contests in the Rahneshan tournaments with more than 15 teams competing from top universities in Iran.

## Contact
Feel free to contact us via email or connect with us on linkedin.

- Amirhossein Heydarian ---  [Linkedin](https://www.linkedin.com/in/amirhosseinh77/), [Github](https://github.com/amirhosseinh77), [Email](mailto:amirhossein4633@gmail.com )
- Aida Mohammadshahi ---  [Linkedin](https://www.linkedin.com/in/aida-mohammadshahi-9845861b3/), [Github](https://github.com/aidamohammadshahi), [Email](mailto:aidamoshahi@gmail.com)
- Milad Soltany --- [Linkedin](https://www.linkedin.com/in/milad-soltany/), [Github](https://github.com/miladsoltany) , [Email](mailto:soltany.m.99@gmail.com)
- Abbas Omidi --- [Linkedin](https://www.linkedin.com/in/abbasomidi77/), [Github](https://github.com/abbasomidi77), [Email](mailto:abbasomidi77@gmail.com)
- Amirhossein Kazerooni ---  [Linkedin](https://www.linkedin.com/in/amirhossein477/), [Github](https://github.com/amirhossein-kz), [Email](mailto:Amirhossein477@gmail.com )

