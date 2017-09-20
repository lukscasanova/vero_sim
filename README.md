# vero_sim
Simulation Environment for project VERO

This project uses the [Dataspeed Drive-by-Wire](https://bitbucket.org/DataspeedInc/dbw_mkz_ros) packages to simulate an Ackerman vehicle.

# Setup

Install DATASPEED gazebo-related packages: https://bitbucket.org/DataspeedInc/dbw_mkz_simulation

Install can-msgs
`sudo apt-get install ros-kinetic-can-msgs`

Get the other repositories

```
cd src
rosws update
```

# Instructions

## Simulation Only

```
roslaunch vero_sim vero_sim.launch
```

## RANSAC Test

```
roslaunch vero_sim vero_ransac_test.launch
```

## Custom Obstacles Worlds

We can create a custom obstacle track for the car composed of cylinders by following the example on this file:
(obstacle.urdf.xacro)[src/vero_sim/objects/obstacles.urdf.xacro]

Create a copy of that file and modify it, then launch:

```
roslaunch vero_sim custom_obstacles world_xacro:=PATH_TO_WORLD_XACRO_HERE
```


# Videos

You can see the simulator working in the video:

[RANSAC Control](https://www.youtube.com/watch?v=ARcp8niD5iI)



