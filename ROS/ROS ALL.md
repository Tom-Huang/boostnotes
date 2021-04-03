# ROS ALL

## Using member functions of a class as callback

[[roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks - ROS Wiki](https://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks)](https://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks )

## Rotation convention in ROS

1. quaternion msg are stored in x,y,z,w order. Access the msg with .x .y .z .w is ok
2. in the tf message, RPY represent first rotate around x axis, and then y, and then z(not sure in which frame)
3. blender rotation is stored in rx, ry, rz format representing first rotate around z for rz, and then around y for ry, and the around x for rx 

## Overlay workspace with catkin build tools

to make two workspace accessible we need to have a working workspace and a overlaid workspace

1. source the overlaid workspace with `source devel/setup.bash`
2. remove the existing build and devel folder with `catkin clean`
3. catkin build the working workspace with `catkin build`
4. source the working workspace with `source devel/setup.bash`