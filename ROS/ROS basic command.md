## ROS basic command

Before running any command, please source the workspace:
```bash
source /opt/ros/melodic/setup.bash
```

### 1. create workspace

#### 1.1. catkin_make

create a workspace

```bash
mkdir -p catkin_ws/src
cd catkin_ws
catkin_make
```

### 2. File navigation

navigate through file system

#### 2.1. roscd
go to the package directory

```bash
roscd [package_name]
```

#### 2.2. rospack find [packagename]

find a ros package\'s path

```bash
rospack find [packagename]
```

#### 2.3. rosls

```bash
rosls [packagename]/subdirectory
```

#### 2.4. rospack depends1 [packagename]

list first order dependencies in one package

```bash
rospack depends1 [packagename]
```

### 3. create a package

#### 3.1. catkin_create_pkg

```bash
cd catkin_wd/scr
# catkin_create_pkg [nameofpackage] [depend1] [depend2] ...
catkin_create_package beginner_tutorials std_msgs rospy roscpp
cd catkin_ws
catkin_make
source devel/setup.bash
```

#### 3.2. interpretation of package.xml file

check the [link](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) to see the meaning of lines in package.xml file.

### 4. ROS node

#### 4.1. roscore

first thing to run

```bash
roscore
```

#### 4.2. rosrun [packagename] [nodename]

run a ros package

```bash
rosrun turtlesim turtlesim_node
```

remap the name of the node

```bash
# rename the node to my_turtle
rosrun turtlesim turtlesim_node __name:=my_turtle
rosnode list
```

the output of `ronode list` will be

```bash
/my_turtle
/rosout
```

#### 4.3. rosnode list

list all ros nodes

```bash
rosnode list
```

#### 4.4. rosnode cleanup

clean all the rosnode that are no longer existing from the rosnode list

```bash
rosnode cleanup
```

### 5. ROS topic

#### 5.1. rqt_graph

use rqt_graph to plot the dynamic graph of relaltionship between different nodes

```bash
sudo apt-get install ros-melodic-rqt
sudo apt-get install ros-melodic-rqt-common-plugins
rosrun rqt_graph rqt_graph
```

#### 5.2. rostopic list

list all rostopic

```bash
rostopic list
```

the output could be:

```bash
/rosout
/rosout_agg
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

#### 5.3. rostopic echo [topic]

show data published on the topic

```bash
rostopic echo /turtle1/pose
```

the output could be:

```bash
---
x: 3.79069209099
y: 5.13080310822
theta: 3.06081461906
linear_velocity: 0.0
angular_velocity: 0.0
---
```

#### 5.4. rostopic type [topic]

show the type of the topic

```bash
rostopic type /turtle1/pose
```

the output could be:

```bash
turtlesim/Pose
```

#### 5.5 rosmsg show [msg_type]

show the message structure

```bash
rosmsg show turtlesim/Pose
```

the output could be:

```bash
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
```

#### 5.6. rostopic pub [topic] [msg_type] [args]

publish data on to a topic

```bash
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```

```bash
-1 # means publish one time
-r 1 # makes it publish at 1Hz repeatedly
```

#### 5.7. rostopic hz [topic]

show at which rate data is published

```bash
rostopic hz /turtle1/cmd_vel
```

the output could be(after running the command in 5.6. with rate of 1Hz):

```bash
average rate: 1.000
        min: 1.000s max: 1.000s std dev: 0.00021s window: 10
```

#### 5.8. use rqt_plot 

plot the data on a certain topic

```bash
rosrun rqt_plot rqt_plot
# input the topic name in the text box in the GUI
```

### 6. ROS service

#### 6.1. rosservice list

list all services

#### 6.2. rosservice type \[service\]

get the type of the service

#### 6.3. rossrv show \[srv_type]

show the information to be sent and to be received in this service type

```bash
rosservice type /spawn | rossrv show
```

the output could be:

```bash
float32 x
float32 y
float32 theta
string name
---
string name
```

#### 6.4. rosservice call \[service\] \[args\]

call a service

```bash
rosservice call /clear
```

or an example with arguments

```bash
rosservice call /spawn 2 2 0.2 "" # args are x, y, theta and name
```

### 7. ROS parameters

#### 7.1. rosparam list

list all parameters for the current node

```bash
rosparam list
```

the output could be:

```bash
/background_b
/background_g
/background_r
/rosdistro
/roslaunch/uris/host_57aea0986fef__34309
/rosversion
/run_id
```

#### 7.2. rosparam set \[param_name] \[args] & rosparam get \[param_name]

set and get parameters

```bash
rosparam get /background_b
255 # output
```

```bash
rosparam set /background_b 150
rosservice call /clear # update
# and then the background color of turtlesim window will be changed
```

#### 7.3. rosparam dump \[filename] \[namespace]

save the current parameters in a file, \[namespace] specifies that we only save parameters in this namespace

```bash
rosparam dump paramters.yaml
```

#### 7.4. rosparam load \[filename] \[namespace]

load parameters from a .yaml file and assign them to a namespace

```bash
rosparam load parameters.yaml copy_namespace
rosparam list
```

the output could be:

```bash
/copy_namespace/background_b
/copy_namespace/background_g
/copy_namespace/background_r
/copy_namespace/rosdistro
/copy_namespace/roslaunch/uris/host_w212_3v_v4_eduroam_dynamic_rbg_tum_de__39895
/copy_namespace/rosversion
/copy_namespace/run_id
```

### 8. rqt_console and roslaunch

#### 8.1. rqt_console

display output of nodes. Run this before running nodes

```bash
rosrun rqt_console rqt_console
```

#### 8.2. rqt_logger_level

change the verbosity of nodes when they run

```bash
rosrun rqt_logger_level rqt_logger_level
```

#### 8.3. roslaunch

create a .launch file in subfolder `launch` in package, and use `roslaunch` to run serveral nodes at a time. Examples of .launch file can be found [here](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch).

```bash
roscd beginner_tutorials
mkdir launch
cd launch
touch mimic.launch # create and edit the launch file
roslaunch beginner_tutorials mimic.launch
```

### 9. rosed

#### 9.1. rosed

edit any file in a package

```bash
rosed [package_name] [filename]
```






















### end of page
