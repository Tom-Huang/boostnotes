## ROS srv and msg

### 1. create a msg

Step 1. create msg file in msg folder
```bash
roscd beginner_tutorials
mkdir msg
cd msg
echo "int64 num" > msg/Num.msg
```

Step 2. modify the package.xml file, add the following lines into it

```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

Step 3. modify CMakeLists.txt file, change the find_package(), catkin_package() and add_message_files() contents

```text
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```

```text
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```

```text
add_message_files(
  FILES
  Num.msg
)
```

```text
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

### 2. create a srv

Step 1. create srv file in msg folder
```bash
roscd beginner_tutorials
mkdir srv
# we copy one srv file from another package
roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
```

Step 2. modify the package.xml file, add the following lines into it

```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

Step 3. modify CMakeLists.txt file, change the find_package(), catkin_package() and add_message_files() contents

```text
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```

```text
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```

```text
add_service_files(
  FILES
  AddTwoInts.srv
)
```

```text
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

### Build DSO in ROS



























### end of page