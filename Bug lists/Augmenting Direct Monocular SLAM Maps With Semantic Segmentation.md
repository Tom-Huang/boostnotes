# Augmenting Direct Monocular SLAM Maps With Semantic Segmentation

## How to build DSO for ROS

1. Create a catkin work space and clone the repositories
    ```bash
    mkdir catkin_ws
    cd catkin_ws
    mkdir src
    cd src
    git clone https://github.com/JakobEngel/dso.git
    git clone https://github.com/NikolausDemmel/dso_ros.git
    cd dso_ros
    git checkout catkin
    cd ../.. # go back to catkin_ws
    ```

2. Build DSO
    ```bash
    cd src/dso
    mkdir build && cd build
    cmake ..
    make
    cd ../../.. # go back to catkin_ws
    ```

3. Build dso_ros

    ```bash
    source /opt/ros/melodic/setup.bash
    catkin init
    export DSO_PATH=/your_path_to/catkin_ws/src/dso/
    catkin build
    ```

4. Run ROS
    run roscore on one terminal
    
    then run the following on the other terminal
    
    ```bash
    source devel/setup.bash
    rosrun dso_ros dso_live image:=/blackfly/image_raw calib=/media/hcg/Data/TUM/semester_project/dataset/heap_data/camera.txt 
    ```

5. Rebuild dso_ros

    ```bash
    catkin clean
    catkin build
    ```






















## end of file