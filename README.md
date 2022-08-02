# Autonoship_simulation
Autonoship simulator based on Gazebo, ROS, FMU and Open Simulation Platform


### 1. System Requirements

This simulation is built on [Ubuntu 20.04](https://releases.ubuntu.com/20.04/) and [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu). If you are looking for the Ubuntu 16.04 version, please go to [this branch](https://github.com/Autonoship/Autonoship_simulation/tree/ubuntu16.04). The Ubuntu can be either a real machine or a virtual machine.
Before start, please follow the link [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) to install ROS noetic on a Ubuntu 20.04 first.

According to the link, open a terminal and run:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt update
    sudo apt install ros-noetic-desktop-full
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo rosdep init
    rosdep update

### 2. Install Gazebo

    sudo apt-get install libgazebo11-dev libignition-math6-dev

### 3. Install Tensorflow (placeholder)

The collision avoidance module will rely on the tensorflow.
    
### 4. Install FMI-library

Install FMI Library according to https://github.com/modelon-community/fmi-library and https://jmodelica.org/fmil/FMILibrary-2.0.3-htmldoc/index.html, or try:

    cd 
    git clone https://github.com/modelon-community/fmi-library.git
    mkdir build-fmil; cd build-fmil
    cmake -DFMILIB_INSTALL_PREFIX=~/FMI_library ~/fmi-library
    make install test

### 5. Install libcosim

Install Conan and the libcosim library according to https://github.com/open-simulation-platform/libcosim, or try:

    sudo pip install conan
    sudo apt-get install libmsgsl-dev
    
    cd 
    git clone https://github.com/open-simulation-platform/libcosim.git
    conan remote add osp https://osp.jfrog.io/artifactory/api/conan/conan-local
    export CONAN_REVISIONS_ENABLED=1
    
    cd libcosim
    mkdir build
    cd build
    conan install -u .. --build=missing --settings build_type=Debug --settings compiler.libcxx=libstdc++11
    cmake .. -DLIBCOSIM_USING_CONAN=TRUE -DCMAKE_BUILD_TYPE=Debug -DLIBCOSIM_TREAT_WARNINGS_AS_ERRORS=OFF -DLIBCOSIM_BUILD_APIDOC=OFF
    cmake --build .

If you are using an Conan older than 1.40.3, and fail to connect to conancenter in the "conan install" process, try:

    conan config install https://github.com/conan-io/conanclientcert.git

For details, please check https://github.com/conan-io/conan/issues/9695

### 6. Create Workspace & Environment Configuration

Download the /autonoship_simulation, /collision_avoidance and /usv_gazebo_plugins into the /home directory. In a terminal, run:
 
    mkdir -p ~/autonoship/src
    cd ~/autonoship/
    catkin_make

    git clone https://github.com/Autonoship/Autonoship_simulation.git
    mv Autonoship_simulation/autonoship_simulation/ Autonoship_simulation/collision_avoidance/ Autonoship_simulation/navigation_control/ Autonoship_simulation/usv_gazebo_plugins/ Autonoship_simulation/osp_ros_demo/ Autonoship_simulation/path_following/ Autonoship_simulation/path_planning/ Autonoship_simulation/situation_awareness/ src

    sudo apt-get install ros-noetic-hector-gazebo-plugins ros-noetic-pid ros-noetic-fmi-adapter libgsl-dev ibmsgsl-dev

    catkin_make
    source devel/setup.bash
    echo  'source ~/autonoship/devel/setup.bash' >> ~/.bashrc 

    rm src/osp_ros_demo/CMakeLists.txt 
    mv Autonoship_simulation/CMakeLists_osp_ros_demo.txt src/osp_ros_demo/CMakeLists.txt
    catkin_make

    roscd autonoship_simulation/scripts
    chmod +x key_publisher.py keys_to_rudder.py setpoint_pub.py

    roscd situation_awareness/scripts
    chmod +x radar_reader.py radar_tracking.py state_reader.py target_state.py true_state.py

    roscd collision_avoidance/scripts
    chmod +x predict_action.py collision_avoidance_placeholder.py
    
    roscd path_following/scripts
    chmod +x path_following.py
    
    roscd path_planning/scripts
    chmod +x path_planning.py show_own_path.py

    pip install pyyaml
    echo  'export GAZEBO_RESOURCE_PATH=~/autonoship/src/usv_gazebo_plugins/fmu:$GAZEBO_RESOURCE_PATH' >> ~/.bashrc 
    
    pip install scipy matplotlib pygame pymunk==4.0.0 opencv-python scikit-fmm pygad

If you failed to find Boost during catkin_make, try to add the Boost_DIR to the ~/autonoship/src/CMakeLists.txt:
    
    set(Boost_DIR /usr/lib/x86_64-linux-gnu/cmake/Boost-1.71.0)


### 7. Usage

The osp_ros_demo package keeps all other functions same to the raw FMU simulation, but simulates the dynamics of every ship with OSP. The connection between OSP and ROS is realized by the node OSP_bridge. To test the ROS-OSP demo, run:

    roslaunch osp_ros_demo osp_autonoship_gazebo.launch scenario:=scenario1

To test the path planning module and OSP simulation based on FMUs provided by USC members, run:

    roslaunch osp_ros_demo osp_autonoship_gazebo.launch scenario:=scenario18 spawn_land:=true

The simulations defined in autonoship_simulation access FMUs with fmi_adapter and FMUCoSimulation. To test the collision avoidance module without using OSP, run scenario 1:

    roslaunch autonoship_simulation autonoship_gazebo.launch scenario:=scenario1
    
To run a certain testing scenario (e.g. scenario8), run:
    
    roslaunch autonoship_simulation autonoship_gazebo.launch scenario:=scenario8

To run the PID autopilot, run: 

    roslaunch autonoship_simulation autonoship_gazebo.launch scenario:=scenario1 autopilot:=PID
    
To control targetships, publish message in the terminal:

    rostopic pub targetship1/u2 std_msgs/Float64 10000000
    
To test the radar module, echo the radar feedback in the terminal:

    rostopic echo ownship/logical_camera

To change the RPM of the radar to 80, run in a terminal:

    rostopic pub ownship/rpm std_msgs/Float64 80
    
To test the radar tracking module, echo the state of a targetship (e.g. targetship1):

    rostopic echo ownship/targetship1/state

