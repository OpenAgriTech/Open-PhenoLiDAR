# Installation Guide - MAVROS - PX4 Autopilot
###What is MAVROS?
Mavros is a ROS package that enables MAVLink extendable communication between computers
running ROS for any MAVLink enabled autopilot, ground station or peripheral. MAVROS is
the official supported bridge between ROS and the MAVLink protocol.

Look for more information about MAVROS [here](http://wiki.ros.org/mavros).

### What is MAVLink?
MAVLink is a very lightweight messaging protocol for communicating with drones and between onboard drone components.
MAVLink follows a moder hybrid publish-subcribe and point-to-point design pattern: Data streams are sent (published
as topics) while configuration sub-protocols such as the mission protocol or parameter protocol are point-to-point
with retransmission. 

Look for more information about MAVLink [here](https://mavlink.io/en/). 
### Installation Guide
This documentation explains how to set up communication between PX4 Autopilot and ROS. 
MAVROS can be installed either from source or binary. Source installation highly recommended for developers.


 + ##### Binary installation (Debian/Ubuntu)
   The ROS repository has binary packages for Ubuntu x86, amd64(x86_64) and armhf (ARMv7).
   Kinetic also supports Debian Jessie amd64 and arm64 (ARMv8). 
   
   Use `apt-get` for installation: 
    ```shell script
    sudo apt-get install ros-kinetic-mavros
    sudo apt-get install ros-kinetic-mavros-extras
    sudo apt-get install ros-kinetic-mavros-msgs
    ```   
   Then install [GeographicLib](https://geographiclib.sourceforge.io/) datasets by running the `install_geographiclib_datasets.sh` script:
    ```shell script
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    sudo bash ./install_geographiclib_datasets.sh 
    ```  
 + ##### Source installation
    1. **Check catkin workspace**
    
        This guide assumes you have a catkin workspace located at `~/agrolaser_ws/catkin_ws`. If you don't create one with: 
        
        ```shell script
        mkdir -p ~/agrolaser_ws/src
        cd ~/agrolaser_ws/
        catkin init
        wstool init src
        ```
       You will be using the ROS Python tools: *wstool (for retrieving sources)*, *rosinstall*,
       and *catkin_tools* (building) for this installation. While they may have been installed
       during your installation of ROS you can also install them with: 
        ```shell script
        sudo apt-get install python-catkin-tools python-rosinstall-generator -y
        ```   
        ***TIP: `catkin_make` is the default method for built packages, the alternative is `catkin_tools`, highly recommended.***
        
        If this your first time using **wstool** you will need to initialize your source space with:
       
        ```shell script
        $ wstool init ~/catkin_ws/src
    
        ```
    
    2. **Install MAVLink**
        ```shell script
        # We use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
        rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
        ```
          
    3. **Install MAVROS and EXTRAS**
        1. Choose between released or latest version:
            + Released/stable
                 ```shell script
                 rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
                 ```
           + Latest source 
             ```shell script
             rosinstall_generator --upstream-development mavros | tee -a /tmp/mavros.rosinstall
             ```
             **TIP:** For fetching all dependencies into your catkin_ws, just add `--deps` to the above scripts, E.g.:
              `rosinstall_generator --upstream mavros --deps | tee -a /tmp/mavros.rosinstall`
             
        2. **Installation of MAVROS-extras**
           
           ***mavros-extras*** is a fundamental packag
             e for monitoring GPS status and odometry.
           ```shell script
           sudo apt-get install ros-kinetic-mavros-extras
           ```
           ***mavros-msgs*** is required package for displaying mavros-extras topics through the shell.  
           ```shell script
           sudo apt-get install ros-kinetic-mavros-msgs
           ```
           Github link of [***mavros-extras***](https://github.com/mavlink/mavros/tree/master/mavros_extras) repository.
           
           Github link of [***mavros-msgs***](https://github.com/mavlink/mavros/tree/master/mavros_msgs) repository.
           
    4. **Create workspace & deps**
         ```shell script
         wstool merge -t src /tmp/mavros.rosinstall
         wstool update -t src -j4
         rosdep install --from-paths src --ignore-src -y
         ```
       
    5. **Install [GeographicLib](https://geographiclib.sourceforge.io/) datasets:**
         ```shell script
         ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
         ```
       
    6. **Build source**
       ```shell script
       cd ~/agrolaser_ws/src/
       catkin build
       ```
    
    7. **Check use of ROS environment variables**
       ```shell script
       #Needed or rosrun can't find nodes from this workspace.
       source  ~/agrolaser_ws/devel/setup.bash
       ```
    
    
#####Resources: 
 - [mavros-extras repository](https://github.com/mavlink/mavros/tree/master/mavros_extras)
 - [mavros-msgs repository](https://github.com/mavlink/mavros/tree/master/mavros_msgs)
 - [MAVLink Developer Guide](https://mavlink.io/en/)    
 - [Package Summary ROS - mavros](http://wiki.ros.org/mavros)
 - [PX4 MAVROS Installation Guide](https://docs.px4.io/master/en/ros/mavros_installation.html)

