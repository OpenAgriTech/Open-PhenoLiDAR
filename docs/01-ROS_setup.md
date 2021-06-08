# ROS SETUP - Step by Step

The operating system running for this guide is Linux Mint 20 Ulyana (OS running at the base is
Ubuntu 20), so it’s highly recommended to install the distribution of ROS ‘Noetic Ninjemys’,
otherwise you could have some compatibility issues.
For the installation process you could follow this link: 
[ROS-Noetic Official website.](http://wiki.ros.org/noetic/Installation/Ubuntu)
But here we have some tips for make it easier and quicklier.

1. **Configure your Ubuntu repositories**

    You need to configure your ubuntu repositories to allow ‘restricted’, ‘universe’ and
    ‘multiverse’. At the distribution of linux (Linux Mint 20 Ulyana) we are using it’s supposed
    to be setup by default.
    The command for enable that repositories are the following:
    
    ```shell script
    sudo add-apt-repository universe
    sudo add-apt-repository multiverse
    sudo add-apt-repository restricted
    ```
   
2. **Set up your sources.list**

    First of all we need to modify a file ‘ros-latest.list’, because the distribution that ROS
    support is Ubuntu focal and we have linux mint Ulyana; so we need to replace ***ulyana*** by
    ***focal***
    
    ```shell script
    nano /etc/apt/sources.list.d/ros-latest.list
    ```
   
    Replace the text inside by the one below: 
    
    ```shell script
    deb http://packages.ros.org/ros/ubuntu focal main
    ```
   
    Now setup your computer to accept software from packages.ros.org and type the following
    command.
    
    ```shell script
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/
    apt/sources.list.d/ros-latest.list'
    ```
   
3. **Set up your keys**

     ```shell script
     sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key
     C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
      ```
   
    if your experience issues connecting to the keyserver, you can try substituting 
    [hkp://pgp.mit.edu:80](hkp://pgp.mit.edu:80)
    or [hkp://keyserver.ubuntu.com:80](hkp://keyserver.ubuntu.com:80) in the previous command.
    Alternatively, you can use curl instead of the apt-key command, which can be helpful if you
    are behind a proxy server:
    
      ```shell script
    curl -sSL 'http://keyserver.ubuntu.com/pks/lookup? op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-
    key add -
      ```   
   
    Also, in the end of May 2021, the key for ROS has expired, which need to be updated if you get the following warning
   when your run `sudo apt update`: 

    ```shell script
    W: An error occurred during the signature verification. The repository is not updated and the previous index files will be used. GPG error: http://packages.ros.org/ros/ubuntu focal InRelease: The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
    W: Failed to fetch http://packages.ros.org/ros/ubuntu/dists/focal/InRelease  The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
    W: Some index files failed to download. They have been ignored, or old ones used instead.
    ```
   
    Update your key with the following command:
    ```sheel script
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    ```
   
    And run `sudo apt update` again. 

   
4. **Installation**

    First, make sure your Debian package index is up-to-date, if you change well the source list no issues with 
    the OS update.
    
      ```shell script
      sudo apt update
      ```   
   
   Then you could choose what version of ROS you would like to install:
   
    +  **Desktop-Full install:** (Recommended): Everything in Desktop plus 2D/3D
       simulators and 2D/3D perception packages.<br>
       ```shell script
       sudo apt install ros-noetic-desktop-full
       ```   
       
    +  **Desktop Install**: Everything in ROS-Base plus tools like rqt and rviz.<br>
       ```shell script
       sudo apt install ros-noetic-desktop
       ```   
       
    +  **ROS-Base**:(Bare Bones) ROS packaging, build, and communication libraries.
       No GUI tools.
       
       ```shell script
       sudo apt install ros-noetic-ros-base
       ```
       
    **NOTE**: *For search all available ROS packages and install an specific package directly, look
    at the commands in the next prompt.*
       
   ```shell script
   //Search ROS packages
   apt search ros-noetic
   
   //Install ROS package directly (PACKAGE= slam-gmapping)
   sudo apt install ros-noetic-slam-gmapping
   ```            
5. **Environment setup**

    You must source this script in every bash terminal you use ROS in.
    
   ```shell script
   source /opt/ros/noetic/setup.bash
   ```    
   
   But instead of that, you can source this script automatically every time 
   a new shell is launched. These command will do that for you.
   
   ```shell script
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```   
   
6. **Next Steps**

    6. ***Check your environment variables***
    
        If you are ever having problems finding or using your ROS packages make sure you
        have your environment properly setup. A good way to ckeck is to ensure hat
        environment variables like ***ROS_ROOT*** and ***ROS_PACKAGE_PATH*** are set:
        If they are not them, you might have to source some setup.*sh files
       ```shell script
        printenv | grep ROS
       ```     
       
    6. ***Installation Catkin-pkg, Python empy and Rospkg***
    
        Catkin is the official build system of ROS and the successor of the original ROS build
        sytem(rosbuild). Also is needed the system dependence python-empy and rospkg. For
        installing all of them just type:
        
          ```shell script
            //Catkin package
            sudo apt-get install python-catkin-pkg
           
            //Python-empy dependence
            sudo pip install empy
           
            //Rospkg
            pip install rospkg
         ```    
            
    6. ***Generate a workspace (catkin make)***
    
        Let's create and build a catkin workspace:
          ```shell script
            mkdir -p ~/agrolaser_ws/src
            cd ~/agrolaser_ws/
            catkin_make
         ``` 
       
       The catkin_make command is a convenience tool for working with catkin workspaces.
       Running in the first time in your workspace, it will create a CmakeLists.txt link in your
       ‘src’ folder. 
              
    6. ***Copy sources from GitHub***
    
       Open a new shell and copy the folder `agrolaser` from the GitHub repo to your workspace:
       
      ```shell script
        git clone https://github.com/OpenAgriTech/Open-PhenoLiDAR.git
        cd Open-PhenoLiDAR/src
        copy -R agrolaser ~/agrolaser_ws/src
        cd ~/agrolaser_ws/
     ```        
       
    6. ***Install rosdep (IMPORTANT STEP!)***
    
       Rosdep is a command-line tool for installing system dependencies.
       + ***For end-users:*** Rosdep helps you install system dependencies for software that
       you are building from source. 
       + ***For developers:*** Rosdep simplifies the problem of installing system dependencies
       on different platforms. 
          ```shell script
            #Installing rosdep (First step)
            sudo apt-get install python3-rosdep
         
            # After installation, rosdep needs to be initialized once
            sudo rosdep init
         
            #Rosdep upgrade (Second step)
            rosdep update
         
            #Install dependency of all packages in the workspace (Third step)
            rosdep install --from-paths src --ignore-src -r -y --os=ubuntu:focal
         ```         



    
     