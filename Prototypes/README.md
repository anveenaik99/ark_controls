# PX4 and MAVROS

This documentation contains information on how to work inside the PX4 system architecture. We will be using Pixhawk as our autopilot hardware.

## Setup ##
The user needs to be part of the group "dialout":


``` sudo usermod -a -G dialout $USER ```



Logout and login again, as this is only changed after a new login.

Update the package list and install the following dependencies for all PX4 build targets:



```sudo add-apt-repository ppa:george-edison55/cmake-3.x -y```

```sudo apt-get update```

```sudo apt-get install python-argparse git-core wget zip \```

```python-empy qtcreator cmake build-essential genromfs -y```


#### Simulation tools

``` sudo apt-get install ant protobuf-compiler libeigen3-dev libopencv-dev openjdk-8-jdk openjdk-8-jre clang-3.5 lldb-3.5 -y ```



Note: replace 3.x with desired version

Ubuntu comes with a serial modem manager which interferes heavily with any robotics related use of a serial port (or USB serial). It can deinstalled without side effects:

``` sudo apt-get remove modemmanager ```


Pixhawk is Nuttx based hardware. So we need to install the following dependencies:

``` sudo apt-get install python-serial openocd \```

``` flex bison libncurses5-dev autoconf texinfo build-essential \```

```libftdi-dev libtool zlib1g-dev \```

```   python-empy  -y ```


We require arm-none-eabi toolchain version 4.9 or 5.4. So remove arm-none-eabi toolchain if it’s not the required version using:

``` sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded```

``` sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa ```


For v4.9

``` pushd . ```

``` cd ~ ```

``` wget https://launchpad.net/gcc-arm-embedded/4.9/4.9-2015-q3-update/+download/gcc-arm-none-eabi-4_9-2015q3-20150921-linux.tar.bz2 ```

``` tar -jxf gcc-arm-none-eabi-4_9-2015q3-20150921-linux.tar.bz2 ```

``` exportline="export PATH=$HOME/gcc-arm-none-eabi-4_9-2015q3/bin:\$PATH" ```

``` if grep -Fxq "$exportline" ~/.profile; then echo nothing to do ; else echo  $exportline >> ~/.profile; fi ```

``` . ~/.profile ```

``` popd ```


For v5.4

``` pushd . ```

``` cd ~ ```

``` wget https://launchpad.net/gcc-arm-embedded/5.0/5-2016-q2-update/+download/gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2 ```

``` tar -jxf gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2 ```

``` exportline="export PATH=$HOME/gcc-arm-none-eabi-5_4-2016q2/bin:\$PATH" ```

``` if grep -Fxq "$exportline" ~/.profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi ```

``` . ~/.profile ```

``` Popd ```




### Installing Qgroundcontrol

Download the tar.gz from Github

https://github.com/mavlink/qgroundcontrol/releases/

Unpack and run the install script




### Installing MAVROS

``` sudo apt-get install ros-indigo-mavros ros-indigo-mavros-extras ```


#### Dependencies:

##### Install wstool

``` sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools ```


Initialize source space using:

``` wstool init ~/catkin_ws/src ```


Build:

``` # 1. get source (upstream - released) ```

``` $ rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall ```

``` # alternative: latest source ```

``` $ rosinstall_generator --upstream-development mavros | tee /tmp/mavros.rosinstall ```

``` # 2. get latest released mavlink package ```

``` # you may run from this line to update ros-*-mavlink package ```

``` $ rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall ```

``` # 3. Setup workspace & install deps ```


``` $ wstool merge -t src /tmp/mavros.rosinstall ```

``` $ wstool update -t src ```

``` $ rosdep install --from-paths src --ignore-src --rosdistro indigo -y ```

``` # finally - build ```

``` $ catkin build ```



## Building Code

PX4 can be built on the console or in a graphical development environment / IDE.


##### Compiling on the Console:

Before moving on to a graphical editor or IDE, it is important to validate the system setup. Do so by bringing up the terminal.

The terminal starts in the home directory. We default to '~/src/Firmware' and clone the upstream repository or its fork.

``` mkdir -p ~/src ```

``` cd ~/src ```

``` git clone https://github.com/PX4/Firmware.git ```

``` cd Firmware ```

``` git submodule update --init --recursive ```

``` cd .. ```



##### Compiling code: NuttX / Pixhawk based boards

``` cd Firmware ```

``` make px4fmu-v2_default ```


Note the syntax: 'make' is the build tool, 'px4fmu-v2' is the hardware / autopilot version and 'default' is the default configuration.


By appending 'upload' to these commands the compiled binary will be uploaded via USB to the autopilot hardware:

``` make px4fmu-v2_default upload ```



##### Raspberry Pi 2/3 boards

The command below builds the target for Raspbian.

Cross-compiler build:

``` cd Firmware ```

``` make posix_rpi_cross # for cross-compiler build ```


The "px4" executable file is in the directory build_posix_rpi_cross/src/firmware/posix. Make sure you can connect to your RPi over ssh.

Then set the IP (or hostname) of your RPi using:
``` export AUTOPILOT_HOST=192.168.X.X ```


And upload it with:

``` cd Firmware	```

``` make posix_rpi_cross upload # for cross-compiler build ```


Then, connect over ssh and run it with (as root):

``` sudo ./px4 px4.config ```



##### Native build

If you're building directly on the Pi, you will want the native build target (posix_rpi_native).

``` cd Firmware ```

``` make posix_rpi_native # for native build ```


The "px4" executable file is in the directory build_posix_rpi_native/src/firmware/posix. Run it directly with:


``` sudo ./build_posix_rpi_native/src/firmware/posix/px4 ./posix-configs/rpi/px4.config ```



## Simulation
### SITL simulation

Software in the Loop Simulation runs the complete system on the host machine and simulates the autopilot. It connects via local network to the simulator. The setup looks like this:

Simulator--->MAVLink--->SITL

Running SITL:

``` make posix_sitl_default jmavsim ```


#### Important Files

The startup script is in the posix-configs/SITL/init folder and named rcS_SIM_AIRFRAME, the default is rcS_jmavsim_iris.

The root file system (the equivalent of / as seen by the) is located inside the build directory: build_posix_sitl_default/src/firmware/posix/rootfs/



### Gazebo simulation (SITL)

Gazebo 7 is the recommended version for running a simulation

If using ROS Indigo, gazebo 2 must be removed from system before installing Gazebo 7

``` sudo apt-get remove gazebo2 ```


Install Gazebo 7

``` sudo apt-get install ros-indigo-gazebo7-ros-pkgs ros-indigo-gazebo7-ros-control ```

Running the simulation:

``` cd ~/src/Firmware ```

``` make posix_sitl_default gazebo ```



### Interfacing Gazebo to ROS
##### Launching MAVROS


To connect to a specific IP:

``` roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557" ```

Fcu_url is the port of SITL


To connect to localhost, use this URL:

``` roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" ```


To run SITL wrapped in ROS the ROS environment needs to be updated, then launch as usual:

``` cd <Firmware_clone> ```

``` make posix_sitl_default gazebo ```

``` source ~/catkin_ws/devel/setup.bash    // (optional) ```

``` source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default ```

``` export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd) ```

``` export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo ```

``` roslaunch px4 posix_sitl.launch ```


Include one of the above mentioned launch files in your own launch file to run your ROS application in the simulation.


### HITL Simulation


PX4 supports HITL for multicopters (using jMAVSim) and fixed wing (using X-Plane demo or full).


##### Using jMAVSim (Quadrotor)

Make sure QGroundControl is not running (or accessing the device via serial port)

Run jMAVSim in HITL mode (replace the serial port if necessary):

``` ./Tools/jmavsim_run.sh -q -d /dev/ttyACM0 -b 921600 -r 250 ```

The console will display mavlink text messages from the autopilot.

Then run QGroundControl and connect via default UDP configuration.


##### Enable HITL in QGroundControl

Widgets -> HIL Config, then select X-Plane 10 in the drop-down and hit connect. Once the system is connected, battery status, GPS status and aircraft position should all become valid




#### Opening system console and shell

For NuttX-based systems (Pixhawk, Pixracer, ...), the nsh console can also be accessed via mavlink. This works via serial link or WiFi (UDP/TCP). Make sure that QGC is not running, then start the shell with:

``` ./Tools/mavlink_shell.py /dev/ttyACM0 ``` (in the Firmware source).

Use -h to get a description of all available arguments. You may first have to install the dependencies with sudo pip install pymavlink pyserial.



## Offboard control using MAVROS

Include mavros_msgs in code

The mavros_msgs package contains all of the custom messages required to operate services and topics provided by the mavros package. All services and topics as well as their corresponding message types are documented in the [mavros wiki](http://wiki.ros.org/mavros).

Code for giving a setpoint to the quadcopter can be found [here](https://github.com/skapuria/ark_controls/blob/master/Prototypes/mavcontrol/src/goto.cpp).

Test run on Gazebo [here](https://drive.google.com/open?id=0B3snoyglu81_ZmVsenU1b3JhWTA).
