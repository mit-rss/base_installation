# Base Installation

These instructions will guide you through the installation of the necessary software for this course.
This includes:

- A unix-like operating system
- ROS, the robotic operating system
- The Racecar simulator

You can install this software via a prebuilt virtual machine, or manually.

## Prebuilt Installation

Download and install VMware, which you can get for free [through IS&T](https://ist.mit.edu/vmware-fusion).

Download the latest VM via bittorrent. If you don't already have a bittorrent client we recommend Transmission on Linux and OS X and qBittorent on Windows.

Open up the virtual machine. Everything is already installed! Skip ahead to the 
[Testing the Racecar Simulator](#testing-the-racecar-simulator) section.

## Manual Installation

### Operating System

We recommend that you install the operating system you will use for this course as a virtual machine.
This will isolate the system from your native operating system and can be much easier for you and the TAs to debug if things go wrong.
Plus, virtual machines can be paused in a particular state and backed up easily.

While it is entirely possible to install ROS and the other software directly on your own native operating system, this is done **at your own risk**.
If you know what you're doing, everything will probably work fine if you are running Debian or Ubuntu.
Other distributions (Arch Linux, Gentoo, OS X) are not as well supported by ROS and you may find yourself having manually building packages or writing patches.
ROS does not work on Windows.

#### Installing a Debian Virtual Machine

Download and install VMware, which you can get for free [through IS&T](https://ist.mit.edu/vmware-fusion).

Download the [Debian](https://www.debian.org/distrib/netinst) operating system. Choose the "small", "amd64" installer.

Open VMWare and select "New Virtual Machine...". Select a "Typical" installation. When promted, select the ```.iso``` Debian image you downloaded. It should automatically detect the correct disto. Click through the rest of the settings until the OS begins to boot

- Choose "Graphical Installer" and begin selecting the default options. When promted, set your hostname to "racecar-vm", leave the domain name blank and set the username to "racecar".

- Continue selecting default settings. When asked about setting up GRUB select "yes" then select "/dev/sda".

You should now have Debian installed! Log in and click the "Activities" button in the top left and open the "Terminal" application.

Type ```su``` and enter your password. As root type:

    apt-get update
    apt-get upgrade
    apt-get install sudo open-vm-tools open-vm-tools-desktop vim git dirmngr make g++

After the installation completes run ```visudo``` and add the following line to the file that opens up:

    racecar ALL=(ALL:ALL) ALL

Write out the file and type ```exit``` in the terminal, then shut the virtual machine down. At this point, you can optionally designate more processors or memory to the VM to allow it to run faster by changing the settings in ```VM > Settings```.

*A useful tip*: Use <kbd>CTRL</kbd>+<kbd>SHIFT</kbd>+<kbd>V</kbd> to paste into the VM.

### Installing ROS

Boot up the virtual machine and log in. Follow the [instructions on the ROS wiki](http://wiki.ros.org/melodic/Installation/Debian)
to install ```ros-melodic-desktop```. You can stop at the step about setting up your enviroment.

### Creating a Catkin workspace

A catkin workspace is where all of your ROS code will live. The workspace will automatically keep track of the packages scripts you write so you can reference them in other pieces of code. Create it by running:

    source /opt/ros/melodic/setup.bash
    mkdir -p ~/racecar_ws/src
    cd ~/racecar_ws/
    catkin_make

Then add the following to your ```~/.bashrc``` file:

    source ~/racecar_ws/devel/setup.bash

The ```~/.bashrc``` file is run anytime you open up a new terminal. By adding that line you are defining a number of ROS variables in every terminal you open.

### Installing the Racecar Simulator

First install some additional ros packages by running:

    sudo apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server

Then navigate to your catkin workspace source folder and use git to clone the simulator:

    cd ~/catkin_ws/src
    git clone https://github.com/mit-racecar/racecar_simulator.git

Then run ```catkin_make``` in the root of your catkin workspace to build it.

    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash

### Testing the Racecar Simulator

Start a ```roscore``` which acts as the communication center for all ROS services. Just run this in a terminal:

    roscore

In another terminal start a graphical utility called ```rviz``` by running:

    rviz

In another terminal start the simulator itself by running

    roslaunch racecar_simulator simulate.launch

Now in the ```rviz``` window that opened, in the left panel at the bottom click the "Add" button, then in the "By topic" tab add the ```/map``` topic and the ```/scan``` topic.
Then in the "By display type" tab add the RobotModel type.
In the left panel under the newly added LaserScan section, change the size to 0.1 meters for a clearer visualization of the lidar (shown in rainbow).

You should see the car sitting in the middle of a 2D map of MIT's building 31 as shown below:

![The racecar in the starting position](https://raw.githubusercontent.com/mit-racecar/racecar_simulator/master/media/racecar_simulator_rviz_1.png)

You can use a USB joystick to drive the car around, or you can place the car manually by clicking the "2D Pose Estimate button" on the top of the screen and dragging your mouse on the desired pose.

![The racecar in a cubicle](https://raw.githubusercontent.com/mit-racecar/racecar_simulator/master/media/racecar_simulator_rviz_2.png)
