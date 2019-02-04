# Base Installation

These instructions will guide you through the installation of the necessary software for this course.
This includes:

- The Ubuntu operating system
- Some essential software
- ROS: The robotic operating system
- The Racecar simulator

## Ubuntu

In this course we recommend that you install Ubuntu as a virtual machine.
This will isolate the system from your native operating system and can be much easier for you and the TAs to debug if things go wrong.
Plus, virtual machines can be paused in a particular state and backed up easily.

While it is entirely possible to install ROS and the other software on your own native Ubuntu installation
or another distribution of Linux this is done **at your own risk**. If you know what you're doing, everything will probably work fine on a native Ubuntu installation and the TAs will probably be able to help you out with most issues. To get other distributions (Fedora, Gentoo, Arch Linux) to work may require you to build a lot of packages manually or write patches for ROS.

### Virtual Machine Installation

- Download and install VMware, which you can get for free [through IS&T](https://ist.mit.edu/vmware-fusion).

- Download [Ubuntu Desktop 18.04.\*](https://www.ubuntu.com/download/desktop).

- Open VMWare and select ```New Virtual Machine...```. When promted, select the ```.iso``` Ubuntu image you downloaded. Optionally, set your username to ```racecar``` (this will make things slightly easier later on). Set an easy to remember password and use default setting for everything else.

At this point the operating system should install and boot. It is worth performing a system update:

    sudo apt-get update
    sudo apt-get upgrade

If your computer has a signifigant around of processors or memory, feel free to designate more to the VM to allow it to run faster. To do this, shutdown the VM and then adjust the allocated processors and memory in ```VM > Settings```.

*A useful tip*: Use <kbd>CTRL</kbd>+<kbd>SHIFT</kbd>+<kbd>V</kbd> to paste into the VM.

## Installing Software

Install ```git``` and ```python``` using ```apt```:

    sudo apt install git python

Then, follow the [instructions on the ROS wiki](http://wiki.ros.org/melodic/Installation/Ubuntu)
to install ```ros-melodic-desktop```. You can stop at the step about setting up your enviroment

## Creating a Catkin workspace

A catkin workspace is where all of your ROS code will live. The workspace will automatically keep track of the packages scripts you write so you can reference them in other pieces of code. Create it by running:

    source /opt/ros/melodic/setup.bash
    mkdir -p ~/racecar_ws/src
    cd ~/racecar_ws/
    catkin_make

Then add the following to your ```~/.bashrc``` file:

    source ~/racecar_ws/devel/setup.bash

The ```~/.bashrc``` file is run anytime you open up a new terminal. By adding that line you are defining a number of ROS variables in every terminal you open.

## Racecar Software

TODO, see the [simulator page](https://github.com/mit-racecar/racecar_simulator)
