# PUT / AI Robotics 1 laboratories

## Motivation

The purpose of this framework is to optimize the work done around Docker in the context of Robotics classes:

- **_reduce redundancy_** - both in routine tasks and memory and network usage
- **_introduce conventions_**

Moreover, this framework will gently familiarize you with Docker<br />
You're highly encouraged to _explore all the scripts_ in this repository as _they are annotated_<br />
This will allow you to _add Docker to your toolset_

The suggested workflow is presented next 👇

## A comprehensive example

Assume you run all the terminals during a single session

Terminal 1:

```bash
# Set up and connect to a container for laboratory number 7
# Because of --rm option it will be removed on exit
./robgo.sh 7 --rm

# Make the ROS running
roscore
```

Terminal 2:

```bash
# Connect to the same running container
./robgo.sh 7

# Pick and download a supplementary BAG file
robget pick

# Prepare the downloaded file for playing
rosbag play /materials/7/walking_robot.bag --clock --pause
```

Terminal 3:

```bash
# Again, connect to the same container
./robgo.sh 7

# Go to the source directory of catkin packages
cd /catkin_ws/src

# Create a new package called center-of-shanks
catkin_create_pkg center-of-shanks std_msgs rospy

# Both /materials/ and /solutions/ directories
# are stored outside of the container

# Move the newly created package there so it
# is not lost when the container is removed
robshare center-of-shanks

# Check whether the package has been moved
# You will see that it is a symlink to
# /solutions/7/center-of-shanks now
ls -l

# Later when you want to restore the package
# you will run the same robshare command,
# and the center-of-shanks symlink will
# appear in you current directory

# The robshare script supports complex
# absolute and relative paths as well

# Finally, you will want to exit the container
exit
```

## Rare cases

```bash
# Run one more independent container for the same laboratory

# The next time robgo will send you to the new container

# Until you remove it you will have to
# operate the previous ones manually

./sources/robrun.sh 7

# Upgrade the image

# apt update && apt upgrade will be run on the existing base image
# The laboratory 7 image will be fully rebuilt

./sources/robbuild.sh 7 upgrade

# Fully upgrade the image

# The latest ROS image will be pulled
# The base and laboratory 7 images
# will be fully rebuilt

./sources/robbuild.sh 7 full-upgrade
```

## Scripts

The list of all available helper scripts provided by this framework:

<table>
    <tr>
        <td>Name</td>
        <td>Path</td>
    </tr>
    <tr>
        <td>robgo</td>
        <td>./</td>
    </tr>
    <tr>
        <td>robget</td>
        <td>./materials</td>
    </tr>
    <tr>
        <td>robshare</td>
        <td>./solutions</td>
    </tr>
    <tr>
        <td>robrun</td>
        <td>./source</td>
    </tr>
    <tr>
        <td>robbuild</td>
        <td>./source</td>
    </tr>
</table>

_Go see help for each of them_ (`<path><name>.sh` without any arguments from the repository root directory)

## More on scripts

In Linux, a script needs special permission to be executed. Grant it by typing `chmod +x <path-to-script>`<br />
To run a script type `<path-to-script> <argument1> <argument2> ... `<br />
As long as it is not an alias, **_remember about ./ and .sh_**

Used argument notation (one of the many others):

<table>
    <tr>
        <td>Notation</td>
        <td>Meaning</td>
    </tr>
    <tr>
        <td>&lt;lab-id&gt;</td>
        <td>To be replaced by a value</td>
    </tr>
    <tr>
        <td>[--rm]</td>
        <td>Optional</td>
    </tr>
    <tr>
        <td>-a/-aq/--quiet</td>
        <td>Non-positional; may be combinable; double dash for full forms</td>
    </tr>
    <tr>
        <td>upgrade|full-upgrade</td>
        <td>Mutually exclusive - pick one</td>
    </tr>
    <tr>
        <td>(&lt;docker-run-options&gt;)</td>
        <td>Can be repeated multiple times</td>
    </tr>
</table>

## WSL configuration

### GUI

Make sure you run [the latest WSL version](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)<br />
You can either choose the default WSL X server or VcxSrv<br />
The last one is much slower but provides a more Windows-like experience:<br />
the windows are easier to interact with (moving, snapping, stacking)

### GPU

Totally not required for these classes, but you can [try it out](https://learn.microsoft.com/en-us/windows/ai/directml/gpu-cuda-in-wsl#get-started-with-nvidia-cuda)

### Devices

WSL provides a very limited amount of drivers<br />
Therefore for something like webcam support, you will have to compile a custom [WSL kernel](https://github.com/microsoft/WSL2-Linux-Kernel)<br />
Also you will need a tool like [usbipd-win](https://github.com/dorssel/usbipd-win) that will handle the control over your device to Linux<br />
There is a [video](https://www.youtube.com/watch?v=t_YnACEPmrM) covering the whole process
