# CDT2019-ERL

This is a competition entry for the European Robotic League and part of our PhD work at the Edinburgh Centre for Robotics Centre for Doctoral Training in Robotics and Autonomous Systems. The goal of this project is to produce a software system which is:

- Competitive, i.e. software supporting HWU ECR entries to this type of competitions (ERL, Robocup@Home, METRICS...).
- Flexible, modular, and easy to configure software (teams typically have half a day / one day) to configure their system to operate in the test-bed where the tournament takes place - ideally most of which should be usable for both HSR and Tiago
- Robust (keep things simple, consider an integration and testing plan)

In addition to the code available in this repo we have produced a [demonstration video]() and [team description paper]().

## Set up

### Before cloning this repo:

1. Install ROS (version x) and other dependencies (TBD)
2. Create a ros workspace and ``cd`` into the ``src`` directory:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```
3. Clone this repo into the ``src`` directory.

### Cloning the repo:

1. Fork the repo on your github acount.
2. clone your fork localy.
3. Set the upstream repo. This will allow you to pull the latest changes from the main repo
```
git remote add upstream https://github.com/LiamWellacott/CDT2019-ERL.git
```

To pull just do, this will also create all the other branches.
```
git fetch upstream
```



### After cloning this repo

1. ``cd`` to the ``src`` folder of the workspace, from the previous steps you should be there already
2. Clone the virtual environments ([RALT virtual environment](https://github.com/LiamWellacott/Virtual-RALT-Standalone), [mbot environment](https://github.com/LiamWellacott/mbot_simulation_environments/tree/melodic)):

```
git clone https://github.com/care-group/Virtual-RALT-Standalone.git
git clone https://github.com/LiamWellacott/mbot_simulation_environments.git
```
3. Install the TIAGo software. Below instructions adapted from [here](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/TiagoSimulation).

```
# if you haven't used rosinstall before you may have to run:
sudo apt install python-rosinstall

# download the file "tiago_public.rosinstall" from  into ~/catkin_ws/
cd ~/catkin_ws/
nano tiago_public.rosinstall
```
Paste the copied content of [this](https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/kinetic-devel/tiago_public-melodic.rosinstall) page and save the file in the root directory of the workspace (watch for the ROS distribution in the file name, make sure it matches what you downloaded)

```
rosinstall src /opt/ros/melodic tiago_public-melodic.rosinstall

# ensure you have the dependencies required to build
sudo rosdep init
rosdep update
## my install of moveit failed... you may need to run sudo apt-get update before running the next line
rosdep install --from-paths src --ignore-src -y --rosdistro melodic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3"

# build
catkin_make
```
## Running

1. Make sure the workspace is on the ros package path, check path with ``echo $ROS_PACKAGE_PATH``. You should make sure to comment out any previous ROS Workspaces that may be sourced in your ``~.bashrc``.

You may want to add it to ``~/.bashrc`` to avoid having to do it for each terminal.
```
source {path to workspace}/devel/setup.bash
```

2. (First time test only) You can launch the default robot sim with:

```
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=titanium
```

3. (First time test only) You can launch the virtual environment with:

```
roslaunch virtual_ralt_standalone virtual_ralt_standalone.launch
```

4. (First time test only) run the hello TIAGo example, familiarise yourself with the interface (TODO)

### If the Robot Model does not load...
If there are issues with ``ModuleNotFoundError: No module named 'rospkg'``

Check the version of Python that you have installed using
```
python --version
echo $ROS_PYTHON_VERSION
```
The ``python --version`` should return ``Python 2.7.17``, and the ``echo $ROS_PYTHON_VERSION`` should return ``2``.

## Change protocol

**You must have your own fork to contribute to the project**, we will use a pull request model for managing contributions each change must be reviewed before merging is allowed.

## Info/useful links for team members

### Other Project Sites
- [Sharepoint](https://heriotwatt.sharepoint.com/sites/CDT2019-ERL) for project documentation, you must use your HW credentials to access, non HW accounts cannot be added to this.
- [Overleaf report](https://www.overleaf.com/read/tbvrxpjrnrkt) for paper (tell me your email to be added as collaborator)

### Resources 
- The main source of information is the [consumer page of the competition website](https://www.eu-robotics.net/robotics_league/erl-consumer/about/index.html), here you can find the [rulebook](https://www.eu-robotics.net/robotics_league/upload/documents-2018/ERL_Consumer_10092018.pdf) which contains detailed descriptions of the tasks to be completed. Some additional information on the [test environments](https://www.eu-robotics.net/robotics_league/erl-consumer/certified-test-beds/index.html) and specifically the [Edinburgh test environment](https://www.eu-robotics.net/robotics_league/upload/documents-2017/ERL-SR_TestBedCertificationForm_HWU_web.pdf)
- Home page for the [Edinburgh assisted living testbed](https://ralt.hw.ac.uk/)
- Some notes on possible [tools/algorithms](https://heriotwatt.sharepoint.com/sites/CDT2019-ERL/_layouts/15/doc.aspx?sourcedoc={c50ef375-786c-45bd-8ff7-3e8696c3442a}&action=edit) which may be useful for coming up with a solution.



