# Set up

We have created a docker image and container to make the set up as easy as possible. However in case you have issues with this environment we also have instructions for setting up the dependencies on your machine.

## Setting up your local machine

### Before cloning this repo:

1. Install ROS (melodic)
2. Create a ros workspace and ``cd`` into the ``src`` directory:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```
3. Clone this repo into the ``src`` directory.

### Cloning the repo:

1. Fork the repo on your github acount.
2. Clone your fork locally in your ``src`` directory.
```
git clone https://github.com/{git_username}/CDT2019-ERL
```
3. Set the upstream repo. This will allow you to pull the latest changes from the main repo
```
cd CDT2019-ERL/
git remote add upstream https://github.com/LiamWellacott/CDT2019-ERL.git
```

To update your local copy use ``fetch``, this will download branches etc. from the main repo.
```
git fetch upstream
```

To bring changes from the main repo into your repo (using the example of the main branch do:
```
# make sure you are on the branch you want to merge into locally
git merge upstream/main

# To push the result of the merge to your fork
git push
```

### After cloning this repo

**make sure you have both the melodic workspace and this workspace on your ROS_PACKAGE_PATH while doing these steps**

1. ``cd`` to the ``src`` folder of the workspace, from the previous steps you should be there already
2. Clone the virtual environment ([RALT virtual environment](https://github.com/LiamWellacott/Virtual-RALT-Standalone)

```
git clone https://github.com/care-group/Virtual-RALT-Standalone.git
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

```
4. clone the RSBB ROS communication repo into your src folder:
```
git clone https://github.com/rockin-robot-challenge/at_home_rsbb_comm_ros.git
cd at_home_rsbb_comm_ros
# make sure you do the following or catkin_make will fail
# a repo with a very similar name is a submodule of this repo
# make sure to rerun this if you ever update this repo
git submodule update --init

```

5. In order for the robot to path through doors you must reduce the `inflation_radius` and `inflation_dist` parameter in the tiago planner configuration. This reduces the distance from an obstacle (e.g. furniture, door, wall) the robot is willing to cross. TODO create a script to do this or find a better way... You can find `inflation_radius` in `pal_navigation_cfg_public/pal_navigation_cfg_tiago/config/base/common/global_costmap_public_sim.yaml` and `pal_navigation_cfg_public/pal_navigation_cfg_tiago/config/base/common/local_costmap_public_sim.yaml`. You can find `inflation_dist` in `pal_navigation_cfg_public/pal_navigation_cfg_tiago/config/base/teb/local_planner.yaml`. I set the value to `0.15`.

6. (untested) install CUDA see dockerfile in ``docker/gaan/nvidia/cuda`` and ``docker/gaan/nvidia/third_party`` for what to install

7. (untested) install DOPE and dependencies (instructions from dockerfile ``docker/gaan/Dockerfile``)

check out the dope repo and install dependencies
```
# in the src folder
git clone -b ros-kinetic https://github.com/kini5gowda/Deep_Object_Pose dope
cd dope
pip install -r requirements.txt
rosdep install --from-paths src -i --rosdistro kinetic
```

download model weights

```
# if you don't have these
pip install gdown
apt install unrar

# download and extract weights into src/dope/weights
cd weights # assume you are in dope folder from previous step
gdown https://drive.google.com/uc?id=1rnjp2kgttraZRmqH51vW_NOaTTa6iiN2 -O weights.rar -O  weights.rar
unrar e weights.rar

```

8. (untested) install face recognition dependencies (instructions from dockerfile ``docker/gaan/Dockerfile``)

```
# get dependencies
pip install face_recognition
apt install ros-melodic-usb-cam -y # used for testing without needing access to Tiago's camera
```

9. install nlp dependencies

make sure you have python 3 tools if you don't already
```
mv /etc/apt/sources.list.d/wheezy-backports-main.list{,.bak} && \
apt update && \
apt install -y python3-dev python3-pip python3-venv
```

rasa bot set up:
```
# create a python virtual env to avoid conflicting with speech_to_text and other python3 tools
python3 -m venv /opt/venv/rasa_env
source /opt/venv/rasa_env/bin/activate

# install dependencies
cd <your_CDT2019-ERL_checkout>/GAAN/nlp/rasa_bot # path given for context change based on your current directory
pip3 install --upgrade pip
pip3 install -r requirements.txt

# train rasa model, you should have a file in the rasa_bot/models dir after this 
rasa train
```

speech_to_text set up
```
# if you are still in the rasa venv
deactivate

# create a python virtual env to avoid conflicting with rasa and other python3 tools
python3 -m venv /opt/venv/speech_env
source /opt/venv/speech_env/bin/activate

# install dependencies
cd <your_CDT2019-ERL_checkout>/GAAN/nlp/speech_to_text # path given for context change based on your current directory
apt install portaudio19-dev espeak -y
pip3 install --upgrade pip
pip3 install deepspeech~=0.9.3 pyaudio~=0.2.11 webrtcvad~=2.0.10 halo~=0.0.18 numpy>=1.15.1 scipy>=1.1.0 requests torch pyaudio

# download weights into nlp/speech_to_text/stt_models
mkdir stt_models
cd stt_models
curl -LO https://github.com/mozilla/DeepSpeech/releases/download/v0.9.3/deepspeech-0.9.3-models.pbmm
curl -LO https://github.com/mozilla/DeepSpeech/releases/download/v0.9.3/deepspeech-0.9.3-models.scorer

```

Set up text to speech
```
cd <your_workspace>/src

git clone https://github.com/ros-drivers/audio_common.git
apt install libgstreamer-plugins-base1.0-dev
rosdep install sound_play -y
```

10. Build workspace

```
cd <your_workspace_route>
catkin_make

```

## Running

1. Make sure the workspace is on the ros package path, check path with ``echo $ROS_PACKAGE_PATH``. You should make sure to comment out any previous ROS Workspaces that may be sourced in your ``~.bashrc``.

You may want to add it to ``~/.bashrc`` to avoid having to do it for each terminal.
```
source {path to workspace}/devel/setup.bash
```

2. (First time test only) You can launch the robot sim with:

```
roslaunch sim_launch base_ralt_tiago.launch
```

3. To run the restricted task 3 scenario:

```
# launch the simulation and start the gaan software controller
roslaunch sim_launch restricted_task_3.launch

# (in a separate terminal) launch the rsbb which will send the start signal (granny annie pushes the summon button)
# note: wait for the robot to initialise and tuck arm before running this command
roslaunch fake_rsbb restricted_task_3.launch
```

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

Here is a step by step for how to contribute to this repo. You start off having cloned a fork of the repo in to your workspace.
```
cd CDT2019-ERL/
git fetch upstream
```
The ```fetch``` is important to ensure your copy (fork) is up to date.

Next, you want to work on a branch of your fork. A branch is a subsection where you copy your fork but you can carry out experimental work and discard it or merge it with your main fork without polluting the original.

To create a branch in your own fork, use ``branch``
```
git checkout -b tutorial
```
Where ``-b`` creates the local branch and checks it out at the same time. Remove ``-b`` if your branch already exists. You are now working on a branch and any changes you make will remain in this branch of the code until you ``merge`` the branches.
Now you can make whatever changes you like to the subsection or subsection you are working on.
When you have done some work (But not too much! Make sure to commit regularly), you need to ``add``, ``commit`` and ``push`` the work to be able to upload it to your branch online.
You can check and see what is different between your local (your PC) and the branch you are working on.
```
git status
```
Then you have to add the files that are untracked that you want to upload.
```
git add tutorial.txt
```
You can replace ``tutorial.txt`` with the file or folder you are adding. Now that all your files that you wish to upload are tracked, you then need to commit them.

```
git commit -m "This is a brief explanation of what has been changed."
```
To upload the commit, use ``push``.
```
git push origin tutorial
```
You should now see your forked copy of the project with a branch named ``tutorial`` that contains all your changes. The next step is to ``merge`` the work using a ``pull request``. The easiest way to do this is to go to github and look at your forked repo and click create a new pull request, where the "base" is what you are adding to and the "head" is what you have been working on. In this case, the base is ``main`` and the head to compare is ``tutorial``.  
As long as there are no conflicts with the base branch, you should be able to merge the pull request. Your main should now reflect the files you have added or changed to the branch you made and that branch can be closed.

The subsequent step is to create a new pull request to merge your forked repo and the original CDT2019-ERL Repo that is held by Liam. The steps are the same but the base is ``LiamWellacot/CDT2019-ERL`` and the head is ``{git_username}/CDT2019-ERL`` and the merge is occuring on the ``main`` branches. This pull request then has to be granted by the owner of the ``base`` repo. In this case, it is Liam. Once the merge has been completed, your work is successfully available to all other collaborators!

Don't forget to ``git fetch upstream`` to ensure you have an up to date repo before you work on your fork.