# Install

To install the docker image with cuda support you need:

[Docker](https://www.docker.com/) and [Nvidia-Docker 2](https://github.com/NVIDIA/nvidia-docker). You need cuda compatible GPU (from Nvidia), make sure you follow the install procedure for both carefully, [this guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker) should provide all of the required steps

# Build

To build the image run

```
make erl_base
```

If needed CPU support only can be added!

# Run

If you are running for the first time do the following. Note: we had a lot of issues at this stage... 

```
xhost +si:localuser:root

touch /tmp/.docker.xauth-n

./dev-first.bash
```

To run the image run ```./dev.bash``` from the erl-docker package.
This will launch the image and mount your erl-ws catkin workspace from your file system to /erl-ws

To have additional terminals inside the docker environment you can re run ```./dev.bash``` from the new terminal.

Before you run any ros nodes you have to ```source devel/setup.bash```, remember that this is separate from your regular environment so any commands in .bashrc are not run.

# Add dependencies

To add new dependencies just instert the terminal command in the right dockerfile.

 - Anything related to Nvidia/cuda/cudnn etc in the ```erl/nvidia/*``` package.
 - Anything related to ros in ```erl/melodic/```
 - Anything related to tiago in ```erl/tiago/```
 - Anything related to the erl competition in ```.```

# NOTES:

 - Not tested on anything else than ubuntu. Need to look at how to forward Xserver equivalent in macos and Windows, or use other tools to launch the image.

 - The camera/webcam device is located in /dev/video0 in ubuntu, need to edit dev.bash to give it the correct camera name. 
