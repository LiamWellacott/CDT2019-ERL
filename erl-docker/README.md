# Install

To install the docker image with cuda support you need:

[Docker](https://www.docker.com/) and [Nvidia-Docker 2](https://github.com/NVIDIA/nvidia-docker). Obviously you need cuda compatible GPUs and follow the install procedure for the two docker softwares.

# Build

To build the image run

```
make erl_base
```

If needed CPU support only can be added!

# Run

To run the image just run ```./dev.bash``` from the erl-docker package.
This will launch the image and mount your erl-ws catkin workspace from your file system to /erl-ws


# Add dependencies

To add new dependencies just instert the terminal command in the right dockerfile.

 - Anything related to Nvidia/cuda/cudnn etc in the ```erl/nvidia/*``` package.
 - Anything related to ros in ```erl/melodic/```
 - Anything related to tiago in ```erl/tiago/```
 - Anything related to the erl competition in ```.```


# NOTES:

 - Not tested on anything else than ubuntu. Need to look at how to forward Xserver equivalent in macos and Windows, or use other tools to launch the image.

 - The camera/webcam device is located in /dev/video0 in ubuntu, need to edit dev.bash to give it the correct camera name. 
