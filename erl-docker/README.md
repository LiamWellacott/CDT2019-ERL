# Install

To install the docker image with cuda support you need:

[Docker](https://www.docker.com/) and [Nvidia-Docker 2](https://github.com/NVIDIA/nvidia-docker). Obviously you need cuda compatible GPUs and follow the install procedure for the two docker softwares.

# Build

To build the image run

```
make erl_ros_melodic_cuda_base
```

If needed CPU support only can be added!

# Run

To run the image just run ```./dev.bash```
