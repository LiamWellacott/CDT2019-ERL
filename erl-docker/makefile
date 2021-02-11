.PHONY: help

help:
	@awk 'BEGIN {FS = ":.*?## "} /^[0-9a-zA-Z_-]+:.*?## / {printf "\033[36m%-42s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

.DEFAULT_GOAL := help

nvidia_cudagl_cudnn:
	docker build -t erl/nvidia:cudagl-cudnn erl/nvidia/cuda
	@printf "\n\033[92mDocker Image: erl/nvidia:cudagl-cudnn\033[0m\n"

nvidia_ros_melodic: nvidia_cudagl_cudnn
	docker build -t erl/ros-melodic:nvidia erl//melodic
	@printf "\n\033[92mDocker Image: erl/nvidia:cudagl-cudnn\033[0m\n"

erl_ros_melodic_cuda_base: nvidia_ros_melodic
	docker build -t erl/base erl
	@printf "\n\033[92mDocker Image: erl/base\033[0m\n"
