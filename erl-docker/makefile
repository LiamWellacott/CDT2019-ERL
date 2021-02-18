.PHONY: help

help:
	@awk 'BEGIN {FS = ":.*?## "} /^[0-9a-zA-Z_-]+:.*?## / {printf "\033[36m%-42s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

.DEFAULT_GOAL := help

nvidia_cudagl_cudnn:
	docker build -t erl/nvidia:cudagl-cudnn erl/nvidia/cuda
	@printf "\n\033[92mDocker Image: erl/nvidia:cudagl-cudnn\033[0m\n"

nvidia_cudagl_cudnn_dep: nvidia_cudagl_cudnn
	docker build -t erl/ros-melodic:nvidia-tensorflow erl/nvidia/third_party
	@printf "\n\033[92mDocker Image: erl/ros-melodic:nvidia-tensorflow\033[0m\n"

nvidia_ros_melodic: nvidia_cudagl_cudnn_dep
	docker build -t erl/ros-melodic:gpu erl/melodic
	@printf "\n\033[92mDocker Image: erl/ros-melodic\033[0m\n"

erl_tiago_melodic_cuda_base: nvidia_ros_melodic
	docker build -t erl/tiago-melodic:gpu erl/tiago
	@printf "\n\033[92mDocker Image: erl/tiago-melodic:gpu\033[0m\n"

erl_base: erl_tiago_melodic_cuda_base
	docker build -t erl/base:gpu erl
	@printf "\n\033[92mDocker Image: erl/base\033[0m\n"
