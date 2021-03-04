.PHONY: help

help:
	@awk 'BEGIN {FS = ":.*?## "} /^[0-9a-zA-Z_-]+:.*?## / {printf "\033[36m%-42s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

.DEFAULT_GOAL := help

nvidia_cudagl_cudnn:
	docker build -t gaan/nvidia:cudagl-cudnn gaan/nvidia/cuda
	@printf "\n\033[92mDocker Image: gaan/nvidia:cudagl-cudnn\033[0m\n"

nvidia_cudagl_cudnn_dep: nvidia_cudagl_cudnn
	docker build -t gaan/ros-melodic:nvidia-tensorflow gaan/nvidia/third_party
	@printf "\n\033[92mDocker Image: gaan/ros-melodic:nvidia-tensorflow\033[0m\n"

nvidia_ros_melodic: nvidia_cudagl_cudnn_dep
	docker build -t gaan/ros-melodic:gpu gaan/melodic
	@printf "\n\033[92mDocker Image: gaan/ros-melodic\033[0m\n"

gaan_tiago_melodic_cuda_base: nvidia_ros_melodic
	docker build -t gaan/tiago-melodic:gpu gaan/tiago
	@printf "\n\033[92mDocker Image: gaan/tiago-melodic:gpu\033[0m\n"

gaan_base: gaan_tiago_melodic_cuda_base
	docker build -t gaan/base:gpu gaan
	@printf "\n\033[92mDocker Image: gaan/base\033[0m\n"
