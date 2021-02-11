FROM nvidia/cudagl:10.1-devel

MAINTAINER Athanasios Tasoglou <athanasios@tasoglou.net>
LABEL Description="ROS-Melodic-Desktop with CUDA 10.1 and cuDNN 7 support (Ubuntu 18.04)" Vendor="TurluCode" Version="1.0"

# Install packages without prompting the user to answer any questions
ENV DEBIAN_FRONTEND noninteractive

## CUDNN Runtime-packages
ENV CUDNN_VERSION 7.6.5.32
LABEL com.turlucode.ros.cudnn="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \
            libcudnn7=$CUDNN_VERSION-1+cuda10.1 && \
    apt-mark hold libcudnn7 && \
    rm -rf /var/lib/apt/lists/*

## CUDNN Devel-packages
RUN apt-get update && apt-get install -y --no-install-recommends \
            libcudnn7=$CUDNN_VERSION-1+cuda10.1 \
            libcudnn7-dev=$CUDNN_VERSION-1+cuda10.1 && \
    apt-mark hold libcudnn7 && \
    rm -rf /var/lib/apt/lists/*

# Terminator Config
RUN mkdir -p /root/.config/terminator/
COPY assets/terminator_config /root/.config/terminator/config

# Entry script - This will also run terminator
COPY assets/entrypoint_setup.sh /
ENTRYPOINT ["/entrypoint_setup.sh"]

# Launch terminator
CMD ["terminator"]
