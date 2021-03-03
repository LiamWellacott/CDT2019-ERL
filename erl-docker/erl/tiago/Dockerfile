FROM erl/ros-melodic:gpu

RUN cmake --version

RUN apt update && apt install -y apt-utils build-essential psmisc vim-gtk wget
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN apt update && apt install -q -y python-catkin-tools
# Install git lfs. Necessary in order to properly clone bobble_description
RUN echo 'deb http://http.debian.net/debian wheezy-backports main' > /etc/apt/sources.list.d/wheezy-backports-main.list
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash
RUN apt install -q -y git-lfs
RUN git lfs install
ENV ERL_WS=/erl-ws

RUN source /opt/ros/melodic/setup.bash && \
    mkdir -p $ERL_WS/src && \
    cd $ERL_WS/src && \
    git clone https://github.com/care-group/Virtual-RALT-Standalone.git && \
    git clone https://github.com/LiamWellacott/mbot_simulation_environments.git && \
    apt install python-rosinstall

RUN cmake --version

RUN source /opt/ros/melodic/setup.bash && \
    cd $ERL_WS && \
    catkin_make && \
    wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/kinetic-devel/tiago_public-melodic.rosinstall && \
    rosinstall src /opt/ros/melodic tiago_public-melodic.rosinstall

RUN source /opt/ros/melodic/setup.bash && \
    cd $ERL_WS && \
    rosdep update

RUN source /opt/ros/melodic/setup.bash && \
    cd $ERL_WS && \
    rosdep install --from-paths src --ignore-src -y --rosdistro melodic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3" && \
    source /opt/ros/melodic/setup.bash && \
    catkin_make -j 4


RUN source /opt/ros/melodic/setup.bash && \
    cd $ERL_WS/src && \
    git clone https://github.com/rockin-robot-challenge/at_home_rsbb_comm_ros.git && \
    cd at_home_rsbb_comm_ros && \
    git submodule update --init && \
    cd ../.. && \
    catkin_make

RUN cmake --version
