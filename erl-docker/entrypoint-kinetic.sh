#!/bin/bash
set -e

source /opt/ros/kinetic/setup.bash
source /erl-ws/install/setup.bash


exec "$@"
