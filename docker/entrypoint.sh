#!/bin/bash
set -e

source /opt/ros/melodic/setup.bash
source /erl-ws/devel/setup.bash

exec "$@"
