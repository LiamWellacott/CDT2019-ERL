#!/bin/bash

OLD_DIR=$(pwd)
#RASA_DIR=$(dirname $(dirname $(realpath $0)))/rasa_bot
RASA_DIR=/tools/rasa-bot

# update this to the right path
#source $(dirname $(dirname $(dirname $(dirname $(dirname $(realpath $0))))))/rasa/bin/activate
# below doesn't work due to alias made in dockerfile, may want to just do this to keep it standard
source /opt/venv/rasa_env/bin/activate
#rasa_env

cd $RASA_DIR
rasa run actions --actions actions.actions
cd $OLD_DIR

deactivate