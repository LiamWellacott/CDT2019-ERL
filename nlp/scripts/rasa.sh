#!/bin/bash

OLD_DIR=$(pwd)
RASA_DIR=$(dirname $(dirname $(realpath $0)))/rasa_bot

# update this to the right path
source $(dirname $(dirname $(dirname $(dirname $(realpath $0)))))/rasa/bin/activate

cd $RASA_DIR
rasa run
cd $OLD_DIR

deactivate