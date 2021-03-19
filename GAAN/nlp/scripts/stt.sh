#!/bin/bash

STT_DIR=$(dirname $(dirname $(realpath $0)))/speech_to_text

# update this to the right path
#source $(dirname $(dirname $(dirname $(dirname $(dirname $(realpath $0))))))/speech/bin/activate
# below doesn't work due to alias made in dockerfile, may want to just do this to keep it standard
source /opt/venv/speech_env/bin/activate
#speech_env

python3 $STT_DIR/speech.py -m /stt_models/deepspeech-0.9.3-models.pbmm -s /stt_models/deepspeech-0.9.3-models.scorer -v 0
#python3 $STT_DIR/speech.py -m $STT_DIR/stt_models/deepspeech-0.9.3-models.pbmm -s $STT_DIR/stt_models/deepspeech-0.9.3-models.scorer -v 0

deactivate