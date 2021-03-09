#!/bin/bash

STT_DIR=$(dirname $(dirname $(realpath $0)))/speech_to_text

# update this to the right path
source $(dirname $(dirname $(dirname $(dirname $(dirname $(realpath $0))))))/speech/bin/activate
#source /opt/venv/speech_env/bin/activate

python3 $STT_DIR/speech.py -m $STT_DIR/stt_models/deepspeech-0.9.3-models.pbmm -s $STT_DIR/stt_models/deepspeech-0.9.3-models.scorer -v 0

deactivate