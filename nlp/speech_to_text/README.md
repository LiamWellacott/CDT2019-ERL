# Set up

**NOTE: you do not have to do this if you are using docker**

## Ubuntu dependencies
``` bash
sudo apt install python3-dev python3-pip python3-venv
sudo apt install portaudio19-dev espeak
```

## Create python environment
- Navigate to speech folder
- It is important to use environment as the dependencies for rasa conflict with the dependencies for speech IO

``` bash
python3 -m venv ./speech
source ./speech/bin/activate
```

## Pip Dependencies
``` bash
pip3 install --upgrade pip
pip3 install -r requirements.txt
```

(requirements moved to docker)

## Download STT Models

Download stt models [here](https://github.com/mozilla/DeepSpeech/releases/download/v0.9.3/deepspeech-0.9.3-models.pbmm) and [here](https://github.com/mozilla/DeepSpeech/releases/download/v0.9.3/deepspeech-0.9.3-models.scorer) into a folder called `stt_models` 

# Run speech.py
``` bash
python3 ./speech.py -m ./stt_models/deepspeech-0.9.3-models.pbmm -s ./stt_models/deepspeech-0.9.3-models.scorer -v 0
```