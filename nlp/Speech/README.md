# Ubuntu dependencies
``` bash
sudo apt install python3-dev python3-pip python3-venv
sudo apt install portaudio19-dev espeak
```

# Create python environment
- Navigate to speech folder
- It is important to use environment as the dependencies for rasa conflict with the dependencies for speech IO

``` bash
python3 -m venv ./speech
source ./speech/bin/activate
```

# Pip Dependencies
``` bash
pip3 install --upgrade pip
pip3 install -r requirements.txt
```

# Download Models
## STT
1. Download stt models [here](https://github.com/mozilla/DeepSpeech/releases/download/v0.9.3/deepspeech-0.9.3-models.pbmm) and [here](https://github.com/mozilla/DeepSpeech/releases/download/v0.9.3/deepspeech-0.9.3-models.scorer)
2. Create folder called `stt_models` and copy them inside

## TTS
1. Download tts model by running
``` bash
tts --text "Hello Tiago" --model_name "tts_models/en/ljspeech/glow-tts" --vocoder_name "vocoder_models/en/ljspeech/mulitband-melgan"
```

2. Create tts_models folder and copy downloaded models from `/home/$USER/.local/share/tts`
3. Rename to `tts_models-en-ljspeech-glow-tts-config.json` and `tts_models-en-ljspeech-glow-tts-model.pth.tar`


# Run speech.py
``` bash
python3 ./speech.py -m ./stt_models/deepspeech-0.9.3-models.pbmm -s ./stt_models/deepspeech-0.9.3-models.scorer -v 0
```

---
# Available models for TTS
 >: tts_models/en/ljspeech/glow-tts 
 >: tts_models/en/ljspeech/tacotron2-DCA 
 >: tts_models/en/ljspeech/speedy-speech-wn 
 >: tts_models/es/mai/tacotron2-DDC 
 >: tts_models/fr/mai/tacotron2-DDC 
 >: vocoder_models/universal/libri-tts/wavegrad 
 >: vocoder_models/universal/libri-tts/fullband-melgan 
 >: vocoder_models/en/ljspeech/mulitband-melgan 

- 
``` bash


tts --text "Hello Granny Annie. How may I help you?" --out_path ./ --config_path ./tts_model/tts_config.json --model_path ./tts_model/tts_model.pth.tar --vocoder_path ./tts_model/vocoder_model.pth.tar --vocoder_config_path ./tts_model/vocoder_config.json 
``` 