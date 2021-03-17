# NLP

NLP is split into three parts, there was little existing (free) software for NLP on Tiago, so this involved more work.

1. `speech_to_text` as the name suggests converts audio to text. Once converted this is published on a ros topic to be processed by the nlp ros node. 
2. nlp ros node `nlp.py`. this accepts incomming commands from the controller to speak (see note on tts below), additionally when `speech_to_text` provides the users input as text, calls `rasa_bot` to interpret the message and respond if required, then feeds the command to the controller which starts an action.
3. `rasa_bot` interprets text commands into an internal grammar and generates a text response

Note: Text to speech functionality is provided from the ``sound_play`` ROS library. This produces a fairly robotic sound (pardon the pun), but the interface is well defined and will definitely work on the real robot so we chose to go with it.

## Running

run ```roslaunch nlp nlp.launch``` which will start all three nlp components

Note on running with docker: If you get the following error

```
OSError: [Errno -9996] Invalid input device (no default output device)
```

and/or

```
ALSA lib pcm_dsnoop.c:618:(snd_pcm_dsnoop_open) unable to open slave
```

Which seems to have something to do with permission to access the sound device, or some other conflict between playing sound within a docker container. We checked our configuration and are confident we forwarded the device correctly... It started working after we tested access to the sound I/O device with `aplay` and `arecord` both inside and outside the container... We don't know why this was enough to get it working. 

## Interfaces

### /gaan/nlp/user_msg

Only currently used internally between the NLP components. Raw user messages interpreted by the speech to text system are received on the user_msg topic in the NLP ROS node.

### /gaan/nlp/command

NLP ROS node is a client of this service. Used to send user messages interpreted as commands to the controller module.

### /gaan/nlp/speech

NLP ROS node provides this service. Used by the controller to request a message be spoken via Tiago's TTS service.

## Limitations and Lessons Learned

- The paths inside the scripts for the rasa and speech_to_text servers are highly sensitive to the location of the virtual environment and the components themseleves... deviate from the suggested locations at your own peril.
- We want to be able to take time after receiving a request to do something and report back the result. rasa allows this as part of the "actions" but it expects that the actions take in the order of ms to complete. Since we are a robot and things take seconds to minutes, rasa will timeout on such requests before we can respond. We have to implement our own "actions" concept using rasa if we want to be able to improve the flow of conversation and maintain information about previous utterances. TODO
- We initially tried using Alexa as an add on to Tiago to attempt to abstract most of the NLP stuff completely. We don't recommend this. Firstly, its not easy to get the result of the utterances in a form we can send to our ROS components. We ended up having to set up a local web server and capture commands on it... Secondly, we couldn't get Alexa to interpret multi part commands such as "Go to the kitchen, retreive the box", Alexa only listened to the first part. We also didn't know if we can/how to initiate a conversation. As is, we don't think Alexa can be used to perform the conversations required for the competition.
- NLP in the docker environment is not stable, I got it working, only to have it cause errors the next day...   