# NLP

NLP is split into three parts due to what is available on Tiago.

1. `speech_to_text` as the name suggests converts audio to text. Once converted this is published on a ros topic to be processed by the nlp ros node. 
2. nlp ros node `nlp.py`. this accepts incomming commands from the controller to speak, additionally when `speech_to_text` provides the users input as text, calls `rasa_bot` to interpret the message and respond if required, then feeds the command to the controller which starts an action.
3. `rasa_bot` interprets text commands into an internal grammar and generates a text response

# Running NLP

run ```roslaunch nlp nlp.launch``` which will start all three nlp components

Note on running with docker: If you get the following error

```
OSError: [Errno -9996] Invalid input device (no default output device)
```

We think it is due to

```
ALSA lib pcm_dsnoop.c:618:(snd_pcm_dsnoop_open) unable to open slave
```

Which seems to have something to do with permission to access the sound device, or some other conflict between playing sound within a docker container. We checked our configuration and are confident we forwarded the device correctly... It started working after we tested access to the sound I/O device with `aplay` and `arecord` both inside and outside the container... We don't know why this was enough to get it working.   

# Interfaces

## /gaan/nlp/user_msg

Only currently used internally between the NLP components. Raw user messages interpreted by the speech to text system are received on the user_msg topic in the NLP ROS node.

## /gaan/nlp/command

NLP ROS node is a client of this service. Used to send user messages interpreted as commands to the controller module.

## /gaan/nlp/speech

NLP ROS node provides this service. Used by the controller to request a message be spoken via Tiago's TTS service.
