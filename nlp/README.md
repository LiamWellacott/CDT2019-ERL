# NLP

NLP is split into three parts due to what is available on Tiago.

1. `speech_to_text` as the name suggests converts audio to text. Once converted this is published on a ros topic to be processed by the nlp ros node. 
2. nlp ros node `nlp.py`. this accepts incomming commands from the controller to speak, additionally when `speech_to_text` provides the users input as text, calls `rasa_bot` to interpret the message and respond if required, then feeds the command to the controller which starts an action.
3. `rasa_bot` interprets text commands into an internal grammar and generates a text response

# Running NLP

run ```roslaunch nlp nlp.launch``` which will start all three nlp components

Note on running with docker: 

# Interfaces

## 
