# Pip Dependencies
``` bash
pip3 install --upgrade pip
pip3 install rasa
pip3 install rasa_core_sdk
```

# Rasa Commands
`rasa init`

Creates a new project with example training data, actions, and config files.

`rasa train`
Trains a model using your NLU data and stories, saves trained model in `./models`.

`rasa train --finetune`
Used for incremental training. Reuses previously trained model and retrains it using newly added data. It might still be useful to run a full training.

`rasa run`

Starts a server with your trained model.

`rasa visualize`

Generates a visual representation of your stories.

`rasa interactive`

Starts an interactive learning session to create new training data by chatting to your assistant.

`rasa shell` or `rasa shell --debug`

Loads your trained model and lets you talk to your assistant on the command line.

`rasa run actions --actions actions.actions`

Run action server

`docker run -p 8000:8000 rasa/duckling`

Run duckling