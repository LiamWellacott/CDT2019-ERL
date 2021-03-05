# Set up

**NOTE: you do not have to do this if you are using docker**

## Ubuntu dependencies
``` bash
sudo apt install -y python3-dev python3-pip python3-venv
```

## Create python environment
- Navigate to Rasa-bot
- It is important to use environment as the dependencies for rasa conflict with the dependencies for speech IO

``` bash
python3 -m venv ./rasa
source ./rasa/bin/activate
```

## Pip Dependencies
``` bash
pip3 install --upgrade pip
pip3 install -r requirements.txt
```
(requirements moved to docker)

# Running Rasa

First time:
- Run `rasa train` to train a model
- Run `rasa run actions --actions actions.actions` first (what is this for?)

Once the above has been done once.
- Use `rasa shell` to run rasa in the terminal or use `rasa run` to run as a web service (use curl to test. See below)

# Test Rasa REST API
``` bash
curl -i -X POST -H "Content-Type: application/json" -d "{\"sender\":\"test_user\", \"message\":\"Hello Tiago\"}" http://localhost:5005/webhooks/rest/webhook

curl -i -X POST -H "Content-Type: application/json" -d "{\"sender\":\"test_user\", \"message\":\"Can you bring me the crackerbox\"}" http://localhost:5005/webhooks/rest/webhook

curl -i -X POST -H "Content-Type: application/json" -d "{\"sender\":\"test_user\", \"message\":\"Can you find my glasses?\"}" http://localhost:5005/webhooks/rest/webhook
```

# Rasa Commands
`rasa init`

Creates a new project with example training data, actions, and config files.

`rasa train`
Trains a model using your NLU data and stories, saves trained model in `./models`.

`rasa train --finetune`
Used for incremental training. Reuses previously trained model and retrains it using newly added data. It might still be useful to run a full training.

`rasa run`

`rasa run --enable-api`

Starts a server with your trained model.

`rasa visualize`

Generates a visual representation of your stories.

`rasa interactive`

Starts an interactive learning session to create new training data by chatting to your assistant.

`rasa interactive --model ----skip-visualization`
Starts an interactive learning session without retraining model and generating visualizationras.


`rasa shell` or `rasa shell --debug`

Loads your trained model and lets you talk to your assistant on the command line.

`rasa run actions --actions actions.actions`

Run action server

`docker run -p 8000:8000 rasa/duckling`

Run duckling


