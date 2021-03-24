# CDT2019-ERL / Granny Annie's Android Nanny (GAAN)

This is a competition entry for the European Robotic League and part of our PhD work at the Edinburgh Centre for Robotics Centre for Doctoral Training in Robotics and Autonomous Systems. The goal of this project is to produce a software system which is:

- Competitive, i.e. software supporting HWU ECR entries to this type of competitions (ERL, Robocup@Home, METRICS...).
- Flexible, modular, and easy to configure software (teams typically have half a day / one day) to configure their system to operate in the test-bed where the tournament takes place - ideally most of which should be usable for both HSR and Tiago
- Robust (keep things simple, consider an integration and testing plan)

In the end we didn't manage to create a competition ready system, but we learned a lot which we hope can be used for other teams to help them get started with the competition. In particular this repo provides:

- A (mostly working) docker image and container for tiago, which should simplify the installation and set up phase for your project, if you are using this robot.
- A virtual environment and task scenario which can be used to test your system functionality "restricted_task_3"
- A software architecture which you can base a competition system on, you may want to replace some components to improve 

## Project documentation

- High level project documentation including software architecture and scenario diagrams, see [this presentation](https://drive.google.com/file/d/1-JlEw5rZ3xLCFbg1VuI-TzUSlgtNpt1-/view?usp=sharing)
- For set up see [docker README](docker/README.md), if you have issues with this I have made [a setup guide for local machines](setup.md)
- Code level documentation:
  - [face_rec](GAAN/face_rec/README.md)
  - [gaan_controller](GAAN/gaan_controller/README.md)
  - [manpulation](GAAN/gaan_controller/README.md)
  - [navigation](GAAN/navigation/README.md)
  - [nlp](GAAN/nlp/README.md)
- Check the github issues for some design decisions made (note the detail is highly variable)


## Setup and run guides

check the readme in ``docker`` for information on using that environment. I also tried to make a "setup your local machine guide" in ``setup.md`` in case you have issues with the docker environment.

## On the test framework

Our idea was to design a number of "simple" scenarios to help bridge the gap between functional benchmarks and the full competition tasks. Currently we have only implemented a single scenario which we call "restricted task 3". In addition to providing the virtual environment we have also implemented enough of a "fake RSBB" so you can receieve a score for running the scenario (TODO). We hope this provides a tool for validating your system.

To run the test scenario, in first terminal run:

```
roslaunch sim_launch restricted_task_3.launch
```

Allow the simulation to launch, arm to tuck, adjust any visuals. Then run:

```
roslaunch fake_rssbb restricted_task_3.launch
```
The score will be returned by the service calls and available in the controller, you could implement this as an automated test case by comparing the score to an expected score for each part of the scenario. 

## Other Project Sites for team members
- [Sharepoint](https://heriotwatt.sharepoint.com/sites/CDT2019-ERL) for project documentation, you must use your HW credentials to access, non HW accounts cannot be added to this.
- [Overleaf report](https://www.overleaf.com/read/tbvrxpjrnrkt) for paper (tell me your email to be added as collaborator)
- Some notes on possible [tools/algorithms](https://heriotwatt.sharepoint.com/sites/CDT2019-ERL/_layouts/15/doc.aspx?sourcedoc={c50ef375-786c-45bd-8ff7-3e8696c3442a}&action=edit) which may be useful for coming up with a solution.

## General Resources 
- The main source of information is the [consumer page of the competition website](https://www.eu-robotics.net/robotics_league/erl-consumer/about/index.html), here you can find the [rulebook](https://www.eu-robotics.net/robotics_league/upload/documents-2018/ERL_Consumer_10092018.pdf) which contains detailed descriptions of the tasks to be completed. Some additional information on the [test environments](https://www.eu-robotics.net/robotics_league/erl-consumer/certified-test-beds/index.html) and specifically the [Edinburgh test environment](https://www.eu-robotics.net/robotics_league/upload/documents-2017/ERL-SR_TestBedCertificationForm_HWU_web.pdf)
- Home page for the [Edinburgh assisted living testbed](https://ralt.hw.ac.uk/)





