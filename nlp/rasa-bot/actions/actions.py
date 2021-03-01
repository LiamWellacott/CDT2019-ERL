# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions


#This is a simple example for a custom action which utters "Hello World!"
from random import choice

from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import SlotSet


class ActionsSearchCommand(Action):

    def name(self) -> Text:
        return "action_get_search_status"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        print(tracker.get_latest_entity_values("item"))
        answer = choice(['success', 'failure'])
        # Wait for ROS respone
        return [SlotSet("search_status", answer)]

class ActionsBringCommand(Action):

    def name(self) -> Text:
        return "action_get_bring_status"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        print(tracker.get_latest_entity_values("item"))
        # answer = choice(['success', 'failure'])
        # Wait for ROS respone
        return [SlotSet("bring_status", "success")]
