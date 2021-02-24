import logging

from random import randint

from flask import Flask, render_template

from flask_ask import Ask, statement, question, session, convert_errors #Added convert_errors ~Emilyann


app = Flask(__name__)

ask = Ask(app, "/")

logging.getLogger("flask_ask").setLevel(logging.DEBUG)


@ask.launch
def new_game():

    #welcome_msg = render_template('welcome')

    return question("Welcome to memory game. I'm going to say three numbers for you to repeat backwards. Ready?")


@ask.intent("YesIntent")
def next_round():

    numbers = [randint(0, 9) for _ in range(3)]

    round_msg = render_template('round', numbers=numbers)

    session.attributes['numbers'] = numbers[::-1]  # reverse

    return question(round_msg)


@ask.intent("AnswerIntent", convert={'first': int, 'second': int, 'third': int})
def answer(first, second, third):

    winning_numbers = session.attributes['numbers']

    if [first, second, third] == winning_numbers:

        msg = render_template('win')

    else:

        msg = render_template('lose')

    return statement(msg)

# @ask.intent("PickupIntent", convert={'object': str})
# def response(object):

#     msg = render_template(object)

#     return statement(msg)

# @ask.intent("ReadinessCommand")
# def introduce():

#     msg = render_template('I, Tiago, am ready to receive a command')

#     return statement(msg)

# @ask.intent("FoundItem", convert={'object': str})
# def found():

#     if 'object' in convert_errors:

#         msg = render_template('I, Tiago, have found the item')

#     else
#         msg = render_template('I, Tiago, have found the {}'.format(object))
    

#     return statement(msg)

# @ask.intent("Handoff", convert={'object': str})
# def handoff():

#     if 'object' in convert_errors:

#         msg = render_template('Here is the item, Granny Annie')

#     else
#         msg = render_template('Here is the {} Granny Annie'.format(object))
    

#     return statement(msg)


@ask.intent("Amazon.NavigateHomeIntent")
def home():
    return statement("home.")

@ask.intent("AMAZON.CancelIntent")
def cancel():
    return statement("cancel.")

@ask.intent("AMAZON.HelpIntent")
def help():
    return question("Help you? burk.")

@ask.intent("AMAZON.StopIntent")
def stop():
    return statement("I'm not stopping for you!")

if __name__ == '__main__':

    app.run(debug=True, host='0.0.0.0')
