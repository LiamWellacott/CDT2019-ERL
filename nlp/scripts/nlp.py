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

    return question("")


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

@ask.intent("PickupIntent")
def response(object):

    msg = render_template('find', object=object)

    session.attributes['object'] = object

    return statement(msg)

@ask.intent("ReadinessCommand")
def introduce():

    msg = render_template('intro')

    return statement(msg)

@ask.intent("FoundItem") ###Still using alexa confirmation - can delete this and use code here instead
def found(object):

    msg = render_template('found', object=object)

    session.attributes['object'] = object

    return statement(msg)

@ask.intent("Handoff")
def handoff(object):

    msg = render_template('handoff', object=object)

    session.attributes['object'] = object

    return statement(msg)


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
