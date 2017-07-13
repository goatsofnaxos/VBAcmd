'''

© 2017 The Trustees of Columbia University in the City of New York. All Rights Reserved.

Started September 24, 2015
@author: Carl Schoonover
'''

from threading import Thread
from numpy import random
from time import sleep
from transitions import Machine

class VBAFSMthread(Thread):

    def __init__(self,window):
        self.window = window
        self.states = ['waiting4start', 'pullingBack', 'slackening', 'launching']
        self.transitions = [
            {'trigger': 'startOver',      'source': '*',              'dest': 'waiting4start',  'after': 'do_waiting4start'},
            {'trigger': 'beginPull',      'source': 'waiting4start',  'dest': 'pullingBack',    'after': 'do_pullingBack',     'conditions': 'olfactometer_says_go'},
            {'trigger': 'slacken',        'source': 'pullingBack',    'dest': 'slackening',     'after': 'do_slackening',      'conditions': 'not_struggling_anymore'},
            {'trigger': 'catchorlaunch',  'source': 'slackening',     'dest': 'pullingBack',    'after': 'do_pullingBack',     'conditions': 'animal_escaped'},
            {'trigger': 'catchorlaunch',  'source': 'slackening',     'dest': 'launching',      'after': 'do_launching',       'conditions': 'ok_to_launch'},
        ]
        Machine(model=self, states=self.states, transitions=self.transitions, initial='waiting4start')
        # Constructor
        Thread.__init__(self)

    def run(self):
        self.startOver()

    def do_waiting4start(self):
        print self.state
        # Set trigger (do0) to low
        # Instruct servo to move to slack position (on ao0)
        # Wait for trial-start signal from loopcloser (on di0); when high,
        while True and (self.state is 'waiting4start'):
            self.beginPull()

    def do_pullingBack(self):
        print self.state
        # Instruct servo to move to pullback position (on ao0)
        # Wait for position (on ai0) and and force (on ai1) to meet conditions (i.e. not_struggling_anymore) for > N seconds (optional: N variable), then slacken
        while True and (self.state is 'pullingBack'):
            self.slacken()

    def do_slackening(self):
        print self.state
        # Instruct servo to move to slack position (on ao0)
        # Check that position (on ai0) meets condition (i.e. animal hasn't escaped), send FSM to pullingBack
        # When position (on ai0) has met condition (i.e. animal hasn't escaped) for > N seconds (optional: N variable), launch
        while True and (self.state is 'slackening'):
            self.catchorlaunch()

    def do_launching(self):
        print self.state
        # Set trigger (do0) to high
        # Wait for trial-end signal from loopcloser (on di0); when low,
        print 'STARTING OVER AGAIN IN 3 SECONDS'
        sleep(3)
        self.startOver()

    def olfactometer_says_go(self):
        bufLen = 0.5
        statevar = random.random_sample() > 0.7
        print '## OLFACTOMETER READY TO START TRIAL? ##', statevar
        sleep(bufLen/2)
        return statevar

    def not_struggling_anymore(self):
        bufLen = 0.5
        statevar = random.random_sample() > 0.7
        print '## ANIMAL DONE STRUGGLING? ##', statevar
        sleep(bufLen/2)
        return statevar

    def animal_escaped(self):
        bufLen = 0.5
        statevar = random.random_sample() > 0.7
        print '## ANIMAL ESCAPED? ##', statevar
        sleep(bufLen/2)
        return statevar

    def ok_to_launch(self):
        bufLen = 0.5
        statevar = random.random_sample() > 0.8
        print '## READY TO LAUNCH? ##', statevar
        sleep(bufLen/2)
        return statevar



if __name__ == '__main__':
    window = 2
    vbafsm = VBAFSMthread(window)
    # Start running the thread
    vbafsm.start()