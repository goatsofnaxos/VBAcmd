'''

© 2017 The Trustees of Columbia University in the City of New York. All Rights Reserved.

Started September 24, 2015
@author: Carl Schoonover
Finite state machine for virtual burrow assay v.3+
'''

from __future__ import division
from threading import Thread
from numpy import random
from time import sleep
from transitions import Machine

class VBAFSMthread(Thread):

    def __init__(self):

        # INPUT VALUES SET BY CMD
        self.doPrint = 1                     # Option to print current state of FSM
        self.rt = 0.5                        # Update rates (default is 0.5 sec; set by CMD)
        self.CMDhaltLoopSignal = 0           # Exit run loop
        self.CMDrestartSignal = 0            # Return to waiting4start state
        self.CMDolfactometerStartSignal = 0  # Up when olfactometer initiates trial, down at end
        self.CMDdoneStruggling = 0           # Up when animal no longer struggling/escaping
        self.CMDanimalEscaped = 0            # Up when animal escaped during slackened state
        self.CMDreadyToLaunch = 0            # Up when animal has been extruded without moving for long enough

        # FSM parameters
        self.states = ['waiting4start', 'pullingBack', 'slackening', 'launching']
        self.transitions = [
            {'trigger': 'startOver',      'source': '*',              'dest': 'waiting4start',  'after': 'do_waiting4start'},
            {'trigger': 'beginPull',      'source': 'waiting4start',  'dest': 'pullingBack',    'after': 'do_pullingBack',     'conditions': 'olfactometer_says_go'},
            {'trigger': 'slacken',        'source': 'pullingBack',    'dest': 'slackening',     'after': 'do_slackening',      'conditions': 'not_struggling_anymore'},
            {'trigger': 'catchorlaunch',  'source': 'slackening',     'dest': 'pullingBack',    'after': 'do_pullingBack',     'conditions': 'animal_escaped'},
            {'trigger': 'catchorlaunch',  'source': 'slackening',     'dest': 'launching',      'after': 'do_launching',       'conditions': 'ok_to_launch'},
            {'trigger': 'doneWithTrial',  'source': 'launching',      'dest': 'waiting4start',                                 'conditions': 'ok_to_wrap_up'},
        ]

        # Initiate FSM
        Machine(model=self, states=self.states, transitions=self.transitions, initial='waiting4start')

        # Constructor for threading
        Thread.__init__(self)

    def run(self):
        self.startOver()
        while True:
            if self.CMDhaltLoopSignal: self.halt_loop(); return
            elif self.CMDrestartSignal:
                self.reset_to_waiting4start()
            elif self.state is 'waiting4start':
                while True and (self.state is 'waiting4start'):
                    if self.CMDrestartSignal: self.reset_to_waiting4start(); break
                    if self.CMDhaltLoopSignal: self.halt_loop(); return
                    self.beginPull()
            elif self.state is 'pullingBack':
                while True and (self.state is 'pullingBack'):
                    if self.CMDrestartSignal: self.reset_to_waiting4start(); break
                    if self.CMDhaltLoopSignal: self.halt_loop(); return
                    self.slacken()
            elif self.state is 'slackening':
                while True and (self.state is 'slackening'):
                    if self.CMDrestartSignal: self.reset_to_waiting4start(); break
                    if self.CMDhaltLoopSignal: self.halt_loop(); return
                    self.catchorlaunch()
            elif self.state is 'launching':
                while True and (self.state is 'launching'):
                    if self.CMDrestartSignal: self.reset_to_waiting4start(); break
                    if self.CMDhaltLoopSignal: self.halt_loop(); return
                    self.doneWithTrial()
            else: pass

    def reset_to_waiting4start(self):
        self.CMDrestartSignal = 0
        self.startOver()

    def halt_loop(self):
        self.reset_to_waiting4start()

    def do_waiting4start(self):
        if self.doPrint: print self.state

    def do_pullingBack(self):
        if self.doPrint: print self.state

    def do_slackening(self):
        if self.doPrint: print self.state

    def do_launching(self):
        if self.doPrint: print self.state

    def olfactometer_says_go(self): # When self.CMDolfactometerStartSignal is high launch the trial
        sleep(self.rt)
        if self.doPrint: print '## OLFACTOMETER READY TO START TRIAL? ##', self.CMDolfactometerStartSignal
        return self.CMDolfactometerStartSignal > 0

    def not_struggling_anymore(self):
        sleep(self.rt)
        if self.doAutoCycle: condition = random.random_sample() > 0.5
        else: condition = self.CMDdoneStruggling > 0
        if self.doPrint: print '## ANIMAL DONE STRUGGLING? ##', condition
        return condition

    def animal_escaped(self):
        sleep(self.rt/2)
        if self.doAutoCycle: condition = random.random_sample() > 0.8
        else: condition = self.CMDanimalEscaped > 0
        if self.doPrint: print '## ANIMAL ESCAPED? ##', condition
        return condition

    def ok_to_launch(self):
        sleep(self.rt/2)
        if self.doAutoCycle: condition = random.random_sample() > 0.95
        else: condition = self.CMDreadyToLaunch > 0
        if self.doPrint: '## READY TO LAUNCH? ##', condition
        return condition

    def ok_to_wrap_up(self): # When self.CMDolfactometerStartSignal is low terminate the trial
        if self.doPrint: print '## IT AINT OVER TILL THE OLFACTOMETER SAYS ITS OVER ##', not self.CMDolfactometerStartSignal
        sleep(self.rt)
        return not self.CMDolfactometerStartSignal


if __name__ == '__main__':
    vbafsm = VBAFSMthread()
    vbafsm.doPrint = 1
    vbafsm.doAutoCycle = 1
    vbafsm.start()
    vbafsm.CMDolfactometerStartSignal = 1