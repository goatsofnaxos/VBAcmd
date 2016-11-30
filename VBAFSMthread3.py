'''
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
        self.doPrint = 1                            # Option to print current state of FSM
        self.rt = 0.5                               # Update rates (default is 0.5 sec; set by CMD)
        self.CMDhaltLoopSignal = 0                  # Exit run loop
        self.CMDrestartSignal = 0                   # Return to waiting4start state
        self.CMDolfactometerSaysPull = [True, True] # Up when olfactometer is not in [odor delivery + behavior] epochs
        self.CMDdoneStruggling = 0                  # Up when animal no longer struggling/escaping
        self.CMDanimalEscaped = 0                   # Up when animal escaped during slackened state

        # OUTPUT VALUES READ BY CMD
        self.CMDtriggerHigh = 0

        # FSM parameters
        self.states = ['waiting4start', 'pullingBack', 'slackening', 'ready', 'launching']
        self.transitions = [
            {'trigger': 'startOver',     'source': '*',             'dest': 'waiting4start', 'after': 'do_waiting4start'},
            {'trigger': 'beginPull',     'source': 'waiting4start', 'dest': 'pullingBack',   'after': 'do_pullingBack'},
            {'trigger': 'slacken',       'source': 'pullingBack',   'dest': 'slackening',    'after': 'do_slackening',     'conditions': 'not_struggling_anymore'},
            {'trigger': 'catchOrReady',  'source': 'slackening',    'dest': 'pullingBack',   'after': 'do_pullingBack',    'conditions': 'animal_escaped'},
            {'trigger': 'catchOrReady',  'source': 'slackening',    'dest': 'ready',         'after': 'do_ready',          'conditions': 'animal_ready'},
            {'trigger': 'catchOrLaunch', 'source': 'ready',         'dest': 'launching',     'after': 'do_launching',      'conditions': 'olf_says_launch'},
            {'trigger': 'catchOrLaunch', 'source': 'ready',         'dest': 'pullingBack',   'after': 'do_pullingBack',    'conditions': 'animal_escaped'},
            {'trigger': 'trialDone',     'source': 'launching',     'dest': 'waiting4start', 'after': 'do_waiting4start',  'conditions': 'olf_says_trial_over'},
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
                    self.catchOrReady()
            elif self.state is 'ready':
                while True and (self.state is 'ready'):
                    if self.CMDrestartSignal: self.reset_to_waiting4start(); break
                    if self.CMDhaltLoopSignal: self.halt_loop(); return
                    self.catchOrLaunch()
            elif self.state is 'launching':
                while True and (self.state is 'launching'):
                    if self.CMDrestartSignal: self.reset_to_waiting4start(); break
                    if self.CMDhaltLoopSignal: self.halt_loop(); return
                    self.trialDone()
            else: pass

    def reset_to_waiting4start(self):
        self.CMDrestartSignal = 0
        self.CMDtriggerHigh = 0
        self.startOver()

    def halt_loop(self):
        self.CMDtriggerHigh = 0
        self.reset_to_waiting4start()

    def do_waiting4start(self):
        self.CMDtriggerHigh = 0
        if self.doPrint: print self.state
        sleep(self.rt*4)

    def do_pullingBack(self):
        self.CMDtriggerHigh = 0
        if self.doPrint: print self.state
        sleep(self.rt*4)

    def do_slackening(self):
        self.CMDtriggerHigh = 0
        if self.doPrint: print self.state
        sleep(self.rt*4)

    def do_ready(self):
        self.CMDtriggerHigh = 1
        if self.doPrint: print self.state
        sleep(self.rt*4)

    def do_launching(self):
        self.CMDtriggerHigh = 0
        if self.doPrint: print self.state
        sleep(self.rt*4)

    def not_struggling_anymore(self):
        sleep(self.rt)
        if self.doAutoCycle: condition = random.random_sample() > 0.8
        else: condition = self.CMDdoneStruggling # CONDITIONS: (1) Servo done pulling; (2) Animal hasn't generated force for N seconds
        if self.doPrint: print '## ANIMAL DONE STRUGGLING? ##', condition
        return condition

    def animal_escaped(self):
        sleep(self.rt/2)
        if self.doAutoCycle: condition = random.random_sample() > 0.8
        else: condition = self.CMDanimalEscaped # CONDITIONS: (1) Laser says animal moved
        if self.doPrint: print '## ANIMAL ESCAPED? ##', condition
        return condition

    def animal_ready(self):
        sleep(self.rt/2)
        if self.doAutoCycle: condition = random.random_sample() > 0.8
        else: condition = self.CMDanimalReady   # CONDITIONS: (1) Servo done slackening; (2) Animal hasn't moved for N seconds
        if self.doPrint: print '## ANIMAL READY? ##', condition
        return condition

    def olf_says_launch(self):
        sleep(self.rt/2)
        if self.doAutoCycle: condition = random.random_sample() > 0.8
        else: condition = self.CMDolfactometerSaysPull[1] == False   # CONDITIONS: (1) Olfactometer not telling VBA to pull anymore (i.e. = 0) because odor presentation has begun
        if self.doPrint: print '## OLFACTOMETER TOLD VBA TO STOP PULLING? ##', condition
        return condition

    def olf_says_trial_over(self):
        sleep(self.rt)
        if self.doAutoCycle: condition = True; sleep(2)
        else: condition = self.CMDolfactometerSaysPull[1] == True   # CONDITIONS: (1) Olfactometer telling VBA to start pulling again (i.e. = 1) because trial is over
        if self.doPrint: print '## OLFACTOMETER TOLD VBA THAT TRIAL IS DONE? ##', condition
        return condition

if __name__ == '__main__':
    vbafsm = VBAFSMthread()
    vbafsm.doPrint = 1
    vbafsm.doAutoCycle = 1
    vbafsm.start()
    vbafsm.CMDolfactometerSaysPull[1] = True