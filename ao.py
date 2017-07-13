# © 2017 The Trustees of Columbia University in the City of New York. All Rights Reserved.

from __future__ import division
from pylab import *
from PyDAQmx import *
import numpy
import time

###data = numpy.arange(2, dtype=numpy.float64)*0
###aonumsamples = 2
servowritedata = numpy.arange(1, dtype=numpy.float64)*0
aonumsamples = 1

# Create Task
ao = Task()
aowrite = int32()
timeout = 10.0
autostart = 0

# DAQmx Configure Code
###ao.CreateAOVoltageChan("dev5/ao0:1","Voltage",0.0,5.0,DAQmx_Val_Volts,None)
ao.CreateAOVoltageChan("dev5/ao1","Voltage",0.0,5.0,DAQmx_Val_Volts,None)
ao.StartTask()
print "UP"
ao.WriteAnalogF64(aonumsamples,autostart,timeout,DAQmx_Val_GroupByChannel,servowritedata+0,aowrite,None)
# time.sleep(10)
# print "DOWN"
# ao.WriteAnalogF64(aonumsamples,autostart,timeout,DAQmx_Val_GroupByChannel,data,write,None)

# Stop and clear task
ao.StopTask()
ao.ClearTask()