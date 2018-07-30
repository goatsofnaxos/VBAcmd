'''

(c) 2017 The Trustees of Columbia University in the City of New York. All Rights Reserved.

Started July 21, 2015
@author: Carl Schoonover
GUI and control of virtual burrow assay v. 4+
now includes analog input
'''

from __future__ import division

# Change working directory, compile UI and load configuration
from os import chdir, path
from PyQt4 import uic
curpath = path.dirname(path.realpath(__file__))
chdir(curpath)
uic.compileUiDir(curpath)
from VBAconfig import VBAconfig
import sys
from ctypes import byref

from PyQt4 import QtGui, QtCore, Qt
import PyQt4.Qwt5 as Qwt
from UIdesign import Ui_MainWindow
from VBAFSMthread3 import VBAFSMthread
from PyDAQmx import *
from time import time, strftime
import numpy
from ParamLoad import ParamLoad
from Scaling import Scaling

"""""""""""""""""""""""""""""""""""""""""""""""""""
NI-DAQmx callback class
"""""""""""""""""""""""""""""""""""""""""""""""""""
class DAQCallbackTask(Task):

    global DAQmx_Val_RSE,DAQmx_Val_Volts, DAQmx_Val_Rising, DAQmx_Val_ContSamps, DAQmx_Val_Acquired_Into_Buffer, DAQmx_Val_GroupByScanNumber, DAQmx_Val_GroupByChannel, DAQmx_Val_ChanForAllLines,DAQmx_Val_OnDemand

    def __init__(self,initialUsrPrms):

        Task.__init__(self)

        # Load hardware configuration and previous parameters if extant
        vbaconfig = VBAconfig()
        self.initialUsrPrms = initialUsrPrms

        # Scaling
        self.scaleLaser    = Scaling(vbaconfig.encoding['rangeLaser'],vbaconfig.encoding['rangeLaserVout'],vbaconfig.encoding['unitLaser'])
        self.scaleForce    = Scaling(vbaconfig.encoding['rangeForce'],vbaconfig.encoding['rangeForceVout'],vbaconfig.encoding['unitForce'])
        self.scaleServo    = Scaling(vbaconfig.encoding['rangeServo'],vbaconfig.encoding['rangeServoVout'],vbaconfig.encoding['unitServo'])
        self.scaleServoSet = Scaling(vbaconfig.encoding['rangeServoSet'],vbaconfig.encoding['rangeServoSetVcmd'],vbaconfig.encoding['unitServoSet'])
        self.scaleAI       = Scaling(vbaconfig.encoding['rangeAI'],vbaconfig.encoding['rangeAIVout'],vbaconfig.encoding['unitAI'])

        # ANALOG INPUT (Laser / Force / Servo position / Optional analog input)
        self.numaiChannels =  vbaconfig.channels['ainumchanels']
        self.aiChannelIDs =   vbaconfig.channels['aiChannelIDs']
        self.numaoChannels =  vbaconfig.channels['aonumchanels']
        self.aoChannelIDs =   vbaconfig.channels['aoChannelIDs']
        self.airt =           1000 # Hz
        self.DAQBufferEpoch = 100 # ms
        self.numBuffsPerSec = int(round(1000 / self.DAQBufferEpoch)) # Hz
        self.DAQBufferSize =  int(round(self.airt/self.numBuffsPerSec)) # Samples
        self.timeout =        10.0 # s
        self.Vrange =         [-10.0, 10.0]
        self.autostart =      0
        self.read =           int32()
        self.totNumBuffers =  int(0)
        # Circular buffers
        self.circBuffEpoch = 8000 # ms
        self.circBuffSamps = int(round( (self.circBuffEpoch/1000) * self.airt))
        self.circBufferAnalogIn       = numpy.zeros((self.circBuffSamps),dtype=float32)
        self.circBufferLaser          = numpy.zeros((self.circBuffSamps),dtype=float32)
        self.circBufferForce          = numpy.zeros((self.circBuffSamps),dtype=float32)
        self.circBufferServo          = numpy.zeros((self.circBuffSamps),dtype=float32)
        self.circBufferDI             = numpy.zeros(numpy.int(self.circBuffSamps/self.DAQBufferSize),dtype=float32)
        self.circBufferServoSlope     = numpy.zeros(numpy.int(self.circBuffSamps/self.DAQBufferSize),dtype=float32)
        self.circBufferLaserThreshold = numpy.zeros(numpy.int(self.circBuffSamps/self.DAQBufferSize),dtype=float32) + self.initialUsrPrms.laserThreshold
        self.circBufferForceThreshold = numpy.zeros(numpy.int(self.circBuffSamps/self.DAQBufferSize),dtype=float32) + self.initialUsrPrms.forceThreshold
        self.circBufferTimeLong       = numpy.linspace(start=0,stop=self.circBuffEpoch/1000,num=self.circBufferLaser.size)
        self.circBufferTimeShort      = numpy.linspace(start=0,stop=self.circBuffEpoch/1000,num=self.circBufferDI.size)
        # DAQ read data buffer
        self.DAQbufferDataIn = numpy.tile(numpy.zeros((self.DAQBufferSize,), dtype=numpy.float64),(self.numaiChannels,1)) # Time column-wise
        # self.DAQbufferDataIn = numpy.tile(numpy.zeros((self.DAQBufferSize,), dtype=numpy.float64),(self.numaiChannels,1)) # Time row-wise
        self.CreateAIVoltageChan(self.aiChannelIDs,"Analog inputs",DAQmx_Val_RSE,self.Vrange[0],self.Vrange[1],DAQmx_Val_Volts,None)
        self.CfgSampClkTiming("",self.airt,DAQmx_Val_Rising,DAQmx_Val_ContSamps,self.DAQBufferSize)
        self.AutoRegisterEveryNSamplesEvent(DAQmx_Val_Acquired_Into_Buffer,self.DAQBufferSize,0,name='EveryNCallback')
        self.AutoRegisterDoneEvent(0,name='DoneCallback')
        self.StartTask()
        # self.ReadAnalogF64(self.DAQBufferSize,self.timeout,DAQmx_Val_GroupByChannel,self.DAQbufferDataIn,self.DAQBufferSize*self.numaiChannels,byref(self.read),None)




        # ANALOG OUTPUT (Servo command)
        self.ao = Task()
# OLD        self.ao.aowritedata = numpy.arange(1, dtype=numpy.float64)*0

        self.ao.numsamples = 1
        self.ao.aowritedata = numpy.tile(numpy.zeros((self.ao.numsamples,), dtype=numpy.float64), (self.numaoChannels, 1))
        self.ao.write = int32()

# OLD        self.ao.CreateAOVoltageChan(vbaconfig.channels['aoChannelIDs'],"Analog output",self.scaleServoSet.signal[0],self.scaleServoSet.signal[1],DAQmx_Val_Volts,None)
        self.ao.CreateAOVoltageChan(vbaconfig.channels['aoChannelIDs'],"",self.Vrange[0],self.Vrange[1],DAQmx_Val_Volts,None)

        self.ao.StartTask()
        self.ao.WriteAnalogF64(self.ao.numsamples,self.autostart,self.timeout,DAQmx_Val_GroupByChannel,self.ao.aowritedata+2.5,self.ao.write,None)




        # DIGITAL INPUT (Toggle closed / open loop mode)
        self.di = Task()
        self.di.fillMode = 1
        self.di.numSampsPerChan = 1
        self.di.read = int32()
        self.di.numBytesPerSamp = int32()
        self.di.readData = numpy.array([0], dtype=numpy.uint8)
        self.di.arraySizeInBytes = self.di.readData.__len__()
        self.di.CreateDIChan(vbaconfig.channels['diChannelIDs'],"Digital input",DAQmx_Val_ChanForAllLines)
        self.di.StartTask()
        # self.di.ReadDigitalLines(self.di.numSampsPerChan, self.timeout, self.di.fillMode, self.di.readData, self.di.arraySizeInBytes, self.di.read, byref(self.di.numBytesPerSamp), None)

        # DIGITAL OUTPUT (Indicate ready state to stimulus control)
        self.do = Task()
        self.do.CreateDOChan(vbaconfig.channels['doChannelIDs'],"Digital output",DAQmx_Val_ChanForAllLines)
        self.do.StartTask()
        # self.do.WriteDigitalScalarU32(1,1,0,None)

        # VBA finite state machine
        self.vbafsm = VBAFSMthread()
        self.vbafsm.rt = self.DAQBufferEpoch/(2*1000)
        self.vbafsm.doPrint = 0
        self.vbafsm.doAutoCycle = 0
        self.vbafsm.CMDolfactometerSaysPull = [True, True]
        self.vbafsm.start()
        self.VBAFSMstate = ['state', 'state']
        self.vbafsm.CMDtriggerHigh = 0

    def EveryNCallback(self):

        # Get analog input
        self.ReadAnalogF64(self.DAQBufferSize,self.timeout,DAQmx_Val_GroupByChannel,self.DAQbufferDataIn,self.DAQBufferSize*self.numaiChannels,byref(self.read),None)
        self.aiDataForSigProc = self.DAQbufferDataIn

        # Get digital input and update window if it changes
        self.di.ReadDigitalLines(self.di.numSampsPerChan, self.timeout, self.di.fillMode, self.di.readData, self.di.arraySizeInBytes, self.di.read, byref(self.di.numBytesPerSamp), None)

        self.vbafsm.CMDolfactometerSaysPull[0] = self.vbafsm.CMDolfactometerSaysPull[1]
        self.vbafsm.CMDolfactometerSaysPull[1] = self.di.readData[0]
        # Update window if digital start signal changes
        if self.vbafsm.CMDolfactometerSaysPull[1] != self.vbafsm.CMDolfactometerSaysPull[0]:
            window.updateDIstateSignal.emit()
        if window.controlMode_manual:
            self.di.readData[0] = 0
        # Get state of finite state machine from thread
        self.VBAFSMstate[0] = self.VBAFSMstate[1]
        self.VBAFSMstate[1] = self.vbafsm.state
        # Update window if finite state machine state changes
        if self.VBAFSMstate[1] != self.VBAFSMstate[0]:
            window.updateStateSignal.emit()

        # Keep track of buffers
        self.totNumBuffers += 1
        window.writeSignal.emit() # Emit signal to write data
        return 0 # The function should return an integer

    def DoneCallback(self, status):
        print "Status ",status.value
        return 0 # The function should return an integer

    def updateOutputTrigger(self,state):
        self.do.WriteDigitalScalarU32(1,1,state,None)

    def updateServoPosition(self,position):
        pass
        # scaledPosition = self.ao.aowritedata + self.scaleServoSet.act2sig(position) #(position/self.rangeServo[1])*self.rangeServoSet[1]
        # self.ao.WriteAnalogF64(self.ao.numsamples,self.autostart,self.timeout,DAQmx_Val_GroupByChannel,scaledPosition,self.ao.write,None)

    def clearDAQ(self):
        self.ao.StopTask()
        self.ao.ClearTask()
        self.do.StopTask()
        self.do.ClearTask()
        self.di.StopTask()
        self.di.ClearTask()
        self.StopTask()
        self.ClearTask()



"""""""""""""""""""""""""""""""""""""""""""""""""""
UI and signal processing
"""""""""""""""""""""""""""""""""""""""""""""""""""
class Main(QtGui.QMainWindow):

    writeSignal = QtCore.pyqtSignal()
    drawTraceSignal = QtCore.pyqtSignal()
    updateDIstateSignal = QtCore.pyqtSignal()
    updateStateSignal = QtCore.pyqtSignal()

    def __init__(self):
        # Load previous user parameters if they exist. or else set them
        self.paramLoad = ParamLoad(__file__)
        self.usrPrms = self.paramLoad.loadUserParams()
        if self.usrPrms.loadedOldParams == 1:
            pass
        else:
            self.usrPrms.laserThreshold = 70.0   # (mm)
            self.usrPrms.forceThreshold = 15.0   # (g)
            self.usrPrms.struggleWait = 1.0      # (s) amount of time to wait after animal done struggling
            self.usrPrms.pullPosition = 17.0     # (mm) "egress"
            self.usrPrms.slackPosition = 40.0    # (mm) "ingress"
            self.usrPrms.movementWait = 1.5      # (s) amount of time to wait to make sure animal not moving
            self.usrPrms.servoWait = 1.0         # (s) amount of time to wait after servo moves before transitioning
            self.usrPrms.aiRangeMax = 10.0       # (V)
            self.usrPrms.aiRangeMin = -10.0      # (V)
            self.usrPrms.laserSDThreshold = 15.0 # ([SD of mm] * 100)
            self.usrPrms.laserServoCalib = 42.0  # (mm) offset to align laser and servo readings
        # Launch the UI
        QtGui.QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle("VBAcmd")
        self.ui.tubeStateTextLabel.setText('MANUAL CONTROL')
        # Set appearance of widgets
        self.ui.analogInPlot.setCanvasBackground(Qt.Qt.white)
        self.ui.analogInPlot.setAxisScale(Qwt.QwtPlot.yLeft,self.usrPrms.aiRangeMin,self.usrPrms.aiRangeMax)
        self.ui.positionPlot.setCanvasBackground(Qt.Qt.white)
        self.ui.forcePlot.setCanvasBackground(Qt.Qt.white)
        self.ui.startSig.setStyleSheet("background-color: white;")
        # Run mode parameters
        self.controlMode_manual = 1
        self.servoState_slackening = 1
        self.outputTriggerHigh = 0
        self.timer = 0
        self.nojiggle = False
        self.noyank = False
        # Set default user values
        self.ui.laserThresholdDoubleSpinBox.setValue(self.usrPrms.laserThreshold)
        self.ui.laserWaitTimeDoubleSpinBox.setValue(self.usrPrms.movementWait)
        self.ui.servoWaitTimeDoubleSpinBox.setValue(self.usrPrms.servoWait)
        self.ui.forceThresholdDoubleSpinBox.setValue(self.usrPrms.forceThreshold)
        self.ui.forceTimeBeforeSlackDoubleSpinBox.setValue(self.usrPrms.struggleWait)
        self.ui.pullPositionDoubleSpinBox.setValue(self.usrPrms.pullPosition)
        self.ui.slackPositionDoubleSpinBox.setValue(self.usrPrms.slackPosition)
        self.ui.aiRangeMaxSpinBox.setValue(self.usrPrms.aiRangeMax)
        self.ui.aiRangeMinSpinBox.setValue(self.usrPrms.aiRangeMin)
        self.ui.manualRadioButton.toggle()
        self.ui.slackPositionRadioButton.toggle()
        self.ui.laserSDThresholdDoubleSpinBox.setValue(self.usrPrms.laserSDThreshold)
        # Set plotting parameters
        self.circBufLaserIndex = 0
        self.circBufForceIndex = 0
        self.plotEveryNBuffers = 1
        self.circBufIndex = 0
        # Connect clicking signals
        self.ui.laserThresholdDoubleSpinBox.valueChanged.connect(self.laserThresholdDoubleSpinBox_changed)
        self.ui.laserWaitTimeDoubleSpinBox.valueChanged.connect(self.laserWaitTimeDoubleSpinBox_changed)
        self.ui.servoWaitTimeDoubleSpinBox.valueChanged.connect(self.servoWaitTimeDoubleSpinBox_changed)
        self.ui.forceThresholdDoubleSpinBox.valueChanged.connect(self.forceThresholdDoubleSpinBox_changed)
        self.ui.forceTimeBeforeSlackDoubleSpinBox.valueChanged.connect(self.forceTimeBeforeSlackDoubleSpinBox_changed)
        self.ui.pullPositionDoubleSpinBox.valueChanged.connect(self.pullPositionDoubleSpinBox_changed)
        self.ui.slackPositionDoubleSpinBox.valueChanged.connect(self.slackPositionDoubleSpinBox_changed)
        self.ui.aiRangeMaxSpinBox.valueChanged.connect(self.aiRangeMaxSpinBox_changed)
        self.ui.aiRangeMinSpinBox.valueChanged.connect(self.aiRangeMinSpinBox_changed)
        self.ui.manualRadioButton.toggled.connect(self.manualRadioButton_toggled)
        self.ui.slackPositionRadioButton.toggled.connect(self.slackPositionRadioButton_toggled)
        self.ui.laserSDThresholdDoubleSpinBox.valueChanged.connect(self.laserSDThreshold_changed)

        self.ui.calibratePushButton.clicked.connect(self.calibratePushButton_clicked)

        # Set up event signals
        app.aboutToQuit.connect(self.shutDown)
        self.writeSignal.connect(self.sigProcess)
        self.drawTraceSignal.connect(self.plotTraces)
        self.updateDIstateSignal.connect(self.updateDIstate)
        self.updateStateSignal.connect(self.updateState)
        # Create the DAQ object
        self.task = DAQCallbackTask(self.usrPrms)
        self.task.animalNotStruggledInNSeconds = False
        # Set some variables and UI values that depend on DAQ initialization
        self.ui.forceTimeBeforeSlackDoubleSpinBox.setMaximum(self.task.circBuffEpoch/1000)
        self.ui.laserWaitTimeDoubleSpinBox.setMaximum(self.task.circBuffEpoch/1000)
        self.ui.servoWaitTimeDoubleSpinBox.setMaximum(self.task.circBuffEpoch/1000)
        self.forceCheckWindow = int(self.usrPrms.struggleWait * self.task.airt)
        self.laserCheckWindow = int(self.usrPrms.movementWait * self.task.airt)
        self.servoCheckWindow = int(self.usrPrms.servoWait * (self.task.airt/self.task.DAQBufferEpoch))
        self.updateDIstate()
        # Set plot title text and appearance (also depends on DAQ initialization
        AItitle    = 'Analog input (' + self.task.scaleAI.unit + ')'
        laserTitle = 'Laser Position (' + self.task.scaleLaser.unit + ')'
        servoTitle = 'Servo Position (' + self.task.scaleServo.unit + ')'
        forceTitle = 'Force (' + self.task.scaleForce.unit + ')'
        self.ui.chartTitle_AnalogInput.setText(AItitle)
        self.ui.chartTitle_laserPosition.setText(laserTitle)
        self.ui.chartTitle_servoPosition.setText(servoTitle)
        self.ui.chartTitle_force.setText(forceTitle)
        self.ui.chartTitle_servoPosition.setStyleSheet("color: red")
        self.ui.chartTitle_laserPosition.setStyleSheet("color: blue")

    def sigProcess(self):

        execTimingStart = time()

        # Check if VBAFSM is in launch state and if so, update the trigger on D.O. (except if jiggling or yanking on the tether)
        if not (self.nojiggle and self.task.animalNotStruggledInNSeconds):
            if self.outputTriggerHigh > 0:
                self.outputTriggerHigh = 0
                self.task.updateOutputTrigger(self.outputTriggerHigh)
        else:
            if self.outputTriggerHigh != self.task.vbafsm.CMDtriggerHigh:
                self.outputTriggerHigh = self.task.vbafsm.CMDtriggerHigh
                self.task.updateOutputTrigger(self.outputTriggerHigh)

        # Set up indexing to move AI and DI to circular buffer
        if self.circBufIndex >= self.task.circBufferDI.size:
            self.circBufIndex = 0
        indexLong = [self.circBufIndex*self.task.DAQBufferSize, self.circBufIndex*self.task.DAQBufferSize+self.task.DAQBufferSize]

        # Construct scaled circular buffer of analog and digital inputs (0:LASER, 1:FORCE, 2:SERVO, 3:ANALOGIN)
        self.task.circBufferLaser[indexLong[0]:indexLong[1]]    = self.task.scaleLaser.sig2act(self.task.aiDataForSigProc[0,:])
        self.task.circBufferForce[indexLong[0]:indexLong[1]]    = self.task.scaleForce.sig2act(self.task.aiDataForSigProc[1,:])
        self.task.circBufferServo[indexLong[0]:indexLong[1]]    = self.task.scaleServo.actual[1] - self.task.scaleServo.sig2act(self.task.aiDataForSigProc[2,:]) + self.usrPrms.laserServoCalib
        self.task.circBufferAnalogIn[indexLong[0]:indexLong[1]] = self.task.scaleAI.sig2act(self.task.aiDataForSigProc[3,:])
        self.task.circBufferDI[self.circBufIndex] = self.task.vbafsm.CMDolfactometerSaysPull[1]
        # Construct circular buffer of thresholds
        self.task.circBufferLaserThreshold[self.circBufIndex] = self.usrPrms.laserThreshold
        self.task.circBufferForceThreshold[self.circBufIndex] = self.usrPrms.forceThreshold
        # Figure out whether animal is below position threshold (measured by laser)
        self.task.belowPositionThreshold = all(self.task.circBufferLaser[indexLong[0]:indexLong[1]] < self.usrPrms.laserThreshold)
        # Figure out whether animal has not moved for N seconds
        circBufferLaserRolled = numpy.roll(self.task.circBufferLaser,-(indexLong[1]))
        self.rescentSTD = numpy.std(circBufferLaserRolled[-self.laserCheckWindow:]) * 100
        self.nojiggle = self.rescentSTD < self.usrPrms.laserSDThreshold
        self.task.animalNotMovedInNSeconds = all(circBufferLaserRolled[-self.laserCheckWindow:] < self.usrPrms.laserThreshold) and self.nojiggle
        # Figure out whether animal has not been struggling for N seconds
        circBufferForceRolled = numpy.roll(self.task.circBufferForce,-(indexLong[1]))
        self.task.animalNotStruggledInNSeconds = all(circBufferForceRolled[-self.forceCheckWindow:] < self.usrPrms.forceThreshold)
        # Figure out whether servo has not moved for N seconds (absolute value of servo signal diff)
        servoSlopeCurrent = abs(numpy.polyfit(numpy.r_[0: self.task.DAQBufferSize],self.task.circBufferServo[indexLong[0]:indexLong[1]],1))*1000
        self.task.circBufferServoSlope[self.circBufIndex] = servoSlopeCurrent[0]
        circBufferServoSlopeRolled = numpy.roll(self.task.circBufferServoSlope,-(self.circBufIndex))
        timeLag = ((time()-self.timer)*1000 > self.task.DAQBufferEpoch*3) # Need to wait a couple of updates before servo signal derivative changes
        self.task.servoStationary = all(circBufferServoSlopeRolled[-self.servoCheckWindow:] < 4.5) and timeLag
        # Set the trigger variables for the finite state machine
        self.task.vbafsm.CMDdoneStruggling = self.task.servoStationary and self.task.animalNotStruggledInNSeconds                                      # CONDITIONS: (1) Servo hasn't moved for N seconds; (2) Animal hasn't crossed force threshold for N seconds
        self.task.vbafsm.CMDanimalEscaped = not self.task.belowPositionThreshold                                                                       # CONDITIONS: (1) Laser says animal moved
        self.task.vbafsm.CMDanimalReady = self.task.servoStationary and self.task.animalNotMovedInNSeconds and self.task.animalNotStruggledInNSeconds  # CONDITIONS: (1) Servo hasn't moved for N seconds; (2) Animal hasn't crossed laser threshold for N seconds; (3) Animal hasn't moved more than Laser SD theta for N seconds; (4) Animal hasn't crossed force threshold for N seconds
        ''' # FSM AUTOCYCLE
        self.task.vbafsm.CMDdoneStruggling =  numpy.random.random_sample() > 0.9
        self.task.vbafsm.CMDanimalEscaped = numpy.random.random_sample() > 0.9
        self.task.vbafsm.CMDanimalReady = numpy.random.random_sample() > 0.9 '''

        # Compute difference between last scaled laser value and last scaled servo reading value for calibration
        self.laserServoDelta = self.task.circBufferLaser[indexLong[0]] - numpy.mean(self.task.scaleServo.actual[1] - self.task.scaleServo.sig2act(self.task.aiDataForSigProc[2,:]))

        # Plot curves in GUI
        if self.task.totNumBuffers % self.plotEveryNBuffers == 0:
            self.drawTraceSignal.emit()
        # Update position of circular buffer for next DAQ read
        self.circBufIndex = self.circBufIndex + 1

        # Make sure signal processing isn't taking too long relative to acquisition buffer length
        execTiming = (time()-execTimingStart) * 1000
        if (self.task.DAQBufferEpoch/2) < execTiming:
            print 'Warning: sigProcess is taking', execTiming, 'ms ; DAQBufferEpoch is set to', self.task.DAQBufferEpoch, 'ms', strftime("%Y-%m-%d %H:%M:%S")

    def plotTraces(self):

        ## TOP PLOT
        # Generate and plot force curve
        self.curveAI = Qwt.QwtPlotCurve()
        self.curveAI.attach(self.ui.analogInPlot)
        penColor = Qt.Qt.black
        penSize = 1
        self.curveAI.setPen(Qt.QPen(penColor, penSize))
        self.curveAI.setData(self.task.circBufferTimeLong,self.task.circBufferAnalogIn)

        ## MIDDLE PLOT
        # Generate and plot laser threshold curve
        self.curveLaserThreshold = Qwt.QwtPlotCurve()
        self.curveLaserThreshold.attach(self.ui.positionPlot)
        self.curveLaserThreshold.setPen(Qt.QPen(Qt.Qt.darkCyan, 1))
        self.curveLaserThreshold.setData(self.task.circBufferTimeShort,self.task.circBufferLaserThreshold)
        # Generate and plot laser position curve
        self.curveLaser = Qwt.QwtPlotCurve()
        self.curveLaser.attach(self.ui.positionPlot)
        if self.task.animalNotMovedInNSeconds: penColor = Qt.Qt.blue;     penSize = 2
        else:                                  penColor = Qt.Qt.darkBlue; penSize = 1
        self.curveLaser.setPen(Qt.QPen(penColor, penSize))
        self.curveLaser.setData(self.task.circBufferTimeLong,self.task.circBufferLaser)
        # Generate and plot servo position curve
        self.curveServo = Qwt.QwtPlotCurve()
        self.curveServo.attach(self.ui.positionPlot)
        if self.task.servoStationary: penColor = Qt.Qt.red;         penSize = 2
        else:                         penColor = Qt.Qt.darkMagenta; penSize = 1
        self.curveServo.setPen(Qt.QPen(penColor, penSize))
        self.curveServo.setData(self.task.circBufferTimeLong,self.task.circBufferServo)
        
        ## BOTTOM PLOT
        # Generate and plot force threshold curve
        self.curveForceThreshold = Qwt.QwtPlotCurve()
        self.curveForceThreshold.attach(self.ui.forcePlot)
        self.curveForceThreshold.setPen(Qt.QPen(Qt.Qt.darkCyan, 1))
        self.curveForceThreshold.setData(self.task.circBufferTimeShort,self.task.circBufferForceThreshold)
        # Generate and plot force curve
        self.curveForce = Qwt.QwtPlotCurve()
        self.curveForce.attach(self.ui.forcePlot)
        if self.task.animalNotStruggledInNSeconds: penColor = Qt.Qt.black;    penSize = 2
        else:                                      penColor = Qt.Qt.darkGray; penSize = 1
        self.curveForce.setPen(Qt.QPen(penColor, penSize))
        self.curveForce.setData(self.task.circBufferTimeLong,self.task.circBufferForce)

        # Update plot windows
        self.ui.analogInPlot.replot()
        self.curveAI.detach()
        self.ui.positionPlot.replot()
        self.curveLaser.detach()
        self.curveServo.detach()
        self.curveLaserThreshold.detach()
        self.ui.forcePlot.replot()
        self.curveForce.detach()
        self.curveForceThreshold.detach()

        # Update STD text
        self.ui.laserSDtext_label.setText("{:.2f}".format(self.rescentSTD))
        if self.nojiggle:
            self.ui.laserSDtext_label.setStyleSheet('color: black')
        else:
            self.ui.laserSDtext_label.setStyleSheet('color: red; font-weight: bold')

        # Update trigger state text
        if self.outputTriggerHigh > 0:
            self.ui.outTrigTextLabel1.setText(' X ')
            self.ui.outTrigTextLabel1.setStyleSheet('color: red; background-color: grey')
            self.ui.outTrigTextLabel2.setText(' X ')
            self.ui.outTrigTextLabel2.setStyleSheet('color: red; background-color: grey')
        else:
            self.ui.outTrigTextLabel1.setText('')
            self.ui.outTrigTextLabel2.setText('')

    def updateDIstate(self):
        if self.task.vbafsm.CMDolfactometerSaysPull[1] and not self.controlMode_manual:
            self.ui.startSig.setText('STIMULUS SAYS PULL')
        elif self.task.vbafsm.CMDolfactometerSaysPull[1] and self.controlMode_manual:
            self.ui.startSig.setText('STIM: PULL BUT MANUAL')
        elif not self.task.vbafsm.CMDolfactometerSaysPull[1]:
            self.ui.startSig.setText('_____________________')

    def updateState(self):
        if self.controlMode_manual: # Set servo position  manually
            if self.servoState_slackening:
                self.task.updateServoPosition(self.ui.slackPositionDoubleSpinBox.value())
            if not self.servoState_slackening:
                self.task.updateServoPosition(self.ui.pullPositionDoubleSpinBox.value())
        else:
            if self.task.VBAFSMstate[1] == 'waiting4start':
                self.ui.tubeStateTextLabel.setText('WAITING FOR START SIGNAL')
                self.ui.tubeStateTextLabel.setStyleSheet('color: black')
                self.timer = time()
                self.task.updateServoPosition(self.ui.slackPositionDoubleSpinBox.value()) # Set servo position to slack
            elif self.task.VBAFSMstate[1] == 'pullingBack':
                self.ui.tubeStateTextLabel.setText('PULLING BACK')
                self.ui.tubeStateTextLabel.setStyleSheet('color: darkCyan')
                self.timer = time()
                self.task.updateServoPosition(self.ui.pullPositionDoubleSpinBox.value()) # Set servo position to pull
            elif self.task.VBAFSMstate[1] == 'slackening':
                self.ui.tubeStateTextLabel.setText('SLACKENING')
                self.ui.tubeStateTextLabel.setStyleSheet('color: darkGreen')
                self.timer = time()
                self.task.updateServoPosition(self.ui.slackPositionDoubleSpinBox.value()) # Set servo position to slack
            elif self.task.VBAFSMstate[1] == 'ready':
                self.ui.tubeStateTextLabel.setText('READY TO LAUNCH')
                self.ui.tubeStateTextLabel.setStyleSheet('color: darkMagenta;')
            elif self.task.VBAFSMstate[1] == 'launching':
                self.ui.tubeStateTextLabel.setText('!!! LAUNCHING !!!')
                self.ui.tubeStateTextLabel.setStyleSheet('color: red')
            else:
                print 'WARNING: Not in manual mode and also not in one of the FSM states'

    def shutDown(self):
        try:
            self.task
        except:
            pass
        else:
            self.task.updateServoPosition(self.ui.slackPositionDoubleSpinBox.value()) # Set servo position to slack
            self.task.vbafsm.CMDhaltLoopSignal = 1
            self.paramLoad.saveUserParams(self.usrPrms)
            self.task.updateOutputTrigger(0)
            self.task.clearDAQ()

    def laserThresholdDoubleSpinBox_changed(self):
        self.usrPrms.laserThreshold = self.ui.laserThresholdDoubleSpinBox.value()
    def laserWaitTimeDoubleSpinBox_changed(self):
        self.usrPrms.movementWait = self.ui.laserWaitTimeDoubleSpinBox.value()
        self.laserCheckWindow = int(self.usrPrms.movementWait * self.task.airt)
    def servoWaitTimeDoubleSpinBox_changed(self):
        self.usrPrms.servoWait = self.ui.servoWaitTimeDoubleSpinBox.value()
        self.servoCheckWindow = int(self.usrPrms.servoWait * (self.task.airt/self.task.DAQBufferEpoch))
    def forceThresholdDoubleSpinBox_changed(self):
        self.usrPrms.forceThreshold = self.ui.forceThresholdDoubleSpinBox.value()
    def forceTimeBeforeSlackDoubleSpinBox_changed(self):
        self.usrPrms.struggleWait = self.ui.forceTimeBeforeSlackDoubleSpinBox.value()
        self.forceCheckWindow = self.usrPrms.struggleWait * self.task.airt
    def pullPositionDoubleSpinBox_changed(self):
        self.usrPrms.pullPosition = self.ui.pullPositionDoubleSpinBox.value()
        self.updateState()
    def slackPositionDoubleSpinBox_changed(self):
        self.usrPrms.slackPosition = self.ui.slackPositionDoubleSpinBox.value()
        self.updateState()
    def aiRangeMaxSpinBox_changed(self):
        self.usrPrms.aiRangeMax = self.ui.aiRangeMaxSpinBox.value()
        self.ui.analogInPlot.setAxisScale(Qwt.QwtPlot.yLeft,self.usrPrms.aiRangeMin,self.usrPrms.aiRangeMax)
    def aiRangeMinSpinBox_changed(self):
        self.usrPrms.aiRangeMin = self.ui.aiRangeMinSpinBox.value()
        self.ui.analogInPlot.setAxisScale(Qwt.QwtPlot.yLeft,self.usrPrms.aiRangeMin,self.usrPrms.aiRangeMax)
    def calibratePushButton_clicked(self):
        self.usrPrms.laserServoCalib = self.laserServoDelta
        print 'Calibrating laser/servo offset:', self.usrPrms.laserServoCalib
    def manualRadioButton_toggled(self):
        self.task.vbafsm.CMDrestartSignal = 1
        if self.ui.manualRadioButton.isChecked():
            self.controlMode_manual = 1
            self.servoState_slackening = 1
            self.updateDIstate()
            self.ui.slackPositionRadioButton.setEnabled(True)
            self.ui.pullPositionRadioButton.setEnabled(True)
            if not self.ui.slackPositionRadioButton.isChecked():
                self.ui.slackPositionRadioButton.toggle()
            self.ui.tubeStateTextLabel.setText('MANUAL CONTROL')
            self.ui.tubeStateTextLabel.setStyleSheet('color: black')
            self.task.updateOutputTrigger(0)
        else:
            self.controlMode_manual = 0
            self.updateDIstate()
            self.ui.slackPositionRadioButton.setEnabled(False)
            self.ui.pullPositionRadioButton.setEnabled(False)
            self.ui.tubeStateTextLabel.setText('AUTOMATIC')
            self.ui.tubeStateTextLabel.setStyleSheet('color: black')
    def slackPositionRadioButton_toggled(self):
        if self.ui.slackPositionRadioButton.isChecked(): self.servoState_slackening = 1
        else: self.servoState_slackening = 0
        self.updateState()
    def laserSDThreshold_changed(self):
        self.usrPrms.laserSDThreshold = self.ui.laserSDThresholdDoubleSpinBox.value()



""""""""""""""""""""""""""""""""""""""""""""""""""""""
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = Main()
    window.show()
    sys.exit(app.exec_())
