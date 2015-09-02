import os
import sys
import vtkAll as vtk
from ddapp import botpy
import math
import time
import types
import functools
import numpy as np

from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp.timercallback import TimerCallback
from ddapp.asynctaskqueue import AsyncTaskQueue
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import applogic as app
from ddapp.debugVis import DebugData
from ddapp import ikplanner
from ddapp.ikparameters import IkParameters
from ddapp import ioUtils
from ddapp.simpletimer import SimpleTimer
from ddapp.utime import getUtime
from ddapp import affordanceitems
from ddapp import robotstate
from ddapp import robotplanlistener
from ddapp import segmentation
from ddapp import planplayback
from ddapp import affordanceupdater
from ddapp import segmentationpanel
from ddapp import vtkNumpy as vnp
import ddapp.applogic as app

from ddapp.tasks.taskuserpanel import TaskUserPanel
from ddapp.tasks.taskuserpanel import ImageBasedAffordanceFit

import ddapp.tasks.robottasks as rt
import ddapp.tasks.taskmanagerwidget as tmw

import drc as lcmdrc
import copy
import time
from PythonQt import QtCore, QtGui


class AnklePDPlanner(object):
    def __init__(self, robotSystem):
        self.robotSystem = robotSystem
        self.footstepsDriver = robotSystem.footstepsDriver

        self.robotModel = robotSystem.robotStateModel
        self.ikPlanner = robotSystem.ikPlanner


    def planWalking(self, type="roll right"):
        startPose = self.getPlanningStartPose()
        stanceFrame = self.getStanceFrame();
        goalFrame = transformUtils.copyFrame(stanceFrame)
        goalFrame.PreMultiply()
        goalFrame.Translate(1,0,0)


        request = self.footstepsDriver.constructFootstepPlanRequest(startPose, goalFrame)
        plan = self.footstepsDriver.sendFootstepPlanRequest(request, waitForResponse=True)

        # time.sleep(2)

        # print 'last footstep plan is'
        # print self.footstepsDriver.lastFootstepPlan

        # self.modifyFootstepPlan(type=type)

    def modifyFootstepPlan(self, type=None, ndx=2, degrees=None):
        if type is None:
            return

        # frameName = "step " + str(ndx) + " frame"
        # print frameName
        # frame = om.findObjectByName(frameName)
        plan = self.footstepsDriver.lastFootstepPlan
        frame = transformUtils.frameFromPositionMessage(plan.footsteps[ndx+2].pos)       

        if type == 'roll right':
            frame.PreMultiply()
            frame.RotateX(10)
        if type == "roll left":
            frame.PreMultiply()
            frame.RotateX(-10)
        if type == "pitch up":
            frame.PreMultiply()
            frame.Translate(0,0,-0.015)
            frame.RotateY(-15)
        if type == "pitch down":
            frame.PreMultiply()
            frame.Translate(0,0,0.015)
            frame.RotateY(15)

        self.footstepsDriver.onStepModified(ndx,frame, frame=frame)

    def getPlanningStartPose(self):
        return self.robotSystem.robotStateJointController.q

    def getStanceFrame(self):
        return self.robotSystem.footstepsDriver.getFeetMidPoint(self.robotSystem.robotStateModel, useWorldZ=False)

