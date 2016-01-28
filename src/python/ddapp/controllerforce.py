__author__ = 'manuelli'
import ddapp
import drc as lcmdrc
import vtkAll as vtk
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp import objectmodel as om
from ddapp import lcmUtils
from ddapp.debugVis import DebugData


import numpy as np

class ControllerForce(object):

    def __init__(self, robotSystem):
        self.robotSystem = robotSystem
        self.robotStateModel = robotSystem.robotStateModel
        self.addSubscribers()
        self.forceDict = {}

    def addSubscribers(self):
        lcmUtils.addSubscriber('CONTROLLER_FORCE', lcmdrc.support_element_forces_t, self.onControllerForceMsg)

    def onControllerForceMsg(self, msg):
        self.forceDict = {}

        for singleBodyMsg in msg.support_bodies:
            bodyForceDict = {}
            bodyName = singleBodyMsg.body_name
            bodyForceDict['bodyName'] = bodyName
            bodyForceDict['numContactPts'] = singleBodyMsg.num_contact_pts
            bodyForceDict['pointForces'] = np.array(singleBodyMsg.force) # will be numContactPts x 3
            bodyForceDict['contactPts'] = np.array(singleBodyMsg.contact_pts) # will be numContactPts x 3
            self.forceDict[bodyName] = bodyForceDict

        # self.computeAllBodiesForceTorque()

    def computeSingleBodyForceTorque(self, bodyForceDict):
        # maybe this will just fill in some fields of bodyForceDict with the
        # totalForce and totalTorque acting on the body
        totalWrench = np.zeros(6)
        for i in xrange(0, bodyForceDict['numContactPts']):
            contactPt = bodyForceDict['contactPts'][i,:]
            pointForce = bodyForceDict['pointForces'][i,:]

            outputFrame = vtk.vtkTransform()
            wrenchFrame = vtk.vtkTransform()
            wrenchFrame.Translate(contactPt)
            forceMomentTransform = transformUtils.forceMomentTransformation(wrenchFrame, outputFrame)
            wrench = np.zeros(6)
            wrench[3,:] = pointForce
            wrenchBodyFrame = np.dot(forceMomentTransform, wrench)
            totalWrench += wrenchBodyFrame

        return totalWrench
        pass

    def computeAllBodiesForceTorque(self):
        for bodyForceDict in self.forceDict:
            totalWrench = self.computeSingleBodyForceTorque(bodyForceDict)
            bodyForceDict['totalWrench'] = totalWrench

    def publishForceTorque(self):
        pass

    def drawContactForces(self):
        # should draw all point forces that are appearing in the current self.forceDict
        pass


