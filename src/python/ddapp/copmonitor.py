import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
import ddapp.objectmodel as om
from ddapp import lcmUtils
from ddapp import applogic as app
from ddapp.utime import getUtime
from ddapp import vtkNumpy as vnp
from ddapp import visualization as vis
from ddapp.debugVis import DebugData
from scipy.spatial import Delaunay
import drc as lcmdrc
import numpy as np
import math
from time import time
from time import sleep

class COPMonitor(object):
    LONG_FOOT_CONTACT_POINTS = [[0.17, 0.0562, 0.0],
                                [0.17, -0.0562, 0.0],
                                [-0.13, -0.0562, 0.0],
                                [-0.13, 0.0562, 0.0]]
    DESIRED_INTERIOR_DISTANCE = 0.05
    printDebugData = True

    def __init__(self, robotSystem, view):

        self.robotStateModel = robotSystem.robotStateModel
        self.robotStateJointController = robotSystem.robotStateJointController
        self.robotSystem = robotSystem
        self.lFootFtFrameId = self.robotStateModel.model.findLinkID( self.robotSystem.ikPlanner.leftFootLink)
        self.rFootFtFrameId = self.robotStateModel.model.findLinkID( self.robotSystem.ikPlanner.rightFootLink)
        self.leftInContact = 0
        self.rightInContact = 0
        self.view = view
        self.ddDrakeWrapper = PythonQt.dd.ddDrakeWrapper()

        self.warningButton = QtGui.QLabel('COP Warning')
        self.warningButton.setStyleSheet("background-color:white")
        self.dialogVisible = False

        d = DebugData()
        vis.updatePolyData(d.getPolyData(), 'measured cop', view=self.view, parent='robot state model')
        om.findObjectByName('measured cop').setProperty('Visible', False)

        vis.updatePolyData(d.getPolyData(), 'measured cop right', view=self.view, parent='robot state model')
        om.findObjectByName('measured cop right').setProperty('Visible', True)

        vis.updatePolyData(d.getPolyData(), 'measured cop left', view=self.view, parent='robot state model')
        om.findObjectByName('measured cop left').setProperty('Visible', True)

        
        self.robotStateModel.connectModelChanged(self.update)

    def update(self, unused):
        if ((om.findObjectByName('measured cop').getProperty('Visible') or om.findObjectByName('measured cop right').getProperty('Visible') or om.findObjectByName('measured cop left').getProperty('Visible')) and self.robotStateJointController.lastRobotStateMessage):

            if self.dialogVisible == False:
                self.warningButton.setVisible(True)
                app.getMainWindow().statusBar().insertPermanentWidget(0, self.warningButton)
                self.dialogVisible = True

            self.leftInContact = self.robotStateJointController.lastRobotStateMessage.force_torque.l_foot_force_z > 200
            self.rightInContact = self.robotStateJointController.lastRobotStateMessage.force_torque.r_foot_force_z > 200

            

            if self.rightInContact or self.leftInContact:

                lFootFt =  [self.robotStateJointController.lastRobotStateMessage.force_torque.l_foot_torque_x, 
                             self.robotStateJointController.lastRobotStateMessage.force_torque.l_foot_torque_y, 
                             0.0,
                             0.0, 
                             0.0, 
                             self.robotStateJointController.lastRobotStateMessage.force_torque.l_foot_force_z]
                rFootFt = [self.robotStateJointController.lastRobotStateMessage.force_torque.r_foot_torque_x, 
                             self.robotStateJointController.lastRobotStateMessage.force_torque.r_foot_torque_y, 
                             0.0,
                             0.0, 
                             0.0, 
                             self.robotStateJointController.lastRobotStateMessage.force_torque.r_foot_force_z]
                lFootTransform = self.robotStateModel.getLinkFrame( self.robotSystem.ikPlanner.leftFootLink )
                rFootTransform = self.robotStateModel.getLinkFrame( self.robotSystem.ikPlanner.rightFootLink)

                rFootOrigin = np.array(rFootTransform.TransformPoint([0, 0, -0.07645]))
                lFootOrigin = np.array(lFootTransform.TransformPoint([0, 0, -0.07645])) # down to sole

                measured_cop = self.ddDrakeWrapper.resolveCenterOfPressure(self.robotStateModel.model, [self.lFootFtFrameId, self.rFootFtFrameId], 
                                lFootFt + rFootFt, [0., 0., 1.], (self.rightInContact*rFootOrigin+self.leftInContact*lFootOrigin)/(self.leftInContact + self.rightInContact))

                measured_cop_right = self.ddDrakeWrapper.resolveCenterOfPressure(self.robotStateModel.model, [self.rFootFtFrameId], rFootFt, [0.0, 0.0, 1.0], rFootOrigin)

                measured_cop_left = self.ddDrakeWrapper.resolveCenterOfPressure(self.robotStateModel.model, [self.lFootFtFrameId], lFootFt, [0.0, 0.0, 1.0], lFootOrigin)



                allFootContacts = np.empty([0, 2])
                if self.rightInContact:
                    rFootContacts = np.array([rFootTransform.TransformPoint(contact_point) for contact_point in self.LONG_FOOT_CONTACT_POINTS])
                    allFootContacts = np.concatenate((allFootContacts, rFootContacts[:, 0:2]))
                if self.leftInContact:
                    lFootContacts = np.array([lFootTransform.TransformPoint(contact_point) for contact_point in self.LONG_FOOT_CONTACT_POINTS])
                    allFootContacts = np.concatenate((allFootContacts, lFootContacts[:, 0:2]))

                num_pts = allFootContacts.shape[0]
                dist = self.ddDrakeWrapper.drakeSignedDistanceInsideConvexHull(num_pts, allFootContacts.reshape(num_pts*2, 1), measured_cop[0:2])

                inSafeSupportPolygon = dist >= 0
                # map dist to color -- green if inside threshold, red if not
                dist = min(max(0, dist), self.DESIRED_INTERIOR_DISTANCE) / self.DESIRED_INTERIOR_DISTANCE
                # nonlinear interpolation here to try to maintain saturation
                r = int(255. - 255. * dist**4 )
                g = int(255. * dist**0.25 )
                b = 0
                colorStatus = [r/255., g/255., b/255.]
                colorStatusString = 'rgb(%d, %d, %d)' % (r, g, b)
                self.warningButton.setStyleSheet("background-color:"+colorStatusString)

                d = DebugData()
                d.addSphere(measured_cop[0:3], radius=0.02)
                vis.updatePolyData(d.getPolyData(), 'measured cop', view=self.view, parent='robot state model').setProperty('Color', colorStatus)


                # handles the right and left cop monitors
                redColorStatus = [1,0,0]
                greenColorStatus = [0,1,0]
                edgeDetectionThreshold = 0.03

                # print 'left foot points'
                # print lFootContacts
                # print type(lFootContacts)

                def computeColorStatus(dist, threshold):
                    # alpha = min(max(0,dist), threshold)/threshold
                    alpha = 1;
                    if dist < edgeDetectionThreshold:
                        alpha = 0


                    colorStatus = [(1-alpha), alpha, 0]
                    return colorStatus

                worldToRFoot = rFootTransform.GetLinearInverse();
                numpyFootContacts = np.array(self.LONG_FOOT_CONTACT_POINTS)
                numpyFootContacts = numpyFootContacts[:,0:2];
                num_pts = 4
                contacts = numpyFootContacts.reshape(2*num_pts,1)

                if self.rightInContact:
                    num_pts = 4
                    cop_right_foot_frame = worldToRFoot.TransformPoint(measured_cop_right[0:3])
                    dist = self.ddDrakeWrapper.drakeSignedDistanceInsideConvexHull(num_pts,contacts, cop_right_foot_frame[0:2])

                    # print "measure COP in foot frame is "
                    # print cop_right_foot_frame

                    edgeDist = self.ddDrakeWrapper.drakeDistanceToEdges(num_pts, contacts, cop_right_foot_frame[0:2])

                    if self.printDebugData:
                        print 'right foot distance to convex hull'
                        print dist

                        print 'min distance edge left/right'
                        print min(edgeDist[1], edgeDist[3])

                        print "min edge distance front/back"
                        print min(edgeDist[0], edgeDist[2])


                    colorStatus = computeColorStatus(dist, edgeDetectionThreshold)
                    # colorStatus = redColorStatus

                    d = DebugData()
                    d.addSphere(measured_cop_right[0:3], radius=0.02)
                    vis.updatePolyData(d.getPolyData(), 'measured cop right', view=self.view, parent='robot state model').setProperty('Color', colorStatus)
                else:
                    d = DebugData()
                    vis.updatePolyData(d.getPolyData(), 'measured cop right', view=self.view, parent='robot state model')

                if self.leftInContact:
                    num_pts = 4
                    contacts = lFootContacts[:, 0:2]
                    contacts = contacts.reshape(2*num_pts,1)
                    dist = self.ddDrakeWrapper.drakeSignedDistanceInsideConvexHull(num_pts,contacts, measured_cop_left[0:2])

                    if self.printDebugData:
                        print 'left foot distance to foot edge'
                        print dist

                    colorStatus = computeColorStatus(dist, edgeDetectionThreshold)
                    # colorStatus = redColorStatus

                    d = DebugData()
                    d.addSphere(measured_cop_left[0:3], radius=0.02)
                    vis.updatePolyData(d.getPolyData(), 'measured cop left', view=self.view, parent='robot state model').setProperty('Color', colorStatus)
                else:
                    d = DebugData()
                    vis.updatePolyData(d.getPolyData(), 'measured cop left', view=self.view, parent='robot state model')


        elif self.dialogVisible:
            app.getMainWindow().statusBar().removeWidget(self.warningButton)
            self.dialogVisible = False
