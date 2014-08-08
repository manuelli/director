import ddapp.applogic as app

from ddapp import lcmUtils
from ddapp import transformUtils
from ddapp import visualization as vis
from ddapp import filterUtils
from ddapp.shallowCopy import shallowCopy
from ddapp.timercallback import TimerCallback
from ddapp import vtkNumpy
import ddapp.vtkAll as vtk
from ddapp.debugVis import DebugData

import PythonQt
from PythonQt import QtCore, QtGui
import bot_core as lcmbotcore
import numpy as np
from ddapp.simpletimer import SimpleTimer
from ddapp import ioUtils
import sys
import drc as lcmdrc
import multisense as lcmmultisense


def clipRange(dataObj, arrayName, thresholdRange):
    if not dataObj.GetPointData().GetArray(arrayName):
        raise Exception('clipRange: could not locate array: %s' % arrayName)

    dataObj.GetPointData().SetScalars(dataObj.GetPointData().GetArray(arrayName))

    f = vtk.vtkClipPolyData()
    f.SetInput(dataObj)
    f.SetValue(thresholdRange[0])
    f.SetInputArrayToProcess(0, 0, 0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, arrayName)

    f2 = vtk.vtkClipPolyData()
    f2.AddInputConnection(f.GetOutputPort())
    f2.SetValue(thresholdRange[1])
    f2.InsideOutOn()
    f2.SetInputArrayToProcess(0, 0, 0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, arrayName)

    f2.Update()
    return shallowCopy(f2.GetOutput())



def makeSphere(radius, resolution):

    s = vtk.vtkSphereSource()
    s.SetThetaResolution(resolution)
    s.SetPhiResolution(resolution)
    s.SetRadius(radius)
    s.SetEndPhi(85)
    s.Update()
    return shallowCopy(s.GetOutput())


def colorizePoints(polyData, cameraName='CAMERA_LEFT'):
    imageManager.queue.colorizePoints(cameraName, polyData)



def sendFOVRequest(channel, imagePoints):

    channelToImageType = {
        'CAMERA_LEFT' : lcmdrc.data_request_t.CAMERA_IMAGE_HEAD_LEFT,
        'CAMERACHEST_LEFT' : lcmdrc.data_request_t.CAMERA_IMAGE_LCHEST,
        'CAMERACHEST_RIGHT' : lcmdrc.data_request_t.CAMERA_IMAGE_RCHEST,
                         }

    dataRequest = lcmdrc.data_request_t()
    dataRequest.type = channelToImageType[channel]

    message = lcmdrc.subimage_request_t()
    message.data_request = dataRequest

    imagePoints = np.array([[pt[0], pt[1]] for pt in imagePoints])
    minX, maxX = imagePoints[:,0].min(), imagePoints[:,0].max()
    minY, maxY = imagePoints[:,1].min(), imagePoints[:,1].max()

    message.x = minX
    message.y = minY
    message.w = maxX - minX
    message.h = maxY - minY

    #print message.x, message.y, message.w, message.h

    requestChannel = 'SUBIMAGE_REQUEST'
    lcmUtils.publish(requestChannel, message)


def testColorize():

    radius = 10
    resolution = 400
    s = makeSphere(radius, resolution)
    cameraView.queue.colorizePoints(s)
    showPolyData(p, 'sphere', colorByName='rgb')


def rayDebug(position, ray):
    d = DebugData()
    d.addLine(position, position+ray*5.0)
    drcView = app.getViewManager().findView('DRC View')
    obj = vis.updatePolyData(d.getPolyData(), 'camera ray', view=drcView, color=[0,1,0])
    obj.actor.GetProperty().SetLineWidth(2)


class ImageManager(object):

    def __init__(self):

        self.images = {}
        self.imageUtimes = {}
        self.textures = {}

        self.queue = PythonQt.dd.ddBotImageQueue(lcmUtils.getGlobalLCMThread())
        self.queue.init(lcmUtils.getGlobalLCMThread())

    def addImage(self, name):

        if name in self.images:
            return

        image = vtk.vtkImageData()
        tex = vtk.vtkTexture()
        tex.SetInput(image)
        tex.EdgeClampOn()
        tex.RepeatOff()

        self.imageUtimes[name] = 0
        self.images[name] = image
        self.textures[name] = tex

    def writeImage(self, imageName, outFile):
        writer = vtk.vtkPNGWriter()
        writer.SetInput(self.images[imageName])
        writer.SetFileName(outFile)
        writer.Write()

    def updateImage(self, imageName):
        imageUtime = self.queue.getCurrentImageTime(imageName)
        if imageUtime != self.imageUtimes[imageName]:
            image = self.images[imageName]
            self.imageUtimes[imageName] = self.queue.getImage(imageName, image)
        return imageUtime

    def updateImages(self):
        for imageName in self.images.keys():
            self.updateImage(imageName)

    def hasImage(self, imageName):
        return imageName in self.images

    def getImage(self, imageName):
        return self.images[imageName]

    def getUtime(self, imageName):
        return self.imageUtimes[imageName]

    def getTexture(self, imageName):
        return self.textures[imageName]


class CameraView(object):

    def __init__(self, imageManager, view=None):

        self.imageManager = imageManager
        self.updateUtimes = {}
        self.sphereObjects = {}
        self.sphereImages = [
                'CAMERA_LEFT',
                'CAMERACHEST_LEFT',
                'CAMERACHEST_RIGHT']

        for name in self.sphereImages:
            imageManager.addImage(name)
            self.updateUtimes[name] = 0

        self.initView(view)
        self.initEventFilter()
        self.rayCallback = rayDebug

        self.timerCallback = TimerCallback()
        self.timerCallback.targetFps = 60
        self.timerCallback.callback = self.updateView
        self.timerCallback.start()

    def onViewDoubleClicked(self, displayPoint):

        obj, pickedPoint = vis.findPickedObject(displayPoint, self.view)

        if pickedPoint is None or not obj:
            return

        imageName = obj.getProperty('Name')
        imageUtime = self.imageManager.getUtime(imageName)

        cameraToLocal = vtk.vtkTransform()
        self.imageManager.queue.getTransform(imageName, 'local', imageUtime, cameraToLocal)

        utorsoToLocal = vtk.vtkTransform()
        self.imageManager.queue.getTransform('utorso', 'local', imageUtime, utorsoToLocal)

        p = range(3)
        utorsoToLocal.TransformPoint(pickedPoint, p)

        ray = np.array(p) - np.array(cameraToLocal.GetPosition())
        ray /= np.linalg.norm(ray)

        if self.rayCallback:
            self.rayCallback(np.array(cameraToLocal.GetPosition()), ray)

    def filterEvent(self, obj, event):
        if event.type() == QtCore.QEvent.MouseButtonDblClick:
            self.eventFilter.setEventHandlerResult(True)
            self.onViewDoubleClicked(vis.mapMousePosition(obj, event))
        elif event.type() == QtCore.QEvent.KeyPress:
            if str(event.text()).lower() == 'p':
                self.eventFilter.setEventHandlerResult(True)
            elif str(event.text()).lower() == 'r':
                self.eventFilter.setEventHandlerResult(True)
                self.resetCamera()

    def initEventFilter(self):
        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        qvtkwidget = self.view.vtkWidget()
        qvtkwidget.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyPress)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)

    def initView(self, view):

        self.view = view or app.getViewManager().createView('Camera View', 'VTK View')
        app.setCameraTerrainModeEnabled(self.view, True)
        self.resetCamera()

    def resetCamera(self):
        self.view.camera().SetViewAngle(90)
        self.view.camera().SetPosition(-7.5, 0.0, 5.0)
        self.view.camera().SetFocalPoint(0.0, 0.0, 0.0)
        self.view.camera().SetViewUp(0.0, 0.0, 1.0)
        self.view.render()

    def getSphereGeometry(self, imageName):

        sphereObj = self.sphereObjects.get(imageName)
        if sphereObj:
            return sphereObj

        if not self.imageManager.getImage(imageName).GetDimensions()[0]:
            return None

        sphereResolution = 50
        sphereRadii = {
                'CAMERA_LEFT' : 9.85,
                'CAMERACHEST_LEFT' : 9.95,
                'CAMERACHEST_RIGHT' : 10
                }

        geometry = makeSphere(sphereRadii[imageName], sphereResolution)
        self.imageManager.queue.computeTextureCoords(imageName, geometry)

        tcoordsArrayName = 'tcoords_%s' % imageName
        vtkNumpy.addNumpyToVtk(geometry, vtkNumpy.getNumpyFromVtk(geometry, tcoordsArrayName)[:,0].copy(), 'tcoords_U')
        vtkNumpy.addNumpyToVtk(geometry, vtkNumpy.getNumpyFromVtk(geometry, tcoordsArrayName)[:,1].copy(), 'tcoords_V')
        geometry = clipRange(geometry, 'tcoords_U', [0.0, 1.0])
        geometry = clipRange(geometry, 'tcoords_V', [0.0, 1.0])
        geometry.GetPointData().SetTCoords(geometry.GetPointData().GetArray(tcoordsArrayName))

        sphereObj = vis.showPolyData(geometry, imageName, view=self.view, parent='cameras')
        sphereObj.actor.SetTexture(self.imageManager.getTexture(imageName))
        sphereObj.actor.GetProperty().LightingOff()

        self.sphereObjects[imageName] = sphereObj
        return sphereObj

    def updateSphereGeometry(self):

        for imageName in self.sphereImages:
            sphereObj = self.getSphereGeometry(imageName)
            if not sphereObj:
                continue

            transform = vtk.vtkTransform()
            self.imageManager.queue.getBodyToCameraTransform(imageName, transform)
            sphereObj.actor.SetUserTransform(transform.GetLinearInverse())

    def updateImages(self):

        updated = False
        for imageName, lastUtime in self.updateUtimes.iteritems():
            currentUtime = self.imageManager.updateImage(imageName)
            if currentUtime != lastUtime:
                self.updateUtimes[imageName] = currentUtime
                updated = True

        return updated

    def updateView(self):

        if not self.view.isVisible():
            return

        if not self.updateImages():
            return

        self.updateSphereGeometry()
        self.view.render()


class ImageWidget(object):

    def __init__(self, imageManager, imageName, view):
        self.view = view
        self.imageManager = imageManager
        self.imageName = imageName

        self.updateUtime = 0
        self.initialized = False

        self.imageWidget = vtk.vtkLogoWidget()
        rep = self.imageWidget.GetRepresentation()
        rep.GetImageProperty().SetOpacity(1.0)
        self.imageWidget.SetInteractor(self.view.renderWindow().GetInteractor())

        self.flip = vtk.vtkImageFlip()
        self.flip.SetFilteredAxis(1)
        self.flip.SetInput(imageManager.getImage(imageName))
        rep.SetImage(self.flip.GetOutput())

        self.timerCallback = TimerCallback()
        self.timerCallback.targetFps = 60
        self.timerCallback.callback = self.updateView
        self.timerCallback.start()


    def hide(self):
        self.imageWidget.Off()

    def show(self):
        self.imageWidget.On()

    def updateView(self):

        if not self.view.isVisible():
            return

        currentUtime = self.imageManager.updateImage(self.imageName)
        if currentUtime != self.updateUtime:
            if not self.initialized:
                self.show()
                self.initialized = True

            self.updateUtime = currentUtime
            self.flip.Update()
            self.view.render()


class CameraImageView(object):

    def __init__(self, imageManager, imageName, viewName=None, view=None):

        imageManager.addImage(imageName)

        self.imageManager = imageManager
        self.viewName = viewName or imageName
        self.imageName = imageName
        self.imageInitialized = False
        self.updateUtime = 0
        self.rayCallback = rayDebug
        self.initView(view)
        self.initEventFilter()


    def onViewDoubleClicked(self, displayPoint):
        dataset, pickedPoint = vis.pickImage(displayPoint, self.view)
        if pickedPoint is None or not dataset:
            return


    def getImagePixel(self, displayPoint):

        worldPoint = [0.0, 0.0, 0.0, 0.0]
        vtk.vtkInteractorObserver.ComputeDisplayToWorld(self.view.renderer(), displayPoint[0], displayPoint[1], 0, worldPoint)

        imageDimensions = self.getImage().GetDimensions()

        if 0.0 <= worldPoint[0] <= imageDimensions[0] and 0.0 <= worldPoint[1] <= imageDimensions[1]:
            return [worldPoint[0], worldPoint[1], 0.0]
        else:
            return None


    def rayDebug(self, displayPoint):

        pickedPoint = self.getImagePixel(displayPoint)
        imageUtime = self.imageManager.getUtime(self.imageName)

        position,ray = self.getPositionAndRay(pickedPoint, imageUtime)


        if self.rayCallback:
            self.rayCallback(position, ray)


    def getPositionAndRay(self, pickedPoint, imageUtime):
        # input is pixel u,v, output is unit x,y,z in camera coordinates
        pickedPoint = self.imageManager.queue.unprojectPixel(self.imageName, pickedPoint[0], pickedPoint[1])

        cameraToLocal = vtk.vtkTransform()
        self.imageManager.queue.getTransform(self.imageName, 'local', imageUtime, cameraToLocal)

        p = range(3)
        cameraToLocal.TransformPoint(pickedPoint, p)
        ray = np.array(p) - np.array(cameraToLocal.GetPosition())
        ray /= np.linalg.norm(ray)

        position = np.array(cameraToLocal.GetPosition())

        return position, ray


    def filterEvent(self, obj, event):
        if self.eventFilterEnabled and event.type() == QtCore.QEvent.MouseButtonDblClick:
            self.eventFilter.setEventHandlerResult(True)
            self.onViewDoubleClicked(vis.mapMousePosition(obj, event))

        elif event.type() == QtCore.QEvent.KeyPress:
            if str(event.text()).lower() == 'p':
                self.eventFilter.setEventHandlerResult(True)
            elif str(event.text()).lower() == 'r':
                self.eventFilter.setEventHandlerResult(True)
                self.resetCamera()

    def onRubberBandPick(self, obj, event):
        displayPoints = self.interactorStyle.GetStartPosition(), self.interactorStyle.GetEndPosition()
        imagePoints = [vis.pickImage(point, self.view)[1] for point in displayPoints]
        sendFOVRequest(self.imageName, imagePoints)

    def getImage(self):
        return self.imageManager.getImage(self.imageName)

    def initView(self, view):
        self.view = view or app.getViewManager().createView(self.viewName, 'VTK View')
        self.view.installImageInteractor()
        self.interactorStyle = self.view.renderWindow().GetInteractor().GetInteractorStyle()
        self.interactorStyle.AddObserver('SelectionChangedEvent', self.onRubberBandPick)

        self.imageActor = vtk.vtkImageActor()
        self.imageActor.SetInput(self.getImage())
        self.imageActor.SetVisibility(False)
        self.view.renderer().AddActor(self.imageActor)

        self.timerCallback = TimerCallback()
        self.timerCallback.targetFps = 60
        self.timerCallback.callback = self.updateView
        self.timerCallback.start()

    def initEventFilter(self):
        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        qvtkwidget = self.view.vtkWidget()
        qvtkwidget.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyPress)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)
        self.eventFilterEnabled = True

    def resetCamera(self):
        camera = self.view.camera()
        camera.ParallelProjectionOn()
        camera.SetFocalPoint(0,0,0)
        camera.SetPosition(0,0,-1)
        camera.SetViewUp(0,-1, 0)
        self.view.resetCamera()
        self.fitImageToView()
        self.view.render()

    def fitImageToView(self):

        camera = self.view.camera()
        image = self.getImage()
        imageWidth, imageHeight, _ = image.GetDimensions()

        viewWidth, viewHeight = self.view.renderWindow().GetSize()
        aspectRatio = float(viewWidth)/viewHeight
        parallelScale = max(imageWidth/aspectRatio, imageHeight) / 2.0
        camera.SetParallelScale(parallelScale)

    def setImageName(self, imageName):
        if imageName == self.imageName:
            return

        assert self.imageManager.hasImage(imageName)

        self.imageName = imageName
        self.imageInitialized = False
        self.updateUtime = 0
        self.imageActor.SetInput(self.imageManager.getImage(self.imageName))
        self.imageActor.SetVisibility(False)
        self.view.render()

    def updateView(self):

        if not self.view.isVisible():
            return

        currentUtime = self.imageManager.updateImage(self.imageName)
        if currentUtime != self.updateUtime:
            self.updateUtime = currentUtime
            self.view.render()

            if not self.imageInitialized and self.imageActor.GetInput().GetDimensions()[0]:
                self.imageActor.SetVisibility(True)
                self.resetCamera()
                self.imageInitialized = True

views = {}


def addCameraView(channel, viewName=None, cameraName=None, imageType=-1):

    cameraName = cameraName or channel
    imageManager.queue.addCameraStream(channel, cameraName, imageType)
    imageManager.addImage(cameraName)
    view = CameraImageView(imageManager, cameraName, viewName)
    global views
    views[channel] = view
    return view


def getStereoPointCloud(decimation=4):

    q = imageManager.queue
    imagesChannel = 'CAMERA'
    cameraName = 'CAMERA_LEFT'

    utime = q.getCurrentImageTime(cameraName)
    if utime == 0:
        return None

    p = vtk.vtkPolyData()
    cameraToLocal = vtk.vtkTransform()

    q.getPointCloudFromImages(imagesChannel, p, decimation)
    q.getTransform(cameraName, 'local', utime, cameraToLocal)
    p = filterUtils.transformPolyData(p, cameraToLocal)

    return p


def init():

    global imageManager
    imageManager = ImageManager()

    global cameraView
    cameraView = CameraView(imageManager)

    addCameraView('CAMERA_LEFT', 'Head camera')
    #addCameraView('CAMERA', 'Head camera right', 'CAMERA_RIGHT', lcmmultisense.images_t.RIGHT)
    #addCameraView('CAMERA', 'Head camera depth', 'CAMERA_DISPARITY', lcmmultisense.images_t.DISPARITY_ZIPPED)
    addCameraView('CAMERACHEST_LEFT', 'Chest left')
    addCameraView('CAMERACHEST_RIGHT', 'Chest right')

