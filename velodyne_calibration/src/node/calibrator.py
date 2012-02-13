#!/usr/bin/env python
#
# This node is used to calibrate the velodyne. Uses opencv &
# wx for visualization.
#
# Copyright (C) 2010 UT-Austin & Austin Robot Technology
# License: Modified BSD Software License 

import sys
import os
import threading
import wx

PKG_NAME = 'velodyne_calibrate'
NODE_NAME = 'velodyne_calibrator'
import roslib
roslib.load_manifest(PKG_NAME)

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image, PointCloud
from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

import cv as cv

# Some global values (for now)

imageWidth = 512
imageHeight = 64
imageSize = (imageWidth, imageHeight)
scaleDisplay = 2
displayWidth = imageWidth * scaleDisplay
displayHeight = imageHeight * scaleDisplay
displaySize = (displayWidth, displayHeight)

ID_TOOL_HEIGHT = 1000
ID_TOOL_INTENSITY = ID_TOOL_HEIGHT + 1
ID_TOOL_EXEC = ID_TOOL_INTENSITY + 1

calib_path = os.path.join(roslib.packages.get_pkg_dir(PKG_NAME), "data")
defaultCalibPath = os.path.join(calib_path, "default.cal")
tempCalibPath = os.path.join(calib_path, "tmp.cal")
icon_path = os.path.join(roslib.packages.get_pkg_dir(PKG_NAME), "icons")

class CalibrationHandler:
    "Handles calibration and the interface to velodyne topics and the window. Run in the gui threadi"

    def __init__(self):

        # define some constants, i.e. calibration modes

        self.CALIBRATE_NONE = 0
        self.CALIBRATE_INTENSITY = 1
        self.CALIBRATE_HEIGHT = 2
        self.CALIBRATE_HEIGHT_PITCH = 3

        self.CURRENT = 0
        self.DEFAULT = 1
        self.NONE = 2

        self.mode = self.CALIBRATE_HEIGHT
        self.calibration = self.DEFAULT

        self.reconfigClient = dynamic_reconfigure.client.Client('velodyne_calibrate_cloud')

        # read default calibration information

        self.iCalib = dict()
        self.hCalib = dict()
        self.pitchCalib = 0
        self.samples = dict()

        self.readDefaultCalibration(defaultCalibPath)
        self.writeTempCalibration()
        
    def readDefaultCalibration(self, filePath):

        f = open(filePath)

        # Intensity Calib

        id = 0
        while id < 64:
            self.iCalib[id] = 0, 1, 0, 0
            id = id + 1

        while True:
            id = eval(f.readline())
            a = eval(f.readline())
            b = eval(f.readline())
            c = eval(f.readline())
            d = eval(f.readline())
            if id == -1:
                break
            self.iCalib[id] = a, b, c, d

        # Pitch Calib

        self.pitchCalib = eval(f.readline());

        # Intensity Calib

        id = 0
        while id < 64:
            self.hCalib[id] = 0
            id = id + 1
        while True:
            id = eval(f.readline())
            a = eval(f.readline())
            if id == -1:
                break
            self.hCalib[id] = a

        f.close()

        # Samples

        f2 = open(filePath + '.sam')

        while True:
            line = f.readline()
            if len(line) == 0:
                break
            id = eval(line)
            self.samples[id] = dict()
            arr = f.readline().split(" ")
            numLasers = eval(arr[0])
            numFrames = eval(arr[1])
            countLasers = 0
            countFrames = 0
            while countLasers < numLasers:
                eval(f.readline())

    def updateWindow(self, calibrationWindow):
        self.window = calibrationWindow

    def updateRosHandler(self, rosHandler):
        self.rosHandler = rosHandler

    def Cleanup(self):
        self.rosHandler.Shutdown()

    def UpdateHeightImage(self, image):
        "Update the current image being displayed"
        img = wx.EmptyImage(image.width, image.height)
        img.SetData(image.tostring())
        self.window.heightImage = img.Rescale(displayWidth, displayHeight)
        self.window.Refresh()

    def UpdateIntensityImage(self, image):
        "Update the current image being displayed"
        img = wx.EmptyImage(image.width, image.height)
        img.SetData(image.tostring())
        self.window.intensityImage = img.Rescale(displayWidth, displayHeight)
        self.window.Refresh()

    def UpdatePointCloud(self, pointCloud):
        self.pc = pointCloud

        # TODO while fitting pick samples from here

    def UpdateCalibration(self, calibration):
        self.calibration = calibration
        params = None
        if calibration == self.NONE:
            params = {'calibrateCloud' : False}
        else:
            params = {'calibrateCloud' : True}
        self.reconfigClient.update_configuration(params)
        
    def FitEquations(self):
        print "Python equations use requested" 
        # will have to resend calibration file to image generator using dynamic reconfigure

class CalibrationWindow(wx.Frame):
    "The main calibration window"

    def __init__(self, parent, id, handler):
        "constructor"

        # begin wxGlade: MyFrame.__init__
        wx.Frame.__init__(self, parent, id)
        self.intensityPanel = wx.Panel(self, -1)
        self.heightPanel = wx.Panel(self, -1)

        # Menu Bar
        self.menuBar = wx.MenuBar()
        self.fileMenu = wx.Menu()
        self.menuExit = wx.MenuItem(self.fileMenu, wx.ID_EXIT, "E&xit", "Exit the calibrator", wx.ITEM_NORMAL)
        self.fileMenu.AppendItem(self.menuExit)
        self.menuBar.Append(self.fileMenu, "&File")
        self.SetMenuBar(self.menuBar)
        # Menu Bar end
        self.statusBar = self.CreateStatusBar(1, 0)
        
        # Tool Bar
        self.toolBar = wx.ToolBar(self, -1)
        self.SetToolBar(self.toolBar)
        self.toolBar.AddLabelTool(ID_TOOL_HEIGHT, "Height Calibration", wx.Bitmap("/home/piyushk/art/sandbox/stacks/velodyne_experimental/velodyne_calibrate/icons/exec.png", wx.BITMAP_TYPE_ANY), wx.NullBitmap, wx.ITEM_CHECK, "Calibrate Height", "Calibrates the velodyne data to remove pitch error, as well as individual laser angle defects.")
        self.toolBar.AddLabelTool(ID_TOOL_INTENSITY, "Intensity Calibration", wx.Bitmap("/home/piyushk/art/sandbox/stacks/velodyne_experimental/velodyne_calibrate/icons/exec.png", wx.BITMAP_TYPE_ANY), wx.NullBitmap, wx.ITEM_CHECK, "Calibrate Intensity", "Calibrates the velodyne data to remove intensity differences between different lasers.")
        self.toolBar.AddSeparator()
        self.toolBar.AddLabelTool(wx.ID_OPEN, "Open Calibration", wx.Bitmap("/home/piyushk/art/sandbox/stacks/velodyne_experimental/velodyne_calibrate/icons/open.png", wx.BITMAP_TYPE_ANY), wx.NullBitmap, wx.ITEM_NORMAL, "Opens existing calibration data.", "Opens existing calibration, along with samples used to generate the calibration information")
        self.toolBar.AddLabelTool(wx.ID_SAVE, "Save Calibration", wx.Bitmap("/home/piyushk/art/sandbox/stacks/velodyne_experimental/velodyne_calibrate/icons/save.png", wx.BITMAP_TYPE_ANY), wx.NullBitmap, wx.ITEM_NORMAL, "Saves current Calibration", "Saves the current calibration, along with the samples used to generate this calibration")
        self.toolBar.AddSeparator()
        self.toolBar.AddLabelTool(ID_TOOL_EXEC, "Calibrate", wx.Bitmap("/home/piyushk/art/sandbox/stacks/velodyne_experimental/velodyne_calibrate/icons/exec.png", wx.BITMAP_TYPE_ANY), wx.NullBitmap, wx.ITEM_NORMAL, "Fit data to generate calibration", "Uses pythonequations to generate calibration from provided data")
        # Tool Bar end
        self.noCalibButton = wx.ToggleButton(self, -1, "No Calibration")
        self.defaultCalibButton = wx.ToggleButton(self, -1, "Default Calibration")
        self.currentCalibButton = wx.ToggleButton(self, -1, "Current Calibration")
        self.heightImagePanel = wx.Panel(self.heightPanel, -1)
        self.htPitchButton = wx.ToggleButton(self.heightPanel, -1, "Pitch")
        self.htIndvButton = wx.ToggleButton(self.heightPanel, -1, "Individual")
        self.intensityImagePanel = wx.Panel(self.intensityPanel, -1)
        self.topMark = wx.StaticText(self, -1, "Top:")
        self.topVal = wx.SpinCtrl(self, -1, "", min=0, max=63)
        self.bottomMark = wx.StaticText(self, -1, "  Bot:")
        self.botVal = wx.SpinCtrl(self, -1, "", min=0, max=63)
        self.leftMark = wx.StaticText(self, -1, "  Left:")
        self.leftVal = wx.SpinCtrl(self, -1, "", min=0, max=511)
        self.rightMark = wx.StaticText(self, -1, "  Right:")
        self.rightVal = wx.SpinCtrl(self, -1, "", min=0, max=511)

        self.__set_properties()
        self.__do_layout()

        self.Bind(wx.EVT_MENU, self.Exit, self.menuExit)
        self.Bind(wx.EVT_TOOL, self.CalibrateHeight, id = ID_TOOL_HEIGHT)
        self.Bind(wx.EVT_TOOL, self.CalibrateIntensity, id = ID_TOOL_INTENSITY)
        self.Bind(wx.EVT_TOOL, self.OpenCalibFile, id = wx.ID_OPEN)
        self.Bind(wx.EVT_TOOL, self.SaveCalibFile, id = wx.ID_SAVE)
        self.Bind(wx.EVT_TOOL, self.ExecuteCalibration, id = ID_TOOL_EXEC)
        self.Bind(wx.EVT_TOGGLEBUTTON, self.NoCalibration, self.noCalibButton)
        self.Bind(wx.EVT_TOGGLEBUTTON, self.DefaultCalibration, self.defaultCalibButton)
        self.Bind(wx.EVT_TOGGLEBUTTON, self.CurrentCalibration, self.currentCalibButton)
        self.Bind(wx.EVT_TOGGLEBUTTON, self.CalibratePitch, self.htPitchButton)
        self.Bind(wx.EVT_TOGGLEBUTTON, self.CalibrateIndividual, self.htIndvButton)
        self.Bind(wx.EVT_SPINCTRL, self.ChangeTopVal, self.topVal)
        self.Bind(wx.EVT_SPINCTRL, self.ChangeBottomVal, self.botVal)
        self.Bind(wx.EVT_SPINCTRL, self.ChangeLeftVal, self.leftVal)
        self.Bind(wx.EVT_SPINCTRL, self.ChangeRightVal, self.rightVal)
        # end wxGlade

        self.Bind(wx.EVT_CLOSE, self.OnShutdown)

        # Image Panel and Configuration Buttons
        self.Bind(wx.EVT_PAINT, self.DisplayImage)
        self.heightImagePanel.Bind(wx.EVT_MOUSE_EVENTS, self.ProcessMouse)
        self.intensityImagePanel.Bind(wx.EVT_MOUSE_EVENTS, self.ProcessMouse)
        self.heightImagePanel.SetCursor(wx.StockCursor(wx.CURSOR_CROSS))
        self.intensityImagePanel.SetCursor(wx.StockCursor(wx.CURSOR_CROSS))

        self.toolBar.ToggleTool(ID_TOOL_HEIGHT, 1)
        self.intensityPanel.Hide()

        self.Show(True)

        self.heightImage = None
        self.intensityImage = None

        # Rectangle
        self.rectangleAvailable = False
        self.rectangleDraw = False
        self.rectPos1 = None
        self.rectPos2 = None
        self.top = 0
        self.bottom = 0
        self.left = 0
        self.right = 0

        # Calibration Handler instance
        self.handler = handler

    def __set_properties(self):
        # begin wxGlade: MyFrame.__set_properties
        self.SetTitle("Velodyne Calibration Tool")
        self.SetSize((1024, 352))
        self.statusBar.SetStatusWidths([-1])
        # statusbar fields
        statusBar_fields = ["frame_1_statusbar"]
        for i in range(len(statusBar_fields)):
            self.statusBar.SetStatusText(statusBar_fields[i], i)
        self.toolBar.SetToolBitmapSize((32, 32))
        self.toolBar.Realize()
        self.defaultCalibButton.SetValue(1)
        self.heightImagePanel.SetMinSize((1024, 128))
        self.htIndvButton.SetValue(1)
        self.heightPanel.SetMinSize((1024,174))
        self.intensityImagePanel.SetMinSize((1024, 128))
        self.intensityPanel.SetMinSize((1024, 174))
        self.topVal.SetMinSize((50, 27))
        self.botVal.SetMinSize((50, 27))
        self.leftVal.SetMinSize((50, 27))
        self.rightVal.SetMinSize((50, 27))
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: MyFrame.__do_layout
        sizer_1 = wx.BoxSizer(wx.VERTICAL)
        sizer_7 = wx.BoxSizer(wx.HORIZONTAL)
        self.sizer_3 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_5 = wx.BoxSizer(wx.VERTICAL)
        sizer_4 = wx.BoxSizer(wx.VERTICAL)
        sizer_6 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_2 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_2.Add(self.noCalibButton, 0, wx.ALL, 3)
        sizer_2.Add(self.defaultCalibButton, 0, wx.ALL, 3)
        sizer_2.Add(self.currentCalibButton, 0, wx.ALL, 3)
        sizer_1.Add(sizer_2, 0, wx.ALL|wx.EXPAND, 3)
        sizer_4.Add(self.heightImagePanel, 0, 0, 0)
        sizer_6.Add(self.htPitchButton, 0, wx.ALL, 3)
        sizer_6.Add(self.htIndvButton, 0, wx.ALL, 3)
        sizer_4.Add(sizer_6, 0, wx.ALL, 3)
        self.heightPanel.SetSizer(sizer_4)
        self.sizer_3.Add(self.heightPanel, 1, wx.EXPAND, 0)
        sizer_5.Add(self.intensityImagePanel, 0, 0, 0)
        self.intensityPanel.SetSizer(sizer_5)
        self.sizer_3.Add(self.intensityPanel, 1, wx.EXPAND, 0)
        sizer_1.Add(self.sizer_3, 0, wx.EXPAND, 0)
        sizer_7.Add(self.topMark, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL, 3)
        sizer_7.Add(self.topVal, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL, 3)
        sizer_7.Add(self.bottomMark, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL, 3)
        sizer_7.Add(self.botVal, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL, 3)
        sizer_7.Add(self.leftMark, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL, 3)
        sizer_7.Add(self.leftVal, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL, 3)
        sizer_7.Add(self.rightMark, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL, 3)
        sizer_7.Add(self.rightVal, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL, 3)
        sizer_1.Add(sizer_7, 0, 0, 0)
        self.SetSizer(sizer_1)
        self.Layout()
        # end wxGlade

    def Exit(self, evt):
        "Close Calibrator"
        self.Close()
        evt.Skip()

    def OnShutdown(self, evt):
        "Cleanup while closing"
        self.handler.Cleanup()
        evt.Skip()

    def Refresh(self):         # overridden as inherited Refresh did not work
        "Refresh the display"
        self.AddPendingEvent(wx.PaintEvent()) 

    def ProcessMouse(self, event):

        panel = self.heightImagePanel
        if self.handler.mode == self.handler.CALIBRATE_NONE:
            event.Skip()
            return
        elif self.handler.mode == self.handler.CALIBRATE_INTENSITY:
            panel = self.intensityImagePanel

        if event.LeftDown():
            pos = event.GetPosition()
            if pos.x >= 0 and pos.y >= 0 and pos.x < displayWidth and pos.y < displayHeight:
                self.rectangleDraw = True
                self.rectangleAvailable = False
                self.rectPos1 = pos
                self.rectPos2 = pos

        elif (event.Dragging() or event.LeftUp()) and self.rectangleDraw:

            pos = event.GetPosition()
            if pos.x < 0:
                pos.x = 0
            if pos.y < 0:
                pos.y = 0
            if pos.x >= displayWidth:
                pos.x = displayWidth - 1
            if pos.y >= displayHeight:
                pos.y = displayHeight - 1
            self.rectPos2 = pos

            if self.rectPos2.y < self.rectPos1.y:
                self.top = self.rectPos2.y
                self.bottom = self.rectPos1.y
            else:
                self.top = self.rectPos1.y
                self.bottom = self.rectPos2.y

            if self.rectPos2.x < self.rectPos1.x:
                self.left = self.rectPos2.x
                self.right = self.rectPos1.x
            else:
                self.left = self.rectPos1.x
                self.right = self.rectPos2.x

            if event.LeftUp():
                self.rectangleDraw = False
                pixelLeft = self.left / scaleDisplay
                pixelRight = self.right / scaleDisplay
                pixelTop = self.top / scaleDisplay
                pixelBottom = self.bottom / scaleDisplay
                if pixelLeft != pixelRight and pixelTop != pixelBottom:
                    self.rectangleAvailable = True;
                    self.topVal.SetValue(pixelTop)
                    self.botVal.SetValue(pixelBottom)
                    self.leftVal.SetValue(pixelLeft)
                    self.rightVal.SetValue(pixelRight)

    def DisplayImage(self, evt):
        if self.heightImage:
            if self.handler.mode == self.handler.CALIBRATE_HEIGHT or self.handler.mode == self.handler.CALIBRATE_HEIGHT_PITCH:
                bmp = wx.BitmapFromImage(self.heightImage)
                wx.BufferedPaintDC(self.heightImagePanel, bmp)
                if self.rectangleDraw or self.rectangleAvailable:
                  dc = wx.ClientDC(self.heightImagePanel)
                  dc.BeginDrawing()
                  dc.SetPen(wx.Pen("blue",width=2,style=wx.DOT))
                  dc.SetBrush(wx.Brush("grey", wx.TRANSPARENT))
                  dc.DrawRectangle(self.left, self.top, (self.right - self. left), (self.bottom - self.top))
                  dc.EndDrawing()

        if self.intensityImage:
            if self.handler.mode == self.handler.CALIBRATE_INTENSITY:
                bmp = wx.BitmapFromImage(self.intensityImage)
                wx.BufferedPaintDC(self.intensityImagePanel, bmp)
                if self.rectangleDraw or self.rectangleAvailable:
                  dc = wx.ClientDC(self.intensityImagePanel)
                  dc.BeginDrawing()
                  dc.SetPen(wx.Pen("blue",width=2,style=wx.DOT))
                  dc.SetBrush(wx.Brush("grey", wx.TRANSPARENT))
                  dc.DrawRectangle(self.left, self.top, (self.right - self. left), (self.bottom - self.top))
                  dc.EndDrawing()
        evt.Skip()

    def SaveCalibFile(self, evt):
        dialog = wx.FileDialog(self, "Open calibration file", calib_path, "", "*.*", wx.FD_OVERWRITE_PROMPT)
        if dialog.ShowModal() == wx.ID_OK:
            filename = dialog.GetFilename()
            dirname = dialog.GetDirectory()
            print os.path.join(dirname, filename)
        dialog.Destroy()

    def OpenCalibFile(self, evt):
        dialog = wx.FileDialog(self, "Open calibration file", calib_path, "", "*.*", wx.FD_OPEN)
        if dialog.ShowModal() == wx.ID_OK:
            filename = dialog.GetFilename()
            dirname = dialog.GetDirectory()
            print os.path.join(dirname, filename)
        dialog.Destroy()

    def CalibrateHeight(self, event): # wxGlade: MyFrame.<event_handler>
        if self.handler.mode == self.handler.CALIBRATE_HEIGHT_PITCH or self.handler.mode == self.handler.CALIBRATE_HEIGHT:
            self.handler.mode = self.handler.CALIBRATE_NONE
            self.heightPanel.Hide()
        else:
            self.toolBar.ToggleTool(ID_TOOL_INTENSITY, 0)
            self.intensityPanel.Hide()
            self.handler.mode = self.handler.CALIBRATE_HEIGHT
            self.heightPanel.Show()
        self.sizer_3.Layout()
        event.Skip()

    def CalibrateIntensity(self, event): # wxGlade: MyFrame.<event_handler>
        if self.handler.mode == self.handler.CALIBRATE_INTENSITY:
            self.handler.mode = self.handler.CALIBRATE_NONE
            self.intensityPanel.Hide()
        else:
            self.toolBar.ToggleTool(ID_TOOL_HEIGHT, 0)
            self.heightPanel.Hide()
            self.handler.mode = self.handler.CALIBRATE_INTENSITY
            self.intensityPanel.Show()          
        self.sizer_3.Layout()
        event.Skip()

    def ChangeTopVal(self, event): # wxGlade: MyFrame.<event_handler>
        self.top = self.topVal.GetValue() * scaleDisplay;
        event.Skip()

    def ChangeBottomVal(self, event): # wxGlade: MyFrame.<event_handler>
        self.bottom = self.botVal.GetValue() * scaleDisplay;
        event.Skip()

    def ChangeLeftVal(self, event): # wxGlade: MyFrame.<event_handler>
        self.left = self.leftVal.GetValue() * scaleDisplay;
        event.Skip()

    def ChangeRightVal(self, event): # wxGlade: MyFrame.<event_handler>
        self.right = self.rightVal.GetValue() * scaleDisplay;
        event.Skip()

    def NoCalibration(self, event): # wxGlade: MyFrame.<event_handler>
        if (self.handler.calibration == self.handler.NONE):
            self.noCalibButton.SetValue(1)
        else:
            self.defaultCalibButton.SetValue(0)
            self.currentCalibButton.SetValue(0)
            self.handler.UpdateCalibration(self.handler.NONE)
        event.Skip()

    def DefaultCalibration(self, event): # wxGlade: MyFrame.<event_handler>
        if (self.handler.calibration == self.handler.DEFAULT):
            self.defaultCalibButton.SetValue(1)
        else:
            self.noCalibButton.SetValue(0)
            self.currentCalibButton.SetValue(0)
            self.handler.UpdateCalibration(self.handler.DEFAULT)
        event.Skip()

    def CurrentCalibration(self, event): # wxGlade: MyFrame.<event_handler>
        if (self.handler.calibration == self.handler.CURRENT):
            self.currentCalibButton.SetValue(1)
        else:
            self.noCalibButton.SetValue(0)
            self.defaultCalibButton.SetValue(0)
            self.handler.UpdateCalibration(self.handler.CURRENT)
        event.Skip()

    def CalibratePitch(self, event): # wxGlade: MyFrame.<event_handler>
        if (self.handler.mode == self.handler.CALIBRATE_HEIGHT_PITCH):
            self.htPitchButton.SetValue(1)
        else:
            self.htIndvButton.SetValue(0)
            self.handler.mode = self.handler.CALIBRATE_HEIGHT_PITCH
        event.Skip()

    def CalibrateIndividual(self, event): # wxGlade: MyFrame.<event_handler>
        if (self.handler.mode == self.handler.CALIBRATE_HEIGHT):
            self.htIndvButton.SetValue(1)
        else:
            self.htPitchButton.SetValue(0)
            self.handler.mode = self.handler.CALIBRATE_HEIGHT
        event.Skip()

    def ExecuteCalibration(self, event): # wxGlade: MyFrame.<event_handler>
        print "Event handler `ExecuteCalibration' not implemented!"
        event.Skip()


class RosHandler:
    "Monitor ROS velodyne topics for data analysis."

    def __init__(self, handler):
        "constructor"
        self.bridge = CvBridge()
        self.colorHeightImage = cv.CreateImage(imageSize, cv.IPL_DEPTH_8U, 3)
        self.colorIntensityImage = cv.CreateImage(imageSize, cv.IPL_DEPTH_8U, 3)
        self.handler = handler

    def ProcessPointCloud(self, msg):
        "ROS callback for Velodyne point cloud."
        rospy.logdebug('got a point cloud')
        self.handler.UpdatePointCloud(msg)

    def ProcessHeightImage(self, data):
        "ROS callback for Velodyne Height Image."
        rospy.logdebug('got the height image')
        try:
            image = self.bridge.imgmsg_to_cv(data)
        except CvBridgeError, e:
            print e
        cv.CvtColor(image, self.colorHeightImage, cv.CV_GRAY2RGB)
        self.handler.UpdateHeightImage(self.colorHeightImage)

    def ProcessIntensityImage(self, data):
        "ROS callback for Velodyne Intensity Image."
        rospy.logdebug('got the intensity image')
        try:
            image = self.bridge.imgmsg_to_cv(data)
        except CvBridgeError, e:
            print e
        cv.CvtColor(image, self.colorIntensityImage, cv.CV_GRAY2RGB)
        self.handler.UpdateIntensityImage(self.colorIntensityImage)

    def SubscribeHeightImage(self):
        self.heightImageSubscriber = rospy.Subscriber("velodyne/heightImage", Image, self.ProcessHeightImage)

    def SubscribeIntensityImage(self):
        self.intensityImageSubscriber = rospy.Subscriber("velodyne/intensityImage", Image, self.ProcessIntensityImage)
    
    def SubscribePointCloud(self):
        self.pointCloudSubscriber = rospy.Subscriber('velodyne/pointcloud', numpy_msg(PointCloud), self.ProcessPointCloud)

    def UnsubscribeHeightImage(self):
        if self.heightImageSubscriber:
            self.heightImageSubscriber.unregister()
            self.heightImageSubscriber = None

    def UnsubscribeIntensityImage(self):
        if self.intensityImageSubscriber:
            self.intensityImageSubscriber.unregister()
            self.intensityImageSubscriber = None

    def UnsubscribePointCloud(self):
        if self.pointCloudSubscriber:
            self.pointCloudSubscriber.unregister()
            self.pointCloudSubscriber = None

    def SubscribeAll(self):
        self.SubscribeHeightImage()
        self.SubscribeIntensityImage()
        self.SubscribePointCloud()

    def UnsubscribeAll(self):
        self.UnsubscribeHeightImage()
        self.UnsubscribeIntensityImage()
        self.UnsubscribePointCloud()

    def Shutdown(self):
        rospy.signal_shutdown("Terminating Program")


class wxThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.app = wx.App(False)
        handler = CalibrationHandler()
        frame = CalibrationWindow(None, wx.ID_ANY, handler)
        velodyneTopics = RosHandler(handler)
        handler.updateWindow(frame)
        handler.updateRosHandler(velodyneTopics)

        velodyneTopics.SubscribeAll()

    def run(self):
        # run the GUI
        exit_status = self.app.MainLoop()
        sys.exit(exit_status)

def main(argv = None):
    rospy.init_node(NODE_NAME)
    wxThread().start()

    rospy.loginfo('begin acquiring Velodyne data')
    try:
        rospy.spin()           # invoke callbacks as data arrive
    except rospy.ROSInterruptException: pass
    rospy.loginfo('done acquiring Velodyne data')

    return 0

if __name__ == "__main__":
    sys.exit(main())
