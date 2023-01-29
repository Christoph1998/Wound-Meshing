#!/usr/bin/env python3
# Edited by: Daniel Blase

################################## IMPORT #######################################################

#from multiprocessing.connection import PipeConnection
#from asyncio.windows_utils import PipeHandle
from PyQt5 import QtCore, QtGui, uic
print('[INFO] Successful import of uic') #often reinstallation of PyQt5 is required

from PyQt5.QtCore import (QCoreApplication, QThread, QThreadPool, pyqtSignal, pyqtSlot, Qt, QTimer, QDateTime, QObject, QMutex)
from PyQt5.QtGui import (QImage, QPixmap, QTextCursor, QIntValidator)
from PyQt5.QtWidgets import (QWidget, QMainWindow, QApplication, QLabel, QPushButton, QVBoxLayout, QGridLayout, QSizePolicy, QMessageBox, QFileDialog, QSlider, QComboBox, QProgressDialog)

import sys
import os #.path
import cv2
import numpy as np

#import imutils
from PIL import Image
from scipy import ndimage

import math
import open3d as o3d

print('[INFO] Loaded Packages and Starting APP...')
################################# VARIABLES #####################################################
# Qt_Designer File to open:
qtCreatorFile = r"C:\Users\Christoph\Downloads\Qt5_Example\Qt5_Example\DepthAiApp_1920x1080.ui"
# Create a GUI window (inherited from in class App())
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtCreatorFile)
# Folder where images are to be stored (can be changed by user)
save_folder = r"C:\Users\Christoph\Downloads\Qt5_Example\Qt5_Example"
# GUI display frame refresh interval in ms
REFRESH_INTERVAL = 10
# Set the FPS for the depthai cameras
FPS = 30
# Set manual focus for depth rgb alignment
MANUAL_FOCUS = 150

################################# CLASSES and FUNCTIONS #####################################################

def updateBlendWeights(percent_rgb):
    """
    Update the rgb and depth weights used to blend depth/rgb image
    @param[in] percent_rgb The rgb weight expressed as a percentage (0..100)
    """
    global depthWeight
    global rgbWeight
    rgbWeight = float(percent_rgb)/100.0
    depthWeight = 1.0 - rgbWeight

#################################### APP ###################################################
# Class for the central widget of the application defining the order/start-up/callbacks etc.; Opens the imported .ui GUI and inherits from it
class App(QMainWindow, Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)
        self.initUI()
        self.init_OAKDPRO()
        

    #DB: As soon as the event changePixmap is emitted by the thread1, this slot is triggered and the image of type QImage is transferred
    # @pyqtSlot(QImage,object)
    # def setImage1(self, image, frame):
    #     #DB: will have to implement a token here to avoid to streams both accessing the l_DisplayFrame_Lepton1 at the same time
    #     if thread1 == "active":
    #         self.l_DisplayFrame_Lepton1.setPixmap(QPixmap.fromImage(image))
    #         #self.l_DisplayFrame_Lepton1_big.setPixmap(QPixmap.fromImage(image))

    #         # Save the image for signal processing (prevent parallel access)
    #         self.mutex.lock()
    #         self.L1_img = frame #image
    #         self.L1_grabbed = True
    #         self.mutex.unlock()

    # Determining the callbacks for certain ui-actions
    def initUI(self):
        self.pB_DisplayLiveImage.clicked.connect(self.start_preview)
        self.pB_Save.clicked.connect(self.save_raw_n_depth)
        self.tE_SavePath.setPlainText(save_folder)
        self.tE_IrFloodLevel.setPlainText("0")
        self.tE_IrPointLevel.setPlainText("0")
        self.rb_DShift.toggled.connect(self.adjust_disparity_shift)
        self.rB_Focus_Manual.toggled.connect(self.show_image)
        self.pB_UpdateConfig.clicked.connect(self.update_configuration)

        # The timer is for displaying the time (dt=1 sec), timerFast (dt=10ms) is for updating the temperatures and timerCSI is for reading a frame from the CSI camera
        self.timer = QTimer(self)
        self.display_timer = QTimer(self)
        self.timerCSI = QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.displayTime)
        self.timer.start()
        self.display_timer.setInterval(REFRESH_INTERVAL)
        self.display_timer.timeout.connect(self.refresh_display)
        self.mutex = QMutex() # For writing data to the class and for retrieving
        self.counter = 0

    # Initialization of the connection to the OAK-D-Pro Depth Camera
    def init_OAKDPRO(self):
        # Here I could put some code for camera initialization.
        print("[INFO] Initialization of OAKDPRO done.")

    # Update all configuration settings and send the message to the camera
    def update_configuration(self):
        # Update the IR flood light and point generator
        print("[INFO] Update the camera settings")

    # Switch RGB-Camera-Focus between automatic and manual
    def show_image(self):
        # Read the example image and display it in all frames
        depth_map_in = np.load(r"C:\Users\Christoph\Downloads\Qt5_Example\Qt5_Example\Images\depth_14-11-2022_14-42-13.npy")
        left_frame_in = np.load(r"C:\Users\Christoph\Downloads\Qt5_Example\Qt5_Example\Images\left_14-11-2022_14-42-13.npy")
        right_frame_in = np.load(r"C:\Users\Christoph\Downloads\Qt5_Example\Qt5_Example\Images\right_14-11-2022_14-42-13.npy")
        RGB_frame_in = np.load(r"C:\Users\Christoph\Downloads\Qt5_Example\Qt5_Example\Images\RGB_14-11-2022_14-42-13.npy")
        
        # Display the images
        dframe_left = cv2.resize(left_frame_in, (320,200)) 
        #cv2.addWeighted(IR_img, 0.6, CSI_trt_img, 0.4, 0, IR_img) #0: gamma; output: destination
        Q_image_l = QImage(dframe_left.data, dframe_left.shape[1], dframe_left.shape[0], QImage.Format_Grayscale8)
        self.l_DisplayFrame_Left.setPixmap(QPixmap.fromImage(Q_image_l))

        dframe_right = cv2.resize(right_frame_in, (320,200)) 
        #cv2.addWeighted(IR_img, 0.6, CSI_trt_img, 0.4, 0, IR_img) #0: gamma; output: destination
        Q_image_r = QImage(dframe_right.data, dframe_right.shape[1], dframe_right.shape[0], QImage.Format_Grayscale8)
        self.l_DisplayFrame_Right.setPixmap(QPixmap.fromImage(Q_image_r))

        dframe_rgb_isp = cv2.resize(RGB_frame_in, (480,270)) 
        dframe_rgb_isp = cv2.cvtColor(dframe_rgb_isp, cv2.COLOR_BGR2RGB)
        #cv2.addWeighted(IR_img, 0.6, CSI_trt_img, 0.4, 0, IR_img) #0: gamma; output: destination
        Q_image = QImage(dframe_rgb_isp.data, dframe_rgb_isp.shape[1], dframe_rgb_isp.shape[0], QImage.Format_RGB888)
        self.l_DisplayFrame_RGB.setPixmap(QPixmap.fromImage(Q_image))

        Q_image_d = QImage(depth_map_in.data, depth_map_in.shape[1], depth_map_in.shape[0], QImage.Format_Grayscale8)
        self.l_DisplayFrame_Depth.setPixmap(QPixmap.fromImage(Q_image_d))

        # If you want to write something on frames:
        # fontType = cv2.FONT_HERSHEY_TRIPLEX
        # cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), self.color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)
        # cv2.putText(depthFrameColor, f"X: {int(depthData.spatialCoordinates.x)} mm", (xmin + 10, ymin + 20), fontType, 0.5, 255)
        # cv2.putText(depthFrameColor, f"Y: {int(depthData.spatialCoordinates.y)} mm", (xmin + 10, ymin + 35), fontType, 0.5, 255)
        # cv2.putText(depthFrameColor, f"Z: {int(depthData.spatialCoordinates.z)} mm", (xmin + 10, ymin + 50), fontType, 0.5, 255)

        print("[INFO] Displayed an image.")

    # Preview of left, right, RGB and depth (live) in GUI labels
    def start_preview(self):
        # dummy variable reset
        self.counter = 0
        # Start refresh timer
        self.display_timer.start()
        print("[INFO] Display Timer started.")

    # Refresh all frames in the GUI
    def refresh_display(self):
        # Here you can put the code that should be executed at every refresh interval
        # Instead of using refresh intervals you could also do multithreading (provided by qt, but dealing with mutexes, locks, etc.)
        # print("[INFO] Performing regular update Step.") # Too annoying ;)
        self.counter += 1

    # GUI: Enable or disable a disparity shift which will decrease the minimum perceiveable distance but then cannot see objects farther away than... 
    # WARNING: This function works but is useless for now and shift=0 will not lead to the original state!
    # FOR THIS FUNCTION to work the nodes and links have to be established in the init
    def adjust_disparity_shift(self):
        print("[INFO] Adjust disparity shift")

    # Function to check the RGB-depth alignment
    def align_rgb_depth(self):
        # Configure windows; trackbar adjusts blending ratio of rgb/depth

        rgbWeight = 0.4
        depthWeight = 0.6
        blendedWindowName = "rgb-depth"
        cv2.namedWindow(blendedWindowName)
        cv2.createTrackbar('Manual Focus %', blendedWindowName, 80, 150, updateBlendWeights)

        depth = self.frame_depth
        rgb = self.frame_rgb

        if len(depth.shape) < 3:
            frameDisp = cv2.cvtColor(depth, cv2.COLOR_GRAY2RGB)
        blended = cv2.addWeighted(rgb, rgbWeight, depth, depthWeight, 0)
        cv2.imshow(blendedWindowName, blended)

    # For displaying the time
    def displayTime(self):
        self.l_Time.setText(QDateTime.currentDateTime().toString('dd.MM.yyyy hh:mm:ss'))

    # Saving all images (2x Mono + RGB + Depth) into a new folder
    def save_raw_n_depth(self):
        global save_folder
        # Check the save folder (might be changed by user)
        save_folder = self.tE_SavePath.toPlainText()
        print(f'[INFO] Saving the Images to {save_folder}.')
        
        # Create a directory if not yet done
        try:
            os.makedirs(save_folder)
        except FileExistsError:
            # Directory already existing
            pass # do nothing

        # Create the file name
        filename_left = save_folder + "/left_" + (self.l_Time.text()).replace(" ","_").replace(".","-").replace(":","-") # + ".jpg"
        filename_right = save_folder + "/right_" + (self.l_Time.text()).replace(" ","_").replace(".","-").replace(":","-") # + ".jpg"
        filename_RGB = save_folder + "/RGB_" + (self.l_Time.text()).replace(" ","_").replace(".","-").replace(":","-") # + ".jpg"
        filename_depth = save_folder + "/depth_" + (self.l_Time.text()).replace(" ","_").replace(".","-").replace(":","-") # + ".jpg"
        filename_disparity = save_folder + "/disparity_" + (self.l_Time.text()).replace(" ","_").replace(".","-").replace(":","-") # + ".jpg"
        filename_spatials = save_folder + (self.l_Time.text()).replace(" ","_").replace(".","-").replace(":","-") # + ".jpg"
        filename_pcd = save_folder + "/spatials_" + (self.l_Time.text()).replace(" ","_").replace(".","-").replace(":","-") + ".ply"
        
        # print(f'[INFO] spatial_map: {spatial_points}')
        # print(f'[INFO] spatial_map uint16: { np.array(spatial_points, dtype=np.float32)}')

        # Save as numpy arrays for now (TODO: RGB raw)
        # np.save(filename_left, self.frame_left)
        # np.save(filename_right, self.frame_right)
        # np.save(filename_RGB, self.frame_rgb)
        # np.save(filename_RGB+"_downsized", rgb_small)
        # np.save(filename_depth, self.frame_depth)
        # np.save(filename_disparity, self.frame_disparity)

        print('[INFO] Saving frames to .npy files done.')

    def closeEvent(self, event):
        print("[INFO] Close event called")

        # Stop the timers
        self.timer.stop()
        self.display_timer.stop()

# Main routine for displaying the GUI
def main():
    # The following sequence is the standard code for opening a custom application's (ui, interactive) window
    app = QApplication(sys.argv)
    # Use the class App as central widget
    window = App()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()