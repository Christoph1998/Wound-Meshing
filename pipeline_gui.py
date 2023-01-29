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

from wound_meshing import *

pcd = read_point_cloud_from_file(path=r"SampleData\spatials_14-11-2022_14-42-13.ply", 
                               transform=[[1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]], plot=False)


################################# VARIABLES #####################################################
# Qt_Designer File to open:
qtCreatorFile = r"C:\Users\Christoph\Desktop\Wound-Meshing\pipelineWatcher.ui"
# Create a GUI window (inherited from in class App())
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtCreatorFile)



#################################### APP ###################################################
# Class for the central widget of the application defining the order/start-up/callbacks etc.; Opens the imported .ui GUI and inherits from it
class App(QMainWindow, Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)
        self.initUI()


    def initUI(self):
        self.tE_Number.setPlainText("Hallo")
        self.rb_PrintX.toggled.connect(self.plotX)
        self.pB_Plot.clicked.connect(self.plot_wound)

    def plotX(self):
        print("was toggeled")

    def plot_wound(self):
        if self.rb_PrintX.isChecked():
            textfield = int(self.tE_Number.toPlainText())
            print(textfield)
            print(textfield.type())
        plot_wound(pcd)


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
