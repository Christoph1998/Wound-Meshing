from PyQt5 import QtCore, QtGui, uic
print('[INFO] Successful import of uic') #often reinstallation of PyQt5 is required

from PyQt5.QtCore import (QCoreApplication, QThread, QThreadPool, pyqtSignal, pyqtSlot, Qt, QTimer, QDateTime, QObject, QMutex, QTemporaryFile)
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
        self.pB_read_pcd.clicked.connect(self.read_pcd)
        self.pB_crop_pcd.clicked.connect(self.crop_pcd)
        self.pB_plot_measurement.clicked.connect(self.plot_measurements)
        self.pB_plot_pipeline_pcd.clicked.connect(self.plot_pipeline_pcd)
        self.pB_ball_pivot.clicked.connect(self.run_bpa)
        self.pB_pois.clicked.connect(self.run_pois)
        self.pB_plot_wound.clicked.connect(self.plot_selection)
        self.pB_hull.clicked.connect(self.run_hull)
        

    def read_pcd(self):
        print("Start reading Point Cloud")
        try:
            pcd = read_point_cloud_from_file(path=self.tE_filename_read_pcd.toPlainText(), plot=self.rB_plotting_read_pcd.isChecked())
            print("Point Cloud reading successful.")
        except:
            print("Point Cloud read failed. Please check the file path.")
        o3d.io.write_point_cloud("input_pcd_tmp.ply", pcd)
        
    def crop_pcd(self):
        # read input data
        pcd = read_point_cloud_from_file(path=self.tE_filename_read_pcd.toPlainText(), plot=False)           

        try:
            print("Start cropping")
            x_min = float(self.tE_x_min.toPlainText())
            x_max = float(self.tE_x_max.toPlainText())
            y_min = float(self.tE_y_min.toPlainText())
            y_max = float(self.tE_y_max.toPlainText())
            z_min = float(self.tE_z_min.toPlainText())
            z_max = float(self.tE_z_max.toPlainText())
            coordinates = [x_min,x_max,y_min,y_max,z_min,z_max]
            pcd = crop_point_cloud(pcd=pcd, coordinates=coordinates,plot=self.rB_plotting_cropping.isChecked(), 
            filename=self.tE_filename_cropping.toPlainText())
            print("Point Cloud cropped successfully")
            
        except:
            print("Error while cropping Point Cloud. Please check data")
        os.remove("input_pcd_tmp.ply")
        o3d.io.write_point_cloud("input_pcd_tmp.ply", pcd)

    def plot_measurements(self):
        plot_measurements(read_point_cloud_from_file(path="input_pcd_tmp.ply", plot=False))

    def plot_pipeline_pcd(self):
        plot_wound(pcd=read_point_cloud_from_file(path="input_pcd_tmp.ply", plot=False))

    def run_bpa(self):
        try:
            print("Create mesh using ball pivot algorithm")
            pcd = read_point_cloud_from_file(path="input_pcd_tmp.ply", plot=False)
            sqm = int(self.tE_sqm.toPlainText())
            filename = self.tE_filename_bpa.toPlainText()
            plot = self.rB_plotting_bpa.isChecked()

            mesh = create_mesh_bpa_algo(pcd, sqm, filename, plot)
        except:
            print("Mesh creation failed")
        o3d.io.write_triangle_mesh("bpa_mesh_tmp.obj", mesh)
        print("Mesh created successfully")

    def run_pois(self):
        try:
            print("Create mesh using poison algorithm")   
            mesh = create_mesh_poisson_algo(read_point_cloud_from_file(path="input_pcd_tmp.ply", plot=False), self.rB_plotting_pois.isChecked())
        except:
            print("Mesh creation failed")
        o3d.io.write_triangle_mesh("pois_mesh_tmp.obj", mesh)
        print("Mesh created successfully")

    def plot_selection(self):
        pcd = read_point_cloud_from_file(path="input_pcd_tmp.ply", plot=False)
        hull, return_list = pcd.compute_convex_hull()
        hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
        hull_ls.paint_uniform_color((1, 0, 0))
        if self.cB_pcd_plot.isChecked():
            pcd = pcd
        else:
            pcd = None
        if self.cB_bpa_plot.isChecked():
            mesh = o3d.io.read_triangle_mesh("bpa_mesh_tmp.obj")
        elif self.cB_pois_plot.isChecked():
            mesh = o3d.io.read_triangle_mesh("pois_mesh_tmp.obj")
        else:mesh = None
        if self.cB_hull_plot.isChecked():
            hull = hull_ls
        else:
            hull = None
        plot_wound(pcd=pcd,mesh=mesh,hull=hull)

    def run_hull(self):
        hull = compute_convex_hull(pcd = read_point_cloud_from_file(path="input_pcd_tmp.ply", plot=False), plot=self.rB_plotting_hull.isChecked())


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
