import cv2
import numpy as np
from PyQt4 import QtGui, QtCore, Qt

class Video():
    def __init__(self,capture):
        self.capture = capture
        self.capture.set(3, 1280)
        self.capture.set(4, 960)
        self.currentFrame=np.array([])

        """ 
        Affine Calibration Variables 
        Currently only takes three points: center of arm and two adjacent, 
        corners of the base board. Use more for better calibration.
        Note that OpenCV requires float32 arrays
        """
        self.aff_npoints = 3
        self.real_coord = np.float32([[0., 0.], [305.,-305.], [-305.,-305.]])
        self.mouse_coord = np.float32([[0.0, 0.0],[0.0, 0.0],[0.0, 0.0]])      
        self.mouse_click_id = 0
        self.aff_flag = 0
        self.aff_matrix = np.float32((2,3))
    
    def captureNextFrame(self):
        """                      
        Capture frame, convert to RGB, and return opencv image      
        """
        ret, frame=self.capture.read()
        if(ret==True):
            self.currentFrame=cv2.cvtColor(frame, cv2.COLOR_BAYER_GB2BGR)

    def convertFrame(self):
        """ Converts frame to format suitable for QtGui  """
        try:
            height,width=self.currentFrame.shape[:2]
            img=QtGui.QImage(self.currentFrame,
                              width,
                              height,
                              #QtGui.QImage.Format_RGB888)
                              QtGui.QImage.Format_RGB888)
            img=QtGui.QPixmap.fromImage(img)
            self.previousFrame = self.currentFrame
            return img
        except:
            return None

    def loadCalibration(self):
        """
        Load csmera distortion calibration from file and applies to the image:
        Look at camera_cal.py final lines to see how that is done
        This is optional, you do not need to implement distortion correction
        """
        pass

    def blobDetector(self):
        """
        Implement your color blob detector here.  
        You will need to detect 5 different color blobs
        """
