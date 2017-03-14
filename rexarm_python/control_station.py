import sys
import cv2
import numpy as np
from PyQt4 import QtGui, QtCore, Qt
from ui import Ui_MainWindow
from rexarm import Rexarm
import functools #Let's us give parameters to callback functions for QT connect

from video import Video

""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592

""" Pyxel Positions of image in GUI """
MIN_X = 310
MAX_X = 950

MIN_Y = 30
MAX_Y = 510
 
class Gui(QtGui.QMainWindow):
    """ 
    Main GUI Class
    It contains the main function and interfaces between 
    the GUI and functions
    """
    def __init__(self,parent=None):
        QtGui.QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        """ Main Variables Using Other Classes"""
        self.rex = Rexarm()
        self.video = Video(cv2.VideoCapture(0))

        """ Other Variables """
        self.last_click = np.float32([0,0])

        """ Set GUI to track mouse """
        QtGui.QWidget.setMouseTracking(self,True)

        """ 
        Video Function 
        Creates a timer and calls play() function 
        according to the given time delay (27mm) 
        """
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.play)
        self._timer.start(27)
       
        """ 
        LCM Arm Feedback
        Creates a timer to call LCM handler continuously
        No delay implemented. Reads all time 
        """  
        self._timer2 = QtCore.QTimer(self)
        self._timer2.timeout.connect(self.rex.get_feedback)
        self._timer2.start()

        """ 
        Connect Sliders to Function
        LAB TASK: CONNECT THE OTHER 5 SLIDERS IMPLEMENTED IN THE GUI 
        """ 
        self.ui.sldrBase.valueChanged.connect(functools.partial(self.sliderChange,0))
        self.ui.sldrShoulder.valueChanged.connect(functools.partial(self.sliderChange,1))
        self.ui.sldrElbow.valueChanged.connect(functools.partial(self.sliderChange,2))
        self.ui.sldrWrist.valueChanged.connect(functools.partial(self.sliderChange,3))
        self.ui.sldrGrip1.valueChanged.connect(functools.partial(self.sliderChange,4))
        self.ui.sldrGrip2.valueChanged.connect(functools.partial(self.sliderChange,5))
        self.ui.sldrMaxTorque.valueChanged.connect(functools.partial(self.sliderChange,6))
        self.ui.sldrSpeed.valueChanged.connect(functools.partial(self.sliderChange,7))

        """ Commands the arm as the arm initialize to 0,0,0,0 angles """
        self.sliderChange(0) 
        
        """ Connect Buttons to Functions 
        LAB TASK: NAME AND CONNECT BUTTONS AS NEEDED
        """
        self.ui.btnUser1.setText("Affine Calibration")
        self.ui.btnUser1.clicked.connect(self.affine_cal)



    def play(self):
        """ 
        Play Funtion
        Continuously called by GUI 
        """

        """ Renders the Video Frame """
        try:
            self.video.captureNextFrame()
            self.video.blobDetector()
            self.ui.videoFrame.setPixmap(
                self.video.convertFrame())
            self.ui.videoFrame.setScaledContents(True)
        except TypeError:
            print "No frame"
        
        """ 
        Update GUI Joint Coordinates Labels
        LAB TASK: include the other slider labels 
        """
        self.ui.rdoutBaseJC.setText(str("%.2f" % (self.rex.joint_angles_fb[0]*R2D)))
        self.ui.rdoutShoulderJC.setText(str("%.2f" % (self.rex.joint_angles_fb[1]*R2D)))
        self.ui.rdoutElbowJC.setText(str("%.2f" % (self.rex.joint_angles_fb[2]*R2D)))
        self.ui.rdoutWristJC.setText(str("%.2f" % (self.rex.joint_angles_fb[3]*R2D)))

        """ 
        Mouse position presentation in GUI
        TO DO: after getting affine calibration make the apprriate label
        to present the value of mouse position in world coordinates 
        """    
        x = QtGui.QWidget.mapFromGlobal(self,QtGui.QCursor.pos()).x()
        y = QtGui.QWidget.mapFromGlobal(self,QtGui.QCursor.pos()).y()
        if ((x < MIN_X) or (x > MAX_X) or (y < MIN_Y) or (y > MAX_Y)):
            self.ui.rdoutMousePixels.setText("(-,-)")
            self.ui.rdoutMouseWorld.setText("(-,-)")
        else:
            x = x - MIN_X
            y = y - MIN_Y
            self.ui.rdoutMousePixels.setText("(%.0f,%.0f)" % (x,y))
            if (self.video.aff_flag == 2):
                """ TO DO Here is where affine calibration must be used """
                self.ui.rdoutMouseWorld.setText("(-,-)")
            else:
                self.ui.rdoutMouseWorld.setText("(-,-)")

        """ 
        Updates status label when rexarm playback is been executed.
        This will be extended to includ eother appropriate messages
        """ 
        if(self.rex.plan_status == 1):
            self.ui.rdoutStatus.setText("Playing Back - Waypoint %d"
                                    %(self.rex.wpt_number + 1))


    def sliderChange(self,selected_slider):
        """ 
        Function to change the slider labels when sliders are moved
        and to command the arm to the given position 
        TO DO: Implement for the other sliders
        """
        #print "Selected joint:", selected_slider
        if 0 <= selected_slider and selected_slider <= 5:
            angle = 0
            if selected_slider == 0:
                self.ui.rdoutBase.setText(str(self.ui.sldrBase.value()))
                angle = self.ui.sldrBase.value()*D2R
            elif selected_slider == 1:
                self.ui.rdoutShoulder.setText(str(self.ui.sldrShoulder.value()))
                angle = self.ui.sldrShoulder.value()*D2R
            elif selected_slider == 2:
                self.ui.rdoutElbow.setText(str(self.ui.sldrElbow.value()))
                angle = self.ui.sldrElbow.value()*D2R
            elif selected_slider == 3:
                self.ui.rdoutWrist.setText(str(self.ui.sldrWrist.value()))
                angle = self.ui.sldrWrist.value()*D2R
            elif selected_slider == 4:
                self.ui.rdoutGrip1.setText(str(self.ui.sldrGrip1.value()))
                angle = self.ui.sldrGrip1.value()*D2R
            elif selected_slider == 5:
                self.ui.rdoutGrip2.setText(str(self.ui.sldrGrip2.value()))
                angle = self.ui.sldrGrip2.value()*D2R

            self.rex.joint_angles[selected_slider] = angle;
        elif 6 <= selected_slider and selected_slider <= 7:
            if selected_slider == 6:
                self.ui.rdoutTorq.setText(str(self.ui.sldrMaxTorque.value()) + "%")
                self.rex.max_torque = self.ui.sldrMaxTorque.value()/100.0
            elif selected_slider == 7:
                self.ui.rdoutSpeed.setText(str(self.ui.sldrSpeed.value()) + "%")
                self.rex.speed = self.ui.sldrSpeed.value()/100.0
        else:
            print "Error: Unrecognized slider index", selected_slider

        self.rex.cmd_publish()

    def mousePressEvent(self, QMouseEvent):
        """ 
        Function used to record mouse click positions for 
        affine calibration 
        """
 
        """ Get mouse posiiton """
        x = QMouseEvent.x()
        y = QMouseEvent.y()

        """ If mouse position is not over the camera image ignore """
        if ((x < MIN_X) or (x > MAX_X) or (y < MIN_Y) or (y > MAX_Y)): return

        """ Change coordinates to image axis """
        self.last_click[0] = x - MIN_X
        self.last_click[1] = y - MIN_Y
       
        """ If affine calibration is been performed """
        if (self.video.aff_flag == 1):
            """ Save last mouse coordinate """
            self.video.mouse_coord[self.video.mouse_click_id] = [(x-MIN_X),
                                                                 (y-MIN_Y)]

            """ Update the number of used poitns for calibration """
            self.video.mouse_click_id += 1

            """ Update status label text """
            self.ui.rdoutStatus.setText("Affine Calibration: Click Point %d" 
                                      %(self.video.mouse_click_id + 1))

            """ 
            If the number of click is equal to the expected number of points
            computes the affine calibration.
            
            LAB TASK: Change this code to use your affine calibration routine
            and NOT openCV pre-programmed function as it is done now.
            """
            if(self.video.mouse_click_id == self.video.aff_npoints):
                """ 
                Update status of calibration flag and number of mouse
                clicks
                """
                self.video.aff_flag = 2
                self.video.mouse_click_id = 0
                
                """ Perform affine calibration with OpenCV """
                self.video.aff_matrix = cv2.getAffineTransform(
                                        self.video.mouse_coord,
                                        self.video.real_coord)
            
                """ Updates Status Label to inform calibration is done """ 
                self.ui.rdoutStatus.setText("Waiting for input")

                """ 
                print affine calibration matrix numbers to terminal
                """ 
                print self.video.aff_matrix

    def affine_cal(self):
        """ 
        Function called when affine calibration button is called.
        Note it only chnage the flag to record the next mouse clicks
        and updates the status text label 
        """
        self.video.aff_flag = 1 
        self.ui.rdoutStatus.setText("Affine Calibration: Click Point %d" 
                                    %(self.video.mouse_click_id + 1))
 
def main():
    app = QtGui.QApplication(sys.argv)
    ex = Gui()
    ex.show()
    sys.exit(app.exec_())
 
if __name__ == '__main__':
    main()
