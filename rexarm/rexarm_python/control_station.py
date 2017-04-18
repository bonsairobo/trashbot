import struct
import sys
import os
import time
import cv2
import numpy as np
import numpy.linalg
from PyQt4 import QtGui, QtCore, Qt
from ui import Ui_MainWindow
from rexarm import Rexarm
import functools #Let's us give parameters to callback functions for QT connect
import socket

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
        self.rex = Rexarm(self.ui.rdoutX,self.ui.rdoutY,self.ui.rdoutZ,self.ui.rdoutT)
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

        #Setting the poses
        #Pick up position
        self.ui.btnUser2.setText("Pick Up Position")
        self.ui.btnUser2.clicked.connect(functools.partial(self.setPose,[-0.063,0.203,-.605,-1.493,-0.107,1.702]))
        #Home position (Outside kinect view)
        self.ui.btnUser4.clicked.connect(functools.partial(self.setPose,[-2.015,-1.89,0.318,-1.135,-0.47,1.723]))
        #self.ui.btnUser4.clicked.connect(functools.partial(self.setPose,[0.622,1.119,-0.069,1.125]))
        #self.ui.btnUser5.clicked.connect(functools.partial(self.setPose,[0,0,0,0]))

        #Robot frame points. Index 3 is phi, the grasping angle with respect to the world frame
        point = [0,0.44,0, 90 * D2R]
        #Regular rexarm position
        point = [0.002,0.189,-0.056,(90+72)*D2R]
        point = [0.004,0.388,-0.008,(90)*D2R]
        point = [-0.002,0.361,0,90 * D2R]
        point = [0,0.24,-0,02,90*D2R]
        point = [0.14,0.04,0.14,90*D2R]
        #Test Case
        point = [.22,0,.22,18*D2R]
        #Different test cases
        point = [.12,0.22,0.1,90*D2R]
        point = [-0.03,-0.17,0.074,90*D2R]
        #Testing a pick up position (Waterbottle)
        point = [0.039,-0.002,0.35,37*D2R]
        point = [0.18,0,0.294,(352 * D2R)]
        point = [0.131,0.139,-0.015, 87 * D2R]
        point = [0.184,0.202,-0.02,38 * D2R]
        self.ui.btnUser6.setText("IK on " + str(point))
        self.ui.btnUser6.clicked.connect(functools.partial(self.runIK,point))

        #point = [0,0.05,0.082, 90 * D2R]
        #self.ui.btnUser7.setText("IK on " + str(point))
        #self.ui.btnUser7.clicked.connect(functools.partial(self.runIK,np.transpose(point)))

        self.ui.btnUser3.setText("Straight Position")
        self.ui.btnUser3.clicked.connect(functools.partial(self.setPose,[0,0,0,0,0,0]))

        self.ui.btnUser11.setText("Recall Position")
        self.ui.btnUser11.clicked.connect(self.recall_pose)

        self.ui.btnUser12.setText("Save Position")
        self.ui.btnUser12.clicked.connect(self.save_pose)

        """ Commands the arm as the arm initialize to 0,0,0,0 angles """
        self.sliderChange(0) 
        
        """ Connect Buttons to Functions 
        LAB TASK: NAME AND CONNECT BUTTONS AS NEEDED
        """
        self.ui.btnUser1.setText("Affine Calibration")
        self.ui.btnUser1.clicked.connect(self.affine_cal)

    #Holds the current rexarm position when torque has been set to zero
    def save_pose(self):
        self.saved_angles = self.rex.joint_angles_fb[:]

    def recall_pose(self):
        self.setPose(self.saved_angles)


    #Takes a list of [x,y,z] in kinect coordinates
    def kinect_world_to_rexarm_world(self,kinect_coords):
        x = kinect_coords[0]
        y = kinect_coords[1]
        z = -kinect_coords[2]

        #produce 
        rot_angle = -30 * D2R
        rot_x = np.array([
            [1,0,0,0],
            [0,np.cos(rot_angle),-np.sin(rot_angle),0],
            [0,np.sin(rot_angle),np.cos(rot_angle),0],
            [0,0,0,1]
        ])

        rot_angle = 90 * D2R
        rot_z = np.array([
            [np.cos(rot_angle),-np.sin(rot_angle),0,0],
            [np.sin(rot_angle),np.cos(rot_angle),0,0],
            [0,0,1,0],
            [0,0,0,1]
        ])

        translation = np.array([
            [1,0,0,.072],
            [0,1,0,0],
            [0,0,1,-.625],
            [0,0,0,1]
        ])

        inv_xform = np.dot(np.dot(rot_x,rot_z),translation)

        #Invert the matrix produced so we can go from kinect coordinates to rexarm coordinates
        xform = numpy.linalg.inv(inv_xform)
        coords = np.array([[x],[y],[z],[1]])

        rex_coords = np.dot(xform,coords)

        #Clamp z if below this limit
        z_limit = -.03
        if rex_coords[2][0] <= z_limit:
            print "z_limit was", rex_coords[2][0],"; clamped it to", z_limit
            rex_coords[2][0] = z_limit

        return [rex_coords[0][0],rex_coords[1][0], rex_coords[2][0]]

    #Runs inverse kinematics and only returns hardware thetas. Up to the
    #user to set the pose
    def runIK_noCommand(self,xyz_phi_world):
        #In the robot frame
        xyz_world = xyz_phi_world[0:3]
        phi_world = xyz_phi_world[3]

        #In the rexarm frame
        xyz_rexarm = None #To be computed

        #print "World Coords:", xyz_world

        #Homogeneous coordinates
        xyz_world = np.append(xyz_world,1)
        xyz_world = xyz_world.reshape(4,1)

        #print "Shape:", xyz.shape
        #Convert xyz to rexarm coordinates (with respect to frame of point right below joint 1) by using inverse matrix

        #shift_horizontal_only = self.rex.trans_base.copy()
        #Don't shift along z axis. Only shift along x
        #shift_horizontal_only[2][3] = 0
        #import pdb
        #pdb.set_trace()

        #transformation = np.dot(np.dot(np.dot(self.rex.trans_magicbase,self.rex.rot_72),self.rex.rot_90),shift_horizontal_only)
        #inv_transformation = np.linalg.inv(transformation)

        #Convert desired IK coordinates from world frame to robot frame
        #xyz_rexarm = np.dot(inv_transformation,xyz_world)
        xyz_rexarm = xyz_world.copy()
        xyz_rexarm = xyz_rexarm[0:3]

        print "Rexarm Frame Coords:", xyz_rexarm

        phi_rexarm = phi_world# - 72 * D2R #Converts grasping angle in world frame to angle in rexarm frame.
        xyz_rexarm = np.append(xyz_rexarm,phi_rexarm)

        xyz_rexarm = xyz_rexarm.reshape(4,1)

        #import pdb
        #pdb.set_trace()

        #Run Inverse kinematics
        DH_thetas = self.rex.rexarm_IK(xyz_rexarm,1)

        #Send arm_thetas to the rexarm
        #First convert from DH parameter angle to hardware servo angle
        hardware_thetas = list(DH_thetas.copy())
        for i in range(len(hardware_thetas)):
            hardware_thetas[i] -= self.rex.joint_offsets[i]

        # Append 0,0 for now for the remaining two joints
        hardware_thetas.append(0)
        hardware_thetas.append(0)

        return hardware_thetas


    #Runs inverse kinematics on xyz_phi_world, which has form
    #[x,y,z,phi_in_radians]
    def runIK(self,xyz_phi_world):
        hardware_thetas = self.runIK_noCommand(xyz_phi_world)
        print "Send thetas:", hardware_thetas

        #Send pose to Rexarm
        self.setPose(hardware_thetas)


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
            #print "No frame"
            pass
        
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

    # angles is a list of floats, an angle for each joint
    #Use this with state machine since lcm feedback handler isn't working.
    #current_angles is provided by Rexarm state machine since rex.joint_angles_fb
    #isn't being updated with the rexarm lcm handler
    #Returns the new pose as a variable
    def setPose(self,desired_angles,current_angles = []):
        #Use the linear motion plan that the professor explained
        step_size = 0.04
        t_range = list(np.arange(0,1 + step_size,step_size))

        desired_angles = np.array(desired_angles)
        
        initial_angles = None
        if len(current_angles) == 0:
            initial_angles = np.array(self.rex.joint_angles_fb)
        else:
            initial_angles = np.array(current_angles)

        for t in t_range:
            new_pose = desired_angles * t + initial_angles * (1-t)
            #Sends motor command to rexarm
            for i in range(len(new_pose)):
                #Obey joint_limits to prevent breaking motors
                if new_pose[i] < self.rex.joint_limits[i][0]:
                    new_pose[i] = self.rex.joint_limits[i][0]
                if new_pose[i] > self.rex.joint_limits[i][1]:
                    new_pose[i] = self.rex.joint_limits[i][1]                    

                self.rex.joint_angles[i] = new_pose[i]
            self.rex.cmd_publish()
            time.sleep(0.1)

        return desired_angles

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
 
    def init_socket(self):
        self.sock = socket.socket(socket.AF_UNIX, # Local computer
                         socket.SOCK_DGRAM) # UDP
        self.kinect_path = "/tmp/rexarm_endpoint"

        try:
            os.remove(self.kinect_path)
        except OSError:
            pass
        self.sock.bind(self.kinect_path)
        print "Socket binded. Ready to receive data."
        # Don't need this?
        #self.sock.listen(1)

        #Blocks until Kinect code connects
        #self.conn, self.addr = self.sock.accept()

    def get_socket_data(self):
        point = None

        #Grasping Point struct is 28 bytes
        while True:
            print "Waiting for data..."
            data,addr = self.sock.recvfrom(28)
            print "Received 28 bytes"
            if not data:
                #Error
                pass
            #p is point. n is normal
            #TODO: Check endianness
            time, p1,p2,p3,n1,n2,n3 = struct.unpack("iffffff", data)
            print "Time:", time
            print "Point (mm):", p1,p2,p3
            print "Normal (mm):", n1,n2,n3
            point = [p1/1000.0,p2/1000.0,p3/1000.0]
            break
        return point


    #Busy waits code until rexarm has reached desired pose 
    def wait_until_reached(self,pose):
        while not self.reached_pose(pose):
            #print "not reached pose yet"
            #print "Pose:", pose
            #print "Rexarm:", self.rex.joint_angles_fb
            continue

    #Instantly publishes pose to rexarm without any motion smoothing
    #Used to save time
    def instant_publish(self,pose):
        self.rex.joint_angles = pose[:]
        self.rex.cmd_publish()

    #Returns true if the rexarm's angles match the pose input approximately.
    #Use this to repeatedly check if rexarm has reached a configuration
    # pose is a list of angles for each rexarm_joint ex. [0,0,0,0,0,0]
    def reached_pose(self,pose):
        reached = True
        allowed_error = 0.025 #radians
        for i in range(len(self.rex.joint_angles_fb)):
            #If error for any of the joints is > 0.01, then arm is not
            #at the desired location.
            if abs(self.rex.joint_angles_fb[i] - pose[i]) > allowed_error:
                reached = False
                break
        return reached

    def trash_state_machine(self):
        self.init_socket()
        tighten_gripper = 115 * D2R
        net_base_angle = -1.71 
        poses = {"HOME": [0,0,0,0,0,tighten_gripper],#Tightens gripper
                 "HIDE_INTERMEDIATE": [0.008,-2.038,0.171,1.30,-0.015,-0.015],
                 "HIDE": [1.557,-2.03,-0.629,1.079,-0.061,1.994]#,
                 #"NET_ARCH": [net_base_angle,-0.135,-1.223,-1.447,-0.061,tighten_gripper]
        }

        #TODO: populate this variable with the pose we're currently trying to reach
        #Use this since LCM feedback handler isn't being called :(
        current_pose = None
        next_pose = None
        
        desired_IK = []
        IK_cmd_thetas = None

        #State1: Turn 90 degrees at base to prevent collision
        states = ["START","RUN_IK_TURN_BASE","RUN_IK_DESCEND", "GRASP", "LIFT_UP", "TURN_TO_NET", "ARCH_TO_NET", "DROP", "UNARCH", "TURN_TO_HOME_FROM_NET", "HIDE_POSITION", "UNHIDE","TURN_TO_HOME_FROM_UNHIDE"]
        curr_state = "START"
        synchro_timer = 0.5
        start = True
        linear = True

        while True:
            print "----------------------------------------"
            print "Current State:", curr_state
            if curr_state == "START":
                next_pose = poses["HOME"][:]
                next_state = "HIDE_POSITION"
            elif curr_state == "RUN_IK_TURN_BASE":
                #Run_IK
                IK_cmd_thetas = self.runIK_noCommand(desired_IK)
                print "IK_result:", IK_cmd_thetas
                #Turn base towards object
                next_pose[0] = IK_cmd_thetas[0]
                linear = False
                self.instant_publish(next_pose)
                print "Published to joint 0:", IK_cmd_thetas[0]
                #TODO: Get lcm joint angles returned from runIK so that
                #we can wait before grasping
                next_state = "RUN_IK_DESCEND"
            elif curr_state == "RUN_IK_DESCEND":
                print "About to descend:"
                print "Current pose:", self.rex.joint_angles_fb
                print "Desired Pose:", IK_cmd_thetas
                #Descend the rest of the IK outside of base
                next_pose = IK_cmd_thetas[:]
                next_state = "GRASP"
            elif curr_state == "GRASP":
                #Set joint 5 to grasp
                next_pose[5] = tighten_gripper
                next_state = "LIFT_TO_HOME"
            elif curr_state == "LIFT_TO_HOME":
                #Set all joints except base joint and gripper joint to 0
                for i in range(1,5):
                    next_pose[i] = 0
                next_state = "TURN_TO_NET"
            elif curr_state == "TURN_TO_NET":
                next_pose[0] = net_base_angle
                linear = False
                self.instant_publish(next_pose)
                next_state = "ARCH_TO_NET"
            elif curr_state == "ARCH_TO_NET":
                next_pose[1] = -0.08
                next_pose[2] = -0.97
                next_pose[3] = -1.58
                next_state = "DROP"
            elif curr_state == "DROP":
                #Set joint 5 to 0 angle
                next_pose[5] = 0
                linear = False
                self.instant_publish(next_pose)
                next_state = "UNARCH"
            elif curr_state == "UNARCH":
                #Set all joints except base to 0
                for i in range(1,6):
                    next_pose[i] = 0
                next_state = "TURN_TO_HOME_FROM_NET"
            elif curr_state == "TURN_TO_HOME_FROM_NET":
                #Set all joints to 0
                next_pose[0] = 0
                linear = False
                self.instant_publish(next_pose)
                next_state = "HIDE_POSITION"
            elif curr_state == "UNHIDE":
                #Go to home position. Then run IK
                next_pose = poses["HOME"][:]
                next_state = "RUN_IK_TURN_BASE"
            elif curr_state == "HIDE_POSITION":
                next_pose = poses["HIDE"][:]
                next_state = "SOCKET_READ"
            elif curr_state == "SOCKET_READ":
                #Block and wait for next point of new object
                kin_point = self.get_socket_data()
                #kin_point = [.057,-.165,.731]
                #Convert to rexarm coordinates from kinect coordinates
                rex_point = self.kinect_world_to_rexarm_world(kin_point)
                #TODO: Do matrix transformation from kinect to rexarm world
                #and populate desired_IK
                desired_IK = [rex_point[0],rex_point[1],rex_point[2], 87 *D2R]
                #desired_IK = [0.131,0.139,-0.015, 87 * D2R]
                #desired_IK = [0.15,0.1,0.05, 87 * D2R]
                print "Inverse Kinematics Target:"
                print "Goal x in Rexarm:", desired_IK[0]
                print "Goal y in Rexarm:", desired_IK[1]
                print "Goal z:", desired_IK[2]
                linear = False
                #Not setting next_pose
                next_state = "UNHIDE"
            if linear:
                if start:
                    self.setPose(next_pose)
                    start = False
                else:
                    self.setPose(next_pose,current_pose)
            else:
                linear = True

            #Setting current_pose to whatever next_pose was 
            #determined to be
            current_pose = next_pose[:]
            time.sleep(synchro_timer)

            print "Next State:", next_state
            print "----------------------------------------"
            curr_state = next_state

def main():
    app = QtGui.QApplication(sys.argv)
    ex = Gui()
    #Successfully uses socket to listen for data
    #ex.init_socket()
    #ex.get_socket_data()
    #Put these back when not testing socket anymore
    #import pdb
    #pdb.set_trace()
    #ex.show()
    ex.trash_state_machine()
    sys.exit(app.exec_())
    """
    """
if __name__ == '__main__':
    main()
