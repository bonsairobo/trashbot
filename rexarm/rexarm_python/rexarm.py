import lcm
import time
import numpy as np
import math.pow as power

from lcmtypes import dynamixel_command_t
from lcmtypes import dynamixel_command_list_t
from lcmtypes import dynamixel_status_t
from lcmtypes import dynamixel_status_list_t

PI = np.pi
D2R = PI/180.0
R2D = 180.0/PI
ANGLE_TOL = 2*PI/180.0 

#joint_3_offset = 31 * D2R

#Represents a transformation matrix for a link computed via DH parameters
class DH_xform:
    def __init__(self,a,d,alph):
        self.a = a
        self.d = d
        self.alph = alph
        self.gen_const_xform()

    #Generates the constant xform matrix, given that link twist, link length, and
    #link offset are constant
    def gen_const_xform(self):
         self.link_twist = np.array([
             [1,0,0,0],
             [0,np.cos(self.alph),-np.sin(self.alph),0],
             [0,np.sin(self.alph),np.cos(self.alph),0],
             [0,0,0,1]
         ])

         self.link_length = np.array([
             [1,0,0,self.a],
             [0,1,0,0],
             [0,0,1,0],
             [0,0,0,1]
         ])

         self.link_offset = np.array([
             [1,0,0,0],
             [0,1,0,0],
             [0,0,1,self.d],
             [0,0,0,1]
         ])

         temp = np.dot(self.link_length,self.link_twist)
         self.const_xform = np.dot(self.link_offset,temp)

         # Don't have theta yet
         self.xform = None

    def gen_xform(self,theta):
        joint_angle = np.array([
            [np.cos(theta),-np.sin(theta),0,0],
            [np.sin(theta),np.cos(theta),0,0],
            [0,0,1,0],
            [0,0,0,1]
        ])

        self.theta = theta
        self.xform = np.dot(joint_angle,self.const_xform)


""" Rexarm Class """
class Rexarm():
    def __init__(self,x_out,y_out,z_out,theta_out):

        """ Commanded Values """
        self.num_joints = 6
        self.joint_angles = [0.0] * self.num_joints # radians
        # you must change this to control each joint speed separately 
        self.speed = 0.5                         # 0 to 1
        self.max_torque = 0.5                    # 0 to 1

        """ Link Lengths """
        #Index 0 is length for link 1
        #TODO: Insert the actual link lengths
        self.link_lengths = [.1,
                             .1,
                             .1,
                             .1]


        """ Joint Limits """
        self.joint_limits = [[-PI,PI], #Joint 0
                             [-2.00,2.00], #Joint 1
                             [-1.87,1.87], #Joint 2
                             [-1.82,2.51]] #Joint 3

        """ DH Table """
        self.DH_table = [DH_xform(0,.044,PI/2), #Joint 0's parameters (Used to form A_0). CORRECT
                         DH_xform(0.1,0,0), #Joint 1's parameters (Used to form A_1)
                         DH_xform(0.1,0,0), #Joint 3
                         DH_xform(0.11,0,0) #Joint 4
        ]

        """ Joint offsets from rexarm frame assignments for FK """
        self.joint_offsets = [0,
                              PI/2,
                              0,
                              0#TODO: Shift on a different rexarm
        ]
        
        """ References to GUI labels for FK """
        self.x_out = x_out
        self.y_out = y_out
        self.z_out = z_out
        self.theta_out = theta_out

        # Position of endeffector in frame of last link. Homogeneous
        # coordinates
        #self.endeffector_pos = np.transpose(np.array([[.108,0,0,1]]))
        self.endeffector_pos = np.transpose(np.array([[0,0,0,1]]))

        """ Feedback Values """
        self.joint_angles_fb = [0.0] * self.num_joints # radians
        self.speed_fb = [0.0] * self.num_joints        # 0 to 1   
        self.load_fb = [0.0] * self.num_joints         # -1 to 1  
        self.temp_fb = [0.0] * self.num_joints         # Celsius               

        """ Waypoint Plan - TO BE USED LATER """
        self.plan = []
        self.plan_status = 0
        self.wpt_number = 0
        self.wpt_total = 0

        """ Setup LCM and subscribe """
        self.lc = lcm.LCM()
        lcmMotorSub = self.lc.subscribe("ARM_STATUS",
                                        self.feedback_handler)

    def cmd_publish(self):
        """ 
        Publish the commands to the arm using LCM. 
        You need to activelly call this function to command the arm.
        You can uncomment the print statement to check commanded values.
        """    
        msg = dynamixel_command_list_t()
        msg.len = 6
        self.clamp()
        for i in range(msg.len):
            cmd = dynamixel_command_t()
            cmd.utime = int(time.time() * 1e6)
            cmd.position_radians = self.joint_angles[i]
            # you SHOULD change this to contorl each joint speed separately 
            cmd.speed = self.speed
            cmd.max_torque = self.max_torque
            #print cmd.position_radians
            msg.commands.append(cmd)
        self.lc.publish("ARM_COMMAND",msg.encode())
    
    def get_feedback(self):
        """
        LCM Handler function
        Called continuously from the GUI 
        """
        self.lc.handle_timeout(50)

    def feedback_handler(self, channel, data):
        """
        Feedback Handler for LCM
        """
        msg = dynamixel_status_list_t.decode(data)
        for i in range(msg.len):
            self.joint_angles_fb[i] = msg.statuses[i].position_radians 
            self.speed_fb[i] = msg.statuses[i].speed 
            self.load_fb[i] = msg.statuses[i].load 
            self.temp_fb[i] = msg.statuses[i].temperature

        """
        Compute forward kinematics
        """
        #Recompute DH Parameters. Add joint offset to account for fact that rexarm lcm angles
        #don't naturally correspond to the theta w/r/t the DH x axis frame assignment
        for i in range(len(self.DH_table)):
            self.DH_table[i].gen_xform(self.joint_angles_fb[i] + self.joint_offsets[i])

        #phi is angle with respect to horizontal
        world_pose,phi = self.rexarm_FK(self.DH_table,3)
        phi = (phi % (2 * PI))
        if phi > PI:
            phi -= 2 * PI

        #Update GUI with world_pose
        self.x_out.setText(str("%.3f" % world_pose[0][0]))
        self.y_out.setText(str("%.3f" % world_pose[1][0]))
        self.z_out.setText(str("%.3f" % world_pose[2][0]))
        self.theta_out.setText(str("%.3f" % (phi * R2D)))

    def clamp(self):
        """
        Clamp Function
        Limit the commanded joint angles to ones physically possible so the 
        arm is not damaged.
        LAB TASK: IMPLEMENT A CLAMP FUNCTION
        """
        for i in range(len(self.joint_angles)):
            if i in range(len(self.joint_limits)):
                if self.joint_angles[i] > self.joint_limits[i][1]:
                    self.joint_angles[i] = self.joint_limits[i][1]
                if self.joint_angles[i] < self.joint_limits[i][0]:
                    self.joint_angles[i] = self.joint_limits[i][0]
        return

    def plan_command(self):
        """ Command planned waypoints """
        pass

    # Link is an integer representing index in joints array. This function computes
    # FK for the endeffector in the frame of that link
    def rexarm_FK(self,dh_table, link):
        """
        Calculates forward kinematics for rexarm
        takes a DH table filled with DH parameters of the arm
        and the link to return the position for
        returns a 4-tuple (x, y, z, phi) representing the pose of the 
        desired link
        """

        #phi is angle of endeffector point w/r/t the vector <1,0> in world frame.
        phi = 0

        #First multiply by +60 degree rotation about z axis to align coordinates with the rexarm board
        rot_60 = np.array([[np.cos(PI/3),-np.sin(PI/3),0,0],
                           [np.sin(PI/3),np.cos(PI/3),0,0],
                           [0,0,1,0],
                           [0,0,0,1]
        ])

        #Then apply transformation to frame of the first motor
        trans_base = np.array([[1,0,0,0.013],
                               [0,1,0,0],
                               [0,0,1,0.066],
                               [0,0,0,1]
        ])

        #Multiply transformations
        final_xform = np.dot(rot_60,trans_base)

        #The world coordinates of the last motor
        init_point = []

        #The world coordinates of the endeffector position
        end_point = []

        #Multiply DH matrices
        #Compute phi by finding world vector between endeffector and motor joint 3.
        #Do this by calculating difference between these points' world coordinates
        for i in range(link + 1):
            temp = np.dot(final_xform,dh_table[i].xform)
            final_xform = temp.copy()
            if len(range(link+1)) > 1:
                if i == (link - 1):
                    init_point = np.dot(final_xform,np.transpose(np.array([[0,0,0,1]])))
            else:#Length = 1. Need to only multiply rot_60, trans_base, and [0,0,0,1]
                temp = np.dot(rot_60,trans_base)
                init_point = np.dot(temp,np.transpose(np.array([[0,0,0,1]])))

        #Multiply final_xform by endeffector position vector
        world_coords = np.dot(final_xform,self.endeffector_pos)

        init_point = init_point[0:3]
        end_point = (world_coords[0:3]).copy()

        #import pdb
        #pdb.set_trace()
        
        print "End Point:", end_point
        print "Initial Point:", init_point

        #Get vector to endeffector in global frame
        endeffector_vec = end_point - init_point
        print "Endeffector Vec:", endeffector_vec

        horizontal_vec = endeffector_vec.copy()
        #Simple projection onto XY plane
        horizontal_vec[2] = 0

        #Get angle with the the x,y plane. Same as angle with the same vector but with the z coordinate flattened to zero. phi is in range 0 to +180
        phi = np.arccos(np.dot(horizontal_vec.flatten(),endeffector_vec.flatten())/(np.linalg.norm(horizontal_vec) * np.linalg.norm(endeffector_vec)))
        #Make angle negative if endeffector_vec z coordinate is below the ground
        if endeffector_vec[2][0] < 0:
            phi = -phi

        #Remove homogeneous coordinate
        return end_point,phi

    def rexarm_IK(pose, cfg):
        """
        Calculates inverse kinematics for the rexarm
        pose is a tuple (x, y, z, phi) which describes the desired
        end effector position and orientation.  
        cfg describe elbow down (0) or elbow up (1) configuration
        returns a 4-tuple of joint angles or NONE if configuration is impossible
        """
        
        #Shorthand
        x = pose[0]
        y = pose[1]
        z = pose[2]
        phi = pose[3]

        #Shorthand
        l1 = self.link_lengths[0]
        l2 = self.link_lengths[1]
        l3 = self.link_lengths[2]
        l4 = self.link_lengths[3]

        theta1 = 0
        theta2 = 0
        theta3 = 0
        theta4 = 0

        #TODO: Check this because a tan returns angle between -pi/2 and pi/2
        #-------------------------------------------------------------------
        theta1 = math.atan(y/x)
        #-------------------------------------------------------------------

        zGoal = z
        rGoal = math.sqrt(x * x + y * y)

        zGoalp = zGoal + l4 * math.sin(phi)
        #TODO: Verify rGoalp + or -
        rGoalp = rGoal - l4 * math.cos(phi)

        delt_z = zGoalp - l1
        delf_r = rGoalp

        #TODO: Check range of acos
        #-------------------------------------------------------------------
        theta3 = math.acos((power(delt_z,2) + power(delt_r,2) - power(l2,2) - power(l3,2))/(2 * l2 * l3))
        #-------------------------------------------------------------------

        #Get beta and psi
        beta = math.atan(delt_z/delt_r)
        psi = math.acos(
            ( power(l3,2) - (power(delt_z,2) + power(delt_r,2)) - power(l2,2) )
                        /
            (-2 * math.sqrt(pow(delt_z,2) + pow(delt_r,2)) * l2)
        )

        #-------------------------------------------------------------------
        #Elbow up or elbow down
        #Elbow down
        if cfg == 0:
            theta2 = PI/2 - beta + psi
        #Elbow up
        else:
            theta2 = PI/2 - beta - psi
        #-------------------------------------------------------------------

        #-------------------------------------------------------------------
        theta4 = phi - theta2 - theta3 + PI/2
        #-------------------------------------------------------------------

        return [theta1,theta2,theta3,theta4]


    def rexarm_collision_check(q):
        """
        Perform a collision check with the ground and the base
        takes a 4-tuple of joint angles q
        returns true if no collision occurs
        """
        pass 
