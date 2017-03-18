import lcm
import time
import numpy as np

from lcmtypes import dynamixel_command_t
from lcmtypes import dynamixel_command_list_t
from lcmtypes import dynamixel_status_t
from lcmtypes import dynamixel_status_list_t

PI = np.pi
D2R = PI/180.0
ANGLE_TOL = 2*PI/180.0 

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

        self.xform = np.dot(joint_angle,self.const_xform)


""" Rexarm Class """
class Rexarm():
    def __init__(self):

        """ Commanded Values """
        self.num_joints = 6
        self.joint_angles = [0.0] * self.num_joints # radians
        # you must change this to control each joint speed separately 
        self.speed = 0.5                         # 0 to 1
        self.max_torque = 0.5                    # 0 to 1

        """ Joint Limits """
        self.joint_limits = [[-PI,PI], #Joint 0
                             [-2.00,2.00], #Joint 1
                             [-1.87,1.87], #Joint 2
                             [-1.4,2.51]] #Joint 3

        """ DH Table """
        self.DH_table = [DH_xform(-.013,.066,0), #Joint 0
                         DH_xform(0,.044,PI/2), #Joint 1
                         DH_xform(-0.1,0,0), #Joint 2
                         DH_xform(-0.1,0,0) #Joint 3
        ]


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
        #Recompute DH Parameters
        for i in range(len(self.joint_angles_fb)):
            self.DH_table[i].gen_xform(self.joint_angles_fb[i])

        rexarm_FK(self.DH_table,0)

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

    # Link is an integer represent index in joints array?
    def rexarm_FK(dh_table, link):
        """
        Calculates forward kinematics for rexarm
        takes a DH table filled with DH parameters of the arm
        and the link to return the position for
        returns a 4-tuple (x, y, z, phi) representing the pose of the 
        desired link
        """
        pass
    	
    def rexarm_IK(pose, cfg):
        """
        Calculates inverse kinematics for the rexarm
        pose is a tuple (x, y, z, phi) which describes the desired
        end effector position and orientation.  
        cfg describe elbow down (0) or elbow up (1) configuration
        returns a 4-tuple of joint angles or NONE if configuration is impossible
        """
        pass
        
    def rexarm_collision_check(q):
        """
        Perform a collision check with the ground and the base
        takes a 4-tuple of joint angles q
        returns true if no collision occurs
        """
        pass 
