from math import sin, cos, acos, asin, pi, sqrt, degrees
from common import *
from copy import copy

class Trajectory:
    def __init__(self, start, end, robot, contact_speed, return_speed, acc, resting_height):
        """
        PosController is a modified position controller to achieve a desired arm movement pattern, using TrajGen, feedback AdcReader and AngleController

        ...

        Attributes
        ----------
        start : float
            desired pcx start
        end : float
            desired pcx end
        robot : Robot()
            robot model
        pos_curr : (float, float)
            current (x,y) position in the trajectory generation
        acc : float
            acceleration parameter which increases the contact_speed between each contact increment
        contact_speed : float
            distance between points during contact path generation
        return_speed : float
            angle return speed during return path generation
        resting_height : float
            height of which the resting point should be placed
        return_margin : float
            heigth of which the contact point should be at minimum during return
        xy_traj : (float,float)[]
            generated x y path
        angles : (float,float)[]
            path translated into angles
        pcx_data : float[]
            pcx positions during contact
        length : int
            amount of points in xy_traj and angles
        
        Methods
        -------
        calc_max()
        calc_min_end()
        calc_startcoords()
        generate_trajectory(param)
        generate_contact_path()
        calc_next(theta_3)
        calc_restpoint()
        generate_return_natural()
        generate_return_constraint(des_l_C)
        xy_traj_to_angles()




        """
        self.start = start
        self.end = end
        self.robot = robot
        self.pos_curr = (0.0, 0.0)
        self.acc = acc
        self.contact_speed = contact_speed
        self.return_speed = return_speed
        self.resting_height = resting_height
        self.return_margin = resting_height
        self.xy_traj = []
        self.angles = []
        self.pcx_data = []
        self.length = 0
    
    def calc_max(self):
        """Calculates maximum forward reach of robot model

        Parameters
        ----------
        none

        Returns
        -------
        pcx : float
            Maximum positive pcx 
        """
        robot = self.robot
        angle = acos(robot.l_B/(robot.l_L+robot.l_E))
        return (robot.l_E+robot.l_B)*sin(angle)
        

    def calc_min_end(self):
        robot = self.robot
        angle = acos(robot.l_B/(2*robot.l_L+robot.l_P))
        return (2*robot.l_L+robot.l_P)*sin(angle)
    

    def calc_startcoords(self):
        robot = self.robot
        robot.l_C = sqrt(robot.l_B**2+self.start**2)
        g1 = acos(self.start/robot.l_C)
        g2 = acos( (robot.l_C**2+robot.l_E**2-robot.l_L**2) / (2*robot.l_C*robot.l_E) )
        g3 = acos( (robot.l_P**2+robot.l_E**2-robot.l_L**2) / (2*robot.l_P*robot.l_E) )
        theta_3 = pi - g1 - g2 - g3
        x = self.start + robot.l_P*cos(theta_3)
        y = robot.l_P*sin(theta_3)
        self.pos_curr = (x, y)
        return 0

    def generate_trajectory(self, param):
        self.generate_contact_path()
        self.calc_restpoint()
        if param == 0:
            self.generate_return_natural()
        else:
            self.generate_return_constraint(self.robot.l_B-self.return_margin)
        self.xy_traj_to_angles()
        self.length = len(self.angles)
        logger.info("traj_len: "+str(self.length))
        return 0

    

    def generate_contact_path(self):
        self.calc_startcoords()
        done = False
        while(not done):
            #1
            self.xy_traj.append( (self.pos_curr[0], self.pos_curr[1]) )
            #2
            theta_3 = asin(self.pos_curr[1]/self.robot.l_P)
            pcx = self.pos_curr[0] - self.robot.l_P*cos(theta_3)
            self.pcx_data.append(pcx)
            pos_next = self.calc_next(theta_3)
            #3
            self.pos_curr = pos_next
            theta_3 = asin(self.pos_curr[1]/self.robot.l_P)
            pcx = self.pos_curr[0] - self.robot.l_P*cos(theta_3) #self.robot.l_P*sin(arm_angles[0]+arm_angles[1]+theta_2)
            #4
            #print(sqrt((self.pos_curr[0]-pcx)**2+self.pos_curr[1]**2))
            if(pcx<=self.end):
                break
            else:
                continue
        return 0
    
    def calc_next(self, theta_3):
        next_pos = (self.pos_curr[0]-self.contact_speed*cos(theta_3), self.pos_curr[1]-self.contact_speed*sin(theta_3))
        self.contact_speed = self.contact_speed*self.acc
        return next_pos
    
    def calc_restpoint(self):
        f_contact = self.xy_traj[0]
        s_contact = self.xy_traj[1]
        # Mirror vector of impact
        imp_v = (f_contact[0]-s_contact[0], f_contact[1]-s_contact[1])
        # Height vector
        height_v = (0, self.resting_height)
        # Find angle
        dot_p = imp_v[0]*height_v[0]+imp_v[1]*height_v[1]
        m_iv = sqrt(imp_v[0]**2+imp_v[1]**2)
        m_hv = sqrt(height_v[0]**2+height_v[1]**2)
        angle = acos(dot_p/(m_iv*m_hv))
        # Pythagoras
        c = self.resting_height/cos(angle)
        resting_point_dx = sin(angle)*c
        resting_point = (f_contact[0]+resting_point_dx, f_contact[1]+self.resting_height)
        self.xy_traj.insert(0,resting_point)
        return 0


    def generate_return_natural(self):
        # Start to end vector
        vect = (self.xy_traj[0][0]-self.xy_traj[len(self.xy_traj)-1][0], self.xy_traj[0][1]-self.xy_traj[len(self.xy_traj)-1][1])
        # Find angle between ground and start to end vector
        dot_p = vect[0]*1
        angle = acos(dot_p/sqrt(vect[0]**2+vect[1]**2))
        point = copy(self.xy_traj[len(self.xy_traj)-1])
        while(True):
            point = ( point[0]+self.return_speed*cos(angle), point[1]+self.return_speed*sin(angle) )
            if(point[0]<self.xy_traj[0][0]):
                self.xy_traj.append(point)
            else:
                break
        return 0
    
    def generate_return_constraint(self, des_l_C):
        r = self.robot
        theta_1 = r.calc_arm_angles(self.xy_traj[len(self.xy_traj)-1])[1]
        l_C_tmp = r.calc_lC(theta_1)
        # Check if l_C is already short enough, else set theta_1 to desired l_C
        if( l_C_tmp < des_l_C):
            theta_1 = theta_1
        else:
            theta_1 = pi - acos( (r.l_E**2+r.l_L**2-r.l_P**2) / (2*r.l_E*r.l_L) ) - acos( (r.l_E**2+r.l_L**2-des_l_C**2) / (2*r.l_E*r.l_L) )
        # Calculate max reach theta 0.
        max_reach_theta_0 = acos(r.l_B/(r.l_L+r.l_E))
        theta_0 = r.calc_arm_angles(self.xy_traj[len(self.xy_traj)-1])[0]
        while(theta_0<max_reach_theta_0*1.5):
            xy = r.calc_hand_pos((theta_0, theta_1))
            self.xy_traj.append(xy)
            theta_0 += self.return_speed
        rp_theta_angles = r.calc_arm_angles(self.xy_traj[0])
        m0_dir = sign(theta_0-rp_theta_angles[0])
        m1_dir = sign(theta_1-rp_theta_angles[1])
        while(sign(theta_0-rp_theta_angles[0])==m0_dir or sign(theta_1-rp_theta_angles[1])==m1_dir):
            if(sign(theta_0-rp_theta_angles[0])==m0_dir):
                theta_0 -= m0_dir*self.return_speed*0.5
            if(sign(theta_1-rp_theta_angles[1])==m1_dir):
                theta_1 -= m1_dir*self.return_speed
            xy = r.calc_hand_pos((theta_0, theta_1))
            self.xy_traj.append(xy)
        return 0

    
    def xy_traj_to_angles(self):
        for pos in self.xy_traj:
            arm_angles = self.robot.calc_arm_angles(pos)
            self.angles.append( (degrees(arm_angles[0]), degrees(arm_angles[1]) ) )
        return 0
    
    