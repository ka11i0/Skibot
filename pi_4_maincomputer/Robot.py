from math import sin, cos, tan, sqrt, pi, acos, atan, asin
from common import *

class Robot:
    def __init__(self, l_L, l_B, l_P, alpha):
        """
        Robot class is a model of the robot and handles all the kinematics

        ...

        Attributes
        ----------
        l_L : float
            limb length 
        l_B : float
            body length
        l_P : float
            pole length
        alpha : float
            resting angle between pole and lower limb
        l_E : float
            length between elbow and pole contact point
        l_C : float
            length between shoulder and pole contact point

        Methods
        -------
        calc_hand_pos(angles)
            Calculates the position of the hand from theta_0 and theta_1.
        calc_poleend_pos(angles)
            Calculates the pole end position from theta_0, theta_1 and theta_2.
        calc_arm_angles(pos)
            Arm inverse kinematics from (x,y) pos of hand end-effector.
        calc_lC(theta_1)
            Calculates the length between pole end and shoulder from theta_1
        calc_l_E()
            Calculates elbow to pole contact length when pole is in its resting position

        """
        self.l_L = l_L
        self.l_B = l_B
        self.l_P = l_P
        self.alpha = alpha
        self.l_E = self.calc_l_E()
        self.l_C = self.calc_lC(0) # made up initial theta_1
        

    def calc_hand_pos(self, angles):
        """Calculates the position of the hand from theta_0 and theta_1.

        Parameters
        ----------
        angles : float
            (theta_0, theta_1)

        Returns
        -------
        (x,y) : (float, float)
            hand xy position
        """
        x = self.l_L*(sin(angles[0]) + sin(angles[0]+angles[1]))
        y = self.l_B - self.l_L*(cos(angles[0]) + cos(angles[0]+angles[1]))
        return (x, y)

    def calc_poleend_pos(self, angles):
        """Calculates the pole end position from theta_0, theta_1 and theta_2.

        Parameters
        ----------
        angles : float
            (theta_0, theta_1, theta_2)

        Returns
        -------
        (x,y) : (float, float)
            pole end xy position
        """
        pcx = self.l_L*(sin(angles[0]) + sin(angles[0]+angles[1])) + self.l_P*sin(angles[0]+angles[1]+angles[2])
        pcy = self.l_B - self.l_L*(cos(angles[0]) - cos(angles[0]+angles[1])) - self.l_P*cos(angles[0]+angles[1]+angles[2])
        return (pcx, pcy)

    def calc_arm_angles(self, pos):
        """Arm inverse kinematics from (x,y) pos of hand end-effector.

        Parameters
        ----------
        pos : float
            (x,y)

        Returns
        -------
        theta_0,theta_1 : (float, float)
        """
        x = pos[0]
        y = pos[1]
        theta_1 = pi - acos( sat(-1,( 2*self.l_L**2-(x**2+(self.l_B-y)**2) ) / (2*self.l_L**2)) )
        if((self.l_B-y)<0):
            t = atan( (x) / (self.l_B-y) ) + pi
        else:
            t = atan( (x) / (self.l_B-y) )
        theta_0 = t-atan( (self.l_L*sin(theta_1)) / (self.l_L+self.l_L*cos(theta_1)) )
        return (theta_0, theta_1)

    def calc_lC(self, theta_1):
        """Calculates the length between pole end and shoulder from theta_1

        Parameters
        ----------
        theta_1 : float

        Returns
        -------
        l_C : float
        """
        theta_l_C = pi - acos( (self.l_E**2+self.l_L**2-self.l_P**2) / (2*self.l_E*self.l_L) ) - theta_1
        self.l_C = self.l_L**2+self.l_E**2-2*self.l_L*self.l_E*cos(theta_l_C)
        return self.l_C
    
    def calc_l_E(self):
        """Calculates elbow to pole contact length when pole is in its resting position

        Parameters
        ----------
        none

        Returns
        -------
        l_E : float
        """
        self.l_E = sqrt(self.l_L**2+self.l_P**2-2*self.l_L*self.l_P*cos(self.alpha))
        return self.l_E

    
    
