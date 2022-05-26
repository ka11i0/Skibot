from turtle import position
from hal import AngleCom, AdcReader
from math import degrees, acos, asin, atan, sin, cos
from copy import copy
import multiprocessing as mp
import time
from common import *


class AngleController:
    def __init__(self, winsize):
        """
        Lower level controller which checks 

        ...

        Attributes
        ----------
        angle_com : AngleCom()
            object which communicates with the pwm microcontroller
        adc_reader : AdcReader()
            object which reads incoming
        adc_process : mp.Process()
            background process of adc_reader
        adc_mapping : int[[]]
            adc mapping arrays for all the motors

        Methods
        -------
        set_angles(angles)
            Transforms calculated model angles to motor angles and sends to adc_com

        get_adc()
            Gets latest read adc from adc_reader

        get_angles()
            Translates ADC readings to angles using saved adc mapping for the respective motors

        create_mapping()
            ADC mapping procedure for all motors

        find_map()
            Helper function that moves the motors between to angles and saves their respective adc values

        save_mapping()
            Saves mapping into csv

        import_mapping()
            Imports saved mapping from csv

        """
        self.angle_com = AngleCom()
        self.adc_reader = AdcReader(winsize)
        self.adc_process = mp.Process(target=self.adc_reader.run).start()
        self.adc_mapping = []
        
    def set_angles(self, angles):
        """Translates desired motor angles to right and left arm motors and sends to angle_com

        Parameters
        ----------
        angles : int[]
            angles that will be set

        Returns
        -------
        none
            none
        """
        t_angles = [0,0,0,0]
        # right motors 0 and 1
        # ardmap io values were manually calibrated for each motor,
        #  as their pw values did not match their real angles
        t_angles[0] = 90 - ardmap(angles[0], (0,180,0,170) )
        t_angles[1] = 180 - ardmap(angles[1], (0,180,8,180) )
        # left motors 0[2] and 1[3]
        t_angles[2] = 90 + ardmap(angles[2], (0,180,5,180) )
        t_angles[3] = ardmap(angles[3], (0,180,0,167) )
        self.angle_com.send_angles(t_angles)

    
    def get_adc(self):
        """ Reads ADC values from the adc_reader process

        Parameters
        ----------
        none

        Returns
        -------
        vals : int[]
            ADC values
        """
        with self.adc_reader.adc.get_lock():
            vals = self.adc_reader.adc.get_obj()[0:4].copy()
        return vals

    
    def get_angles(self):
        """ Translates ADC readings to angles using saved adc mapping for the respective motors

        Parameters
        ----------
        none

        Returns
        -------
        vals : float[]
            motor angles
        """
        vals = self.get_adc()
        angles = [0,0,0,0]
        for i, val in enumerate(vals):
            angles[i] = ardmap(val, self.adc_mapping[i])
        logger.debug("m_angles: "+str(angles))
        return angles
        

    def create_mapping(self):
        """ ADC mapping procedure for all motors

        Parameters
        ----------
        none

        Returns
        -------
        none

        """
        position = [0, 0, 0, 0]
        self.set_angles(position)

        m0 = self.find_map(-90, 90, 0, position)
        # right and left m0 motor 
        rm0 = (m0[0], m0[1], -90, 90)
        lm0 = (m0[2], m0[3], -90, 90)
        time.sleep(1)
        m1 = self.find_map(0, 137, 1, position)
        rm1 = (m1[0], m1[1], 0, 137)
        lm1 = (m1[2], m1[3], 0, 137)
        for m in [rm0, rm1, lm0, lm1]:
            self.adc_mapping.append(m)
        self.save_mapping()
        

    def find_map(self, a, b, motors_i, position):
        """ ADC mapping procedure for all motors

        Parameters
        ----------
        a : int
            starting angle
        b : int
            ending angle
        motors_i : int
            theta_0 motors or theta_1 motors.
        position : int
            to save initial position

        Returns
        -------
        m : int[]
            mapping (adc_minright, adc_maxright, adc_minleft, adc_maxleft)
        
        """
        initial_angles = position.copy()
        m = [0,0,0,0]
        t = 0.02
        # find lower limit
        while(position[motors_i]>a):
            position[motors_i] -= 1 
            position[motors_i+2] -= 1 
            self.set_angles(position)
            time.sleep(t)
        time.sleep(1)
        vals = self.get_adc()
        m[0] = vals[motors_i]
        m[2] = vals[motors_i+2]

        # find upper limit
        while(position[motors_i]<b):
            position[motors_i] += 1 
            position[motors_i+2] += 1 
            self.set_angles(position)
            time.sleep(t)
        time.sleep(1)
        vals = self.get_adc()
        m[1] = vals[motors_i]
        m[3] = vals[motors_i+2]

        # return to initial position
        while(position[motors_i]>initial_angles[motors_i]):
            position[motors_i] -= 1 
            position[motors_i+2] -= 1 
            self.set_angles(position)
            time.sleep(t)
        return m


    def save_mapping(self):
        """ Saves mapping into csv

        Parameters
        ----------
        none

        Returns
        -------
        none

        """
        file1 = open("adc_mapping.txt", "wb") 
        pickle.dump(self.adc_mapping, file1)
        file1.close
        logger.info("Map saved: "+str(self.adc_mapping))


    def import_mapping(self):
        """ Imports saved mapping from csv

        Parameters
        ----------
        none

        Returns
        -------
        none

        """
        with open('adc_mapping.txt', 'rb') as f:
            self.adc_mapping = pickle.load(f)
        logger.info("Map loaded: "+str(self.adc_mapping))
        

