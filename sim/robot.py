from common import *
import numpy as np
from math import sin, cos, radians
import logging
import pychrono as chrono

logger = logging.getLogger(args.log)


class MotorTorqueControl(chrono.ChFunction_SetpointCallback):
    """
    A class used to represent the actuation of a digital servo

    ...

    Attributes
    ----------
    stall_torque : float
        torque for which the motor stops
    speed : float
        the operating speed of the motor
    init_angle : float
        the initial angle of the motor
    motor : chrono.ChMotorLink
        the motor object

    Methods
    -------
    SetpointCallback(x)
        Calculates the momentary angle of the motor as a function of time, error and speed.

    Setref(ref)
        Sets the reference angle.
    """
    def __init__(self, name, stall_torque, pid, speed, init_angle, motor):
        """
        Parameters
        ----------
        name : str
            for debugging
        stall_torque : float
            torque for which the motor stops (Nm)
        speed : float
            the operating speed of the motor (rad/s)
        init_angle : float
            the initial angle of the motor (rad)
        motor : chrono.ChLinkMotorRotationSpeed
            the motor object
        """
        chrono.ChFunction_SetpointCallback.__init__(self)
        self.name = name
        self.stall_torque = stall_torque
        self.error = [0, 0, 0]
        self.pid = pid
        self.speed = speed
        self.motor = motor
        self.angle = init_angle
        self.ref_angle = init_angle
        self.last_time = 0.0

    def SetpointCallback(self, x):
        """Calculates the momentary angle of the motor as a function of time, error and speed.

        Parameters
        ----------
        x : float
            x = time variable of the simulation

        Returns
        -------
        float
            the calculated momentary angle of the motor
        """
        time = x
        dt = time - self.last_time
        self.error[0] = self.ref_angle - self.motor.GetMotorRotPeriodic() # P-error
        self.error[1] += self.error[0]*dt # I-error
        self.error[2] = (self.error[0] - self.error[2]) # D-error
        self.last_time = time
        torque = self.error[0]*self.pid[0] + self.error[1]*self.pid[1] + self.error[2]*self.pid[2]
        logger.debug("error: "+str(self.error[0]))
        logger.debug("torque: "+str(torque))
        torque = cutoff(maximum=self.stall_torque, val=torque)
        return torque


    def SetRef(self, ref):
        """Sets the reference angle.

        Parameters
        ----------
        ref : float
            ref = refrence angle (rad)
        """
        self.ref_angle = ref

class MotorControl(chrono.ChFunction_SetpointCallback):
    """
    A class used to represent the actuation of a digital servo

    ...

    Attributes
    ----------
    stall_torque : float
        torque for which the motor stops
    speed : float
        the operating speed of the motor
    init_angle : float
        the initial angle of the motor
    motor : chrono.ChMotorLink
        the motor object

    Methods
    -------
    SetpointCallback(x)
        Calculates the momentary angle of the motor as a function of time, error and speed.

    Setref(ref)
        Sets the reference angle.
    """
    def __init__(self, name, stall_torque, speed, init_angle, motor):
        """
        Parameters
        ----------
        name : str
            for debugging
        stall_torque : float
            torque for which the motor stops (Nm)
        speed : float
            the operating speed of the motor (rad/s)
        init_angle : float
            the initial angle of the motor (rad)
        motor : chrono.ChLinkMotorRotationSpeed
            the motor object
        """
        chrono.ChFunction_SetpointCallback.__init__(self)
        self.name = name
        self.stall_torque = stall_torque
        self.speed = speed
        self.motor = motor
        self.angle = init_angle
        self.ref_angle = init_angle
        self.last_time = 0.0


    def SetpointCallback(self, x):
        """Calculates the momentary angle of the motor as a function of time, error and speed.

        Parameters
        ----------
        x : float
            x = time variable of the simulation

        Returns
        -------
        float
            the calculated momentary angle of the motor
        """
        time = x
        if (time > self.last_time):
            dt = time - self.last_time
            error = self.ref_angle - self.motor.GetMotorRot()
            angle_dt = cutoff(error, self.speed*dt)
            logger.debug(self.name+"_motor_error:%20s",str(error))
            logger.debug(self.name+"_motor_angle_dt:%20s",str(angle_dt))
            if error>0:
                self.angle += angle_dt
            else:
                self.angle -= angle_dt
            self.last_time = time
        return self.angle


    def SetRef(self, ref):
        """Sets the reference angle.

        Parameters
        ----------
        ref : float
            ref = refrence angle (rad)
        """
        self.ref_angle = ref

class SpringDynamics(chrono.ChFunction_SetpointCallback):
    """
    A class used to represent the dynamics of the limb-pole spring

    ...

    Attributes
    ----------
    k : float
        torque for which the motor stops
    angle : float
        limb-pole angle (rad)
    motor : chrono.ChLinkMotorRotationTorque
            motor link acting as spring

    Methods
    -------
    SetpointCallback(x)
        Calculates the angle 

    """
    def __init__(self, name, k, init_angle, stop_angle, lev_len, motor, pole_contact, lock):
        """
        Parameters
        ----------
        name : str
            for debugging
        k : float
            spring stiffness constant
        angle : float (rad)
            limb-pole angle 
        stop_angle : float (rad)
            angle for which the spring is in its retracted position
        lev_len : float (m)
            distance forming the lever arm vector
        motor : chrono.ChLinkMotorRotationTorque
            motor link acting as spring
        limb : 
        """
        chrono.ChFunction_SetpointCallback.__init__(self)
        self.name = name
        self.k = k
        self.angle = init_angle
        self.stop_angle = stop_angle
        self.lev_len = lev_len
        self.motor = motor
        self.pole_contact = pole_contact
        self.link = lock

    def SetpointCallback(self, x):
        """Calculates the momentary torque of the motor as a function of k and angle.

        Parameters
        ----------
        x : float
            x = time variable of the simulation

        Returns
        -------
        float
            the calculated momentary torque of the motor
        """
        self.angle = chrono.CH_C_PI - self.motor.GetMotorRot()
        logger.debug(self.name+"_pole-limb_angle:%-12s",str(self.angle))
        dx = self.lev_len * 2 * sin(self.angle/2)
        F = dx*self.k
        Ft = abs(cos(self.angle/2) * F)
        torque = Ft*self.lev_len
        logger.debug(self.name+"_spring_torque:%-12s",str(torque))
        if(self.angle<=self.stop_angle and self.pole_contact.GetPos().y>-0.89):# and self.pole_contact.GetContact()):
            print("haha")
            self.link.Lock(True)
            logger.debug(self.angle)
            logger.debug("stop angle reached")
            return 0.0
        else:
            print("hihi")
            self.link.Lock(False)
            return 0.0


    

class ArmController:
    """
    A class for controlling the motors of a single arm.

    ...

    Attributes
    ----------
    motors : list[motor]
        torque for which the motor stops
    margin : float
        margin of error before the 
    schedule : list
        safd
    schedule_index : int
        current schedule index
    running : bool
        

    Methods
    -------
    run()
        sets running bool to True.
    stop()
        sets running bool to False.
    reset()
        sets running bool to False and resets the motor reference angles to schedule initial value.
    loop_check()
        checks if margin has been reached for both motors then popagates to next scheduled angle.
    propagate()
        sets motor controller refrence to next scheduled position.
    calcError()
        calculates and returns refrence angle error for a single motor.
    """
    def __init__(self, motors, margin, schedule):
        self.motors = motors
        self.margin = margin
        self.schedule = schedule
        self.schedule_index = 0
        self.running = False
        
    
    def run(self):
        """sets running bool to True."""
        self.running = True
    
    def stop(self):
        """sets running bool to False."""
        self.running = False
        for motor_c in self.motors:
            motor_c.SetRef(self.motor.GetMotorRot())

    def reset(self):
        """sets running bool to False and resets the motor reference angles to schedule initial value."""
        self.running = False
        for i, motor_c in enumerate(self.motors):
            motor_c.SetRef(self.schedule[i][0])
            self.schedule_index = 0
    
    def loop_check(self):
        """checks if margin has been reached for both motors then popagates to next scheduled angle."""
        if(self.running):
            if abs( self.calcError(self.motors[0]) ) < self.margin and abs( self.calcError(self.motors[1]) ) < self.margin:
                self.propagate()
    
    def propagate(self):
        """sets motor controller refrence to next scheduled position."""
        self.schedule_index += 1
        if(self.schedule_index == self.schedule[0].size):
            self.schedule_index = 0
        for i, motor_c in enumerate(self.motors):
            motor_c.SetRef( radians(self.schedule[i][self.schedule_index]) )
        
    
    def calcError(self, motor_c):
        """calculates and returns refrence angle error for a single motor."""
        return motor_c.ref_angle - motor_c.motor.GetMotorRot()

      