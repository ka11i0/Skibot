from smbus2 import SMBus, i2c_msg
from common import *

class AngleCom:
    def __init__(self):
        """
        AngleCom handles the translation between desired angle to pulse width

        ...

        Attributes
        ----------
        devbus : int
            i2c bus to be used on Pi
        i2caddr : hex
            i2c address of raspberry Pico
        bus : SMBus
            i2c instance

        Methods
        -------
        send_angles(angles)
            Sends pulse width angles over the I2C bus to the designated address.
        convert_to_bytearray(angles)
            Converts angles to bytearray form to send over I2C.
        map_to_pw(angle)
            Maps angle 0-180 to pw 500-2500microseconds.

        """
        self.devbus = 1
        self.i2caddr = 0x08
        self.bus = SMBus(self.devbus)

    def send_angles(self, angles):
        """Sends pulse width angles over the I2C bus to the designated address.

        Parameters
        ----------
        angles : int[]

        Returns
        -------
        none
        """
        b_angles = self.convert_to_bytearray(angles)
        msg = i2c_msg.write(self.i2caddr, b_angles)
        self.bus.i2c_rdwr(msg)

    def convert_to_bytearray(self, angles):
        """Converts angles to bytearray form to send over I2C.

        Parameters
        ----------
        angles : int[]

        Returns
        -------
        none
        """
        b = bytearray(2*4) 
        for i, angle in enumerate(angles):
            pw = self.map_to_pw(angle).to_bytes(2, 'little')
            b[i*2] = b[i*2] | pw[0]
            b[i*2+1] = b[i*2+1] | pw[1]
        return b

    def map_to_pw(self, angle):
        """Maps angle 0-180 to pw 500-2500microseconds.

        Parameters
        ----------
        angle : int

        Returns
        -------
        pw : int
        """
        pw = ardmap(angle, (0, 180, 500, 2500))
        return round(pw)

