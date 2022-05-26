import serial
import time
import multiprocessing as mp
from common import *

class AdcReader:
    def __init__(self, window_size):
        """
        Class which handles the adc values UART communication.

        ...

        Attributes
        ----------
        window_size : int
            average window filtering window size
        adc_window : int[][]
            adc windows to average
        adc : mp.Array 
            multiprocess array to be used as accesspoint by external processes to the adc values
        com : serial.Serial
            Serial UART port

        Methods
        -------
        run()
            UART reading loop
        insert_raw(raw, i)
            Converts the raw serial readings to int values and inserts into adc window
        init_windows(size, n)
            Inits n averaging windows of size n

        """
        self.window_size = window_size
        self.adc_window = self.init_windows(window_size, 4)
        self.adc = mp.Array('i', 4)
        self.com = serial.Serial("/dev/serial0", 115200, timeout=1)

    def run(self):
        """UART reading loop

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        i = 0
        while(True):
            raw = self.com.read(10)
            if(raw[9]<<8 | raw[8] != 31354): #Sync solution
                time.sleep(0.01)
                self.com.reset_input_buffer()
                continue
            self.insert_raw(raw, i)
            with self.adc.get_lock():
                values = self.adc.get_obj()
                for j in range(4): #If in sync, set vals
                    values[j] = sum(self.adc_window[j])//self.window_size
                logger.debug("ADC"+str(values[0:4]))
            i = (i + 1)%self.window_size
            
        
    def insert_raw(self, raw, i):
        """UART reading loop

        Parameters
        ----------
        raw : bytes
            raw bytes readings from UART Port
        i : int
            windows index position

        Returns
        -------
        none
        """

        for j in range(4):
            self.adc_window[j][i] = (raw[j*2+1]<<8) | raw[j*2]
    
    def init_windows(self, size, n):
        """Inits n averaging windows of size n

        Parameters
        ----------
        size : int
            window size
        n : int
            n windows

        Returns
        -------
        windows = [[]]
            array of arraywindows
        """
        windows = []
        for i in range(0,n):
            sub_window = []
            for j in range(0,size):
                sub_window.append(0)
            windows.append(sub_window)
        return windows

                    

    
    
    


