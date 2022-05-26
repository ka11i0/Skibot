from hal import AngleCom
from common import *

a_ctrl = AngleCom()
i = 0
while(True):
    for i in range(0, 180):
        a_ctrl.send_angles([i, i, i, i])
        time.sleep(0.01)
    for i in range(180, 0, -1):
        a_ctrl.send_angles([i, i, i, i])
        time.sleep(0.01)