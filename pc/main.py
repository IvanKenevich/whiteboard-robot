import logging
import time
import numpy as np
import modern_robotics as mr

from UI import Paint
from controls import Robot, MotorController

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(relativeCreated)6d %(threadName)s %(message)s', filename='log.txt', filemode='w')
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    console.setFormatter(logging.Formatter('%(relativeCreated)6d %(threadName)s %(message)s'))
    logging.getLogger('').addHandler(console)

    # mc = MotorController("COM7", 57600)
    # Paint("COM7", 57600)
    Paint(None, None)