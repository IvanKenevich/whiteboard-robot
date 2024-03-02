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

    Paint("COM7", 57600)
    # Paint(None, None)


    # r = Robot()
    # T = np.array([
    #     [0, 0, 1, -30.4],
    #     [0, 1, 0, 36.25],
    #     [-1, 0, 0, 15.45],
    #     [0, 0, 0, 1]
    # ])
    # thetalist0 = np.array([-0.01745329,  0.10471976, -1.32645023, -0.17453293, -0.01745329])#np.deg2rad(np.array([-1, 6, -76, -10, -1]))
    # eomg = 0.0872
    # ev = 2
    # [thetalist, success] = mr.IKinSpace(
    #     np.array([r.S1, r.S2, r.S3, r.S4, r.S5]).T,
    #     r.M, T,
    #     thetalist0,
    #     eomg, ev
    # )
    # print(np.rad2deg(thetalist), success)

    # T = mr.FKinSpace(r.M, np.array([r.S1, r.S2, r.S3, r.S4, r.S5]).T, np.array([np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2]))
    # print(T)

    # mc = MotorController("COM7", 57600)
    # mc.setAngles([0, 0, 0, 0, 0, 0])
    # time.sleep(3)
    # mc.setAngles(list(np.rad2deg(thetalist)))
    # time.sleep(6)
    # T = np.array([
    #     [0, 0, 1, 40.412],
    #     [0, 1, 0, 15.543],
    #     [-1, 0, 0, 23.69],
    #     [0, 0, 0, 1]
    # ])
    # thetalist0 = thetalist
    # eomg = 0.0872
    # ev = 2
    # [thetalist, success] = mr.IKinSpace(
    #     np.array([r.S1, r.S2, r.S3, r.S4, r.S5]).T,
    #     r.M, T,
    #     thetalist0,
    #     eomg, ev
    # )
    # if success:
    #     mc.setAngles(list(np.rad2deg(thetalist)))
    #     time.sleep(6)
    # mc.setAngles([0, 0, 0, 0, 0, 0])
    # time.sleep(6)

    # f = 0.3 # Hz
    # omega = 2 * math.pi * f # rad/s
    # Ts = 0.25 # seconds
    # ncycles = 3
    # t_end = ncycles / f
    # for i in range(int(t_end / Ts)):
    #     t = i * Ts
    #     pos = 10 * math.sin(omega * t)
    #     print(pos)
    #     time.sleep(Ts)