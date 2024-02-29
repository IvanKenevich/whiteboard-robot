from tkinter import *
import serial
import logging
import time
import threading
import numpy as np
import modern_robotics as mr

from UI_utils import Path

class MotorController:
    AX_12_MAXPOS = 1023
    AX_12_MINANG = 0
    AX_12_MAXANG = 300
    MX_64_MAXPOS = 4095
    MX_64_MINANG = 0
    MX_64_MAXANG = 360
    N_MOTORS = 5

    def __init__(self, COM: str, baud: int):
        self.COM = COM
        self.baud = baud
        self.s = serial.Serial(COM, baud)

        self.s_lock = threading.Lock()
        # self.pos_thread = threading.Thread(target=self.readPositions)
        # self.stop_reading = False
        # self.pos_thread.start()


    def __del__(self):
        with self.s_lock:
            self.s.close()
        print(f"Port {self.COM} closed successfully")


    def sendPosition(self, id: int, pos: int):
        with self.s_lock:
            self.s.write(pos.to_bytes(2, "little"))
            self.s.write(id.to_bytes(1, "little"))

    
    def readPositions(self):
        while not self.stop_reading:
            time.sleep(0.5)
            with self.s_lock:
                for id in range(self.N_MOTORS):
                    pos = self.s.read(2)


    def setAngle(self, id: int, angle: float):
        if (id < 3): # MX-64
            angle = angle + 180
            pos = self.map(angle, self.MX_64_MINANG, self.MX_64_MAXANG, 0, self.MX_64_MAXPOS)
        else: # AX-12
            angle = angle + 150
            pos = self.map(angle, self.AX_12_MINANG, self.AX_12_MAXANG, 0, self.AX_12_MAXPOS)
        self.sendPosition(id, pos)


    def setAngles(self, angles: list[float]):
        for i, angle in enumerate(angles):
            self.setAngle(i+1, angle)


    def map(self, x: float, in_min: float, in_max: float, out_min: int, out_max: int) -> int:
        res = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        if res < out_min:
            return out_min
        elif res > out_max:
            return out_max
        else:
            return int(res)
        

class Robot:
    WB_WIDTH = 33.02 # centimeters
    WB_HEIGHT = 25.4 # centimeters

    #vector from S to top left corner of whiteboard 
    p_sw = [40.412, 18.595, 25.4] # centimeters

    PATH_LEAD_IN = 5 # centimeters
    PATH_LEAD_OUT = 5 # centimeters

    def __init__(self, port = None, baud = None):
        if port is not None and baud is not None:
            self.mc = MotorController(port, baud)
        else:
            self.mc = None

        self.M = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 5.48],
            [0, 0, 1, 60.26],
            [0, 0, 0, 1]
        ])

        self.T_sw = np.array([
            [0,-1, 0,self.p_sw[0]],
            [0, 0,-1,self.p_sw[1]],
            [1, 0, 0,self.p_sw[2]],
            [0, 0, 0, 1]
        ])

        r1 = np.array([0, 0, 0])
        r2 = np.array([0, -2.35, 3.1])
        r3 = np.array([0, -1.95, 21.3])
        r4 = np.array([0, -1.95, 36.66])
        r5 = np.array([-2.4, 0.45, 44.96])

        w1 = np.array([0, 0, 1])
        w2 = np.array([0, 1, 0])
        w3 = np.array([0, -1, 0])
        w4 = np.array([0, 1, 0])
        w5 = np.array([1, 0, 0])

        v1 = np.cross(r1, w1)
        v2 = np.cross(r2, w2)
        v3 = np.cross(r3, w3)
        v4 = np.cross(r4, w4)
        v5 = np.cross(r5, w5)

        self.S1 = np.concatenate((w1, v1))
        self.S2 = np.concatenate((w2, v2))
        self.S3 = np.concatenate((w3, v3))
        self.S4 = np.concatenate((w4, v4))
        self.S5 = np.concatenate((w5, v5))


    def planTrajectory(self, paths: list[Path], CANVAS_WIDTH, CANVAS_HEIGHT):
        trajectory = None
        for path in paths:
            # extract whiteboard points
            points = path.extract_points()
            # convert to centimeters, add dimensions
            points = [(x * self.WB_WIDTH/CANVAS_WIDTH, y * self.WB_HEIGHT/CANVAS_HEIGHT, 0, 1) for (x, y) in points]
            # convert to numpy
            points = np.array(points).T
            # apply whiteboard to space transformation
            points_s = mr.TransInv(self.T_sw) @ points
            points_s = points_s[0:3, :]
            # add enter and exit trajectories
            first = points_s[:,0].copy(); first[0] -= self.PATH_LEAD_IN
            last = points_s[:,-1].copy(); last[0] -= self.PATH_LEAD_OUT
            points_s = np.hstack([first[:,None], points_s, last[:,None]])
            trajectory = points_s if trajectory is None else np.hstack([trajectory, points_s])
        
        res = []
        thetalist0 = np.deg2rad(np.array([-1, 6, -76, -10, -1]))
        prev_point = np.array([0, 5.48, 60.26])
        for i in range(trajectory.shape[1]):
            point = trajectory[:,i]
            T = np.array([
                [0, 0, 1, point[0]],
                [0, 1, 0, point[1]],
                [-1, 0, 0, point[2]],
                [0, 0, 0, 1]
            ]); logging.info(f"T matrix:\n{T}")
            eomg = 0.0872
            ev = 2
            [thetalist, success] = mr.IKinSpace(
                np.array([self.S1, self.S2, self.S3, self.S4, self.S5]).T,
                self.M, T,
                thetalist0,
                eomg, ev
            )
            if success:
                thetalist0 = thetalist
                dt = np.linalg.norm(point - prev_point)
                prev_point = point
                res.append((dt, thetalist))
            else:
                logging.info(f"Failed to do inverse kinematics at index {i}.\n"
                             f"Prev point: {prev_point}\n"
                             f"Current point: {point}\n"
                             f"T matrix: {T}\n"
                             f"thetalist0: {thetalist0}")
                return None
            
        return res
