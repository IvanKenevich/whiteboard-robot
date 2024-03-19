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

    
    # def readPositions(self):
    #     while not self.stop_reading:
    #         time.sleep(0.5)
    #         with self.s_lock:
    #             for id in range(self.N_MOTORS):
    #                 pos = self.s.read(2)


    def setAngle(self, id: int, angle: float):
        if (id in [2, 3]): # MX-64
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
    p_sw = [40.41, 16.04, 25.2] # centimeters

    PATH_LEAD_IN = 2 # centimeters
    PATH_LEAD_OUT = 2 # centimeters

    GRABBER_OPEN_ANGLE = 90 # degrees
    GRABBER_CLOSED_ANGLE = -2.5 # degrees

    MAX_POINT_DIST = 1   # centimeters, maximum distance between two points in 3d space
                         # anything below that will be interpolated
    
    INITIAL_DRAWING_ANGLES = np.deg2rad(np.array([-5, -5, 109, -14, -5]))
    INITIAL_DRAWING_POSITION = np.array([38.22, 3.23, 14.63]).reshape(3,1)
    
    ANGLE_PRECISION = np.deg2rad(2.5) # precision values for IK solver
    POSITION_PRECISION = 0.1           # precision values for IK solver

    def __init__(self, port = None, baud = None):
        if port is not None and baud is not None:
            self.mc = MotorController(port, baud)
        else:
            self.mc = None

        self.M = np.array([
            [1, 0, 0, 0.10],
            [0, 1, 0, 5.23],
            [0, 0, 1, 58.46],
            [0, 0, 0, 1]
        ])

        self.T_sw = np.array([
            [0, 0, 1,self.p_sw[0]],
            [-1, 0,0,self.p_sw[1]],
            [0, -1, 0,self.p_sw[2]],
            [0, 0, 0, 1]
        ])

        # initial position - touching middle of the board
        self.T = np.array([
            [0, 0, 1,40.41],
            [0, 1, 0,6.35],
            [-1, 0, 0,12.86],
            [0, 0, 0, 1]
        ])

        self.grabber_open = True

        r1 = np.array([0, 0, 0])
        r2 = np.array([0, 2.35, 4])
        r3 = np.array([0, 2.35, 18.5])
        r4 = np.array([0.10, 2.30, 34.86])
        r5 = np.array([2.5, 0.2, 43.16])

        w1 = np.array([0, 0, 1])
        w2 = np.array([0, 1, 0])
        w3 = np.array([0, 1, 0])
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

    def move_next_point(self):
        thetalist0 = self.thetalist
        eomg = np.deg2rad(3)
        ev = 1
        [thetalist, success] = mr.IKinSpace(
            np.array([self.S1, self.S2, self.S3, self.S4, self.S5]).T,
            self.M, self.T,
            thetalist0,
            eomg, ev
        )
        self.thetalist = thetalist
        if success:
            self.mc.setAngles(list(np.rad2deg(self.thetalist)))
            logging.info(f"Successfully solved for T:\n{self.T}")
        else:
            logging.info("IK failed to converge")

    def toggleGrabber(self):
        if self.grabber_open:
            self.mc.setAngle(6, self.GRABBER_CLOSED_ANGLE)
            self.grabber_open = False
        else:
            self.mc.setAngle(6, self.GRABBER_OPEN_ANGLE)
            self.grabber_open = True

    def left(self):
        # increment y
        self.T[1, 3] += 0.5
        self.move_next_point()

    def right(self):
        # decrement y
        self.T[1, 3] -= 0.5
        self.move_next_point()

    def forward(self):
        # increment x
        self.T[0, 3] += 0.5
        self.move_next_point()

    def backward(self):
        # decrement x
        self.T[0, 3] -= 0.5
        self.move_next_point()

    def up(self):
        # increment z
        self.T[2, 3] += 0.5
        self.move_next_point()

    def down(self):
        # decrement z
        self.T[2, 3] -= 0.5
        self.move_next_point()

    def interp3(self, trajectory):
        new_trajectory = None

        for i in range(trajectory.shape[1] - 1):
            p1 = trajectory[:,i].copy().reshape(3,1)
            p2 = trajectory[:,i+1].copy().reshape(3,1)
            logging.info(f"p1: {p1}  p2: {p2}")

            dist = np.linalg.norm(p2 - p1)
            if dist < self.MAX_POINT_DIST:
                logging.info(f"Two points were close enough: {dist}")
                new_trajectory = np.hstack((p1, p2)) if new_trajectory is None else np.hstack((new_trajectory, p1, p2))
                continue
                
            logging.info(f"Two points were too far apart: {dist}")
            u = (p2 - p1) / dist
            nsteps = int(dist // self.MAX_POINT_DIST)
            logging.info(f"Will create {nsteps} intermediate points")

            for j in range(nsteps + 1):
                p = p1 + j * self.MAX_POINT_DIST * u
                new_trajectory = p if new_trajectory is None else np.hstack((new_trajectory, p))

            new_trajectory = np.hstack([new_trajectory, p2])
        
        return new_trajectory
    

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
            points_s = self.T_sw @ points
            points_s = points_s[0:3, :]
            # add enter and exit points
            first = points_s[:,0].copy(); first[0] -= self.PATH_LEAD_IN
            last = points_s[:,-1].copy(); last[0] -= self.PATH_LEAD_OUT
            points_s = np.hstack((first[:,None], points_s, last[:,None]))
            trajectory = points_s if trajectory is None else np.hstack((trajectory, points_s))
        
        # add initial point for the solver
        initial_point = self.INITIAL_DRAWING_POSITION
        trajectory = np.hstack((initial_point, trajectory))

        # interpolate if points are too far apart
        trajectory = self.interp3(trajectory)

        # do inverse kinematics
        res = []
        thetalist0 = self.INITIAL_DRAWING_ANGLES
        for i in range(trajectory.shape[1]):
            point = trajectory[:,i]
            T = np.array([
                [0, 0, 1, point[0]],
                [0, 1, 0, point[1]],
                [-1, 0, 0, point[2]],
                [0, 0, 0, 1]
            ]); logging.info(f"T matrix:\n{T}")
 
            [thetalist, success] = mr.IKinSpace(
                np.array([self.S1, self.S2, self.S3, self.S4, self.S5]).T,
                self.M, T,
                thetalist0,
                self.ANGLE_PRECISION, self.POSITION_PRECISION
            )
            if success:
                thetalist0 = thetalist
                logging.info(f"thetalist: {np.rad2deg(thetalist)}")
                prev_point = point
                res.append(thetalist)
            else:
                logging.info(f"Failed to do inverse kinematics at index {i}/{trajectory.shape[1]}.\n"
                             f"Prev point: {prev_point}\n"
                             f"Current point: {point}\n"
                             f"T matrix: {T}\n"
                             f"thetalist0: {np.rad2deg(thetalist0)}")
                return trajectory, None
            
        return trajectory, res
    

    def executeTrajectory(self, angles):
        # go to home position
        self.mc.setAngles([0] * 5)
        time.sleep(15)

        # grab the pen
        self.toggleGrabber()
        time.sleep(5)

        # go to initial position
        self.mc.setAngles(list(np.rad2deg(self.INITIAL_DRAWING_ANGLES)))
        logging.info(f"Going to: {list(np.rad2deg(self.INITIAL_DRAWING_ANGLES))}")
        time.sleep(15)

        # execute drawing path
        for angle_set in angles:
            angle_set = list(np.rad2deg(angle_set))
            self.mc.setAngles(angle_set)
            logging.info(f"Going to: {angle_set}")
            time.sleep(0.5)
