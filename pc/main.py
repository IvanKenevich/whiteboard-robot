from tkinter import *
import serial
import logging
import time
import numpy as np

class Paint(object):
    LINE_WIDTH = 3
    CANVAS_WIDTH = 600
    CANVAS_HEIGHT = 400
    paths = []
    lines = []

    def __init__(self, COM: str, baud: int):
        self.COM = COM; self.baud = baud
        self.init_gui()
        self.root.mainloop()


    def init_gui(self):
        self.root = Tk()

        self.init_button = Button(self.root, text='Init', command=self.init_robot)
        self.init_button.grid(row=0, column=0)

        self.init_button = Button(self.root, text='Grabber', command=self.toggle_grabber)
        self.init_button.grid(row=0, column=1)

        self.draw_button = Button(self.root, text='Draw', command=self.draw_letters)
        self.draw_button.grid(row=0, column=2)

        self.clear_button = Button(self.root, text='Clear', command=self.clear_canvas)
        self.clear_button.grid(row=0, column=3)

        self.stop_button = Button(self.root, text='STOP', command=self.stop_drawing)
        self.stop_button.grid(row=0, column=4)

        self.c = Canvas(self.root, bg='white', width=self.CANVAS_WIDTH, height=self.CANVAS_HEIGHT)
        self.c.grid(row=1, columnspan=5)

        self.old_x = None
        self.old_y = None
        self.c.bind('<B1-Motion>', self.paint)
        self.c.bind('<ButtonRelease-1>', self.reset)

    def init_robot(self):
        try:
            self.mc = MotorController(self.COM, self.baud)
            self.mc.homePosition()
        except Exception as e:
            logging.error(e)
        else:
            logging.info(f"Motor comms established at {self.COM}::{self.baud}")

    def toggle_grabber(self):
        pass

    def draw_letters(self):
        pass

    def clear_canvas(self):
        for id in self.lines:
            self.c.delete(id)
        self.lines = []

    def stop_drawing(self):
        pass

    def paint(self, event):
        if self.old_x and self.old_y:
            self.lines.append(self.c.create_line(self.old_x, self.old_y, event.x, event.y,
                               width=self.LINE_WIDTH, fill='black',
                               capstyle=ROUND, smooth=TRUE, splinesteps=36))
        self.old_x = event.x
        self.old_y = event.y

    def reset(self, event):
        self.old_x, self.old_y = None, None


class MotorController:
    AX_12_MAXPOS = 1023
    AX_12_MINANG = 0
    AX_12_MAXANG = 300
    MX_64_MAXPOS = 4095
    MX_64_MINANG = 0
    MX_64_MAXANG = 360

    def __init__(self, COM: str, baud: int):
        self.COM = COM
        self.baud = baud
        self.s = serial.Serial(COM, baud)


    def __del__(self):
        self.s.close()
        print(f"Port {self.COM} closed successfully")


    def sendPosition(self, id: int, pos: int):
        self.s.write(pos.to_bytes(2, "little"))
        self.s.write(id.to_bytes(1, "little"))


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

    def __init__(self):
        r1 = np.array([0, 0, 0])
        r2 = np.array([0, -2.35, 3.1])
        r3 = np.array([0, -1.65, 21.3])
        r4 = np.array([-2.65, 0.45, 37.76])
        r5 = np.array([-2.65, 2.55, 40.36])

        w1 = np.array([0, 0, 1])
        w2 = np.array([0, 1, 0])
        w3 = np.array([0, -1, 0])
        w4 = np.array([0, 0, 1])
        w5 = np.array([0, 1, 0])

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


if __name__ == '__main__':
    # Paint()
    # mc = MotorController("COM6", 57600)

    # time.sleep(3)

    # mc.setAngles([0, 0, 0, 0, 0, 0])
    # time.sleep(3)

    # mc.setAngles(6 * [30])

    r = Robot()
    print(r.S1)

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