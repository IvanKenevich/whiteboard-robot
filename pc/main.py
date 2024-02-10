from tkinter import *
import serial
import time

class Paint(object):

    DEFAULT_PEN_SIZE = 5.0
    DEFAULT_COLOR = 'black'
    paths = []

    def __init__(self):
        self.root = Tk()

        self.init_button = Button(self.root, text='Init', command=self.init_robot)
        self.init_button.grid(row=0, column=0)

        self.draw_button = Button(self.root, text='Draw', command=self.draw_letters)
        self.draw_button.grid(row=0, column=1)

        self.clear_button = Button(self.root, text='Clear', command=self.clear_canvas)
        self.clear_button.grid(row=0, column=2)

        self.stop_button = Button(self.root, text='STOP', command=self.stop_drawing)
        self.stop_button.grid(row=0, column=3)

        self.c = Canvas(self.root, bg='white', width=600, height=400)
        self.c.grid(row=1, columnspan=4)

        self.setup()
        self.root.mainloop()


    def setup(self):
        self.old_x = None
        self.old_y = None
        self.line_width = 3
        self.color = self.DEFAULT_COLOR
        self.c.bind('<B1-Motion>', self.paint)
        self.c.bind('<ButtonRelease-1>', self.reset)

    def init_robot(self):
        pass

    def draw_letters(self):
        pass

    def clear_canvas(self):
        pass

    def stop_drawing(self):
        pass

    def paint(self, event):
        if self.old_x and self.old_y:
            self.c.create_line(self.old_x, self.old_y, event.x, event.y,
                               width=self.line_width, fill=self.color,
                               capstyle=ROUND, smooth=TRUE, splinesteps=36)
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
        

if __name__ == '__main__':
    mc = MotorController("COM6", 57600)

    # zero all out
    mc.setAngle(1, 0)
    mc.setAngle(2, 0)
    mc.setAngle(3, 0)
    mc.setAngle(4, 0)
    mc.setAngle(5, 0)
    mc.setAngle(6, 0)

    time.sleep(3)

    mc.setAngles([0, 90, 0, 0, 0, 0])
    time.sleep(3)
    mc.setAngles([45, 90, 0, 0, 0, 0])

    time.sleep(3)

    mc.setAngles(6 * [0])

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