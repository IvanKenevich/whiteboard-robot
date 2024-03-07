from tkinter import *
import logging
import time

from controls import Robot, MotorController
from UI_utils import Line, Path

class Paint(object):
    LINE_WIDTH = 3
    CANVAS_WIDTH = 1000
    CANVAS_HEIGHT = CANVAS_WIDTH * Robot.WB_HEIGHT / Robot.WB_WIDTH

    def __init__(self, COM: str, baud: int):
        self.COM = COM; self.baud = baud
        self.r = None
        self.init_gui()
        logging.info("Paint started")
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

        self.paths: list[Path] = []
        self.current_path = None

        self.old_x = None
        self.old_y = None
        self.c.bind('<B1-Motion>', self.mouse_drag)
        self.c.bind('<ButtonRelease-1>', self.mouse_release)
        self.c.bind('<ButtonPress-1>', self.mouse_press)
        self.root.bind('<KeyPress>', self.key_press)

    def init_robot(self):
        try:
            self.r = Robot(self.COM, self.baud)
        except Exception as e:
            logging.error(e)
        else:
            logging.info(f"Motor comms established at {self.COM}::{self.baud}")


    def toggle_grabber(self):
        self.r.toggleGrabber()
        logging.info(f"Grabber toggled. New grabber open: {self.r.grabber_open}")


    def draw_letters(self):
        self.r.planTrajectory(self.paths, self.CANVAS_WIDTH, self.CANVAS_HEIGHT)
        

    def clear_canvas(self):
        for path in self.paths:
            for line in path.lines:
                self.c.delete(line.id)
        self.paths = []


    def stop_drawing(self):
        pass


    def mouse_drag(self, event):
        if self.old_x and self.old_y:
            time.sleep(0.15)
            id = self.c.create_line(self.old_x, self.old_y, event.x, event.y,
                               width=self.LINE_WIDTH, fill='black',
                               capstyle=ROUND, smooth=TRUE, splinesteps=36)
            self.current_path.add(Line(id, self.old_x, self.old_y, event.x, event.y))

        self.old_x = event.x
        self.old_y = event.y


    def mouse_release(self, event):
        self.old_x, self.old_y = None, None
        assert(self.current_path is not None)
        self.paths.append(self.current_path)
        logging.info(f"New path added: {str(self.current_path)}")
        self.current_path = None
        

    def mouse_press(self, event):
        assert(self.current_path is None)
        self.current_path = Path()
        logging.info("Button press event")

    
    def key_press(self, event):
        c = event.char
        logging.info(f"Key pressed: {c}")

        if c == 'w':
            self.r.forward()
        elif c == 's':
            self.r.backward()
        elif c == 'a':
            self.r.left()
        elif c == 'd':
            self.r.right()
        elif c == 'r':
            self.r.up()
        elif c == 'f':
            self.r.down()

        