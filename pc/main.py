from tkinter import *
from tkinter.colorchooser import askcolor


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

        self.c = Canvas(self.root, bg='white', width=600, height=600)
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


def sendPosition():
    import serial

    s = serial.Serial("COM7", 57600)
    val: int = 4000
    s.write(val.to_bytes(2, "little"))

    s.close()


if __name__ == '__main__':
    # Paint()
    sendPosition()