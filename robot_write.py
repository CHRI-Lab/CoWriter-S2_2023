import tkinter
from tkinter import *
from tkinter import ttk, Frame, Tk, messagebox, Menu
from PIL import Image, ImageDraw
import pickle
import csv
import numpy as np
from sklearn.datasets import load_digits
from sklearn.preprocessing import LabelBinarizer
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
import imageio
from ssim_similarity import strugg_letter


class Window(Frame):

    def __init__(self, master=None):
        super().__init__()
        self.master = master
        self.init_window()

        self.lastDraw = 0

        self.foreColor = '#000000'
        self.backColor = '#FFFFFF'


        self.yesno = tkinter.IntVar(value=0)

        self.what = tkinter.IntVar(value=1)

        self.X = tkinter.IntVar(value=0)
        self.Y = tkinter.IntVar(value=0)

        self.samples = np.array([])
        self.labels = np.array([])

    def init_window(self):
        self.master.title('NAO Write Board')

        menubar = Menu(self.master)
        self.master.config(menu=menubar)


        self.frame_pad = ttk.LabelFrame(self.master, text="Robot Write Area")
        self.frame_pad.place(x=10, y=100, width=128, height=128)

        # 创建画布
        image = tkinter.PhotoImage()
        self.canvas = tkinter.Canvas(self.frame_pad, bg='white', width=128, height=128)
        self.canvas.create_image(128, 128, image=image)
        self.canvas.bind('<B1-Motion>', self.onLeftButtonMove)
        self.canvas.bind('<Button-1>', self.onLeftButtonDown)
        self.canvas.bind('<ButtonRelease-1>', self.onLeftButtonUp)
        self.canvas.pack(fill=tkinter.BOTH, expand=tkinter.YES)
        self.base = Image.new("RGB", (128, 128), (255, 255, 255))
        self.d = ImageDraw.Draw(self.base)

        action_frame = ttk.Frame(root)
        action_frame.place(x=150, y=100, width=70, height=50)
        button_cl = ttk.Button(action_frame, text="Rewrite", command=self.Clear)
        button_cl.pack(pady=5)
        # button_start = ttk.Button(action_frame, text="", command=self.load_model)
        # button_start.pack(pady=15)
        # button_reg = ttk.Button(action_frame, text="", command=self.predict)
        # button_reg.pack(pady=5)

        # self.frame2 = ttk.LabelFrame(self.master, text="")
        # self.frame2.place(x=20, y=400, width=150, height=150)
        # image2 = tkinter.PhotoImage()
        # self.canvas2 = tkinter.Canvas(self.frame2, bg='white', width=200, height=200)
        # self.canvas2.create_image(120, 120, image=image2)
        # self.canvas2.pack(fill=tkinter.BOTH, expand=tkinter.YES)

        button_save = ttk.Button(self.master, text="Submit", command=self.convImage)
        button_save.place(x=150, y=150, width=70, height=30)

# 按住鼠标左键移动，画图
    def onLeftButtonMove(self, event):
        # global lastDraw
        if self.yesno.get() == 0:
            return
        if self.what.get() == 1:

            # canvas.create_line(X.get(), Y.get(), event.x, event.y, width=8, fill=foreColor)
            self.canvas.create_oval(self.X.get(), self.Y.get(), event.x, event.y, width=8, fill=self.foreColor)
            self.d.line([self.X.get(), self.Y.get(), event.x, event.y],
                        width=8,
                        fill='black')

            self.X.set(event.x)
            self.Y.set(event.y)


    def onLeftButtonDown(self, event):
        self.yesno.set(1)
        self.X.set(event.x)
        self.Y.set(event.y)
        if self.what.get() == 4:
            self.canvas.create_text(event.x, event.y, text=text)


    def onLeftButtonUp(self, event):
        self.yesno.set(0)
        self.lastDraw = 0

    def Clear(self):

        self.d.rectangle([0, 0, 128, 128], fill='white')

        for item in self.canvas.find_all():
            self.canvas.delete(item)

    def convImage(self):
        img = self.base
        x, y = img.size
        img = img.convert('L')
        raw_data = img.load()
        imageio.imwrite('robot.jpg', img) #rename your file
        print("Robot writing saved")

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    root = Tk()
    sw = root.winfo_screenwidth()
    sh = root.winfo_screenheight()
    ww = 400
    wh = 300
    x = (sw - ww) / 2 - 100
    y = 200
    root.geometry("%dx%d+%d+%d" % (ww, wh, x, y))
    app = Window(root)


    root.mainloop()
