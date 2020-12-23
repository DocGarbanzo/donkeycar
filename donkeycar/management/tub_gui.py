
import tkinter as tk
from tkinter import ttk
import time
from threading import Thread
from PIL import ImageTk, Image

from donkeycar import load_config
from donkeycar.parts.tub_v2 import Tub
from donkeycar.pipeline.types import TubRecord


class TubUI:
    def __init__(self, window):
        self.window = window
        self.window.title("Tub GUI")
        #self.window.configure(background='grey45')
        self.run = False
        self.config = load_config("/Users/dirk/mycar/config.py")
        self.base_path = "/Users/dirk/mycar/data3"
        self.tub = Tub(self.base_path)
        self.records = [TubRecord(self.config, self.tub.base_path, record)
                        for record in self.tub]
        self.len = len(self.records)
        self.i = 0
        self.current_rec = self.records[self.i]
        self.img = self.get_img(self.current_rec)
        self.thread = None
        self.build_frame()
        print('Length', self.len)

    def get_img(self, record):
        img_arr = record.image()
        img = Image.fromarray(img_arr)
        return ImageTk.PhotoImage(img)

    def build_frame(self):

        # Image
        self.img_frame = tk.Label(self.window, image=self.img, bg='black',
                                  relief=tk.SUNKEN, borderwidth=3)
        self.img_frame.grid(row=0, column=3, columnspan=4, rowspan=3,
                            padx=15, pady=15)

        # data box
        self.data_fram = tk.LabelFrame(self.window,
                                       padx=10, pady=10)
        self.data_fram.grid(row=0, column=0, rowspan=3)
        self.steering_label = tk.Label(self.data_fram, text='steering',
                                        relief=tk.SUNKEN)
        self.steering_label.grid(row=0, column=0, columnspan=3)
        self.barVar = tk.DoubleVar()
        self.barVar.set(0)
        self.steering_bar = ttk.Progressbar(self.data_fram,
                                            variable=self.barVar,
                                            orient=tk.HORIZONTAL,
                                            length=100, mode='determinate')
        self.steering_bar.grid(row=1, column=0, columnspan=3)

        # control box
        w, h = (3, 1)
        self.ctr_fram = tk.LabelFrame(self.window,
                                      padx=10, pady=10)
        self.ctr_fram.grid(row=0, column=7, rowspan=3)

        self.rec_txt = tk.StringVar(self.ctr_fram, f"Record {self.i}")
        self.record_label = tk.Label(self.ctr_fram, textvariable=self.rec_txt,
                                     relief=tk.SUNKEN)
        self.record_label.grid(row=0, column=0, columnspan=2)


        self.button_bwd = tk.Button(self.ctr_fram, text="Bwd",
                                    command=lambda: self.step(False),
                                    width=w, height=h)
        self.button_bwd.grid(row=1, column=0)
        self.button_fwd = tk.Button(self.ctr_fram, text="Fwd",
                                    command=lambda: self.step(True),
                                    width=w, height=h)
        self.button_fwd.grid(row=1, column=1)

        self.btn_rwd_txt = tk.StringVar(self.ctr_fram, "Rewind")
        self.btn_rwd = tk.Button(self.ctr_fram, textvariable=self.btn_rwd_txt,
                                 command=lambda: self.thread_func(False),
                                 width=w, height=h)
        self.btn_rwd.grid(row=2, column=0)
        self.btn_play_txt = tk.StringVar(self.ctr_fram, "Play")
        self.btn_play = tk.Button(self.ctr_fram, textvariable=self.btn_play_txt,
                                  command=lambda: self.thread_func(True),
                                  width=w, height=h)
        self.btn_play.grid(row=2, column=1)

        self.slider = tk.Scale(self.ctr_fram, from_=0, to=self.len - 1,
                               orient=tk.HORIZONTAL, command=self.slide)
        self.slider.grid(row=3, columnspan=2)

        # quit button
        self.but_exit = tk.Button(self.window, text="Quit",
                                  command=self.quit,
                                  fg='tomato', borderwidth=0)
        self.but_exit.grid(row=4, column=3, columnspan=2)

    def step(self, fwd=True):
        self.i += 1 if fwd else -1
        if self.i >= self.len and fwd:
            self.i = 0
        elif self.i < 0 and not fwd:
            self.i = self.len - 1
        self.update()

    def update(self):
        self.current_rec = self.records[self.i]
        index = self.current_rec.underlying['_index']
        self.rec_txt.set(f"Record {index}")
        self.img = self.get_img(self.current_rec)
        self.img_frame.configure(image=self.img)
        # the slider needs to count continuously through the records
        self.slider.set(self.i)
        angle = self.current_rec.underlying['user/angle']
        self.barVar.set(angle * 50 + 50)

    def loop(self, fwd=True):
        count = 0
        tic = time.time()
        while self.run:
            self.step(fwd)
            time.sleep(0.05)
            count += 1
        print('self.run', self.run)
        #print(f'Took {time.time() - tic}s where we expected {1.0}s')

    def thread_func(self, fwd=True):
        self.run = not self.run
        if fwd:
            self.btn_play_txt.set("Stop" if self.run else "Play")
            self.btn_play.configure(fg='red' if self.run else 'black')
        else:
            self.btn_rwd_txt.set("Stop" if self.run else "Rewind")
            self.btn_rwd.configure(fg='red' if self.run else 'black')
        self.thread = Thread(target=self.loop, args=(fwd, ))
        self.thread.start()

    def slide(self, val):
        self.i = int(val)
        self.update()

    def quit(self):
        self.run = False
        if self.thread:
            print("sleep")
            time.sleep(3.0)
            print("join")
            self.thread.join()
        try:
            print("Try destroying window")
            self.window.destroy()
        except Exception as e:
            exit(str(e))
        finally:
            exit(0)


if __name__ == "__main__":
    # This creates the main window of an application
    window = tk.Tk()
    ui = TubUI(window)
    window.mainloop()