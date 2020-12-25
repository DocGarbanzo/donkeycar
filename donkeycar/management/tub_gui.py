import math
from collections import namedtuple
import tkinter as tk
from tkinter import ttk
import time
from threading import Thread, active_count, Lock
from PIL import ImageTk, Image

from donkeycar import load_config
from donkeycar.parts.tub_v2 import Tub
from donkeycar.pipeline.types import TubRecord

LookUp = namedtuple('LookUp', ['record_field', 'max_value_id', 'centered'])
lookup_entries = [
    LookUp('user/angle', '', centered=True),
    LookUp('user/throttle', '', centered=False),
    LookUp('car/speed', 'MAX_SPEED', centered=False),
    LookUp('car/inst_speed', 'MAX_SPEED', centered=False),
    LookUp('imu/gyro', 'IMU_GYRO_NORM', centered=True),
    LookUp('imu/accel', 'IMU_ACCEL_NORM', centered=True)
]

record_map = {entry.record_field: entry for entry in lookup_entries}


class LabelBar:
    row = 0

    def __init__(self, context, name, colwidth=3):
        self.context = context
        self.text = tk.StringVar()
        self.label = tk.Label(self.context.data_frame, textvariable=self.text)
        self.label.grid(row=self.row, column=0, sticky=tk.NW)
        self.bar_val = tk.DoubleVar()
        self.bar = ttk.Progressbar(self.context.data_frame,
                                   variable=self.bar_val,
                                   orient=tk.HORIZONTAL,
                                   length=100, mode='determinate')
        self.bar.grid(row=self.row, column=1, columnspan=colwidth - 1)
        self.name = name
        lookup = record_map[self.name]
        self.max = getattr(self.context.config, lookup.max_value_id, 1.0)
        self.center = lookup.centered
        LabelBar.row += 1

    def update(self):
        val = self.context.current_rec.underlying[self.name]
        norm_val = val / self.max
        new_bar_val = (norm_val + 1) * 50 if self.center else norm_val * 100
        self.bar_val.set(new_bar_val)
        self.text.set(self.name + f': {val:+3.2f}')

    def destroy(self):
        self.label.destroy()
        self.bar.destroy()


class TubUI:
    def __init__(self, window):
        self.window = window
        self.window.title("Tub GUI")
        # self.window.configure(background='grey45')
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
        self.bars = dict()
        self.build_frame()
        self.update()
        self.count = 0

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
        self.data_frame = tk.LabelFrame(self.window, padx=10, pady=10)
        self.data_frame.grid(row=0, column=0, rowspan=3)

        self.var_label = tk.Label(self.data_frame, text='Add/remove')
        self.var_label.grid(row=0, column=0, sticky=tk.W)
        self.var_select = tk.StringVar()
        self.w = ttk.OptionMenu(self.data_frame, self.var_select,
                                *self.tub.manifest.inputs,
                                command=self.add_remove_bars)
        self.w.grid(row=0, column=1, columnspan=2)
        LabelBar.row += 1

        # control box
        w, h = (3, 1)
        self.ctr_fram = tk.LabelFrame(self.window, padx=10, pady=10)
        self.ctr_fram.grid(row=0, column=7, rowspan=4)

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

        self.btn_rwd = tk.Button(self.ctr_fram, text="Rewind",
                                 command=lambda: self.thread_run(False),
                                 width=w, height=h)
        self.btn_rwd.grid(row=2, column=0)

        self.btn_play = tk.Button(self.ctr_fram, text="Play",
                                  command=lambda: self.thread_run(True),
                                  width=w, height=h)
        self.btn_play.grid(row=2, column=1)

        self.btn_stop = tk.Button(self.ctr_fram, text="Stop",
                                  command=self.thread_stop,
                                  width=w, height=h)
        self.btn_stop.grid(row=3, column=0, columnspan=2)

        self.slider = ttk.Scale(self.window, from_=0, to=self.len - 1,
                                orient=tk.HORIZONTAL, command=self.slide,)
        self.slider.grid(row=4, column=0, columnspan=8, sticky='NSEW', padx=10)

        # quit button
        self.but_exit = tk.Button(self.window, text="Quit",
                                  command=self.quit,
                                  fg='tomato', borderwidth=0)
        self.but_exit.grid(row=5, column=7, columnspan=2, sticky=tk.E)

    def step(self, fwd=True):
        self.i += 1 if fwd else -1
        if self.i >= self.len and fwd:
            self.i = 0
        elif self.i < 0 and not fwd:
            self.i = self.len - 1
        self.update()

    def update(self, update_slider=True):
        self.current_rec = self.records[self.i]
        index = self.current_rec.underlying['_index']
        self.rec_txt.set(f"Record {index}")
        self.img = self.get_img(self.current_rec)
        self.img_frame.configure(image=self.img)
        for field, bar in self.bars.items():
            bar.update()
        if update_slider:
            # the slider needs to count continuously through the records
            self.slider.set(self.i)

    def add_remove_bars(self, field):
        # stop loop if running
        was_running = False
        if self.run:
            self.run = False
            was_running = True
        if field in record_map:
            if field in self.bars:
                self.bars[field].destroy()
                del (self.bars[field])
            else:
                self.bars[field] = LabelBar(self, field)
        if was_running:
            self.run = True
        self.update()

    def loop(self, fwd=True):
        while self.run:
            self.step(fwd)
            time.sleep(0.01)

    def thread_run(self, fwd=True):
        self.run = True
        self.thread = Thread(target=self.loop, args=(fwd,))
        self.thread.start()
        print('Active threads:', active_count())

    def thread_stop(self):
        self.run = False

    def slide(self, val):
        self.i = int(math.floor(float(val)))
        self.update(False)

    def quit(self):
        self.run = False
        try:
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
