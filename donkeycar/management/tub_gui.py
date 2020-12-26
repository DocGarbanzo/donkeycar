import math
import os
from collections import namedtuple
import tkinter as tk
from tkinter import ttk, filedialog
import time
from threading import Thread, active_count
from PIL import ImageTk, Image
import pandas as pd

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, \
    NavigationToolbar2Tk
from matplotlib.backend_bases import key_press_handler

from donkeycar import load_config
from donkeycar.parts.tub_v2 import Tub
from donkeycar.pipeline.types import TubRecord

LookUp = namedtuple('LookUp', ['record_field', 'max_value_id', 'centered'])
lookup_entries = [
    LookUp('user/angle', '', centered=True),
    LookUp('user/throttle', '', centered=False),
    LookUp('car/speed', 'MAX_SPEED', centered=False),
    LookUp('car/inst_speed', 'MAX_SPEED', centered=False),
    LookUp('car/gyro', 'IMU_GYRO_NORM', centered=True),
    LookUp('car/accel', 'IMU_ACCEL_NORM', centered=True)
]

record_map = {entry.record_field: entry for entry in lookup_entries}


def decompose(field):
    field_split = field.split('_')
    if len(field_split) > 1 and field_split[-1].isdigit():
        return '_'.join(field_split[:-1]), int(field_split[-1])
    return field, None


class LabelBar:
    row = 0

    def __init__(self, context, field, colwidth=3):
        self.context = context
        self.text = tk.StringVar()
        self.label = tk.Label(self.context.data_frame,
                              textvariable=self.text, width=16, anchor=tk.E)
        self.label.grid(row=self.row, column=0)
        self.bar_val = tk.DoubleVar()
        self.bar = ttk.Progressbar(self.context.data_frame,
                                   variable=self.bar_val,
                                   orient=tk.HORIZONTAL,
                                   length=100, mode='determinate')
        self.bar.grid(row=self.row, column=1, columnspan=colwidth - 1)
        self.field = field
        lookup = record_map[decompose(self.field)[0]]
        self.max = getattr(self.context.config, lookup.max_value_id, 1.0)
        self.center = lookup.centered
        LabelBar.row += 1

    def update(self):
        decomp_name = decompose(self.field)
        val = self.context.current_rec.underlying[decomp_name[0]]
        if decomp_name[1] is not None:
            val = val[decomp_name[1]]

        norm_val = val / self.max
        new_bar_val = (norm_val + 1) * 50 if self.center else norm_val * 100
        self.bar_val.set(new_bar_val)
        self.text.set(self.field + f' {val:+07.3f}')

    def destroy(self):
        self.label.destroy()
        self.bar.destroy()


class TubUI:
    def __init__(self, window):
        self.window = window
        self.window.title("Tub GUI")
        # self.window.configure(background='grey45')
        self.run = False
        self.config = None
        self.base_path = None
        self.tub = None
        self.records = None
        self.len = 1
        self.i = 0
        self.current_rec = None
        self.img = None
        self.thread = None
        self.bars = dict()
        self.car_dir = None
        self.drop_down = []
        self.build_frame()
        self.count = 0
        self.df = None

    def get_img(self, record):
        img_arr = record.image()
        img = Image.fromarray(img_arr)
        return ImageTk.PhotoImage(img)

    def update_tub(self):
        self.tub = Tub(self.base_path)

        self.records = [TubRecord(self.config, self.tub.base_path, record)
                        for record in self.tub]
        self.len = len(self.records)
        self.i = 0
        self.current_rec = self.records[self.i]
        self.img = self.get_img(self.current_rec)
        self.slider.configure(to=self.len - 1)

        self.df = pd.DataFrame(list(self.tub))
        to_drop = {'_timestamp_ms', 'cam/image_array', 'timestamp', 'car/lap'}
        to_drop = to_drop.intersection(self.df.columns)
        self.df = self.df.drop(labels=to_drop, axis=1)
        self.df = self.df.set_index('_index')
        self.unravel_df()
        self.drop_down = list(self.df.columns)
        self.var_menu.config(value=self.drop_down)

        self.update_plot(self.df)

    def unravel_df(self):
        for k, v in zip(self.tub.manifest.inputs, self.tub.manifest.types):
            if v == 'vector' or v == 'list':
                dim = len(self.current_rec.underlying[k])
                df_keys = [k + f'_{i}' for i in range(dim)]
                self.df[df_keys] = pd.DataFrame(self.df[k].tolist(),
                                                index=self.df.index)
                self.df = self.df.drop(k, axis=1)

    def update_plot(self, df):
        ax1 = self.figure.add_subplot(111)
        ax1.clear()
        df.plot(kind='line', legend=True, ax=ax1)
        self.graph.draw()

    def build_frame(self):
        # running row
        row = 0
        self.btn_car_dir = tk.Button(self.window, text="Car dir",
                                        command=self.browse_car)
        self.btn_car_dir.grid(row=row, column=0, sticky=tk.W)
        self.car_dir_label = tk.Label(self.window)
        self.car_dir_label.grid(row=row, column=1, sticky=tk.W)
        self.btn_tub_dir = tk.Button(self.window, text="Tub dir",
                                        command=self.browse_tub)
        self.btn_tub_dir.grid(row=row, column=2, sticky=tk.W)
        self.tub_dir_label = tk.Label(self.window)
        self.tub_dir_label.grid(row=row, column=3, columnspan=3, sticky=tk.W)

        # Image
        row += 1
        self.img_frame = tk.Label(self.window, image=self.img, bg='black',
                                  relief=tk.SUNKEN, borderwidth=3)
        self.img_frame.grid(row=row, column=2, columnspan=2, rowspan=3,
                            padx=15, pady=15)

        # data box
        self.data_frame = tk.LabelFrame(self.window, padx=10, pady=10)
        self.data_frame.grid(row=row, column=0, columnspan=2, rowspan=3,
                             padx=10)

        self.var_label = tk.Label(self.data_frame, text='Add or remove')
        self.var_label.grid(row=row, column=0, sticky=tk.W)

        self.var_menu = ttk.Combobox(self.data_frame, value=self.drop_down)
        self.var_menu.bind('<<ComboboxSelected>>', self.add_remove_bars)
        self.var_menu.grid(row=row, column=1, columnspan=2)
        LabelBar.row = row + 1

        # control box
        self.ctr_fram = tk.LabelFrame(self.window, padx=10, pady=10)
        self.ctr_fram.grid(row=row, column=4, columnspan=2, rowspan=4, padx=10)

        self.rec_txt = tk.StringVar(self.ctr_fram, f"Record {self.i}")
        self.record_label = tk.Label(self.ctr_fram, textvariable=self.rec_txt,
                                     relief=tk.SUNKEN)
        self.record_label.grid(row=row, column=0, columnspan=2)

        self.btn_bwd = tk.Button(self.ctr_fram, text="<",
                                 command=lambda: self.step(False), )
        #                            width=w, height=h)
        self.btn_bwd.grid(row=row + 1, column=0, sticky=tk.NSEW)
        self.btn_fwd = tk.Button(self.ctr_fram, text=">",
                                    command=lambda: self.step(True))
        self.btn_fwd.grid(row=row + 1, column=1, sticky=tk.NSEW)

        self.btn_rwd = tk.Button(self.ctr_fram, text="<<",
                                 command=lambda: self.thread_run(False))
        self.btn_rwd.grid(row=row + 2, column=0, sticky=tk.NSEW)

        self.btn_play = tk.Button(self.ctr_fram, text=">>",
                                  command=lambda: self.thread_run(True))
        self.btn_play.grid(row=row + 2, column=1, sticky=tk.NSEW)

        self.btn_stop = tk.Button(self.ctr_fram, text="Stop",
                                  command=self.thread_stop)
        self.btn_stop.grid(row=row + 3, column=0, columnspan=2, sticky=tk.NSEW)

        # slider
        row += 4
        self.slider = ttk.Scale(self.window, from_=0, to=self.len - 1,
                                orient=tk.HORIZONTAL, command=self.slide,)
        self.slider.grid(column=0, columnspan=6, sticky=tk.NSEW, padx=10)

        row += 1
        self.figure = Figure(dpi=100)
        self.graph = FigureCanvasTkAgg(self.figure, self.window)
        self.graph.draw()
        self.graph.get_tk_widget().grid(column=0, columnspan=6, sticky=tk.NSEW,
                                        padx=10)
        row += 1
        self.toolbar = NavigationToolbar2Tk(self.graph, self.window,
                                            pack_toolbar=False)
        self.toolbar.update()
        self.toolbar.grid(row=row, column=0, columnspan=5, sticky=tk.W, padx=10)
        self.graph.mpl_connect("key_press_event", self.on_key_press)

        # quit button
        self.but_exit = tk.Button(self.window, text="Quit",
                                  command=self.quit, fg='tomato')
        self.but_exit.grid(row=row, column=5, sticky=tk.E)

    def on_key_press(self, event):
        print("you pressed {}".format(event.key))
        key_press_handler(event, self.graph, self.toolbar)

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

    def add_remove_bars(self, inp):
        field = self.var_menu.get()
        # stop loop if running
        was_running = False
        if self.run:
            self.run = False
            was_running = True
        if decompose(field)[0] in record_map:
            self.manage_bar_entry(field)

        if was_running:
            self.run = True
        self.update()

        df = self.df[self.bars.keys()]
        self.update_plot(df)

    def manage_bar_entry(self, field):
        if field in self.bars:
            self.bars[field].destroy()
            del(self.bars[field])
        else:
            self.bars[field] = LabelBar(self, field)

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

    def browse_car(self):
        self.car_dir = filedialog.askdirectory(
            initialdir=os.path.expanduser('~'),
            title="Select the car dir")
        self.car_dir_label.configure(text=self.car_dir)
        self.config = load_config(os.path.join(self.car_dir, 'config.py'))

    def browse_tub(self):
        start_dir = self.car_dir if self.car_dir else os.path.expanduser('~')
        self.base_path = filedialog.askdirectory(initialdir=start_dir,
                                                 title="Select the tub dir")
        self.tub_dir_label.config(text=self.base_path)
        self.update_tub()
        self.update()

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
