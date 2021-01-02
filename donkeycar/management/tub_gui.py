#!/usr/bin/env python3
import math
import os
from collections import namedtuple
import tkinter as tk
from tkinter import ttk, filedialog
import time
from threading import Thread, active_count
from PIL import ImageTk, Image
import pandas as pd
import yaml
import datetime
import platform
import atexit

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, \
    NavigationToolbar2Tk
from matplotlib.backend_bases import key_press_handler

from donkeycar import load_config
from donkeycar.parts.tub_v2 import Tub
from donkeycar.pipeline.types import TubRecord, create_filter_string

LookUp = namedtuple('LookUp', ['record_field', 'max_value_id', 'centered'])
lookup_entries = [
    LookUp('user/angle', '', centered=True),
    LookUp('user/throttle', '', centered=False),
]

record_map = {entry.record_field: entry for entry in lookup_entries}
rc_path = os.path.expanduser('~/.donkeyrc')


def combine_record_map(field_list):
    if field_list is None:
        field_list = {}
    new_lookups = []
    for entry in field_list:
        assert isinstance(entry, dict), \
            'Dictionary required in lookup conversion'
        lookup = LookUp(**entry)
        new_lookups.append(lookup)

    new_record_map = {entry.record_field: entry for entry in new_lookups}
    record_map.update(new_record_map)
    return record_map


def read_rc():
    if os.path.exists(rc_path):
        with open(rc_path) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            print(f'Donkey file {rc_path} loaded.')
            return data
    else:
        print(f'Donkey file {rc_path} does not exist.')
        return {}


def write_rc(data):
    if os.path.exists(rc_path):
        print(f'Donkey file {rc_path} updated.')

    with open(rc_path, mode='w') as f:
        now = datetime.datetime.now()
        data['time_stamp'] = now
        data = yaml.dump(data, f)
        return data


def decompose(field):
    field_split = field.split('_')
    if len(field_split) > 1 and field_split[-1].isdigit():
        return '_'.join(field_split[:-1]), int(field_split[-1])
    return field, None


class LabelBar:
    row = 0

    def __init__(self, context, field, colwidth=3):
        self.context = context
        self.field = field
        self.text = tk.StringVar()
        self.label = ttk.Label(self.context.data_frame,
                               textvariable=self.text, anchor=tk.E,
                               font='TkFixedFont')
        self.label.grid(row=self.row, column=0, sticky=tk.E)

        # only add bar if we have normalisation data
        self.lookup = context.record_map.get(decompose(self.field)[0])
        text = f'Added field {self.field}'
        if self.lookup:
            self.bar_val = tk.DoubleVar()
            self.bar = ttk.Progressbar(self.context.data_frame,
                                       variable=self.bar_val,
                                       orient=tk.HORIZONTAL,
                                       length=100, mode='determinate')
            self.bar.grid(row=self.row, column=1, columnspan=colwidth - 1)
            self.max = getattr(self.context.config,
                               self.lookup.max_value_id,
                               1.0)
            self.center = self.lookup.centered
            text += f' with max value={self.max} centered={self.center}'
        else:
            text += "... not showing a bar because it is a string field " \
                    "or there is no field list entry in your .donkeyrc"
        LabelBar.row += 1
        self.context.status.configure(text=text)

    def update(self):
        decomp_field = decompose(self.field)
        val = self.context.current_rec.underlying[decomp_field[0]]
        if decomp_field[1] is not None:
            val = val[decomp_field[1]]
        # update bar if present
        if self.lookup:
            norm_val = val / self.max
            new_bar_val = (norm_val + 1) * 50 if self.center else norm_val * 100
            self.bar_val.set(new_bar_val)
        if isinstance(val, float):
            text = f' {val:+07.3f}'
        elif isinstance(val, int):
            text = f' {val:10}'
        else:
            text = ' ' + val
        self.text.set(self.field + text)

    def destroy(self):
        self.label.destroy()
        if self.lookup:
            self.bar.destroy()
        self.context.status.configure(text=f'Removed field {self.field}')


class TubUI:
    def __init__(self, window, rc_data):
        self.window = window
        self.rc_data = rc_data
        self.window.title("Donkey Tub Manager")
        self.record_map = combine_record_map(self.rc_data.get('field_mapping'))
        # self.window.configure(background='grey45')
        self.run = False
        self.config = None
        self.base_path = self.rc_data.get('last_tub')
        self.tub = None
        self.records = None
        self.len = 1
        self.i = 0
        self.current_rec = None
        self.img = None
        self.thread = None
        self.bars = dict()
        self.car_dir = self.rc_data.get('car_dir')
        self.drop_down = []
        self.speed_settings = ['0.25', '0.50', '1.00', '1.50', '2.00',
                               '3.00', '4.00']
        self.speed = 1.0
        self.df = None
        self.lr = [0, 0]
        self.filter_expression = None
        self.record_filter = self.rc_data.get('record_filter', '')
        self.build_frame()
        self.count = 0
        self.enable_keys = True
        self.window.bind("<Key>", self.handle_char_key)
        self.window.bind("<Left>", self.handle_left_key)
        self.window.bind("<Right>", self.handle_right_key)
        # Register at exit
        atexit.register(write_rc, data=self.rc_data)

    def get_img(self, record):
        img_arr = record.image()
        img = Image.fromarray(img_arr)
        return ImageTk.PhotoImage(img)

    def update_tub(self, reload=False):
        if self.base_path is None or self.config is None:
            return
        if not os.path.exists(os.path.join(self.base_path, 'manifest.json')):
            self.update_status(f'Path {self.base_path} is not a valid tub.')
        self.tub = Tub(self.base_path)

        def select(underlying):
            if self.filter_expression is None:
                return True
            else:
                record = TubRecord(self.config, self.tub.base_path, underlying)
                res = eval(self.filter_expression)
                return res

        self.records = [TubRecord(self.config, self.tub.base_path, record)
                        for record in self.tub if select(record)]
        self.len = len(self.records)
        self.i = 0
        self.current_rec = self.records[self.i]
        self.slider.configure(to=self.len - 1)

        underlying_generator = (t.underlying for t in self.records)
        self.df = pd.DataFrame(underlying_generator).dropna()
        to_drop = {'cam/image_array'}
        self.df.drop(labels=to_drop, axis=1, errors='ignore', inplace=True)
        self.df.set_index('_index', inplace=True)
        self.unravel_df()
        self.drop_down = list(self.df.columns)
        self.var_menu.config(value=self.drop_down)
        self.tub_dir_label.config(text=self.base_path)
        index = self.current_rec.underlying['_index']
        self.lr = [index, index]
        self.update_status(f'Loaded tub {self.base_path} with '
                                   f'{self.len} records.')
        # clear bars for new tub only but not for reloading
        if not reload:
            for bar in self.bars.values():
                bar.destroy()
            self.bars.clear()
        # update graph
        self.update_plot_from_current_bars()

    def unravel_df(self):
        for k, v in zip(self.tub.manifest.inputs, self.tub.manifest.types):
            if v == 'vector' or v == 'list':
                dim = len(self.current_rec.underlying[k])
                df_keys = [k + f'_{i}' for i in range(dim)]
                self.df[df_keys] = pd.DataFrame(self.df[k].tolist(),
                                                index=self.df.index)
                self.df.drop(k, axis=1, inplace=True)

    def update_plot(self, df):
        df = df.drop(labels=['_timestamp_ms'], axis=1, errors='ignore')
        ax1 = self.figure.add_subplot(111)
        ax1.clear()
        df.plot(kind='line', legend=True, ax=ax1)
        self.graph.draw()

    def build_frame(self):
        # running row
        row = 0
        self.btn_car_dir = tk.Button(self.window, text="Car dir",
                                     command=self.browse_car)
        self.btn_car_dir.grid(row=row, column=0, sticky=tk.W, padx=10)
        self.car_dir_label = ttk.Label(self.window)
        self.car_dir_label.grid(row=row, column=1, sticky=tk.W)

        self.btn_tub_dir = tk.Button(self.window, text="Tub dir",
                                     command=self.browse_tub,
                                     state=tk.DISABLED)
        self.btn_tub_dir.grid(row=row, column=2, sticky=tk.W)
        self.tub_dir_label = ttk.Label(self.window)
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

        self.var_label = ttk.Label(self.data_frame, text='Add or remove')
        self.var_label.grid(row=row, column=0, sticky=tk.W)

        self.var_menu = ttk.Combobox(self.data_frame, value=self.drop_down,
                                     width=12, state='readonly')
        self.var_menu.bind('<<ComboboxSelected>>', self.add_remove_bars)
        self.var_menu.grid(row=row, column=1, columnspan=2)
        LabelBar.row = row + 1

        # control box
        self.ctr_fram = tk.LabelFrame(self.window, padx=10, pady=10)
        self.ctr_fram.grid(row=row, column=4, columnspan=2, rowspan=4, padx=10)

        self.rec_txt = tk.StringVar(self.ctr_fram)
        self.record_label = ttk.Label(self.ctr_fram, textvariable=self.rec_txt,
                                      font='TkFixedFont')
        self.record_label.grid(row=row, column=0, sticky=tk.E)

        self.speed_var = tk.StringVar(self.window)
        self.speed_var.set("1.00")  # default value
        self.speed_menu = ttk.OptionMenu(self.ctr_fram, self.speed_var, '1.00',
                                         *self.speed_settings,
                                         command=self.set_speed)
        self.speed_menu.grid(row=row, column=1)

        self.btn_bwd = tk.Button(self.ctr_fram, text="<",
                                 command=lambda: self.step(False))
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
                                  state=tk.DISABLED,
                                  command=self.thread_stop)
        self.btn_stop.grid(row=row + 3, column=0, columnspan=2, sticky=tk.NSEW)

        # slider
        row += 4
        self.slider = ttk.Scale(self.window, from_=0, to=self.len - 1,
                                orient=tk.HORIZONTAL, command=self.slide, )
        self.slider.grid(column=0, columnspan=6, sticky=tk.NSEW, padx=10)

        row += 1
        self.btn_set_l = tk.Button(self.window, text="Set left index",
                                   command=lambda: self.set_lr(True))
        self.btn_set_r = tk.Button(self.window, text="Set right index",
                                   command=lambda: self.set_lr(False))
        self.btn_set_l.grid(row=row, column=0, sticky=tk.W, padx=10)
        self.btn_set_r.grid(row=row, column=1, sticky=tk.W)
        self.lr_txt = tk.StringVar(self.window)
        self.lr_txt.set(f'Index range [{self.lr[0]}, {self.lr[1]})')
        self.lr_label = ttk.Label(self.window, textvariable=self.lr_txt)
        self.lr_label.grid(row=row, column=2)
        self.btn_del_lr = tk.Button(self.window, text="Delete",
                                    command=lambda: self.del_lr(True))
        self.btn_undel_lr = tk.Button(self.window, text="Undelete",
                                      command=lambda: self.del_lr(False))
        self.btn_del_lr.grid(row=row, column=3)
        self.btn_undel_lr.grid(row=row, column=4, sticky=tk.E, )
        self.btn_refresh_tub = tk.Button(self.window, text="Reload tub",
                                         command=lambda: self.update_tub(True))
        self.btn_refresh_tub.grid(row=row, column=5, sticky=tk.E, padx=10)

        row += 1
        self.label_filter = tk.Button(text='Set filter',
                                       command=self.update_filter)
        self.label_filter.grid(row=row, column=0, sticky=tk.W, padx=10)
        self.entry_filter_var = tk.StringVar()
        self.entry_filter_var.trace_add("write", self.filter_on_entry)
        self.entry_filter = tk.Entry(self.window,
                                     textvariable=self.entry_filter_var)
        self.entry_filter.grid(row=row, column=1, columnspan=5,
                               sticky=tk.NSEW, padx=10)

        row += 1
        self.figure = Figure(figsize=(6, 4))
        self.graph = FigureCanvasTkAgg(self.figure, self.window)
        self.graph.draw()
        self.graph.get_tk_widget().grid(column=0, columnspan=6, sticky=tk.NSEW,
                                        padx=10, pady=10)
        row += 1
        self.toolbar = NavigationToolbar2Tk(self.graph, self.window,
                                            pack_toolbar=False)
        self.toolbar.update()
        self.toolbar.grid(row=row, column=0, columnspan=5, sticky=tk.W, padx=10)
        self.graph.mpl_connect("key_press_event", self.on_key_press)

        # quit button
        self.but_exit = tk.Button(self.window, text="Quit", command=self.quit)
        self.but_exit.grid(row=row, column=5, sticky=tk.E)
        # status bar
        row += 1
        self.status = ttk.Label(self.window, text="Donkey ready...")
        self.status.grid(row=row, column=0, columnspan=6, sticky=tk.W)

        self.update_config()
        self.update_tub()
        self.update()
        self.window.bind('<Return>', self.enable_keys)

    def enable_keys(self, event):
        self.enable_keys = True

    def on_key_press(self, event):
        key_press_handler(event, self.graph, self.toolbar)

    def step(self, fwd=True):
        self.i += 1 if fwd else -1
        if self.i >= self.len and fwd:
            self.i = 0
        elif self.i < 0 and not fwd:
            self.i = self.len - 1
        self.update()
        if not self.run:
            self.update_status(f'Donkey step '
                               f'{"forward" if fwd else "backward"}')

    def update(self, update_slider=True):
        if self.records is None or self.config is None:
            return
        self.current_rec = self.records[self.i]
        index = self.current_rec.underlying['_index']
        self.rec_txt.set(f"Record {index:06}")
        try:
            self.img = self.get_img(self.current_rec)
            self.img_frame.configure(image=self.img)
            for field, bar in self.bars.items():
                bar.update()
        except KeyError as e:
            print(f"Bad record {index}", e)
        except Exception as e:
            print(f"Bad record {index}", e)
        if update_slider:
            # the slider needs to count continuously through the records
            self.slider.set(self.i)

    def add_remove_bars(self, inp):
        field = self.var_menu.get()
        self.manage_bar_entry(field)
        self.update()
        self.update_plot_from_current_bars()

    def update_plot_from_current_bars(self):
        """ DataFrame for print should contain all bars as long as they don't
            contain strings and everything if bars are empty
        """
        field_map = dict(zip(self.tub.manifest.inputs, self.tub.manifest.types))
        cols = [c for c in self.bars.keys() if decompose(c)[0] in field_map and
                field_map[decompose(c)[0]] != 'str']
        df = self.df[cols] if cols else self.df
        self.update_plot(df)

    def set_speed(self, inp):
        self.speed = float(inp)
        self.update_status(f'Setting speed to {inp} - you can also use the '
                           f'+/- keys for that.')

    def manage_bar_entry(self, field):
        if field in self.bars:
            self.bars[field].destroy()
            del (self.bars[field])

        else:
            self.bars[field] = LabelBar(self, field)

    def loop(self, fwd=True):
        self.update_status('Donkey running...')
        while self.run:
            self.step(fwd)
            time.sleep(1.0 / (self.speed * self.config.DRIVE_LOOP_HZ))
        self.update_status('Donkey stopped')

    def thread_run(self, fwd=True):
        self.run = True
        self.thread = Thread(target=self.loop, args=(fwd,))
        self.thread.start()
        self.btn_play.config(state=tk.DISABLED)
        self.btn_rwd.config(state=tk.DISABLED)
        self.btn_stop.config(state=tk.NORMAL)

    def thread_stop(self):
        self.run = False
        self.btn_play.config(state=tk.NORMAL)
        self.btn_rwd.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)

    def slide(self, val):
        self.i = int(math.floor(float(val)))
        self.update(False)

    def browse_car(self):
        self.car_dir = filedialog.askdirectory(
            initialdir=os.path.expanduser('~'),
            title="Select the car dir")
        if self.update_config():
            self.rc_data['car_dir'] = self.car_dir

    def update_config(self):
        if self.car_dir:
            try:
                self.config = load_config(os.path.join(self.car_dir, 'config.py'))
                self.set_speed(1)
                self.car_dir_label.configure(text=self.car_dir)
                self.btn_tub_dir.configure(state=tk.NORMAL)
                return True
            except FileNotFoundError:
                print(f'Directory {self.car_dir} has no config.py')
            except Exception as e:
                print(e)
            return False

    def browse_tub(self):
        start_dir = self.car_dir if self.car_dir else os.path.expanduser('~')
        self.base_path = filedialog.askdirectory(initialdir=start_dir,
                                                 title="Select the tub dir")
        self.update_tub()
        self.update()
        self.rc_data['last_tub'] = self.base_path

    def set_lr(self, is_l=True):
        self.lr[0 if is_l else 1] = self.current_rec.underlying['_index']
        self.lr_txt.set(f'Index range [{self.lr[0]}, {self.lr[1]})')

    def del_lr(self, is_del):
        del_list = list(range(*self.lr))
        if is_del:
            for d in del_list:
                self.tub.delete_record(d)
            self.update_status(f'Deleting records {self.lr}')
        else:
            for d in del_list:
                self.tub.un_delete_record(d)
            self.update_status(f'Undeleting records {self.lr}')

    def update_filter(self):
        filter = self.entry_filter_var.get()
        # empty string resets the filter
        if filter == '':
            self.record_filter = ''
            self.filter_expression = None
            self.enable_keys = True
            self.rc_data['record_filter'] = self.record_filter
            return
        filter_expression = create_filter_string(
            filter, self.current_rec.underlying.keys(), record_name='record')
        try:
            record = self.current_rec
            res = eval(filter_expression)
            status = f'Filter on current record: {res}'
            if isinstance(res, bool):
                self.record_filter = filter
                self.filter_expression = filter_expression
                self.rc_data['record_filter'] = self.record_filter
            else:
                status += ' - non bool expression can\'t be applied'
            self.update_status(status)
                
        except Exception as e:
            self.update_status(f'Filter error on current record: {e}')

        self.enable_keys = True

    def filter_on_entry(self, a, b, c):
        self.enable_keys = False
        filter_txt = self.entry_filter_var.get()
        self.update_status('Received filter: ' + filter_txt)

    def update_status(self, msg: str) -> None:
        self.status.configure(text=msg)

    def quit(self):
        self.run = False
        write_rc(self.rc_data)
        try:
            self.window.destroy()
        except Exception as e:
            exit(str(e))
        finally:
            exit(0)

    def handle_char_key(self, event=None):
        if not self.enable_keys:
            return
        if event.char == ' ':
            if self.run:
                self.thread_stop()
            else:
                self.thread_run()
        elif event.char == 'q':
            self.quit()
        elif event.char == '+':
            index = self.speed_settings.index(self.speed_var.get())
            if index < len(self.speed_settings) - 1:
                self.speed_var.set(self.speed_settings[index + 1])
                self.set_speed(self.speed_var.get())
        elif event.char == '-':
            index = self.speed_settings.index(self.speed_var.get())
            if index > 0:
                self.speed_var.set(self.speed_settings[index - 1])
                self.set_speed(self.speed_var.get())

    def handle_left_key(self, event=None):
        if self.enable_keys:
            self.step(fwd=False)

    def handle_right_key(self, event=None):
        if self.enable_keys:
            self.step(fwd=True)


if __name__ == "__main__":
    data_rc = read_rc()
    window = tk.Tk()
    # For Linux spice up the optics from 80s to 90s at least
    if platform.system() == 'Linux':
        from ttkthemes import ThemedStyle
        style = ThemedStyle(window)
        style.set_theme("scidblue")
        window.style = ttk.Style()

    ui = TubUI(window, data_rc)
    window.mainloop()
