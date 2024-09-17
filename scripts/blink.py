#!/usr/bin/env python3

import queue
from threading import Thread
from gpiozero import RGBLED
led = RGBLED(red='GPIO6', green='GPIO13', blue='GPIO19')

red = (1, 0, 0)
green = (0, 1, 0)
blue = (0, 0, 1)
yellow = (1, 1, 0)
q = queue.Queue()
num_tasks = 0


def blink_continuous():
    led.blink(on_color=green, on_time=0.3,
              off_time=0.3, n=5, background=True)


def worker():
    global num_tasks
    while True:
        item = q.get()
        print("Received task...")
        func, kwargs = item
        func(**kwargs)
        print("Finished task")
        num_tasks += 1
        q.task_done()
        blink_continuous()


if __name__ == "__main__":
    run = True
    worker_thread = Thread(target=worker, daemon=True)
    worker_thread.start()
    print('Worker thread started')
    blink_continuous()

    while run:
        val = input("Enter color string of r, b, y or q for quit:\n")
        print(f"Received input {val}")
        if val == 'q':
            run = False
            break
        color = None
        for c in val:
            if c == 'r':
                color = red
            elif c == 'b':
                color = blue
            elif c == 'y':
                color = yellow
            if color:
                item = (led.blink, dict(on_color=color, on_time=0.3,
                                        off_time=0.3, n=5, background=False))
                q.put(item)
                print(f'Task {color} added to queue')
            else:
                print('Invalid color string')

    led.off()
    print(f'Exiting main loop, called {num_tasks} tasks')


