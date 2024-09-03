import time
import numpy as np
import logging
from threading import Thread
from .memory import Memory
from prettytable import PrettyTable
import traceback
from donkeycar.parts.pico import instance

logger = logging.getLogger(__name__)


class PartProfiler:
    def __init__(self):
        self.records = {}

    def profile_part(self, p):
        self.records[p] = {'start_time': 0.0, 'dt': []}

    def on_part_start(self, p):
        self.records[p]['start_time'] = time.time()

    def on_part_finished(self, p):
        now = time.time()
        start_time = self.records[p]['start_time']
        thresh = 0.000001
        delta = max(now - start_time, thresh)
        self.records[p]['dt'].append(delta)

    def report(self):
        logger.info("Part Profile Summary: (times in ms)")
        pt = PrettyTable()
        field_names = ["part", "max", "min", "avg"]
        pctile = [50, 90, 99, 99.9]
        pt.field_names = field_names + [str(p) + '%' for p in pctile]
        for p, val in self.records.items():
            # remove first and last entry because you there could be one-off
            # time spent in initialisations, and the latest diff could be
            # incomplete because of user keyboard interrupt
            arr = val['dt'][1:-1]
            if len(arr) == 0:
                continue
            row = [p.__class__.__name__,
                   "%.2f" % (max(arr) * 1000),
                   "%.2f" % (min(arr) * 1000),
                   "%.2f" % (sum(arr) / len(arr) * 1000)]
            row += ["%.2f" % (np.percentile(arr, p) * 1000) for p in pctile]
            pt.add_row(row)
        logger.info('\n' + str(pt))


class Vehicle:
    def __init__(self, mem=None):

        if not mem:
            mem = Memory()
        self.mem = mem
        self.parts = []
        self.on = True
        self.threads = []
        self.profiler = PartProfiler()
        self.loop_count = 0
        self.loop_exceed = 0
        self.excess_time = 0.0
        self.run_time = 0.0

    def add(self, part, inputs=[], outputs=[],
            threaded=False, run_condition=None):
        """
        Method to add a part to the vehicle drive loop.

        Parameters
        ----------
            part: class
                donkey vehicle part has run() attribute
            inputs : list
                Channel names to get from memory.
            outputs : list
                Channel names to save to memory.
            threaded : boolean
                If a part should be run in a separate thread.
            run_condition : str
                Channel name if a part should be run or not
        """
        assert type(inputs) is list, f"inputs is not a list: {repr(inputs)}"
        assert type(outputs) is list, f"outputs is not a list: {repr(outputs)}"
        assert type(threaded) is bool, f"threaded is not a boolean: " \
                                       f"{repr(threaded)}"

        if run_condition:
            assert type(run_condition) is str, \
                f"run_condition is not a str: {repr(threaded)}"

        logger.info(f'Adding part {part.__class__.__name__}.')
        entry = {'part': part, 'inputs': inputs, 'outputs': outputs,
                 'run_condition': run_condition}
        all_parts = [p['part'] for p in self.parts]
        # allow adding a part multiple times, but start its thread only once
        if threaded and part not in all_parts:
            t = Thread(target=part.update, args=())
            t.daemon = True
            entry['thread'] = t

        self.parts.append(entry)
        self.profiler.profile_part(part)

    def remove(self, part):
        """
        remove part form list
        """
        self.parts.remove(part)

    def start(self, rate_hz=10, max_loop_count=None):
        """
        Start vehicle's main drive loop.

        This is the main thread of the vehicle. It starts all the new
        threads for the threaded parts then starts an infinite loop
        that runs each part and updates the memory.

        Parameters
        ----------

        rate_hz : int
            The max frequency that the drive loop should run. The actual
            frequency may be less than this if there are many blocking parts.
        max_loop_count : int
            Maximum number of loops the drive loop should execute. This is
            used for testing that all the parts of the vehicle work.
        """
        loop_time = 1.0 / rate_hz
        self.loop_count = 0
        self.loop_exceed = 0
        self.excess_time = 0.0
        self.run_time = 0.0
        try:
            self.on = True
            for entry in self.parts:
                if entry.get('thread'):
                    # start the update thread
                    entry.get('thread').start()

            # wait until the parts warm up.
            logger.info(f'Starting vehicle at {rate_hz} Hz')

            while self.on:
                start_time = time.time()
                self.update_parts()
                # stop drive loop if loop_count exceeds max_loop_count
                if max_loop_count and self.loop_count > max_loop_count:
                    self.on = False
                # stop drive loop if a part publishes a 'user/stop' entry
                stop = self.mem.get(['user/stop'])
                if stop and stop[0]:
                    self.on = False
                this_loop_time = time.time() - start_time
                sleep_time = loop_time - this_loop_time
                self.run_time += this_loop_time
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
                else:
                    self.loop_exceed += 1
                    self.excess_time -= sleep_time
                self.loop_count += 1
                # print a message when exceeding more than 1ms but at
                # most all 10s:
                if self.loop_count % (10 * rate_hz) == 0:
                    avg_exceed_time = 1e3 * self.excess_time / self.loop_count
                    if avg_exceed_time > 1:
                        logger.warning(f'jitter violation in vehicle loop with '
                                       f'{avg_exceed_time:5.1f}ms')

        except KeyboardInterrupt:
            pass
        except Exception as e:
            traceback.print_exc()
        finally:
            self.stop()
            instance.stop()
            logging.shutdown()

    def update_parts(self):
        '''
        loop over all parts
        '''
        for entry in self.parts:
            run = True
            # check run condition, if it exists
            if entry.get('run_condition'):
                run_condition = entry.get('run_condition')
                run = self.mem.get([run_condition])[0]
            
            if run:
                # get part
                p = entry['part']
                # start timing part run
                self.profiler.on_part_start(p)
                # get inputs from memory
                inputs = self.mem.get(entry['inputs'])
                # run the part
                if entry.get('thread'):
                    outputs = p.run_threaded(*inputs)
                else:
                    outputs = p.run(*inputs)
                # save the output to memory
                if outputs is not None:
                    self.mem.put(entry['outputs'], outputs)
                # finish timing part run
                self.profiler.on_part_finished(p)

    def stop(self):        
        logger.info('Shutting down vehicle and its parts...')
        for entry in self.parts:
            try:
                entry['part'].shutdown()
            except AttributeError:
                # usually from missing shutdown method, which should be optional
                pass
            except Exception as e:
                logger.error(e)

        count = max(self.loop_count, 1)
        logger.info(f'Ran {self.loop_count} vehicle loops with '
                    f'{100.0 * self.loop_exceed / count:.2f}% ' 
                    f'exceeding loop time.')
        logger.info(f'Average excess time '
                    f'{1000.0 * self.excess_time / count:.1f}ms, average loop '
                    f'time {1000.0 * self.run_time / count:.1f}ms.')

        self.profiler.report()
