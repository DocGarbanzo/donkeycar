#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jun 24 20:10:44 2017
@author: wroscoe
remotes.py
The client and web server needed to control a car remotely.
"""


import os
import sys
import json
import logging
import time
import asyncio

import requests
from tornado.ioloop import IOLoop
from tornado.web import Application, RedirectHandler, StaticFileHandler, \
    RequestHandler
import tornado.gen
import tornado.websocket
from socket import gethostname

from ... import utils

logger = logging.getLogger(__name__)


class RemoteWebServer():
    '''
    A controller that repeatedly polls a remote webserver and expects
    the response to be angle, throttle and drive mode.
    '''

    def __init__(self, remote_url, connection_timeout=.25):

        self.control_url = remote_url
        self.time = 0.
        self.angle = 0.
        self.throttle = 0.
        self.mode = 'user'
        self.mode_latch = None
        self.recording = False
        # use one session for all requests
        self.session = requests.Session()

    def update(self):
        '''
        Loop to run in separate thread the updates angle, throttle and
        drive mode.
        '''

        while True:
            # get latest value from server
            self.angle, self.throttle, self.mode, self.recording = self.run()

    def run_threaded(self):
        '''
        Return the last state given from the remote server.
        '''
        return self.angle, self.throttle, self.mode, self.recording

    def run(self):
        '''
        Posts current car sensor data to webserver and returns
        angle and throttle recommendations.
        '''

        data = {}
        response = None
        while response is None:
            try:
                response = self.session.post(self.control_url,
                                             files={'json': json.dumps(data)},
                                             timeout=0.25)

            except requests.exceptions.ReadTimeout as err:
                print("\n Request took too long. Retrying")
                # Lower throttle to prevent runaways.
                return self.angle, self.throttle * .8, None

            except requests.ConnectionError as err:
                # try to reconnect every 3 seconds
                print("\n Vehicle could not connect to server. Make sure you've " +
                    "started your server and you're referencing the right port.")
                time.sleep(3)

        data = json.loads(response.text)
        angle = float(data['angle'])
        throttle = float(data['throttle'])
        drive_mode = str(data['drive_mode'])
        recording = bool(data['recording'])

        return angle, throttle, drive_mode, recording

    def shutdown(self):
        pass


class LocalWebController(tornado.web.Application):

    def __init__(self, port=8887, mode='user'):
        """
        Create and publish variables needed on many of
        the web handlers.
        """
        logger.info('Starting Donkey Server...')

        this_dir = os.path.dirname(os.path.realpath(__file__))
        self.static_file_path = os.path.join(this_dir, 'templates', 'static')
        self.img_arr = None
        self.angle = 0.0
        self.throttle = 0.0
        self.mode = mode
        self.mode_latch = None
        self.recording = False
        self.recording_latch = None
        self.buttons = {}  # latched button values for processing
        self.sliders = {'s1': 1.0, 's2': 1.0, 's3': 1.0}
        self.port = port

        self.num_records = 0
        self.wsclients = []
        self.loop = None
        
        # IMU path visualization data (set by CLI when --web flag used)
        self.imupath_builder = None

        handlers = [
            (r"/", RedirectHandler, dict(url="/drive")),
            (r"/drive", DriveAPI),
            (r"/wsDrive", WebSocketDriveAPI),
            (r"/wsCalibrate", WebSocketCalibrateAPI),
            (r"/calibrate", CalibrateHandler),
            (r"/video", VideoAPI),
            (r"/wsTest", WsTest),
            (r"/imupath", IMUPathHandler),
            (r"/imupath/docs", IMUPathDocsHandler),
            (r"/api/imupath/data", IMUPathDataAPI),
            (r"/api/imupath/fields", IMUPathFieldsAPI),
            (r"/api/imupath/stats", IMUPathStatsAPI),
            (r"/api/imupath/shutdown", IMUPathShutdownAPI),
            (r"/api/imupath/restart", IMUPathRestartAPI),

            (r"/static/(.*)", StaticFileHandler,
             {"path": self.static_file_path}),
        ]

        settings = {'debug': True}
        super().__init__(handlers, **settings)
        logger.info(f"You can now go to {gethostname()}.local:{port} to "
                    f"drive your car.")

    def update(self):
        """ Start the tornado webserver. """
        asyncio.set_event_loop(asyncio.new_event_loop())
        self.listen(self.port)
        self.loop = IOLoop.instance()
        self.loop.start()

    def update_wsclients(self, data):
        if data:
            for wsclient in self.wsclients:
                try:
                    data_str = json.dumps(data)
                    logger.debug(f"Updating web client: {data_str}")
                    wsclient.write_message(data_str)
                except Exception as e:
                    logger.warning("Error writing websocket message",
                                   exc_info=e)
                    pass

    def run_threaded(self, img_arr=None, num_records=0, mode=None, recording=None):
        """
        :param img_arr: current camera image or None
        :param num_records: current number of data records
        :param mode: default user/mode
        :param recording: default recording mode
        """
        self.img_arr = img_arr
        self.num_records = num_records
        #
        # enforce defaults if they are not none.
        #
        changes = {}
        if mode is not None and self.mode != mode:
            self.mode = mode
            changes["driveMode"] = self.mode
        if self.mode_latch is not None:
            self.mode = self.mode_latch
            self.mode_latch = None
            changes["driveMode"] = self.mode
        if recording is not None and self.recording != recording:
            self.recording = recording
            changes["recording"] = self.recording
        if self.recording_latch is not None:
            self.recording = self.recording_latch
            self.recording_latch = None
            changes["recording"] = self.recording

        # Send record count to websocket clients
        if self.num_records is not None and self.recording is True:
            if self.num_records % 10 == 0:
                changes['num_records'] = self.num_records

        #
        # get latched button presses then clear button presses
        # Next iteration will clear press in memory
        #
        buttons = self.buttons
        self.buttons = {}
        for button, pressed in buttons.items():
            if pressed:
                self.buttons[button] = False

        # if there were changes, then send to web client
        if changes and self.loop is not None:
            logger.debug(str(changes))
            self.loop.add_callback(lambda: self.update_wsclients(changes))

        return self.angle, self.throttle, self.mode, self.recording, buttons,\
            self.sliders

    def run(self, img_arr=None, num_records=0, mode=None, recording=None):
        return self.run_threaded(img_arr, num_records, mode, recording)

    def shutdown(self):
        pass


class DriveAPI(RequestHandler):

    def get(self):
        data = {}
        self.render("templates/vehicle.html", **data)

    def post(self):
        '''
        Receive post requests as user changes the angle
        and throttle of the vehicle on the index webpage
        '''
        data = tornado.escape.json_decode(self.request.body)

        if data.get('angle') is not None:
            self.application.angle = data['angle']
        if data.get('throttle') is not None:
            self.application.throttle = data['throttle']
        if data.get('drive_mode') is not None:
            self.application.mode = data['drive_mode']
        if data.get('recording') is not None:
            self.application.recording = data['recording']
        if data.get('buttons') is not None:
            print('Received push')
            latch_buttons(self.application.buttons, data['buttons'])
        if data.get('sliders') is not None:
            print(data['sliders'])
            self.application.sliders.update(data['sliders'])


class WsTest(RequestHandler):
    def get(self):
        data = {}
        self.render("templates/wsTest.html", **data)


class CalibrateHandler(RequestHandler):
    """ Serves the calibration web page"""
    async def get(self):
        await self.render("templates/calibrate.html")


def latch_buttons(buttons, pushes):
    """
    Latch button pushes
    buttons: the latched values
    pushes: the update value
    """
    if pushes is not None:
        #
        # we got button pushes.
        # - we latch the pushed buttons so we can process the push
        # - after it is processed we clear it
        #
        for button in pushes:
            # if pushed, then latch it
            if pushes[button]:
                buttons[button] = True


class WebSocketDriveAPI(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        logger.info("New client connected")
        self.application.wsclients.append(self)

    def on_message(self, message):
        data = json.loads(message)
        self.application.angle = data.get('angle', self.application.angle)
        self.application.throttle = data.get('throttle', self.application.throttle)
        if data.get('drive_mode') is not None:
            self.application.mode = data['drive_mode']
            self.application.mode_latch = self.application.mode
        if data.get('recording') is not None:
            self.application.recording = data['recording']
            self.application.recording_latch = self.application.recording
        if data.get('buttons') is not None:
            latch_buttons(self.application.buttons, data['buttons'])
        if data.get('sliders') is not None:
            self.application.sliders.update(data['sliders'])

    def on_close(self):
        logger.info("Client disconnected")
        self.application.wsclients.remove(self)


class WebSocketCalibrateAPI(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        logger.info("New client connected")

    def on_message(self, message):
        logger.info(f"wsCalibrate {message}")
        data = json.loads(message)
        if 'throttle' in data:
            print(data['throttle'])
            self.application.throttle = data['throttle']

        if 'angle' in data:
            print(data['angle'])
            self.application.angle = data['angle']

        if 'config' in data:
            config = data['config']
            if self.application.drive_train_type == "PWM_STEERING_THROTTLE" \
                or self.application.drive_train_type == "I2C_SERVO":
                if 'STEERING_LEFT_PWM' in config:
                    self.application.drive_train['steering'].left_pulse = config['STEERING_LEFT_PWM']

                if 'STEERING_RIGHT_PWM' in config:
                    self.application.drive_train['steering'].right_pulse = config['STEERING_RIGHT_PWM']

                if 'THROTTLE_FORWARD_PWM' in config:
                    self.application.drive_train['throttle'].max_pulse = config['THROTTLE_FORWARD_PWM']

                if 'THROTTLE_STOPPED_PWM' in config:
                    self.application.drive_train['throttle'].zero_pulse = config['THROTTLE_STOPPED_PWM']

                if 'THROTTLE_REVERSE_PWM' in config:
                    self.application.drive_train['throttle'].min_pulse = config['THROTTLE_REVERSE_PWM']

            elif self.application.drive_train_type == "MM1":
                if ('MM1_STEERING_MID' in config) and (config['MM1_STEERING_MID'] != 0):
                        self.application.drive_train.STEERING_MID = config['MM1_STEERING_MID']
                if ('MM1_MAX_FORWARD' in config) and (config['MM1_MAX_FORWARD'] != 0):
                        self.application.drive_train.MAX_FORWARD = config['MM1_MAX_FORWARD']
                if ('MM1_MAX_REVERSE' in config) and (config['MM1_MAX_REVERSE'] != 0):
                    self.application.drive_train.MAX_REVERSE = config['MM1_MAX_REVERSE']

    def on_close(self):
        logger.info("Client disconnected")


class VideoAPI(RequestHandler):
    '''
    Serves a MJPEG of the images posted from the vehicle.
    '''

    async def get(self):
        placeholder_image = utils.load_image_sized(
                        os.path.join(self.application.static_file_path,
                                     "img_placeholder.jpg"), 160, 120, 3)

        self.set_header("Content-type",
                        "multipart/x-mixed-replace;boundary=--boundarydonotcross")

        served_image_timestamp = time.time()
        my_boundary = "--boundarydonotcross\n"
        while True:

            interval = .005
            if served_image_timestamp + interval < time.time():
                #
                # if we have an image, then use it.
                # otherwise show placeholder
                #
                if hasattr(self.application, 'img_arr') and self.application.img_arr is not None:
                    img = utils.arr_to_binary(self.application.img_arr)
                else:
                    img = utils.arr_to_binary(placeholder_image)

                self.write(my_boundary)
                self.write("Content-type: image/jpeg\r\n")
                self.write("Content-length: %s\r\n\r\n" % len(img))
                self.write(img)
                served_image_timestamp = time.time()
                try:
                    await self.flush()
                except tornado.iostream.StreamClosedError:
                    pass
            else:
                await tornado.gen.sleep(interval)


class IMUPathHandler(RequestHandler):
    """Serves the IMU path visualization page."""

    async def get(self):
        data = {}
        await self.render("templates/imupath.html", **data)


class IMUPathDocsHandler(RequestHandler):
    """Serves the IMU path visualizer user documentation."""

    async def get(self):
        data = {}
        await self.render("templates/imupath_docs.html", **data)


class IMUPathFieldsAPI(RequestHandler):
    """
    API endpoint for getting available fields from tub data.
    Returns list of fields with metadata (type, dimensions, etc.)
    """

    async def get(self):
        # Check if data is available
        if self.application.imupath_builder is None:
            self.set_status(404)
            self.write({'error': 'No IMU path data loaded'})
            return

        try:
            fields = self.application.imupath_builder.get_available_fields()
            self.set_header('Content-Type', 'application/json')
            self.write(json.dumps({'fields': fields}, ensure_ascii=False,
                                 separators=(',', ':')))
        except Exception as e:
            logger.error(f"Error getting available fields: {e}",
                        exc_info=True)
            self.set_status(500)
            self.write({'error': str(e)})


class IMUPathStatsAPI(RequestHandler):
    """
    API endpoint for computing segment statistics on-demand.
    Query parameters:
        - field: Field name (required)
        - method: Aggregation method (required)
        - dimension: Vector component index (optional, for vector fields)
    """

    async def get(self):
        # Check if data is available
        if self.application.imupath_builder is None:
            self.set_status(404)
            self.write({'error': 'No IMU path data loaded'})
            return

        # Get query parameters
        field_name = self.get_argument('field', default=None)
        method = self.get_argument('method', default=None)
        dimension_str = self.get_argument('dimension', default=None)

        # Validate required parameters
        if not field_name or not method:
            self.set_status(400)
            self.write({'error': 'Missing required parameters: field, method'})
            return

        # Parse dimension parameter
        dimension = None
        if dimension_str is not None and dimension_str != 'null' and dimension_str != '':
            try:
                dimension = int(dimension_str)
            except ValueError:
                self.set_status(400)
                self.write({'error': 'Invalid dimension parameter: must be integer'})
                return

        try:
            # Compute segment statistics
            rankings = self.application.imupath_builder.compute_segment_statistics(
                field_name, method, dimension)

            self.set_header('Content-Type', 'application/json')
            self.write(json.dumps({'rankings': rankings}, ensure_ascii=False,
                                 separators=(',', ':')))
        except Exception as e:
            logger.error(f"Error computing segment statistics: {e}",
                        exc_info=True)
            self.set_status(500)
            self.write({'error': str(e)})


class SecuredAPIHandler(RequestHandler):
    """Base handler with origin validation and JSON parsing."""

    def validate_origin(self):
        """
        Validate Origin header matches Host to prevent CSRF.
        Returns True if valid, sets error response and returns False otherwise.
        """
        origin = self.request.headers.get('Origin')
        if not origin:
            return True
        host = self.request.headers.get('Host')
        origin_host = origin.split('://')[-1]
        if origin_host != host:
            logger.warning(f"Request blocked: origin={origin} host={host}")
            self.set_status(403)
            self.write({'error': 'Forbidden'})
            return False
        return True

    def parse_json_body(self):
        """
        Parse request body as JSON.
        Returns parsed data on success, None on error (sets error response).
        """
        try:
            return tornado.escape.json_decode(self.request.body)
        except (ValueError, TypeError, UnicodeDecodeError) as e:
            logger.warning(f"Invalid JSON in request: {e}")
            self.set_status(400)
            self.write({'error': 'Invalid JSON'})
            return None

    def validate_confirmation(self, data, expected_value):
        """
        Validate confirmation field in request data.
        Returns True if valid, sets error response and returns False otherwise.
        """
        if data.get('confirm') != expected_value:
            self.set_status(400)
            self.write({'error': 'Missing confirmation'})
            return False
        return True


class IMUPathShutdownAPI(SecuredAPIHandler):
    """
    API endpoint to shutdown the server.
    Uses Origin header validation to prevent cross-origin requests.
    Intended for local development use only.
    """

    async def post(self):
        if not self.validate_origin():
            return
        data = self.parse_json_body()
        if data is None:
            return
        if not self.validate_confirmation(data, 'shutdown'):
            return
        self.write({'status': 'shutting_down'})
        await self.finish()
        IOLoop.current().call_later(0.5, self._shutdown)

    def _shutdown(self):
        logger.info("Shutdown requested via web UI")
        IOLoop.current().stop()


class IMUPathRestartAPI(SecuredAPIHandler):
    """
    API endpoint to restart the server.
    Uses Origin header validation to prevent cross-origin requests.
    Intended for local development use only.
    """

    async def post(self):
        if not self.validate_origin():
            return
        data = self.parse_json_body()
        if data is None:
            return
        if not self.validate_confirmation(data, 'restart'):
            return
        self.write({'status': 'restarting'})
        await self.finish()
        IOLoop.current().call_later(0.5, self._restart)

    def _restart(self):
        logger.info("Server restart requested via web UI")
        python = sys.executable
        os.execv(python, [python] + sys.argv)


class IMUPathDataAPI(RequestHandler):
    """
    API endpoint for IMU path data.
    Returns JSON data for visualization.
    """
    
    async def get(self):
        # Get optional query parameters
        num_laps = self.get_argument('num_laps', default=None)
        segment_method = self.get_argument('segment_method', default=None)
        
        # Validate segment_method parameter
        valid_methods = ['threshold', 'extrema', 'gradient', 'hybrid']
        if segment_method is not None and segment_method not in valid_methods:
            self.set_status(400)
            msg = (f'Invalid segment_method. '
                   f'Must be one of: {", ".join(valid_methods)}')
            self.write({'error': msg})
            return
        
        # Convert parameters with error handling
        try:
            if num_laps is not None:
                num_laps = int(num_laps)
        except ValueError:
            self.set_status(400)
            self.write({'error': 'Invalid query parameter: num_laps must be an integer.'})
            return
        
        # Check if data is available
        if self.application.imupath_builder is None:
            self.set_status(404)
            self.write({'error': 'No IMU path data loaded'})
            return
        
        try:
            # Build JSON payload
            data = self.application.imupath_builder.build_json_payload(
                num_laps=num_laps,
                segment_method=segment_method
            )
            
            self.set_header('Content-Type', 'application/json')
            self.write(json.dumps(data, ensure_ascii=False, separators=(',', ':')))
        except Exception as e:
            logger.error(f"Error building IMU path data: {e}", exc_info=True)
            self.set_status(500)
            self.write({'error': str(e)})


class BaseHandler(RequestHandler):
    """ Serves the FPV web page"""
    async def get(self):
        data = {}
        await self.render("templates/base_fpv.html", **data)


class WebFpv(Application):
    """
    Class for running an FPV web server that only shows the camera in real-time.
    The web page contains the camera view and auto-adjusts to the web browser
    window size. Conjecture: this picture up-scaling is performed by the
    client OS using graphics acceleration. Hence a web browser on the PC is
    faster than a pure python application based on open cv or similar.
    """

    def __init__(self, port=8890):
        self.port = port
        this_dir = os.path.dirname(os.path.realpath(__file__))
        self.static_file_path = os.path.join(this_dir, 'templates', 'static')

        """Construct and serve the tornado application."""
        handlers = [
            (r"/", BaseHandler),
            (r"/video", VideoAPI),
            (r"/static/(.*)", StaticFileHandler,
             {"path": self.static_file_path})
        ]

        settings = {'debug': True}
        self.img_arr = None
        super().__init__(handlers, **settings)
        logger.info(f"Started Web FPV server. You can now go to "
                    f"{gethostname()}.local:{self.port} to view the car camera")

    def update(self):
        """ Start the tornado webserver. """
        asyncio.set_event_loop(asyncio.new_event_loop())
        self.listen(self.port)
        IOLoop.instance().start()

    def run_threaded(self, img_arr=None):
        self.img_arr = img_arr

    def run(self, img_arr=None):
        self.img_arr = img_arr

    def shutdown(self):
        pass


