import os
import logging


KIVY_MODE = 'KIVY'
PYTHON_MODE = 'PYTHON'


def configure_kivy_logging():
    os.environ.setdefault('KIVY_LOG_MODE', KIVY_MODE)
    mode = os.environ['KIVY_LOG_MODE']
    if mode == PYTHON_MODE:
        return
    # Must happen before kivy is imported: kivy attaches its
    # ConsoleHandler (and logs its startup banner through it) at
    # import time, so removing the handler afterwards is too late.
    os.environ.setdefault('KIVY_NO_CONSOLELOG', '1')
    _remove_root_stream_handlers()


def _remove_root_stream_handlers():
    root = logging.getLogger()
    handlers = list(root.handlers)
    for handler in handlers:
        if not isinstance(handler, logging.StreamHandler):
            continue
        root.removeHandler(handler)
