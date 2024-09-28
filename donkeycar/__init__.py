import os
import sys
from pyfiglet import Figlet
import logging
from pkg_resources import get_distribution

__version__ = '5.1.dev1+DocGarbanzo'

log_level = os.environ.get('DONKEY_LOG_LEVEL', 'INFO')
logging.basicConfig(level=log_level)
logger = logging.getLogger(__name__)

f = Figlet(font='speed')


print(f.renderText('Donkey Car'))
print(f'using donkey v{__version__} ...')
print(f'using log level {log_level} ...')

if sys.version_info.major < 3 or sys.version_info.minor < 8:
    msg = f'Donkey Requires Python 3.8 or greater. You are using {sys.version}'
    raise ValueError(msg)

# The default recursion limits in CPython are too small.
sys.setrecursionlimit(10**5)

from .vehicle import Vehicle
from .memory import Memory
from . import utils
from . import config
from . import contrib
from .config import load_config
