from art import *
__version__ = '3.1.2 DocGarbanzo'

logo = text2art('Donkey Car', font='speed')
print(logo)
print('using donkey v{} ...'.format(__version__))

import sys

if sys.version_info.major < 3:
    msg = 'Donkey Requires Python 3.4 or greater. You are using {}'\
        .format(sys.version)
    raise ValueError(msg)

from . import parts
from .vehicle import Vehicle
from .memory import Memory
from . import utils
from . import config
from . import contrib
from .config import load_config
