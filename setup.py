from setuptools import setup, find_packages

import os
from donkeycar import __version__

# include the non python files
def package_files(directory, strip_leading):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            package_file = os.path.join(path, filename)
            paths.append(package_file[len(strip_leading):])
    return paths


car_templates = ['templates/*']
web_controller_html = package_files('donkeycar/parts/controllers/templates', 'donkeycar/')

extra_files = car_templates + web_controller_html
print('extra_files', extra_files)

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(name='donkeycar',
    version=__version__,
    long_description = long_description,
    description='Self driving library for python.',
    url='https://github.com/autorope/donkeycar',
    author='Will Roscoe, Adam Conway, Tawn Kramer',
    author_email='wroscoe@gmail.com, adam@casaconway.com, tawnkramer@gmail.com',
    license='MIT',
    entry_points={
        'console_scripts': [
            'donkey=donkeycar.management.base:execute_from_command_line',
        ],
    },
    install_requires=['numpy', 
                      'pillow',
                      'docopt',
                      'tornado',
                      'requests',
                      'h5py',
                      'moviepy',
                      'pandas',
                      'PrettyTable',
                      'paho-mqtt',
                      'progress'
                     ],

    extras_require={
                    'pi': [
                        'picamera',
                        'Adafruit_PCA9685',
                        'Adafruit_SSD1306',
                        'RPi.GPIO',
                        'pyserial',
                        ],
                    'nano': [
                        'Adafruit_PCA9685',
                        'Adafruit_SSD1306',
                        ],
                    'pc': [
                        'matplotlib',
                        ],
                    'dev' : [
                        'pytest',
                        'pytest-cov',
                        'responses',
                        ],
                    'ci': ['codecov'],
                    'tf': ['tensorflow==1.13.1'],
                    'tf_gpu': ['tensorflow-gpu==1.13.1'],
                    },
    package_data={
        'donkeycar': extra_files, 
        },

      include_package_data=True,

      classifiers=[
          # How mature is this project? Common values are
          #   3 - Alpha
          #   4 - Beta
          #   5 - Production/Stable
          'Development Status :: 3 - Alpha',

          # Indicate who your project is intended for
          'Intended Audience :: Developers',
          'Topic :: Scientific/Engineering :: Artificial Intelligence',

          # Pick your license as you wish (should match "license" above)
          'License :: OSI Approved :: MIT License',

          # Specify the Python versions you support here. In particular, ensure
          # that you indicate whether you support Python 2, Python 3 or both.

          'Programming Language :: Python :: 3.5',
          'Programming Language :: Python :: 3.6',
          'Programming Language :: Python :: 3.7',
      ],
      keywords='selfdriving cars donkeycar diyrobocars',

      packages=find_packages(exclude=(['tests', 'docs', 'site', 'env'])),
      )
