from distutils.core import setup

setup(
    name='ArduinoDAQ',
    version='0.0.1',
    author='Floris van Breugel',
    author_email='floris@caltech.edu',
    packages = ['arduino_daq'],
    license='BSD',
    description='python api for streaming data',
    long_description=open('README.txt').read(),
)



