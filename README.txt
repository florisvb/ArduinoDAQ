This is a simple collection of code to turn an arduino into a DAQ board that streams data from one of the analog inputs to the computer. You can either use the python api to acquire and/or save data, or you can stream data using the ros package.

Expect ~1 kHz data rate if streaming data and time stamps.

If streaming to ROS, the default is not to stream time stamps - expect ~2 kHz.



Some useful knowledge bits:

On Ubuntu you can find a persistent port for your arduino here:

/dev/serial/by-id/

if you have permissions issues, add your user to the group dialout. on Ubuntu:

$ useradd -G {group-name} username
