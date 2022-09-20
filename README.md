# GIMSAT
PID, Rotations, Bluetooth, Tasks, Coordinate Transformation, Sun Sensing

This code allows the Gimbal Satellite to complete 3 main tasks:
1. Rotation about 3-axes
2. Rotation to point at sun (light source)
3. Rotation to point at location on earth (map) relative to position of sun

These are completed using reaction wheels for actuation which rotate at an angular velocity determined by the code given. 
Tasks are carried out by typing in the serial monitor to execute the task, with the task number also stated. This sends a bluetooth signal
from the ground station (laptop) to the GIMSAT and allows the code to run to perform the necessary rotations.  
