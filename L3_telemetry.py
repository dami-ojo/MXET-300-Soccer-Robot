# This example drives the right and left motors.
# Intended for Beaglebone Blue hardware.
# This example uses rcpy library. Documentation: guitar.ucsd.edu/rcpy/rcpy.pdf

# Import external libraries
import rcpy
import rcpy.motor as motor
import time                                     # only necessary if running this program as a loop
import numpy as np                              # for clip function
import L2_find_ball as find
import L2_heading as heading
import L2_inverse_kinematics as inv
import L2_speed_control as sc
from math import *

motor_l = 1 	                                # Left Motor (ch1)
motor_r = 2 	                                # Right Motor (ch2)
R = 0.041
L = 0.201

# NOTE: THERE ARE 4 OUTPUTS.  3 & 4 ACCESSIBLE THROUGH diode & accy functions

rcpy.set_state(rcpy.RUNNING)                    # initialize the rcpy library


# define functions to command motors, effectively controlling PWM
def MotorL(speed):                              # takes argument in range [-1,1]
    motor.set(motor_l, speed)


def MotorR(speed):                              # takes argument in range [-1,1]
    motor.set(motor_r, speed)


def diode(state, channel):                      # takes argument in range [0,1]
    np.clip(state, 0, 1)                        # limit the output, disallow negative voltages
    motor.set(channel, state)


def accy(state, channel):                       # takes argument in range [-1,1]
    motor.set(channel, state)


if __name__ == "__main__":
    g = (114.5)
    g *= (pi/180)
    while rcpy.get_state() != rcpy.EXITING:     # exit loop if rcpy not ready
        if rcpy.get_state() == rcpy.RUNNING:    # execute loop when rcpy is ready
            ballData = find.findBall()
            axes = heading.getXY()                              # call xy function
            axesScaled = heading.scale(axes)                    # perform scale function
            h = heading.getHeading(axesScaled)
            h *= (pi/180)
            print('Heading:', h * (180/pi), 'Goal angle:', g * (180/pi))
            
            #back up
            s = 0.75
            xd = -0.1/s
            pd = inv.convert(np.array([xd, 0]))
            sc.driveOpenLoop(pd)
            time.sleep(s)
            
            #turn to 90 from ball
            s = 1.5
            if h > g:
                deltaT = (g-(pi/2)) - h
            else:
                deltaT = (g+(pi/2)) - h
            td = deltaT/s
            #xd = 0.2*abs(deltaT)/1.5
            pd = inv.convert(np.array([0, td]))
            sc.driveOpenLoop(pd)
            time.sleep(s)
            
            #drive to in line w ball
            s = 1
            xd = ((ballData[1] + 0.1) * cos(abs(deltaT))) / s
            pd = inv.convert(np.array([xd, 0]))
            sc.driveOpenLoop(pd)
            time.sleep(s)
            
            #turn 90
            s = 0.5
            if h > g:
                td = (pi/2)/s
            else:
                td = -(pi/2)/s
            pd = inv.convert(np.array([0, td]))
            sc.driveOpenLoop(pd)
            time.sleep(s)
            
            #drive and kick
            s = 1
            xd = ((ballData[1] + 0.1) * sin(abs(deltaT)) ) / s
            pd = inv.convert(np.array([xd, 0]))
            sc.driveOpenLoop(pd)
            time.sleep(s)
