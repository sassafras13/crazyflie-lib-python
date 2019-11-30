# tell ROS this is a Python script
#!/usr/bin/env python

# import libraries
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import logging
import time
from threading import Timer

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander

import rospy
from std_msgs.msg import String

###############
# GLOBAL VARS #
###############

# URI
uri = 'radio://0/80/250K'

# time
d = 4.0 # [sec] total duration of trajectory

# x position
x0 = 0.0 # [m] initial position x
xdot0 = 0 # [m/s] initial velocity xdot
xddot0 = 0 # [m/s2] initial accln xddot

xf = 0.5 # [m] final position x
xdotf = 0 # [m/s] final velocity xdot
xddotf = 0 # [m/s2] final accln xddot

# y position
y0 = 0.0 # [m] initial position y
ydot0 = 0 # [m/s] initial velocity ydot
yddot0 = 0 # [m/s2] initial accln yddot

yf = 0.5 # [m] final position y
ydotf = 0 # [m/s] final velocity ydot
yddotf = 0 # [m/s2] final accln yddot

# z position
z0 = 0.3 # [m] initial position z
zdot0 = 0 # [m/s] initial velocity zdot
zddot0 = 0 # [m/s2] initial accln zddot

zf = 0.8 # [m] final position z
zdotf = 0 # [m/s] final velocity zdot
zddotf = 0 # [m/s2] final accln zddot 

#############
# SETUP LOG #
#############

logging.basicConfig(level=logging.ERROR)

###################
# FN:COEFFICIENTS #
###################

def coefficients(v0, vdot0, vddot0, vf, vdotf, vddotf, d):
    a0 = v0
    a1 = vdot0
    a2 = 0.5*vddot0
    a3 = (-10.0 / (d**3) )*v0 - (6.0 / (d**2) )*vdot0 - (3.0 / (2.0*d) ) *vddot0 + (10.0 / (d**3))*vf - (4.0/(d**2))*vdotf + (1.0/(2.0*d))*vddotf
    a4 = (15.0 / (d**4))*v0 + (8.0 / (d**3))*vdot0 + (3.0 / (2.0*(d**2)))*vddot0 - (15.0 / (d**4))*vf + (7.0 / (d**3))*vdotf - (1.0 / (d**2))*vddotf
    a5 = (-6.0 / (d**5))*v0 - (3.0 / (d**4))*vdot0 - (1.0 / (2.0*(d**3)))*vddot0 + (6.0 / (d**5))*vf - (3.0 / (d**4))*vdotf + (1.0 / (2.0*(d**3)))*vddotf 

    return [a0, a1, a2, a3, a4, a5]

##################
# FN: TRAJECTORY #
##################

def trajectory(coeffs, t): 
    a0 = coeffs[0] 
    a1 = coeffs[1]
    a2 = coeffs[2]
    a3 = coeffs[3]
    a4 = coeffs[4]
    a5 = coeffs[5]

    x = a0 + a1*t + a2*np.power(t,2) + a3*np.power(t,3) + a4*np.power(t,4) + a5*np.power(t,5)

    return x

########################
# FN:FOLLOW TRAJECTORY #
########################

def followTrajectory(tf,dt):
    
    # generate time array
    t = np.arange(0,tf,dt) # [s]

    # generate coefficients in x,y,z
    coeffX = coefficients(x0, xdot0, xddot0, xf, xdotf, xddotf, tf)
    coeffY = coefficients(y0, ydot0, yddot0, yf, ydotf, yddotf, tf)
    coeffZ = coefficients(z0, zdot0, zddot0, zf, zdotf, zddotf, tf)

    # generate trajectory in x,y,z
    trajX = trajectory(coeffX, t)
    trajY = trajectory(coeffY, t)
    trajZ = trajectory(coeffZ, t)

    # generate a plot of trajectory
#    fig = plt.figure()
#    ax1 = fig.add_subplot(211, projection='3d')
#    ax1.scatter(trajX, trajY, trajZ)
#    ax1.set_xlabel('X')
#    ax1.set_ylabel('Y')

#    plt.show()

    # send trajectory commands to crazyflie in a loop
    n = np.prod(t.shape)

#    ax2 = fig.add_subplot(212, projection='3d')
#    plt.show()

    for i in range(n):
        cf.commander.send_position_setpoint(trajX[i], trajY[i], trajZ[i],0)
#        ax2.scatter(trajX[i], trajY[i], trajZ[i])
#        plt.pause(0.05)
        print(' x = ',trajX[i],' y = ',trajY[i],' z = ',trajZ[i],'\n')
        time.sleep(dt)
    cf.commander.send_stop_setpoint()
     
#    plt.show()

################
# FN:GET START #
################
def getStart(topic_name):

    rospy.init_node('pathPlanner', anonymous=True)
    rospy.Subscriber(topic_name, String)

#################
# FN:GET TARGET #
#################
def getTarget(topic_name):
    rospy.Subscriber(topic_name, String)

########
# MAIN #
########
 
if __name__ == '__main__':
    try:
        cflib.crtp.init_drivers(enable_debug_driver=False)

        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

            # reset the Crazyflie's kalman filter
            cf = scf.cf
            cf.param.set_value('kalman.resetEstimation','1')
            time.sleep(0.1)
            cf.param.set_value('kalman.resetEstimation','0')
            time.sleep(2)

            # get the starting point and target point from Optitrack
            topic_name = '/optitrack/rigid_bodies'
            getStart(topic_name)
            getTarget(topic_name)

            # run the trajectory planner
            tf = 10.0 # [s] elapsted time
            dt = 0.5 # [s] timestep
            followTrajectory(tf,dt)

            # land the drone
            cf.commander.send_stop_setpoint()

    except rospy.ROSInterruptException:
        pass
