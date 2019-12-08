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
from cflib.positioning.motion_commander import MotionCommander

import rospy
from std_msgs.msg import String
# Messages
from geometry_msgs.msg import Pose, Point, Quaternion
from optitrack.msg import RigidBody, RigidBodyArray
from optitrack.utils import get_ip_address, read_parameter
#tf stuff for converting quaternion to euler cuz i dont wanna do it myself. Thanks grant for the tip man
from tf.transformations import euler_from_quaternion



###############
# GLOBAL VARS #
###############

# URI
uri = 'radio://0/80/250K'

# time
d = 4.0 # [sec] total duration of trajectory

# initial conditions x 
x0 = 0 
xdot0 = 0 # [m/s] initial velocity xdot
xddot0 = 0 # [m/s2] initial accln xddot

xdotf = 0 # [m/s] final velocity xdot
xddotf = 0 # [m/s2] final accln xddot

# initial conditions y
ydot0 = 0 # [m/s] initial velocity ydot
yddot0 = 0 # [m/s2] initial accln yddot

ydotf = 0 # [m/s] final velocity ydot
yddotf = 0 # [m/s2] final accln yddot

# initial conditions z 
zdot0 = 0 # [m/s] initial velocity zdot
zddot0 = 0 # [m/s2] initial accln zddot

zf_margin = 0.1 # [m] final position margin
zdotf = 0 # [m/s] final velocity zdot
zddotf = 0 # [m/s2] final accln zddot 

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

def followTrajectory(tf,dt,adjStartPos,adjTargetPos):
    eps_z = 0.1
    eps_x = 0.05
    eps_y = 0.05
    #print('i am in trajectory')

    # pull out position coordinates
    x0 = adjStartPos[0]
    y0 = adjStartPos[1]
    z0 = adjStartPos[2]

    xf = adjTargetPos[0]
    yf = adjTargetPos[1]
    zf = adjTargetPos[2]

    # generate time array
    t = np.arange(0,tf,dt) # [s]

    # add margin to final z position
    zf = zf + zf_margin # [m]

    # generate coefficients in x,y,z
    coeffX = coefficients(x0, xdot0, xddot0, xf, xdotf, xddotf, tf)
    coeffY = coefficients(y0, ydot0, yddot0, yf, ydotf, yddotf, tf)
    coeffZ = coefficients(z0, zdot0, zddot0, zf, zdotf, zddotf, tf)

    # generate trajectory in x,y,z
    trajX = trajectory(coeffX, t)
    trajY = trajectory(coeffY, t)
    trajZ = trajectory(coeffZ, t)

    # generate a plot of trajectory
    # fig = plt.figure()
    # ax1 = fig.add_subplot(211, projection='3d')
    # ax1.scatter(trajX, trajY, trajZ)
    # ax1.set_xlabel('X')
    # ax1.set_ylabel('Y')
    # plt.show()

    # send trajectory commands to crazyflie in a loop
    n = np.prod(t.shape)

    dx = trajX[1] - trajX[0]
    dy = trajY[1] - trajY[0]
    dz = trajZ[1] - trajZ[0]
    mc.move_distance(dx,dy,dz)
    #cf.commander.send_position_setpoint(trajX[i], trajY[i], trajZ[i],0)
    cf.commander.send_hover_setpoint(0,0,0,dz)
    print(' x = ',dx,' y = ',dy,' z = ',dz,'\n')
    
    if abs(dx) < eps_x and abs(dy) < eps_y and abs(dz) < eps_z:
        flag = True
    else: 
        flag = False

    return flag
     

#####################
# FN:GET START DATA #
#####################
def getStartData(topic_name): 
    data = rospy.wait_for_message(topic_name, RigidBodyArray)
    #print(data)
    #print(data.bodies[0].pose.position.x)
    x = data.bodies[0].pose.position.x
    y = data.bodies[0].pose.position.y
    z = data.bodies[0].pose.position.z
    # yaw
    quat_ori = data.bodies[0].pose.orientation
    ori_list = [quat_ori.x, quat_ori.y, quat_ori.z, quat_ori.w]
    (roll, pitch, yaw) = euler_from_quaternion(ori_list)
    yaw = abs(yaw * (180. / 3.14159))
    return [x,y,z,yaw]

######################
# FN:GET TARGET DATA #
######################
def getTargetData(topic_name): 
    data = rospy.wait_for_message(topic_name, RigidBodyArray)
    #print(data)
    #print(data.bodies[1].pose.position.x)
    x = data.bodies[1].pose.position.x
    y = data.bodies[1].pose.position.y
    z = data.bodies[1].pose.position.z


    return [x,y,z]

####################
# FN:ADJUST TARGET #
####################
def adjustTarget(targetPos):
    x = targetPos[0]
    y = targetPos[1]
    z = targetPos[2]

    xa = x - x
    ya = y - y
    za = z - z

    return [xa,ya,za]

###################
# FN:ADJUST START #
###################
def adjustStart(startPos,targetPos):
    xs = startPos[0]
    ys = startPos[1]
    zs = startPos[2]

    xt = targetPos[0]
    yt = targetPos[1]
    zt = targetPos[2]

    xa = xs - xt 
    ya = ys - yt
    za = zs - zt

    return [xa,ya,za]


########
# MAIN #
########
 
if __name__ == '__main__':
    # initialize this script as a ROS node
    rospy.init_node('pathPlanner', anonymous=True)

    try:
        cflib.crtp.init_drivers(enable_debug_driver=False)

        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

            # reset the Crazyflie's kalman filter
            cf = scf.cf
            cf.param.set_value('kalman.resetEstimation','1')
            time.sleep(0.1)
            cf.param.set_value('kalman.resetEstimation','0')
            time.sleep(2)

            height = 0.7 # 0.1m is min height it can hold 
            velocity = 0.4 

            with MotionCommander(scf) as mc: 
                mc.move_distance(0,0,height,velocity) # (x,y,z,velocity)

                for _ in range(10):
                    cf.commander.send_hover_setpoint(0,0,0,height)
                    print('i am hovering')
                    time.sleep(0.1)

                # mc.left(5.0,velocity)

                landed_flag = False;

                while landed_flag == False:

                    # get the starting point and target point from Optitrack
                    topic_name = '/optitrack/rigid_bodies'
                    startData = getStartData(topic_name)
                    targetData = getTargetData(topic_name)

                    # shift the coordinate system so startPos is the origin
                    # get targetPos relative to startPos
                    adjStartPos = adjustStart(startData,targetData)
                    adjTargetPos = adjustTarget(targetData)

                    print('start pos')
                    print(startData)
                    print('target pos')
                    print(targetData)

                    print('adj start pos') 
                    print(adjStartPos)
                    print('adj target pos')
                    print(adjTargetPos)

                    # yaw = startData[3]
                    # rate = 10.0 # deg/sec 
                    # print('yaw')
                    # print(yaw)

                    # run the trajectory planner
                    tf = 3.0 # [s] elapsed time
                    dt = 1.0 # [s] timestep

                    # rotate so the Crazyflie is aligned to the Optitrack system
                    # mc.turn_left(yaw,rate)                

                    landed_flag = followTrajectory(tf,dt,adjStartPos,adjTargetPos)


    except rospy.ROSInterruptException:
        pass
