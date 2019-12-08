# meeting point function test

# calculate the point where the two vehicles should meet
# drone velocity = 0.4 m/s
# ugv velocity = 0.8 m/s
# distance = sqrt(dx^2 + dy^2 + dz^2)
# time to cover distance = distance / (drone velocity + ugv velocity)
# point = drone velocity * time = ugv velocity * time
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def meetingPoint(droneVel, ugvVel, dronePos, ugvPos):
	
	# get difference in x,y,z components between 2 vehicles
	dx = dronePos[0] - ugvPos[0]
	dy = dronePos[1] - ugvPos[1]
	dz = dronePos[2] - ugvPos[2]
	print('dx = ', dx, ' dy = ', dy, ' dz = ', dz)
	#print('drone pos x = ', dronePos[0], 'drone pos y = ', dronePos[1], 'drone pos z = ', dronePos[2]) 
	#print('ugv pos x = ', ugvPos[0], 'ugv pos y = ', ugvPos[1], 'ugv pos z = ', ugvPos[2])

	# calculate azimuth and zenith angles
	phi = math.atan2(dy,dx)
	theta = math.atan2(dz,dy)
	print('phi = ', phi, 'theta = ', theta)

	# calculate net distance to travel
	dist = math.sqrt((dx**2) + (dy**2) + (dz**2))
	print('dist = ', dist)

	# calculate time required to get to crossing point
	time = dist / abs(droneVel - ugvVel)
	print('time = ', time, 'drone vel = ', droneVel, ' ugv vel = ', ugvVel)

	# calculate distance traveled at time of crossing for each vehicle
	droneDist = abs(droneVel * time) 
	ugvDist = abs(ugvVel * time)
	print('droneDist = ', droneDist)
	print('ugvDist = ', ugvDist)

	# decompose distances into components
	# drone
	droneZi = droneDist * math.sin(theta)
	drone_distxy = droneDist * math.cos(theta)
	print('drone dist xy = ', drone_distxy)
	droneXi = drone_distxy * math.cos(phi)
	droneYi = drone_distxy * math.sin(phi)

	# ugv
	ugvZi = ugvDist * math.sin(theta)
	ugv_distxy = ugvDist * math.cos(theta)
	print('ugv dist xy = ', ugv_distxy)
	ugvXi = ugv_distxy * math.cos(phi)
	ugvYi = ugv_distxy * math.sin(phi)

	print('##drone dx = ', droneXi, 'drone dy = ', droneYi, 'drone dz = ', droneZi)
	print('##ugv dx = ', ugvXi, 'ugv dy = ', ugvYi, 'ugv dz = ', ugvZi)

	# add distances to current positions
	# SUBTRACT distance from DRONE
	# ADD distance to UGV
	droneX = dronePos[0] - droneXi
	droneY = dronePos[1] - droneYi
	droneZ = dronePos[2] - droneZi
	ugvX = ugvPos[0] + ( ugvXi)
	ugvY = ugvPos[1] + ( ugvYi)
	ugvZ = ugvPos[2] + ( ugvZi)

	print('drone dx = ', droneX, 'drone dy = ', droneY, 'drone dz = ', droneZ)
	print('ugv dx = ', ugvX, 'ugv dy = ', ugvY, 'ugv dz = ', ugvZ)

	# plot current and destination points for each vehicle
	fig = plt.figure()
	ax1 = fig.add_subplot(211, projection='3d')
	ax1.scatter(dronePos[0], dronePos[1], dronePos[2],c='blue')
	ax1.scatter(ugvPos[0], ugvPos[1], ugvPos[2],c='green')
	ax1.scatter(droneX, droneY, droneZ,c='blue')
	ax1.scatter(ugvX, ugvY, ugvZ,c='green')
	ax1.set_xlabel('X')
	ax1.set_ylabel('Y')
	plt.show()
	# return desired target points for each vehicle as x,y,z components
	return [droneX,droneY,droneZ,ugvX,ugvY,ugvZ]

if __name__ == '__main__':

	droneVel = -0.4 
	ugvVel = 0.4
	dronePos = [1.0,1.0,1.0]
	ugvPos = [0.0,0.0,0.0]

	#print('drone vel = ', droneVel, ' ugvVel = ', ugvVel)
	point = meetingPoint(droneVel, ugvVel, dronePos, ugvPos)

	#print('dx = ', point[0], ' dy = ', point[1], ' dz = ', point[2], 'ux = ', point[3], ' uy = ', point[4], ' uz = ', point[5 ]) 

