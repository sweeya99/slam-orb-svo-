import rosbag
import numpy as np
#bag = rosbag.Bag("/home/aanal/pose/rosbag16_speed_increase_decrease/2020-12-27-22-47-35.bag")
# bag = rosbag.Bag("/home/aanal/2021-01-21-19-19-15.bag")

bag = rosbag.Bag("/home/panyam/Downloads/2021-01-26-17-13-31.bag")
bag1=rosbag.Bag("/home/panyam/mybot_ws/2021-01-27-18-18-56.bag")


groundx = []
groundy = []
groundz = []

velx = []
vely = []
velz = []

#for topic, msg, t in bag.read_messages(topics = '/ground_truth_to_tf/pose'):
	
#	groundx.append(msg.pose.position.x)
#	groundy.append(msg.pose.position.y)
#	groundz.append(msg.pose.position.z)
#	break
ground_time = []
for topic, msg, t in bag.read_messages(topics = '/gazebo/model_states'):
	n = list(msg.name).index('iris')	
#	if float(str(t))/10**9>305:
#		break
	ground_time.append(t)
	groundx.append(msg.pose[n].position.x )
	groundy.append(msg.pose[n].position.y )
	groundz.append(msg.pose[n].position.z)
	velx.append(msg.twist[n].linear.x)
	vely.append(msg.twist[n].linear.y)
	velz.append(msg.twist[n].linear.z)
#print(msg.name)
print(msg.name[n])
#	break

orbx = []
orby = []
orbz = []

diffx = []
diffy = []
diffz = []

i = 0
for topic, msg, t in bag1.read_messages(topics = '/svo/pose'):

	# orbx.append(msg.pose.pose.position.x )
	# orby.append(msg.pose.pose.position.y )
	# orbz.append(msg.pose.pose.position.z )
	orby.append(msg.pose.pose.position.x*2)
	orbx.append(msg.pose.pose.position.y*2)
	orbz.append(msg.pose.pose.position.z*2)
	# orbx.append(msg.pose.position.x )
	# orby.append(msg.pose.position.y )
	# orbz.append(msg.pose.position.z )

	while i < len(ground_time):

		if float(str(t)) < float(str(ground_time[i])):
			
			diffx.append(groundx[i] - orbx[-1])
			diffy.append(groundy[i] - orby[-1])
			diffz.append(groundz[i] - orbz[-1])
			i += 1
			break
		i += 1
#	if float(str(t))/10**9>305:
#		break
	

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1, projection = '3d')
n = 2*len(groundx)/4
#ax.scatter(groundx[0:n], groundy[0:n], groundz[0:n], label = 'ground', color = 'tab:blue')
#ax.scatter(groundx[n:], groundy[n:], groundz[n:], label = 'ground', color = 'tab:blue')
ax.scatter(groundx, groundy, groundz, label = 'ground', color = 'tab:blue')



ax.scatter(diffx, diffy, diffz, label = 'diff', color = 'tab:green')
n = 3*len(orbx)/4
ax.scatter(orbx, orby, orbz, label = 'orb', color = 'tab:orange')
print(np.mean(np.abs(np.array(diffx))))
print(np.mean(np.abs(np.array(diffy))))
print(np.mean(np.abs(np.array(diffz))))
print(np.var(np.array(diffx)))
print(np.var(np.array(diffy)))
print(np.var(np.array(diffz)))


n = len(groundx)/len(orbx)
ax.set_xlim([-1, 3])
ax.set_ylim([-1, 3])
ax.set_zlim([-1, 3])

"""for i in range(0, len(orbx)):
	ax.scatter(orbx[i], orby[i], orbz[i], color = 'tab:green')
	ax.scatter(groundx[i*n:(i+1)*n], groundy[i*n:(i+1)*n], groundz[i*n:(i+1)*n], label = 'ground', color = 'tab:blue')
	plt.pause(2.2250738585072014e-308)"""
#ax.scatter(orbx, orby, orbz, label = 'orb', color = 'tab:orange')

print("range of pose: x, y, z respectively")
print(np.max(np.array(orbx)), np.min(np.array(orbx)))
print(np.max(np.array(orby)),  np.min(np.array(orby)))
print(np.max(np.array(orbz)), np.min(np.array(orbz)))
print("initial pose by slam and ground truth")
print(orbx[0], orby[0], orbz[0])
print(groundx[0], groundy[0], groundz[0])
print("velocity")
print(np.mean(np.abs(np.array(velx))))
print(np.mean(np.abs(np.array(vely))))
print(np.mean(np.abs(np.array(velz))))
#print(groundx[1:5])
plt.show()
