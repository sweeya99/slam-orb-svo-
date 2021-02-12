import rosbag
import numpy as np
#bag = rosbag.Bag("/home/aanal/pose/rosbag16_speed_increase_decrease/2020-12-27-22-47-35.bag")
bag = rosbag.Bag("/home/panyam/Downloads/2021-01-26-15-22-37.bag")
bag1=rosbag.Bag("/home/panyam/mybot_ws/2021-01-26-16-03-10.bag")

groundx = []
groundy = []
groundz = []

orbx = []
orby = []
orbz = []

for topic, msg, t in bag.read_messages(topics = '/gazebo/model_states'):
# for topic, msg, t in bag.read_messages(topics = '/ground_truth_to_tf/pose'):
	
#	f.write("%s %s %s %s %s %s %s %s"%(t, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,  msg.pose.orientation.x,  msg.pose.orientation.y,  msg.pose.orientation.z,  msg.pose.orientation.w))
#	f.write("\n")
#	print(msg.name)
	print("groundtruth")
	print(t)
	groundx.append(msg.pose.position.x)
	groundy.append(msg.pose.position.y)
	groundz.append(msg.pose.position.z)
	groundx.append(msg.pose.pose.position.x)
	groundy.append(msg.pose.pose.position.y)
	groundz.append(msg.pose.pose.position.z)
#	break


#orb = []
"""for topic, msg, t1 in bag.read_messages(topics = '/orb_slam2_rgbd/pose'):
#	f.write("%s %s %s %s %s %s %s %s"%(t, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,  msg.pose.orientation.x,  msg.pose.orientation.y,  msg.pose.orientation.z,  msg.pose.orientation.w))
#	f.write("\n")
#	orbx.append(msg1.pose.position.x*1000)
#	orby.append(msg1.pose.position.y*1000)
#	orbz.append(msg1.pose.position.z*1000)

	orbx.append(msg.pose.position.x)
	orby.append(msg.pose.position.y)
	orbz.append(msg.pose.position.z)
"""

orbx = []
orby = []
orbz = []
for topic, msg, t in bag1.read_messages(topics ='/svo/pose'):
	print("/svo/pose")
	print(t)
	orbx.append(msg.pose.pose.position.x )
	orby.append(msg.pose.pose.position.y )
	orbz.append(msg.pose.pose.position.z )

	# 	orbx.append(msg.pose.position.x )
	# orby.append(msg.pose.position.y )
	# orbz.append(msg.pose.position.z )


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1, projection = '3d')
n = 3*len(groundx)/4
#ax.scatter(groundx[0:n], groundy[0:n], groundz[0:n], label = 'ground', color = 'tab:blue')
#ax.scatter(groundx[n:], groundy[n:], groundz[n:], label = 'ground', color = 'tab:blue')
ax.scatter(groundx, groundy, groundz, label = 'ground', color = 'tab:blue')
#ax.set_ylim([-5, 20])
#ax.set_zlim([-5, 20])



diffx = []
diffy = []
diffz = []
"""n = len(groundx)/len(orbx)
for i in range(len(orbx)):
	diffx.append(groundx[i*n] - orbx[i])
	diffy.append(groundy[i*n] - orby[i])
	diffz.append(groundz[i*n] - orbz[i])"""
#ax.scatter(diffx, diffy, diffz, label = 'diff', color = 'tab:green')
n = 3*len(orbx)/4
#ax.scatter(orbx[0:n], orby[0:n], orbz[0:n], label = 'orb', color = 'tab:orange')
ax.scatter(orbx, orby, orbz, label = 'orb', color = 'tab:orange')
#ax.scatter(orbx[n:], orby[n:], orbz[n:], label = 'orb', color = 'tab:orange')
ax.legend()
plt.show()


#print(orbx[0], orby[0], orbz[0])
#print(groundx[0], groundy[0], groundz[0])

print(groundx[1:5])
