#! /usr/bin/python
import rospy
from gazebo_fiducial_spawner.msg import FiducialModel

rospy.init_node("multi_fid_requester")
req_pub = rospy.Publisher("/new_fid_models", FiducialModel, queue_size=10)
"""
an example of how to spawn a bunch of fids 
"""

fids_to_spawn = [
    [1, 0, 0.1, -0.7, 1, 0.1, 0.7, 0, 0, 0.7],
    [2, 0, 0.1, 0, 1, 0.1, 0.7, 0, 0, 0.7],
    [3, 0, 0.1, -0.7, -1, 0.1, 0.7, 0, 0, 0.7],
    [4, 0, 0.1, -0.7, 1.45, 0.1, 0.7, 0, 0, 0.7],
    [5, 0, 0.1, 1.4, 1.5, 0.1, 0.5, 0.5, 0.5, 0.5], 
    [6, 0, 0.1, -2.3, -1, 0.1, 0.5, 0.5, 0.5, 0.5]
]

for f in fids_to_spawn:
    rospy.sleep(1)  # sleep time is needed
    model = FiducialModel()
    model.id = f[0]
    model.dictionary = f[1]
    model.size = f[2]
    model.pose.position.x = f[3]
    model.pose.position.y = f[4]
    model.pose.position.z = f[5]
    model.pose.orientation.x = f[6]
    model.pose.orientation.y = f[7]
    model.pose.orientation.z = f[8]
    model.pose.orientation.w = f[9]
    req_pub.publish(model)
