#! /usr/bin/python
# first 4 imports should be installed with ROS
import rospy
import cv2 
import cv2.aruco  # my pylinter in vscode says this doesn't exist, don't worry it does.
from tf.transformations import euler_from_quaternion
# packages that are bundled by default with python
from os.path import dirname, realpath
from os import sep, system, rename
import sys
from distutils.dir_util import copy_tree
# catkin make this package for this message to be added
from gazebo_fiducial_spawner.msg import FiducialModel
# tl;dr no pip should be required for this node 

def get_nearby_file(filename):
    return dirname(realpath(sys.argv[0])) + sep + filename


def new_model(fid_id, fid_dict, size=0):
    # copy model folder
    copy_tree(get_nearby_file("../models/arucotag"),get_nearby_file("../models/arucotag{}".format(fid_id)))
    # create correct fid tag picture
    aruco_dict = cv2.aruco.Dictionary_get(fid_dict)
    img = cv2.aruco.drawMarker(aruco_dict, fid_id, 2000)
    img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    cv2.imwrite(get_nearby_file("../models/arucotag{0}/arucotag{0}.png".format(fid_id)), img)
    # make changes to sdf file
    with open(get_nearby_file("../models/arucotag{}/arucotag.sdf".format(fid_id)), 'r+') as sdf:
        # this pattern is generally the same for all three files
        output = sdf.read().format(fid_id, size if size else 0.15)  # read file and fill placeholders
        sdf.seek(0)  # seek back to start of file
        sdf.write(output)  # write new formatted file
        sdf.truncate()  # truncate any leftover text from template after write
    # make changes to config file
    with open(get_nearby_file("../models/arucotag{}/arucotag.config".format(fid_id)), 'r+') as config:
        output = config.read().format(fid_id)
        config.seek(0)
        config.write(output)
        config.truncate()
    # make changes to material file
    with open(get_nearby_file("../models/arucotag{}/arucotag.material".format(fid_id)), 'r+') as mat:
        output = mat.read().format(fid_id)
        output = output.replace("[", "{")  # the material file needs to have curly braces, but the template has sqare bracket placeholders for python string formatting to work
        output = output.replace("]", "}")
        mat.seek(0)
        mat.write(output)
        mat.truncate()
    # rename files
    rename(get_nearby_file("../models/arucotag{}/arucotag.sdf".format(fid_id)), get_nearby_file("../models/arucotag{0}/arucotag{0}.sdf".format(fid_id)))
    rename(get_nearby_file("../models/arucotag{}/arucotag.config".format(fid_id)), get_nearby_file("../models/arucotag{0}/arucotag{0}.config".format(fid_id)))
    rename(get_nearby_file("../models/arucotag{}/arucotag.material".format(fid_id)), get_nearby_file("../models/arucotag{0}/arucotag{0}.material".format(fid_id)))

def spawn_cb(msg):
    if not msg.id:
        rospy.logerr("cannot spawn model, no id provided")
    else:
        new_model(msg.id, msg.dictionary if msg.dictionary else 7, size=msg.size)
        # stuff related to model pose
        try:
            command = "rosrun gazebo_ros spawn_model -file {} -sdf -model arucotag{} ".format(get_nearby_file("../models/arucotag{0}/arucotag{0}.sdf".format(msg.id)), msg.id)
            # get pose information
            pp = msg.pose.position
            po = msg.pose.orientation
            # add position to command
            if pp.x:
                command += "-x {} ".format(pp.x)
            if pp.y:
                command += "-y {} ".format(pp.y)
            if pp.z:
                command += "-z {} ".format(pp.z)
            # add orientation to command
            if po.x or po.y or po.z or po.w:
                po = euler_from_quaternion([po.x, po.y, po.z, po.w])
                command += "-R {} -P {} -Y {} ".format(po[0], po[1], po[2])
            
            result = system(command)
            if not result:
                rospy.loginfo("fiducial loaded successfully")
            else:
                rospy.logerr("could not spawn fiducial, spawner error")
        except:
            rospy.logerr("could not spawn fiducial. Is gazebo running?")

if __name__ == '__main__':
    rospy.init_node("fid_spawner")
    spawner_sub = rospy.Subscriber("/new_fid_models", FiducialModel, spawn_cb)
    rospy.spin()
