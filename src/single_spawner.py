#! /usr/bin/python
from fiducial_spawner import new_model, get_nearby_file
import argparse
from os import system

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("id", help="The ID number of the desired tag", type=int)
    parser.add_argument("-dict", help="The aruco dictionary the tag belongs to. Default 7", type=int)
    parser.add_argument("-size", help="The height/width of the square tag in simulation. Default 0.15", type=float)
    parser.add_argument("-x", help="Position of tag on x axis (meters)", type=float)
    parser.add_argument("-y", help="Position of tag on y axis (meters)", type=float)
    parser.add_argument("-z", help="Position of tag on z axis (meters)", type=float)
    parser.add_argument("-R", help="Roll component of tag orientation (radians)", type=float)
    parser.add_argument("-P", help="Pitch component of tag orientation (radians)", type=float)
    parser.add_argument("-Y", help="Yaw component of tag orientation (radians)", type=float)
    args = parser.parse_args()

    print(args.id, args.size)
    new_model(args.id, args.dict if args.dict else 7, args.size if args.size else 0)

    command = "rosrun gazebo_ros spawn_model -file {} -sdf -model arucotag{} ".format(get_nearby_file("../models/arucotag{0}/arucotag{0}.sdf".format(args.id)), args.id)
    if args.x:
        command += " -x {}".format(args.x)
    if args.y:
        command += " -y {}".format(args.y)
    if args.z:
        command += " -z {}".format(args.z)
    if args.R:
        command += " -R {}".format(args.R)
    if args.P:
        command += " -P {}".format(args.P)
    if args.Y:
        command += " -Y {}".format(args.Y)

    print(command)
    result = system(command)
    if result:
        print("failure to load model")
    else:
        print("Successful model load")