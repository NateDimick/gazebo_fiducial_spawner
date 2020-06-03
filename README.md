# Gazebo Fiducial Spawner

This is a basic package which allows for fidicuals to be spawned into gazebo easily.

It handles both the creation of new files required to make a model for each fiducial (.sdf, .config, .material and a texture image) and makes the service request to gazebo to put the newly created model in the simulation.

This package might not work if you have another package in your workspace that exports `gazebo_ros gazebo_media_path`. Use at your own risk.


## fiducial_spawner.py

For applications that require multiple fiducials that are generated procedurally, use this node. It will subscribe to the topic **/new_fiducial_models** which uses the included **FiducialModel** message type. At the bare minimum, you must provide an id in this message. The dictionary will default to 7 if not provided and the size will default to 0.15m if not provided. The pose will default to [0,0,0] if not provided. Each time a fiducial is spawned, a new directory will be created in side the `models/` directory which will mimic the contents of `arucotag/`. The good news is that because there can only be up to 1000 arucotags, there can only be 1001 directories in the `models/` directory. Each model directory takes up about 32kB if that matters to you - 32mB in total if all 1000 fiducials are generated.

## single_spawner.py

A script to spawn a single fiducial. Use `rosrun gazebo_fiducial_spawner single_spawner.py -h` to view arguments or see list below. All are optional except id. Same defaults as `fiducial_spawner.py`. 

### Single Spawner args

* `id` is the only positional argument. It should be an integer that fits your dictionary's range. 
* `-dict` an integer between 0 and 16 that corresponds to a dictionary of aruco tags. See documentation [here](http://wiki.ros.org/aruco_detect#Parameters). If this argument is not supplied, then it defaults to `7`
* `-size` is a float that represents the height and width of the model, as it is a square. Value is represented in meters. If not supplied, defaults to 15cm.
* `-x`, `-y` and `-z` all floats which represent components of the model's desired location in the simulation. Unit is meters.
* `-R`, `-P` and `-Y` all floats as well which represent components of the model's orientation in the simulation, **r**oll, **p**itch and **y**aw. Units in radians. NB that y-axis position is lowercase and Yaw rotation is uppercase.

