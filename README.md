# Gazebo Fiducial Spawner

This is a basic package which allows for fidicuals to be spawned into gazebo easily.

This package might not work if you have another package in your workspace that exports `gazebo_ros gazebo_media_path`. Use at your own risk.


## fiducial_spawner.py

For applications that require multiple fiducials that are generated procedurally, use this node. It will subscribe to the topic **/new_fiducial_models** which uses the included **FiducialModel** message type. At the bare minimum, you must provide an id in this message. The dictionary will default to 7 if not provided and the size will default to 0.15m if not provided. The pose will default to [0,0,0] if not provided. Each time a fiducial is spawned, a new directory will be created in side the `models/` directory which will mimic the contents of `arucotag/`. The good news is that because there can only be up to 1000 arucotags, there can only be 1001 directories in the `models/` directory. Each model directory takes up about 32kB if that matters to you.

## single_spawner.py

A script to spawn a single fiducial. Use `rosrun gazebo_fiducial_spawner single_spawner.py -h` to view arguments. All are optional except id. Same defaults as `fiducial_spawner.py`. 
