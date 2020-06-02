# Gazebo Fiducial Spawner

This is a basic package which allows for fidicuals to be spawned into gazebo easily. Two utilities are provided:

* fiducial_spawner.py, which is a full ros node which subscribes to /new_fid_models, which uses the included gazebo_fiducial_spawner/FiducialModel message type. sending a message to this node will spawn a fiducial in a gazebo world. Useful for any continual or procedural fiducial spawning
* spawn.sh, a one-time fiducial spawner. good for if you just need one or a handful of fiducials, toss this in a launch file a few times.


## material backup 
<uri>file://Media/arucotag.material</uri>
material arucotag
{
  technique
  {
    pass
    {
      texture_unit
      {
        texture arucotag.png
      }
    }
  }
}