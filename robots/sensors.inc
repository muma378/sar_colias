# sensors.inc - list of sensors which are not built-in for colias
# but necessay for the algorithm
# Author: Yang Xiao
# Date: 10 September 2015

# it is already implemented in the Colias, however can not
# be simulated in player/stage since it's not able to receive 
# IR emitted by other robots. So we use the fiducial finder
# with 360 degree to replace.
define robot_detector fiducial
(
  range_min 0.0
  range_max 0.3
  # the maximum range at which the sensor can detect the ID of a fiducial
  range_max_id 0.3
  # feild of view is 360 degree
  fov 360
  # ignores the z component when checking a fiducial
  ignore_zloc 1
  fiducial_key 2  # only can detect robots
  size [ 0.004 0.006 0.004 ]
)

# assume a detector developed to identify the strength of the signal which 
# called fitness. we adopt Fmax - (the distance to the fiducial point) 
# as the value of fitness
define source_detector fiducial
(
  range_min 0.0
  range_max 3.0
  range_max_id 3.0

  fov 360
  ignore_zloc 1
  fiducial_key 3  # only can detect signal source
  size [ 0.004 0.006 0.004 ]

)