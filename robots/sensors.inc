# sensors.inc - list of sensors which are not built-in for colias
# but necessay for the algorithm
# Author: Yang Xiao
# Date: 10 September 2015

# To measure the bearing and distance of the neighbour robots
# It is already implemented in the Colias, however can not
# be simulated in player/stage since it's not able to receive 
# IR emitted by other robots. So we use the fiducial finder
# with 360 degree to replace.
define robot_detector fiducial
(
  # with the short-range communication, colias exhibited relatively reliable 
  # coommunication at a distance of less than 12cm
  # with the long-range communcation, it could communicate up to
  # 2 m with an accptable error rate
  # to simplify the question, we set the range as [0.0, 2.0]
  range_min 0.0 
  range_max 0.5
  # the maximum range at which the sensor can detect the ID of a fiducial
  range_max_id 0.5
  # feild of view is 360 degree
  fov 360
  # ignores the z component when checking a fiducial
  ignore_zloc 1
  fiducial_key 2  # only can detect robots
  size [ 0.004 0.006 0.004 ]
)


# To identify the strength of the signal which called fitness.
# We adopt Fmax - (the distance to the fiducial point) as the value.
define source_detector fiducial
(
  # 
  range_min 0.0
  range_max 1.5
  range_max_id 1.5

  fov 360
  ignore_zloc 1
  fiducial_key 3  # only can detect signal source
  size [ 0.004 0.006 0.004 ]

)