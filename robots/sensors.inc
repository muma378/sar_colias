# sensors.inc - list of sensors which are not built-in for colias
# but necessay for the algorithm
# Author: Yang Xiao
# Date: 10 September 2015

# assume a detector developed to identify the target
define detector fiducial
(
  range_min 0.0
  range_max 0.5
  # the maximum range at which the sensor can detect the ID of a fiducial
  range_max_id 0.2
  # feild of view is 120 degree
  fov 3.1415*2/3
  # ignores the z component when checking a fiducial
  ignore_zloc 1
  size [ 0.004 0.006 0.004 ]

)