# survivor.inc - config for the target to be collected 
# Author: Yang Xiao
# Date: 10 September 2015

# the model of survivor, can be detected as fiducial
define survivor model
(
  color "cyan"
  size [ 0.05 0.05 0.1]

  block(
    points 12
    point[0] [ 0.0250 0.000 ]
    point[1] [ 0.0220 0.0125 ]
    point[2] [ 0.0125 0.0220 ]
    point[3] [ 0.0000 0.0250 ]
    point[4] [ -0.0125 0.0220 ]
    point[5] [ -0.0220 0.0125 ]
    point[6] [ -0.0250 0.000 ]
    point[7] [ -0.0220 -0.0125 ]
    point[8] [ -0.0125 -0.0220 ]
    point[9] [ 0.0000 -0.0250 ]
    point[10] [ 0.0125 -0.0220 ]
    point[11] [ 0.0220 -0.0125 ]
    z [ 0.000 0.100 ]
  )

  obstacle_return 1
  gripper_return 0
  ranger_return 0
  # fiducial_return 100   # count from 101
  fiducial_key 3
)

# noise to increase the effort to find targets
# can be detected as fiducial, but will return 0
define obstacle model
(
  color "DarkBlue"
  size [ 0.1 0.1 0.1 ]
  # bitmap "../maps/bitmaps/human_outline.png"

  obstacle_return 1
  gripper_return 0
  ranger_return 0
  fiducial_return 200
  fiducial_key 3

)