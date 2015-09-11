# survivor.inc - config for the target to be collected 
# Author: Yang Xiao
# Date: 10 September 2015

# the model of survivor, can be detected as fiducial
define survivor model
(
   size [ 0.5 1.2 0.2 ]
   origin [ 0.0 0.0 0.0 0.0 ]
   fiducial_return 1
   obstacle_return 1
   gripper_return 0
   bitmap "../maps/bitmaps/human_outline.png"
)

# noise to increase the effort to find targets
# can be detected as fiducial, but will return 0
define obstacle model
(
  color "cyan"
  size [ 0.3 0.3 0.3 ]
  fiducial_return 1
  fiducial_key 0
  obstacle_return 1
  gripper_return 0
)