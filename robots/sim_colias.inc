
include "../robots/colias.inc"
include "../robots/sensors.inc"

define sim_colias colias
(
  # colias_bumpers() ranger 0
  # colias_irs() ranger 1
  robot_detector()  # fiducialfinder 0
  source_detector() # fiducialfinder 1
)

sim_colias( name "robot1" pose [ 1.44 -3.12 0.0 2] fiducial_return 1 )
sim_colias( name "robot2" pose [ -1.12 -0.72 0.0 -67] fiducial_return 2 )
sim_colias( name "robot3" pose [ 2.24 -0.48 0.0 -19] fiducial_return 3 )
