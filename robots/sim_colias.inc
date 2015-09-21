
include "../robots/colias.inc"
include "../robots/sensors.inc"

define sim_colias colias
(
  # colias_bumpers() ranger 0
  # colias_irs() ranger 1
  robot_detector()  # fiducialfinder 0
  source_detector() # fiducialfinder 1
)

sim_colias( name "robot1" pose [ 3.36 -2.80 0.0 -6] fiducial_return 1 )
sim_colias( name "robot2" pose [ -0.56 0.08 0.0 63] fiducial_return 2 )
sim_colias( name "robot3" pose [ 3.28 -0.08 0.0 -160] fiducial_return 3 )
