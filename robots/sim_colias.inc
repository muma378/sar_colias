
include "../robots/colias.inc"
include "../robots/sensors.inc"

define sim_colias colias
(
  # colias_bumpers() ranger 0
  # colias_irs() ranger 1
  robot_detector()  # fiducialfinder 0
  source_detector() # fiducialfinder 1
)

sim_colias( name "robot1" pose [ -2.08 2.88 0.0 -11] fiducial_return 1 )
sim_colias( name "robot2" pose [ 2.48 -2.08 0.0 -25] fiducial_return 2 )
sim_colias( name "robot3" pose [ -1.04 -1.20 0.0 50] fiducial_return 3 )
sim_colias( name "robot4" pose [ 3.52 0.40 0.0 74] fiducial_return 4 )
sim_colias( name "robot5" pose [ -2.08 2.08 0.0 8] fiducial_return 5 )
