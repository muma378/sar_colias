
include "../robots/colias.inc"
include "../robots/sensors.inc"

define sim_colias colias
(
  # colias_bumpers() ranger 0
  # colias_irs() ranger 1
  robot_detector()  # fiducialfinder 0
  target_detector() # fiducialfinder 1
)

sim_colias( name "robot1" pose [ -1.36 2.16 0.0 -23] fiducial_return 1 )
sim_colias( name "robot2" pose [ -3.44 -3.04 0.0 -69] fiducial_return 2 )
sim_colias( name "robot3" pose [ 0.88 3.44 0.0 -42] fiducial_return 3 )
sim_colias( name "robot4" pose [ -3.28 -3.04 0.0 -109] fiducial_return 4 )
sim_colias( name "robot5" pose [ -3.95 0.88 0.0 91] fiducial_return 5 )
sim_colias( name "robot6" pose [ 0.64 -2.48 0.0 40] fiducial_return 6 )
sim_colias( name "robot7" pose [ 2.72 1.60 0.0 131] fiducial_return 7 )
sim_colias( name "robot8" pose [ -1.36 -1.20 0.0 -74] fiducial_return 8 )
sim_colias( name "robot9" pose [ -3.76 0.48 0.0 -9] fiducial_return 9 )
sim_colias( name "robot10" pose [ 2.40 2.08 0.0 -85] fiducial_return 10 )
