
include "../robots/colias.inc"
include "../robots/sensors.inc"

define sim_colias colias
(
  # colias_bumpers() ranger 0
  # colias_irs() ranger 1
  robot_detector()  # fiducialfinder 0
  source_detector() # fiducialfinder 1
)

sim_colias( name "robot1" pose [ -2.48 -3.52 0.0 123] fiducial_return 1 )
sim_colias( name "robot2" pose [ 3.28 -0.56 0.0 69] fiducial_return 2 )
sim_colias( name "robot3" pose [ 3.04 0.40 0.0 -63] fiducial_return 3 )
sim_colias( name "robot4" pose [ 2.88 1.76 0.0 42] fiducial_return 4 )
sim_colias( name "robot5" pose [ -0.24 3.52 0.0 93] fiducial_return 5 )
sim_colias( name "robot6" pose [ -3.12 1.04 0.0 56] fiducial_return 6 )
sim_colias( name "robot7" pose [ -2.96 2.16 0.0 18] fiducial_return 7 )
sim_colias( name "robot8" pose [ 0.80 1.76 0.0 75] fiducial_return 8 )
sim_colias( name "robot9" pose [ 1.12 3.20 0.0 24] fiducial_return 9 )
sim_colias( name "robot10" pose [ 1.28 -1.52 0.0 79] fiducial_return 10 )
