
include "../robots/colias.inc"
include "../robots/sensors.inc"

define sim_colias colias
(
  # colias_bumpers() ranger 0
  # colias_irs() ranger 1
  robot_detector()  # fiducialfinder 0
  source_detector() # fiducialfinder 1
)

sim_colias( name "robot1" pose [ -1.52 -0.32 0.0 -75] fiducial_return 1 )
sim_colias( name "robot2" pose [ 2.96 2.80 0.0 -31] fiducial_return 2 )
sim_colias( name "robot3" pose [ -2.96 -3.90 0.0 171] fiducial_return 3 )
sim_colias( name "robot4" pose [ 0.16 -3.52 0.0 135] fiducial_return 4 )
sim_colias( name "robot5" pose [ 0.16 0.40 0.0 114] fiducial_return 5 )
sim_colias( name "robot6" pose [ -1.44 -2.48 0.0 -134] fiducial_return 6 )
sim_colias( name "robot7" pose [ 3.36 1.44 0.0 -130] fiducial_return 7 )
sim_colias( name "robot8" pose [ -1.12 -3.68 0.0 -160] fiducial_return 8 )
sim_colias( name "robot9" pose [ -3.28 0.72 0.0 -62] fiducial_return 9 )
sim_colias( name "robot10" pose [ -1.84 -2.40 0.0 154] fiducial_return 10 )
sim_colias( name "robot11" pose [ -3.68 -0.56 0.0 -152] fiducial_return 11 )
sim_colias( name "robot12" pose [ -1.68 -2.72 0.0 -14] fiducial_return 12 )
sim_colias( name "robot13" pose [ -3.90 -0.08 0.0 56] fiducial_return 13 )
sim_colias( name "robot14" pose [ -3.92 3.76 0.0 95] fiducial_return 14 )
sim_colias( name "robot15" pose [ 0.64 1.85 0.0 -117] fiducial_return 15 )
sim_colias( name "robot16" pose [ -2.96 -0.64 0.0 102] fiducial_return 16 )
sim_colias( name "robot17" pose [ 2.24 -2.32 0.0 52] fiducial_return 17 )
sim_colias( name "robot18" pose [ 3.04 0.00 0.0 112] fiducial_return 18 )
sim_colias( name "robot19" pose [ 2.96 -1.68 0.0 166] fiducial_return 19 )
sim_colias( name "robot20" pose [ 2.96 -1.60 0.0 3] fiducial_return 20 )
