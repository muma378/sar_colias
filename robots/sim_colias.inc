
include "../robots/colias.inc"
include "../robots/sensors.inc"

define sim_colias colias
(
  # colias_bumpers() ranger 0
  # colias_irs() ranger 1
  robot_detector()  # fiducialfinder 0
  target_detector() # fiducialfinder 1
)

sim_colias( name "robot1" pose [ 1.50 1.18 0.0 83] fiducial_return 1 )
sim_colias( name "robot2" pose [ 1.65 -3.80 0.0 98] fiducial_return 2 )
sim_colias( name "robot3" pose [ 1.34 -2.69 0.0 49] fiducial_return 3 )
sim_colias( name "robot4" pose [ 3.08 2.05 0.0 -31] fiducial_return 4 )
sim_colias( name "robot5" pose [ 0.07 -0.48 0.0 -81] fiducial_return 5 )
sim_colias( name "robot6" pose [ 0.39 3.31 0.0 146] fiducial_return 6 )
sim_colias( name "robot7" pose [ -2.93 1.02 0.0 85] fiducial_return 7 )
sim_colias( name "robot8" pose [ -3.72 2.37 0.0 102] fiducial_return 8 )
sim_colias( name "robot9" pose [ 1.83 -3.48 0.0 61] fiducial_return 9 )
sim_colias( name "robot10" pose [ -3.84 0.58 0.0 -160] fiducial_return 10 )
sim_colias( name "robot11" pose [ -3.24 0.86 0.0 -124] fiducial_return 11 )
sim_colias( name "robot12" pose [ 3.87 3.39 0.0 144] fiducial_return 12 )
sim_colias( name "robot13" pose [ 0.79 3.00 0.0 -171] fiducial_return 13 )
sim_colias( name "robot14" pose [ -2.14 0.47 0.0 68] fiducial_return 14 )
sim_colias( name "robot15" pose [ 3.00 1.26 0.0 -127] fiducial_return 15 )
sim_colias( name "robot16" pose [ -1.82 -3.64 0.0 -140] fiducial_return 16 )
sim_colias( name "robot17" pose [ -3.24 0.23 0.0 -5] fiducial_return 17 )
sim_colias( name "robot18" pose [ 0.86 -3.80 0.0 -10] fiducial_return 18 )
sim_colias( name "robot19" pose [ 3.55 3.55 0.0 -129] fiducial_return 19 )
sim_colias( name "robot20" pose [ -0.95 0.79 0.0 -13] fiducial_return 20 )
