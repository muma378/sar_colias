
include "../robots/colias.inc"
include "../robots/sensors.inc"

define sim_colias colias
(
  # colias_bumpers() ranger 0
  # colias_irs() ranger 1
  robot_detector()  # fiducialfinder 0
  source_detector() # fiducialfinder 1
)

sim_colias( name "robot1" pose [ 2.96 2.48 0.0 37] fiducial_return 1 )
sim_colias( name "robot2" pose [ -3.68 -1.84 0.0 116] fiducial_return 2 )
sim_colias( name "robot3" pose [ -3.68 0.00 0.0 -54] fiducial_return 3 )
sim_colias( name "robot4" pose [ -3.20 -0.24 0.0 66] fiducial_return 4 )
sim_colias( name "robot5" pose [ -1.44 -2.00 0.0 4] fiducial_return 5 )
sim_colias( name "robot6" pose [ -0.80 -2.56 0.0 89] fiducial_return 6 )
sim_colias( name "robot7" pose [ 2.08 2.80 0.0 -141] fiducial_return 7 )
sim_colias( name "robot8" pose [ 0.72 -0.24 0.0 161] fiducial_return 8 )
sim_colias( name "robot9" pose [ 1.20 0.16 0.0 -28] fiducial_return 9 )
sim_colias( name "robot10" pose [ -0.48 3.20 0.0 -121] fiducial_return 10 )
sim_colias( name "robot11" pose [ 3.44 1.04 0.0 -163] fiducial_return 11 )
sim_colias( name "robot12" pose [ 0.56 -1.60 0.0 66] fiducial_return 12 )
sim_colias( name "robot13" pose [ -3.92 0.56 0.0 162] fiducial_return 13 )
sim_colias( name "robot14" pose [ -0.80 2.08 0.0 -75] fiducial_return 14 )
sim_colias( name "robot15" pose [ 2.48 -2.56 0.0 49] fiducial_return 15 )
sim_colias( name "robot16" pose [ -2.16 0.72 0.0 -106] fiducial_return 16 )
sim_colias( name "robot17" pose [ 2.40 -1.84 0.0 23] fiducial_return 17 )
sim_colias( name "robot18" pose [ 2.08 -1.44 0.0 174] fiducial_return 18 )
sim_colias( name "robot19" pose [ -2.08 -0.56 0.0 13] fiducial_return 19 )
sim_colias( name "robot20" pose [ -1.60 -2.72 0.0 -175] fiducial_return 20 )
