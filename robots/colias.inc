# colias.inc - simulation based on the swarm robot Colias developed by Farshad Arvin
# Author: Yang Xiao
# Date: 8 September 2015


# modelling the single colias IR sensor
define colias_ir sensor
(
  #color_rgba [ 0 1 0 0.6 ]
  color "cyan"
  # size of each transducer 4*15*8mm
  size [ 0.010 0.015 0.008 ]
  # Long-range system with 15 +(-)1cm  of max range, 60 degree of wide viewing angle
  range [ 0.000 0.150 ]
  # field of view is 60 degree
  fov 60

)

define colias_ir_block model
(
  color "gray40"
  size [ 0.004 0.008 0.004 ]

  block(
  	points 4
  	point[0] [ 0.0020 0.004 ]
  	point[1] [ -0.0020 0.004 ]
  	point[2] [ -0.0020 -0.004 ]
  	point[3] [ 0.0020 -0.004 ]
  	z [ 0.000 0.004 ]
  )
)


# long-range system is composed of 6 IR proximity sensors (each 60 degree on
# on the upper board) for obstacle and robot detection. 
define colias_irs ranger
(
  
  # uniformly distributed around the circle
  colias_ir( pose [ 0.0125 0.0220 0.0 60] )
  colias_ir( pose [ 0.0250 0.0000 0.0 0] )
  colias_ir( pose [ 0.0125 -0.0220 0.0 -60] )
  colias_ir( pose [-0.0125 -0.0220 0.0 -120] )
  colias_ir( pose [-0.0250 0.0000 0.0 180] )
  colias_ir( pose [-0.0125 0.0220 0.0 120] )
)

# approximates the shape of all ir sensers
define colias_ir_blocks model
(
  size [ 0.0001 0.0001 0.000 ]
  color_rgba [ 1 0 0 1 ]
  colias_ir_block( pose [ 0.0125 0.0220 0.0 60] )
  colias_ir_block( pose [ 0.0250 0.0000 0.0 0] )
  colias_ir_block( pose [ 0.0125 -0.0220 0.0 -60] )
  colias_ir_block( pose [-0.0125 -0.0220 0.0 -120] )
  colias_ir_block( pose [-0.0250 0.0000 0.0 180] )
  colias_ir_block( pose [-0.0125 0.0220 0.0 120] )

)

# to detect bump with short range
define colias_bumper sensor
(
  color "gray"
  size [ 0.010 0.020 0.008 ] # NOT accurate
  range [ 0.000 0.030 ] # 3 +(-)0.5cm
  fov 15

)

# short-range system is composed of 3 bump sensors (each 30 degree placed
# at the bottom board). mostly used as a cue in the groud-level task.
define colias_bumpers ranger
(
  colias_bumper( pose [ 0.0220 0.0125 0.0 30 ] )
  colias_bumper( pose [ 0.0250 0.0000 0.0 0 ] )
  colias_bumper( pose [ 0.0220 0.0125 0.0 -30] )
)

define colias_wheel model
(
  color "yellow"
  # diameter of the wheel is 26mm
  size [ 0.0260 0.0050 0.0130 ]

  block(
  	points 4
    point[0] [ 0.0130   0.0025 ]
    point[1] [ 0.0130  -0.0025 ]
    point[2] [-0.0130  -0.0025 ]
    point[3] [-0.0130   0.0025 ]
    z [ -0.0130 0.0130 ] 
  	)

)

define colias_upper_board model
(
  color "red"
  # diameter of the board is 50mm and height is 34mm
  size [ 0.050 0.050 0.005 ]

  # bitmap "../maps/bitmaps/circle.png"
  # the block approximates the circular shape of upper board
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
  	z [ 0.0300 0.0350 ]
  )

  colias_irs()
  colias_ir_blocks()
 )

define colias_bottom_board model
(
  color "green"

  # the height equals to the radius of wheels
  size [ 0.050 0.040 0.005 ]

  # the block approximates the boat shape
  block(
   points 10
   point[0] [ 0.0150 0.0200 ]
   point[1] [ 0.0220 0.0125 ]
   point[2] [ 0.0250 0.0000 ]
   point[3] [ 0.0220 -0.0125 ]
   point[4] [ 0.0150 -0.0200 ]
   point[5] [ -0.0150 -0.0200 ]
   point[6] [ -0.0220 -0.0125 ]
   point[7] [ -0.0250 0.0000 ]
   point[8] [ -0.0220 0.0125 ]
   point[9] [ -0.0150 0.0200 ]
   z [ 0.008 0.013 ]
  )

  colias_wheel(
  	pose [ -0.008 0.022 0.0 0.000 ]
  )
  colias_wheel(
  	pose [ -0.008 -0.022 0.0 0.000 ]
  )
  colias_bumpers()

)

define colias position
(
  color "red"
  # make it small so that upper board can be displayed
  size [ 0.001 0.001 0.000 ]

  # wheels are controlled independently
  drive "diff"
  # center of rotation
  origin [ 0.0 0.0 0.0 0.0 ] 
  gui_nose 1
  mass 0.028 # kilogram
  ranger_return 1.0 # to distinguish robots from obstacles

  # plugins defined above
  colias_bottom_board( pose [ 0.0 0.0 0.012 0.0 ] )
  colias_upper_board( pose [ 0.0 0.0 0.034 0.0 ] )
)