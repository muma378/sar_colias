# maps.inc - setup for maps
# Author: Yang Xiao
# Date: 8 September 2015

# map used to test the simulation
define simpleMap model
(
    # color of the map to be rendered
    color "black"

    boundary 1
    gui_nose 0
    
    # display the grid of map
    gui_grid 1
    gui_move 0
    gui_outline 0
    gripper_return 0
    fiducial_return 1
    # return laser when detects walls
    ranger_return 1
    bitmap "bitmaps/cave.png"

)

# map used to test the algorithm
define disasterSpot model
(
    # color of the map to be rendered
    color "black"

    boundary 1
    gui_nose 0
    gui_grid 1
    fiducial_return 1
    ranger_return 1
)
