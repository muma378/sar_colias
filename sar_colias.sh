#!/bin/bash
# sar_colias.sh -  generates the .inc and .cfg file that contains the
# configuration about robots and drives, which makes it possible to create
# multi robots automaically. Meanwhile, compiles and runs the program. 
# Author: Yang Xiao
# Date: 17 September 2015

if [ -z "$1" ]; then
	if [[ -e 'sar.cfg' ]]; then
		player sar.cfg &
		exit
	fi
    echo usage: $0 robots_count [x_size*y_size]
    exit
fi

ROBOTS_COUNT=$1

X_SIZE=8	# default size of map
Y_SIZE=8

if [ ! -z "$2" ]; then
	X_SIZE=${2%*\*}
	Y_SIZE=${2#*\*}
fi

# TODO: detects bc installed
function random_pose_generator {
	random_float=$(( $RANDOM%1000 ))
	x_pose=`echo -e "scale=2\n $random_float/1000.0*${X_SIZE}-${X_SIZE}/2"| bc -l`
	random_float=$(( $RANDOM%1000 ))
	y_pose=`echo -e "scale=2\n $random_float/1000.0*${Y_SIZE}-${Y_SIZE}/2"| bc -l`
	angle=$(( $RANDOM%360-180 ))
}

function output_robots_config {

local SIM_COLIAS_DEF='include "../robots/colias.inc"
include "../robots/sensors.inc"

define sim_colias colias
(
  # colias_bumpers() ranger 0
  # colias_irs() ranger 1
  robot_detector()  # fiducialfinder 0
  source_detector() # fiducialfinder 1
)

'
local SIM_COLIAS_DECLARE='sim_colias( name "robot%d" pose [ %.2f %.2f 0.0 %d] fiducial_return %d )\n'

	printf "$SIM_COLIAS_DEF"
    for i in `seq 1 $ROBOTS_COUNT`; do
    	random_pose_generator
    	printf "$SIM_COLIAS_DECLARE" $i $x_pose $y_pose $angle $i
    done
}


function output_drives_config {

local DRIVES_MAP_DECLARE='driver(
  name "stage"
  provides [ "simulation:0" ]
  plugin "stageplugin"

  worldfile "maps/sar.world"
)

'
local DRIVES_DECLARE='driver(
  name "stage"
  provides [ "position2d:%d" "ranger:%d" "ranger:%d" "fiducial:%d" "fiducial:%d" ]
  model "robot%d"
)

'

	printf "$DRIVES_MAP_DECLARE"
	for i in `seq 0 $(($ROBOTS_COUNT-1))`; do
		printf "$DRIVES_DECLARE" $i $((i*2)) $((i*2+1)) $((i*2)) $((i*2+1)) $((i+1)) 
	done
}


# given the first parameter as output function
# the second as file path
function generate_config {
	if [ -d `dirname $2` ]; then
	    if [ -e $2 ]; then
	    	echo '' > $2
	    fi
	    $1 >> $2
	fi
}

generate_config output_robots_config 'robots/sim_colias.inc'
generate_config output_drives_config './sar.cfg'
player sar.cfg &

cd controllers
# replace the parameter in header config.h
PATTERN="s/^\(#define ROBOTS_COUNT\) \([0-9]*\)/\1 $ROBOTS_COUNT/g"
sed -i.bak "$PATTERN" 'config.h'
make clean
make

