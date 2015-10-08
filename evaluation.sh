#!/bin/bash

LOOP=20
for i in `seq 1 $LOOP`; do
    ./sar_colias.sh
    pkill player
done