#!/bin/bash
nr=$1
command=$2
echo "executing " $nr " with " $command
if [ -z "$commandi" ];
then
	BOARD_FUNCTION="BOARD_FUNCTION=-D$command=1 "
else
 BOARD_FUNCTION="not defined"
fi
echo $BOARD_FUNCTION
make clean && make singlecamera CAMNUMBER=$nr $BOARD_FUNCTION && make uploadsinglecamera CAMNUMBER=$nr
