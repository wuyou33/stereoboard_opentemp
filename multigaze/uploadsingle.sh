#!/bin/bash
nr=$1
command=$2
echo "executing " $nr " with " $command
if [ -z "$commandi" ];
then
   PROJECT=$2
else
 PROJECT="not defined"
fi
echo $BOARD_FUNCTION
make clean && make singlecamera CAMNUMBER=$nr PROJECT=$PROJECT && make uploadsinglecamera CAMNUMBER=$nr
