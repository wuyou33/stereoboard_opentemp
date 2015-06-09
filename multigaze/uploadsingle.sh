#!/bin/bash
nr=$1
command=$2
echo "executing " $nr " with " $command
if [ -z "$commandi" ];
then
	SOMETHING="SOMETHING=-D$command=1 "
else
 SOMETHING="not defined"
fi
echo $SOMETHING
#make clean && make singlecamera CAMNUMBER=$nr SOMETHING=-DSEND_COMMANDS=1 && make uploadsinglecamera CAMNUMBER=$nr
make clean && make singlecamera CAMNUMBER=$nr $SOMETHING && make uploadsinglecamera CAMNUMBER=$nr
