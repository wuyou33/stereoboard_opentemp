#!/bin/bash
nr=$1

make clean && make singlecamera CAMNUMBER=$nr && make uploadsinglecamera CAMNUMBER=$nr
