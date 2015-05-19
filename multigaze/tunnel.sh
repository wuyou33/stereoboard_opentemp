#!/bin/bash
nr=$1

make clean && make TUNNEL=$1 && make upload2
