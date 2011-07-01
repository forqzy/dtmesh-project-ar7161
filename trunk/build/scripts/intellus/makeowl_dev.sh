#!/bin/bash

# source the variables needed for build
TOP=$1/build
cd $TOP

make BOARD_TYPE=pb42fus

echo "---------------------"
find . -name \* -user root -exec sudo chown build {} \; -print 
find . -name \.config  -exec chmod 777 {} \; -print 
