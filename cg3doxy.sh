#!/bin/bash
rm -r /home/alessandro/cg3lib_doc/html/*
cd cg3lib
( cat doxygen_config ; echo "OUTPUT_DIRECTORY=/home/alessandro/cg3lib_doc" ) | doxygen -
cd ..
