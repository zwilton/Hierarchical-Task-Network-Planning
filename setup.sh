#! /bin/bash
cd satellite-generator
make clean
make
cd ..
cd bwstates_src
make clean
make
cd ..
cd Metric-FF
make clean
make

