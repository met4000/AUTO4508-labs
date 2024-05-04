#!/bin/bash

set -e

g++ -c -o bezier.o bezier.cpp -lstdc++
gcc -shared -o libbezier.so bezier.o
