#!/bin/bash

g++ -std=c++11 -DNDEBUG -Wall -Wstrict-prototypes -fPIC -I/home/nobug-ros/anaconda3/include/python3.7m -c myIKFastwrapNew.cpp -o ikModule.o -llapack

g++ -shared ikModule.o -o ikModule.so -llapack

python installer.py
