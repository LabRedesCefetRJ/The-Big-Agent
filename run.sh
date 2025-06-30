#!/bin/bash

git clone https://github.com/daniel4244/The-Big-Agent.git
cd The-Big-Agent || exit

sudo chmod 666 /dev/ttyExogenous0
sudo chmod +x ./controllers/Grindor.c

make clean all
make clean

webots worlds/mundo.wbt 
