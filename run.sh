#!/bin/bash

git clone https://github.com/daniel4244/Projeto_Drone
cd Projeto_Drone/Projeto_Drone || exit

sudo chmod 666 /dev/ttyExogenous0

sudo chmod +x ./controllers/Drone_Controller_Test/Grindor

make clean all
make clean

webots worlds/mundo.wbt 
cd Projeto_Drone/MAS/droneMAS
