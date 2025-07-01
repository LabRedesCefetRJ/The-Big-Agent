# The-Big-Agent

# Introduction:

This project is composed of 3 main parts:

* controllers: The robot's controller for the Webots simulator
* MAS : Starting MAS project with ChonIDE
* worlds : The world for the simulator

# Prerequisites:

To run this example, you have to be running one of the following Linux distributions :

* Ubuntu ( 22.04 or 24.04 )
* Mint
* Debian 12

1 - Install the Webots simulator from (https://github.com/cyberbotics/webots)

2 - Install the following packages using apt:

    sudo apt install git make gcc

3 - Install the following packages from ChonOS:

* Add ChonOS repositories following the instructions in https://packages.chon.group/
* Install the following packages using apt:

        sudo apt install chonide chonos-serial-port-emulator

# Execution:

1 - Open a CLI terminal and create/go to the folder you wish to download the project to.

2 - Download the project with git.

    git clone https://github.com/daniel4244/The-Big-Agent.git

3 - Enter the folder and run make

    cd The-Big-Agent
    sudo chmod 666 /dev/ttyExogenous0/dev/
    sudo chmod +x ./Projeto_Drone/controllers/Drone_Controller_Test/Drone_Controller_Test
    
    make clean all
    make clean
    webots worlds/mundo.wbt

This will download and compile the library JavinoInC and run the world

4 - In other terminal, run the agents

    cd MAS
    jasonEmbedded Grindor.mas2j

5 - Done!

# Apresentation Video

https://youtu.be/oFyfsq-eDTg

