# The-Big-Agent

Introduction:
This project is composed of 3 main parts:

    controllers: The robot's controller for the Webots simulator
    MAS : Starting MAS project with ChonIDE
    worlds : The world for the simulator

Prerequisites:
To run this example, you have to be running one of the following Linux distributions :

    Ubuntu ( 22.04 or 24.04 )
    Mint
    Debian 12

    Install the Webots simulator from (https://github.com/cyberbotics/webots)

    Install the following packages using apt:

sudo apt install git make gcc

    Install the following packages from ChonOS:

    Add ChonOS repositories following the instructions in https://packages.chon.group/
    Install the following packages using apt:

sudo apt install chonide chonos-serial-port-emulator

Compilation:
To compile this example from the Command Line Interface (CLI):

    Open a CLI terminal and create/go to the folder you wish to download the project to.

For exemple, if you wish to install it in the home folder of the current logged user:

cd ~/

    Download the project with git.

git clone https://github.com/bptfreitas/Exemplo_Nardin

    Enter the folder and run make

sudo chmod 666 /dev/ttyExogenous0
sudo chmod +x ./controllers/Grindor.c
make clean all
make clean

This will download and compile the library JavinoInC

Execution:

1 - To start the scenario from the CLI, you have to be in the project folder and run:

webots --mode=pause worlds/world01.wbt

This will start the simulator paused.

2 - Start the ChonIDE Embedded MAS IDE

3 - Navigate to the folder of the downloaded project and, in the MAS folder, select the corresponding MAS project for ChonIDE.

4 - Done! 

