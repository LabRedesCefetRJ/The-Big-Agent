# Exemplo_Nardin

# Introduction

This project is an example of how to run a simulation of an Embedded Multiagent System (MAS). This simulation example is a robotic vehicle with 4 wheels, two distance sensors, and a battery, using the Webots simulator. 

The project structure is composed of 3 main parts:

* _controllers_: The robot's controller for the Webots simulator
* _MAS_ : Starting MAS project with ChonIDE
* _worlds_ : The scenario for the simulator

# Prerequisites 

To run this example, you have to be running one of the following Linux distributions :

* Ubuntu ( 22.04 or 24.04 )
* Mint
* Debian 12

1. Install the Webots simulator from (https://github.com/cyberbotics/webots)

2. Install the following packages using apt:

`sudo apt install git make gcc`

3. Install the following packages from ChonOS:

 - Add ChonOS repositories following the instructions in https://packages.chon.group/
 - Install the following packages using apt: 

`sudo apt install chonide chonos-serial-port-emulator`

# Compilation

To compile this example from the Command Line Interface (CLI):

1. Open a CLI terminal and create/go to the folder you wish to download the project to.

For exemple, if you wish to install it in the home folder of the current logged user:

`cd ~/` 

2. Download the project with git.

`git clone https://github.com/bptfreitas/Exemplo_Nardin`

2. Enter the folder and run _make_

```
cd Exemplo_Nardin
make all
```

This will download and compile the library [_JavinoInC_](https://github.com/chon-group/JavinoInC.git)

This process also assumes that the default installation folder of Webots resides in the folder _/usr/local/webots_ . If that is not the case, you should supply the correct folder in make's invocation:

`make WEBOTS_HOME=/path/to/correct/folder all`

# Execution

1. To start the scenario from the CLI, you have to be in the project folder and run:

`webots --mode=pause worlds/world01.wbt`

This will start the simulator paused so you can open the MAS project first.

To run it detached from the CLI instead:

`nohup webots --mode=pause worlds/world01.wbt &`

The simulator should open with the following screen:

![Webots first execution](https://github.com/bptfreitas/Exemplo_Nardin/blob/39d070d261670e95cb5ce104e7132b04a69a161b/doc/imgs-github/exec_webots-first-execution.png)

2. Start the ChonIDE Embedded MAS IDE

For example, if you are running the Ubuntu 22.04, press the SUPER key and enter on the search "_ChonIDE_".

![Start ChonIDE](https://github.com/bptfreitas/Exemplo_Nardin/blob/39d070d261670e95cb5ce104e7132b04a69a161b/doc/imgs-github/exec_start-chonide.png)

3. After logging in, select "Import Project"

![Import Project](https://github.com/bptfreitas/Exemplo_Nardin/blob/39d070d261670e95cb5ce104e7132b04a69a161b/doc/imgs-github/exec_import-project.png)

4. Navigate to the folder of the downloaded project and, in the MAS folder, select the corresponding MAS project for ChonIDE.

![Select world project](https://github.com/bptfreitas/Exemplo_Nardin/blob/39d070d261670e95cb5ce104e7132b04a69a161b/doc/imgs-github/exec_select-MAS-proj.png)

5. Done! You can start programming the MAS for the loaded scenario :) 







