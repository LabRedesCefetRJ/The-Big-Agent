#! /bin/bash
FRAMEWORK="/usr/bin/jasonEmbedded"
WEBOTS="/usr/local/bin/webots"
SERIALPORT="/dev/ttyEmulatedPort0"
XTERM="/usr/bin/xterm"

clear
if [[ ! -f "$FRAMEWORK"  ]] || [[ ! -f "$WEBOTS" ]] || [[ ! -e "$SERIALPORT" ]] || [[ ! -e "$XTERM" ]]
then
    echo "Installing dependencies..."
    sudo clear
    echo "deb [trusted=yes] http://packages.chon.group/ chonos main" | sudo tee /etc/apt/sources.list.d/chonos.list
    sudo apt update
    sudo apt install linux-headers-`uname -r` -y
    sudo apt install jason-embedded chonos-serial-port-emulator webots xterm -y
    clear
else
    echo "The computer has JasonEmbedded and Webots"
fi

echo "Starting Secretary MAS..."
sleep 5
cd Secretary/MAS/
xterm -e "jasonEmbedded secretary.mas2j" &
echo "Starting Simulated World..."

sleep 5
cd ../../Drone/simulator/worlds/
webots mundo.wbt &
echo "Start Drone MAS ??? (Press Enter to continue...)"
read 
cd ../../MAS/
xterm -e "jasonEmbedded drone.mas2j" &
cd ../../
