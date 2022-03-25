#!/bin/bash

#Upload app and init scripts to BB target 
#using default IP / users parameters

SSH_USER="debian"
BUILD_FOLDER="build/"
INIT_SCRIPTS="*.sh"
SCRIPT_FOLDER="script/"
CONFIG_FOLDER="config/"

#use default BB address (dependent on OS used)
if [[ "$OSTYPE" == "cygwin" ]] || [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "win32" ]]; then 
  #Any flavour of Windows (Cygwin, Mingw...)
  SSH_IP_ADDR="192.168.7.2"
else
  #Any other case (Linux, Unix, OSX...)
  SSH_IP_ADDR="192.168.7.2"
fi

#Check if BB is connected
ping -W 1 -c 1 $SSH_IP_ADDR >/dev/null 2>&1
if [ $? -ne 0 ] ; then 
	echo "Nothing connected on ${SSH_IP_ADDR}. Exiting."
	exit
fi

#set in root folder
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"/..


echo "## Create remote folder (~/CANOpenRobotController/):"
echo -n "ssh -q $SSH_USER@$SSH_IP_ADDR "mkdir -p ~/CANOpenRobotController/" ... "
ssh -q $SSH_USER@$SSH_IP_ADDR "mkdir -p ~/CANOpenRobotController/"
echo "done."

echo ""
echo "## Copy scripts:"
echo -n "rsync -chaz -e 'ssh -q' ${SCRIPT_FOLDER} $SSH_USER@$SSH_IP_ADDR:~/CANOpenRobotController/${SCRIPT_FOLDER} ... "
rsync -chaz -e 'ssh -q' ${SCRIPT_FOLDER} $SSH_USER@$SSH_IP_ADDR:~/CANOpenRobotController/${SCRIPT_FOLDER}
echo "done."

echo ""
echo "## Copy config files:"
echo -n "rsync -chaz -e 'ssh -q' ${CONFIG_FOLDER} $SSH_USER@$SSH_IP_ADDR:~/CANOpenRobotController/${CONFIG_FOLDER} ... "
rsync -chaz -e 'ssh -q' ${CONFIG_FOLDER} $SSH_USER@$SSH_IP_ADDR:~/CANOpenRobotController/${CONFIG_FOLDER}
echo "done."

echo ""
echo "## Copy APP(s):"
echo -n "rsync -chaz -e 'ssh -q' --include='*APP' --include='*APP_NOROBOT' --exclude='*' ${BUILD_FOLDER} $SSH_USER@$SSH_IP_ADDR:~/CANOpenRobotController/${BUILD_FOLDER} ... "
rsync -chaz -e 'ssh -q' --include='*APP' --include='*APP_NOROBOT' --exclude='*' ${BUILD_FOLDER} $SSH_USER@$SSH_IP_ADDR:~/CANOpenRobotController/${BUILD_FOLDER}
echo "done."
