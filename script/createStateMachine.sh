#!/bin/bash
# Script intended to quickly create a new CORC state machine from a simple template.
# It creates a state machine in a separate folder and create the minimal cpp and headers files
# for state machine and states derived classes.
# It does not edit the CMakeFileList which should be edited manually to compile the new state machine.
#
#   vcrocher - Unimelb - 16/04/2025

read -p "Enter the new state machine name in CamelCase (e.g. EMUDemo): " name
read -p "Enter the name of platform folder to use (folder name, e.g. M2, M3...): " platform_folder
read -p "Enter the name of platform class to use (folder name, e.g. RobotM3...): " platform_name

state_machine_name="${name}Machine"
states_name="${name}States"

echo "Creating ${state_machine_name}.cpp/h with states in ${states_name}.cpp/h and platform ${platform_name} (${platform_folder})".


## Create relevant folder
echo "Creating folder ${state_machine_name} ..."
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"
cd ../../
cp -r CANOpenRobotController/src/apps/StateMachineTemplate "./${state_machine_name}"
cd "${state_machine_name}"



#Rename files
echo "Renaming files..." 
mv StateMachineTemplate.h "${state_machine_name}.h" #old school because rename too not convenient and not necessarily installed
mv StateMachineTemplate.cpp "${state_machine_name}.cpp"
mv StatesTemplate.h "${states_name}.h"
mv StatesTemplate.cpp "${states_name}.cpp"


#Templates replacement in files
echo "Editing files..."
sed -i "s/StateMachineTemplate/${state_machine_name}/g" *
sed -i "s/StatesTemplate/${states_name}/g" *
sed -i "s/PlatformName/${platform_name}/g" *
sed -i "s/PLATFORM_FOLDER/${platform_folder}/g" app.cmake
sed -i "s/STATEMACHINETEMPLATE/${state_machine_name^^}/g" *
sed -i "s/STATESTEMPLATE/${name^^}/g" *

echo "$(pwd):"
echo $(ls)
echo "Done"



