################################## USER FLAGS ##################################

## Which platform (robot) is the state machine using?
## this is the correspondig folder name in src/hardware/platform to use
set(PLATFORM PLATFORM_FOLDER)

## State machine uses FLNL for network communication
set(USE_FLNL TRUE)

################################################################################

################## AUTOMATED PATH AND NAME DEFINITION ##########################

## StateMachine name is (and should be!) current folder name
get_filename_component(STATE_MACHINE_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
## And its relative path to the root folder is:
file(RELATIVE_PATH STATE_MACHINE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_LIST_DIR}/)

################################################################################
