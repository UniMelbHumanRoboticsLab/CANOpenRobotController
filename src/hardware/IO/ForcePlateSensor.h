/*/**
 * \file ForcePlateSensor.h
 * \author Justin Fong
 * \brief  Class representing a Robotous Force Torque sensor. Note that this is not a CANOpen Device.
 * \version 0.1
 * \date 2021-01-12
 * \version 0.1
 * \copyright Copyright (c) 2021
 *
 */

#ifndef ForcePlateSensor_H_INCLUDED
#define ForcePlateSensor_H_INCLUDED

#include <CANopen.h>
#include <CO_command.h>


#include <Eigen/Dense>
 
#include "ForcePlateConstants.h"
#include "InputDevice.h"
#include "logging.h"
#include "RPDO.h"
#include "TPDO.h"

class ForcePlateSensor : public InputDevice {
    private:
        int commandID;     //   COB-ID of command messages    
        int responseID1; // COB-ID of 1st received message
        int responseID2;  // COB-ID of 2nd received message

        bool streaming=false;

        // Objects representing the PDOs (used to create the PDOs in the OD)
        TPDO *tpdo1;
        RPDO *rpdo1;
        RPDO *rpdo2;

        /// Raw data - these variables are linked to the PDOs
        ForcePlateCommand command = NONE;

        // Data variables
        Eigen::VectorXi forces;

       public:
        /**
        * \brief Sets up the Robotous sensor, including data storage and setting up PDOs
        *
        * \param commandID_ the COB-ID used to send messages to this device
        * \param responseID1_ the COB-ID of the first data message (sent from this device) 
        * \param responseID2_ the COB-ID of the second data message (sent from this device) 
        */
        ForcePlateSensor(int commandID_, int responseID1_, int responseID2_);


        /**
         * @brief Get the Command ID object (can be used as an identifier as there should only be one of each)
         * 
         * @return int 
         */
        int getCommandID();

        /**
         * \brief Sets up the receiving PDOs (note: will have issues if commands are sent, as the response are on the same COB-IDs)
         * 
         */
        bool configureMasterPDOs();

        /**
         * \brief Updates the forces from the raw data
         * 
         */
        void updateInput();

        /**
         * @brief Starts the Robotous Sensor Streaming data (sends 0x0B)
         * 
         * @return true if the sensor was previously not streaming (i.e. the stream is starting)
         * @return false if the sensor was previously streaming (i.e. no change in state)
         */
        bool startStream();

        /**
         * @brief Stops the Robotous Sensor Streaming data (sends 0x0B)
         * 
         * @return true if the sensor was previously streaming (i.e. the stream is starting)
         * @return false if the sensor was previously not streaming (i.e. no change in state)
         */
        bool stopStream();

        /**
         * @brief Check if the system is streaming
         * 
         * @return true if streaming
         * @return false if not streaming
         */
        bool getStreaming();

        /**
         * \brief Get the Forces object
         * 
         * \return Eigen::VectorXd X,Y,Z forces
         */
        Eigen::VectorXi &getForces();

        /**
         * \brief  Zero the force sensors
         * 
         * \return true always
         * \return false no reason for this yet
         */
        bool zero();

};
#endif