/*/**
 * \file RobotousRFT.h
 * \author Justin Fong
 * \brief  Class representing a Robotous Force Torque sensor. Note that this is not a CANOpen Device.
 * \version 0.1
 * \date 2021-01-12
 * \version 0.1
 * \copyright Copyright (c) 2021
 *
 */

#ifndef ROBOTOUSRFT_H_INCLUDED
#define ROBOTOUSRFT_H_INCLUDED

#include <CANopen.h>
#include <CO_command.h>
#include <string.h>
#include <Eigen>

#include "logging.h"

class RobotousRFT {
    private:
        int commandID;     //   COB-ID of command messages    
        int responseID1; // COB-ID of 1st received message
        int responseID2;  // COB-ID of 2nd received message

        // Raw Data store
        // Because the Robotous people are stupid, one of the variables is split over the two messages. So we have to
        // store the raw data and then convert it
        // Resp1: [D1 D2 D3 D4 D5 D6 D7 D8]
        // [0x10, Fx_u, Fx_l, Fy_u, Fy_l, Fz_u, Fz_l, Tx_u] 
        // Resp2: [D9 D10 D11 D12 D13 D14 D15 D16]
        // [Tx_l, Ty_u, Ty_l, Tz_u, Tz_l, 0x00, 0x00]

        /// Raw data
        UNSIGNED8 rawData[16] = {0};

        // Data variables
        Eigen::VectorXd forces;
        Eigen::VectorXd torques;
        

    public:
        /**
        * \brief Sets up the Robotous sensor, including data storage and setting up PDOs
        *
        * \param commandID_ the COB-ID used to send messages to this device
        * \param responseID1_ the COB-ID of the first data message (sent from this device) 
        * \param responseID2_ the COB-ID of the second data message (sent from this device) 
        */
        RobotousRFT(int commandID_, int responseID1_, int responseID2_);

        /**
         * \brief Sets up the receiving PDOs (note: will have issues if commands are sent, as the response are on the same COB-IDs)
         * 
         */
        void setupPDO();

        /**
         * \brief Updates the forces from the raw data
         * 
         */
        void update();

        /**
         * \brief Get the Forces object
         * 
         * \return Eigen::VectorXd X,Y,Z forces
         */
        Eigen::VectorXd getForces();

        /**
         * \brief Get the Forces object
         * 
         * \return Eigen::VectorXd 
         */
        Eigen::VectorXd getTorques();
    
}
#endif