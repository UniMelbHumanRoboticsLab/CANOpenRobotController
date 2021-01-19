/*/**
 * \file RobotousRFT.h
 * \author Justin Fong
 * \brief  Class representing a Robotous Force Torque sensor. 
 * 
 *    NOTE: this is not a CANOpen Device, and the PDO-like messages are send on non-standard COB-IDs
 * 
 * \version 0.1
 * \date 2021-01-12
 * \copyright Copyright (c) 2021
 *
 */

#ifndef ROBOTOUSRFT_H_INCLUDED
#define ROBOTOUSRFT_H_INCLUDED

#include <CANopen.h>
#include <CO_command.h>
#include <string.h>

#include <Eigen/Dense>

#include "InputDevice.h"
#include "logging.h"
#include "RPDO.h"
#include "TPDO.h"

class RobotousRFT : public InputDevice {
    private:
        int commandID;     //   COB-ID of command messages    
        int responseID1; // COB-ID of 1st received message
        int responseID2;  // COB-ID of 2nd received message

        bool streaming=false; 

        // Raw Data store
        // Because the Robotous people are stupid, one of the variables is split over the two messages. So we have to
        // store the raw data and then convert it
        // RespH: [D1 D2 D3 D4 D5 D6 D7 D8]
        // [0x10, Fx_u, Fx_l, Fy_u, Fy_l, Fz_u, Fz_l, Tx_u] 
        // RespL: [D9 D10 D11 D12 D13 D14 D15 D16]
        // [Tx_l, Ty_u, Ty_l, Tz_u, Tz_l, OL_status, 0x00, 0x00]

        // Objects representing the PDOs (used to create the PDOs in the OD)
        TPDO *tpdo1;
        RPDO *rpdo1;
        RPDO *rpdo2;

        /// Raw data - these variables are linked to the PDOs
        UNSIGNED8 rawData[16] = {0};
        UNSIGNED8 cmdData = 0;
        UNSIGNED32 cmdDataPad = 0; // This is to make sure that the message is the full 8 bytes because of Robotous' not-CANopen implementation

        // Number of mapped parameters for RPDOs (lengthData) and TPDO (lengthCmd)
        UNSIGNED8 lengthData =8; // 8 for each of the RPDOs - I cheat and reuse this variable
        UNSIGNED8 lengthCmd = 2; // Second one is for padding

        // OD Parameters
        // Will need to be modified to take into number of items, data size and location
        // Data size and number of items will be constant, function will be used to change location
        OD_RPDOMappingParameter_t RPDOMapParamH = {0x8L, 0x00000108L, 0x00000208L, 0x00000308L, 0x00000408L, 0x00000508L, 0x00000608L, 0x00000708L, 0x00000808L};
        OD_RPDOMappingParameter_t RPDOMapParamL = {0x8L, 0x00000108L, 0x00000208L, 0x00000308L, 0x00000408L, 0x00000508L, 0x00000608L, 0x00000708L, 0x00000808L};
        OD_TPDOMappingParameter_t TPDOMapParam = {0x2L, 0x00000108L, 0x00000238L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L};

        OD_RPDOCommunicationParameter_t RPDOcommParaH = {0x2L, 0x0L, 0xffL}; // {0x2L, COB-ID, 0xffL}
        OD_RPDOCommunicationParameter_t RPDOcommParaL = {0x2L, 0x0L, 0xffL};  // {0x2L, COB-ID, 0xffL}
        OD_TPDOCommunicationParameter_t TPDOcommPara = {0x6L, 0x0L, 0xffL, 0x0, 0x0, 0x0, 0x0};  // {0x2L, COB-ID, 0xffL}

        // Records which are linked to from the OD
        // These might need to be public
        CO_OD_entryRecord_t dataStoreRecordH[9] = {
            {(void *)&lengthData, 0x06, 0x1},
            {(void *)&rawData[0], 0xfe, 0x2},
            {(void *)&rawData[1], 0xfe, 0x2},
            {(void *)&rawData[2], 0xfe, 0x2},
            {(void *)&rawData[3], 0xfe, 0x2},
            {(void *)&rawData[4], 0xfe, 0x2},
            {(void *)&rawData[5], 0xfe, 0x2},
            {(void *)&rawData[6], 0xfe, 0x2},
            {(void *)&rawData[7], 0xfe, 0x2},
        };
        CO_OD_entryRecord_t dataStoreRecordL[9] = {
            {(void *)&lengthData, 0x06, 0x1},
            {(void *)&rawData[8], 0xfe, 0x2},
            {(void *)&rawData[9], 0xfe, 0x2},
            {(void *)&rawData[10], 0xfe, 0x2},
            {(void *)&rawData[11], 0xfe, 0x2},
            {(void *)&rawData[12], 0xfe, 0x2},
            {(void *)&rawData[13], 0xfe, 0x2},
            {(void *)&rawData[14], 0xfe, 0x2},
            {(void *)&rawData[15], 0xfe, 0x2},
        };
        CO_OD_entryRecord_t dataStoreRecordCmd[3] = {
            {(void *)&lengthCmd, 0x06, 0x1},
            {(void *)&cmdData, 0xfe, 0x2},
            {(void *)&cmdDataPad, 0xfe, 0x8},
        };

        CO_OD_entryRecord_t RPDOCommEntryH[3] = {
            {(void *)&RPDOcommParaH.maxSubIndex, 0x06, 0x1},
            {(void *)&RPDOcommParaH.COB_IDUsedByRPDO, 0x8e, 0x4},
            {(void *)&RPDOcommParaH.transmissionType, 0x0e, 0x1},
        };

        CO_OD_entryRecord_t RPDOCommEntryL[3] = {
            {(void *)&RPDOcommParaL.maxSubIndex, 0x06, 0x1},
            {(void *)&RPDOcommParaL.COB_IDUsedByRPDO, 0x8e, 0x4},
            {(void *)&RPDOcommParaL.transmissionType, 0x0e, 0x1},
        };
        CO_OD_entryRecord_t TPDOCommEntry[3] = {
            {(void *)&TPDOcommPara.maxSubIndex, 0x06, 0x1},
            {(void *)&TPDOcommPara.COB_IDUsedByTPDO, 0x8e, 0x4},
            {(void *)&TPDOcommPara.transmissionType, 0x0e, 0x1},
        };

        // This is constant (pointers to above)
        CO_OD_entryRecord_t RPDOMapParamEntryH[9] = {
            {(void *)&RPDOMapParamH.numberOfMappedObjects, 0x0e, 0x1},
            {(void *)&RPDOMapParamH.mappedObject1, 0x8e, 0x4},
            {(void *)&RPDOMapParamH.mappedObject2, 0x8e, 0x4},
            {(void *)&RPDOMapParamH.mappedObject3, 0x8e, 0x4},
            {(void *)&RPDOMapParamH.mappedObject4, 0x8e, 0x4},
            {(void *)&RPDOMapParamH.mappedObject5, 0x8e, 0x4},
            {(void *)&RPDOMapParamH.mappedObject6, 0x8e, 0x4},
            {(void *)&RPDOMapParamH.mappedObject7, 0x8e, 0x4},
            {(void *)&RPDOMapParamH.mappedObject8, 0x8e, 0x4},
        };

        CO_OD_entryRecord_t RPDOMapParamEntryL[9] = {
            {(void *)&RPDOMapParamL.numberOfMappedObjects, 0x0e, 0x1},
            {(void *)&RPDOMapParamL.mappedObject1, 0x8e, 0x4},
            {(void *)&RPDOMapParamL.mappedObject2, 0x8e, 0x4},
            {(void *)&RPDOMapParamL.mappedObject3, 0x8e, 0x4},
            {(void *)&RPDOMapParamL.mappedObject4, 0x8e, 0x4},
            {(void *)&RPDOMapParamL.mappedObject5, 0x8e, 0x4},
            {(void *)&RPDOMapParamL.mappedObject6, 0x8e, 0x4},
            {(void *)&RPDOMapParamL.mappedObject7, 0x8e, 0x4},
            {(void *)&RPDOMapParamL.mappedObject8, 0x8e, 0x4},
        };
        CO_OD_entryRecord_t TPDOMapParamEntry[9] = {
            {(void *)&TPDOMapParam.numberOfMappedObjects, 0x0e, 0x1},
            {(void *)&TPDOMapParam.mappedObject1, 0x8e, 0x4},
            {(void *)&TPDOMapParam.mappedObject2, 0x8e, 0x4},
            {(void *)&TPDOMapParam.mappedObject3, 0x8e, 0x4},
            {(void *)&TPDOMapParam.mappedObject4, 0x8e, 0x4},
            {(void *)&TPDOMapParam.mappedObject5, 0x8e, 0x4},
            {(void *)&TPDOMapParam.mappedObject6, 0x8e, 0x4},
            {(void *)&TPDOMapParam.mappedObject7, 0x8e, 0x4},
            {(void *)&TPDOMapParam.mappedObject8, 0x8e, 0x4},
        };
        // Data variables
        Eigen::VectorXd forces;
        Eigen::VectorXd torques;
        Eigen::VectorXd forceOffsets;
        Eigen::VectorXd torqueOffsets;

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

        bool startStream();

        bool stopStream();

        bool getStreaming();

        /**
         * \brief Get the Forces object
         * 
         * \return Eigen::VectorXd X,Y,Z forces
         */
        Eigen::VectorXd& getForces();

        /**
         * \brief Get the Forces object
         * 
         * \return Eigen::VectorXd 
         */
        Eigen::VectorXd& getTorques();

        /**
         * \brief Set the offsets for the forces and torques
         *  
         */
        void setOffsets(Eigen::VectorXd forceOffset, Eigen::VectorXd torqueOffset);
};
#endif