/**
 * \file TPDO.h
 * \author Justin Fong
 * \brief  The <code>TPDO</code> class is used to create TPDOs in the Programmatic PDO
 * 
 * \version 0.1
 * \date 2021-02-02
 *
 * \copyright Copyright (c) 2021
 *
 */

#ifndef TPDO_H_INCLUDED
#define TPDO_H_INCLUDED


#include <CANopen.h>

#include "logging.h"

class TPDO {
    private:
        // Dummy Variables for extraneous OD entries
        UNSIGNED8 nullData = 0;
        UNSIGNED8 lengthData = 0;
        UNSIGNED32 myCOBID = 0;

    public:
        // Storage for the configuration parameters for the TPDO
        OD_TPDOMappingParameter_t mappingParam = {0x8L, 0x00000108L, 0x00000208L, 0x00000308L, 0x00000408L, 0x00000508L, 0x00000608L, 0x00000708L, 0x00000808L};
        OD_TPDOCommunicationParameter_t commParam = {0x2L, 0x0L, 0xffL, 0x00, 0x0L, 0x00, 0x0L};

        // Object Dictionary Entries for the parameters, along with the pointers to the linked variables
        CO_OD_entryRecord_t dataRecord[9] = {
            {(void *)&lengthData, 0x06, 0x1},
            {(void *)&nullData, 0xfe, 0x1},
            {(void *)&nullData, 0xfe, 0x1},
            {(void *)&nullData, 0xfe, 0x1},
            {(void *)&nullData, 0xfe, 0x1},
            {(void *)&nullData, 0xfe, 0x1},
            {(void *)&nullData, 0xfe, 0x1},
            {(void *)&nullData, 0xfe, 0x1},
            {(void *)&nullData, 0xfe, 0x1},
        };

        CO_OD_entryRecord_t mappingRecord[9] = {
            {(void *)&mappingParam.numberOfMappedObjects, 0x0e, 0x1},
            {(void *)&mappingParam.mappedObject1, 0x8e, 0x4},
            {(void *)&mappingParam.mappedObject2, 0x8e, 0x4},
            {(void *)&mappingParam.mappedObject3, 0x8e, 0x4},
            {(void *)&mappingParam.mappedObject4, 0x8e, 0x4},
            {(void *)&mappingParam.mappedObject5, 0x8e, 0x4},
            {(void *)&mappingParam.mappedObject6, 0x8e, 0x4},
            {(void *)&mappingParam.mappedObject7, 0x8e, 0x4},
            {(void *)&mappingParam.mappedObject8, 0x8e, 0x4},
        };

        CO_OD_entryRecord_t commRecord[7] = {
            {(void *)&commParam.maxSubIndex, 0x06, 0x1},
            {(void *)&commParam.COB_IDUsedByTPDO, 0x8e, 0x4},
            {(void *)&commParam.transmissionType, 0x0e, 0x1},
            {(void *)&commParam.inhibitTime, 0x8e, 0x2},
            {(void *)&commParam.compatibilityEntry, 0x0e, 0x1},
            {(void *)&commParam.eventTimer, 0x8e, 0x2},
            {(void *)&commParam.SYNCStartValue, 0x0e, 0x1},
        };

        /**
      * \brief Creates TPDO with appropriate mapping parameters 
      *
      * \param COBID The COB-ID on which the TPDO is sent
      * \param tranmissionType SYNCs per tranmissions or on change if 0xff (see CO_PDO.h)
      * \param dataEntry An array of addresses of variables to link to the TPDO
      * \param dataSize An array indicating the size of the variables (in bytes)
      * \param numMappedObjects Number of mapped objects in the TPDO  
      */
        TPDO(UNSIGNED32 COBID, UNSIGNED8 transmissionType, void *dataEntry[], UNSIGNED16 dataSize[], UNSIGNED8 numMappedObjects);

        // Makes sense to overload the constructor to allow access to the other communication parameters
        // This could be added if required

        /**
      * \brief returns the COBID of this RPDO
      * 
      * @return UNSIGNED32 COB-ID of this RPDO
      */
        UNSIGNED32 getCOBID();
};

#endif 