/**
 * \file RPDO.h
 * \author Justin Fong
 * \brief  The <code>RPDO</code> class is used to create RPDOs in the Programmatic PDO
 * 
 * \version 0.1
 * \date 2021-02-02
 * \version 0.1
 *
 * \copyright Copyright (c) 2021
 *
 */

#ifndef RPDO_H_INCLUDED
#define RPDO_H_INCLUDED

#include <CANopen.h>

#include "logging.h"

class RPDO {
    private:
        // Dummy Variables for extraneous OD entries
        UNSIGNED8 nullData = 0;
        UNSIGNED8 lengthData = 0;
        UNSIGNED32 myCOBID =0;

    public:
    // Storage for the configuration parameters for the RPDO
     OD_RPDOMappingParameter_t mappingParam = {0x8L, 0x00000108L, 0x00000208L, 0x00000308L, 0x00000408L, 0x00000508L, 0x00000608L, 0x00000708L, 0x00000808L};
     OD_RPDOCommunicationParameter_t commParam = {0x2L, 0x0L, 0xffL};

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

    CO_OD_entryRecord_t commRecord[3] = {
         {(void *)&commParam.maxSubIndex, 0x06, 0x1},
         {(void *)&commParam.COB_IDUsedByRPDO, 0x8e, 0x4},
         {(void *)&commParam.transmissionType, 0x0e, 0x1},
    };

    /**
      * \brief Creates RPDO with appropriate mapping parameters 
      *
      * \param COBID The COB-ID on which the RPDO is received
      * \param tranmissionType SYNCs per processing of data or on change if 0xff (see CO_PDO.h)
      * \param dataEntry An array of addresses of variables to link to the RPDO
      * \param dataSize An array indicating the size of the variables (in bytes)
      * \param numMappedObjects Number of mapped objects in the RPDO  
      */    
     RPDO(UNSIGNED32 COBID, UNSIGNED8 transmissionType, void *dataEntry[], UNSIGNED16 dataSize[], UNSIGNED8 numMappedObjects);

     /**
      * \brief returns the COBID of this RPDO
      * 
      * @return UNSIGNED32 COB-ID of this RPDO
      */
     UNSIGNED32 getCOBID();
};

#endif 