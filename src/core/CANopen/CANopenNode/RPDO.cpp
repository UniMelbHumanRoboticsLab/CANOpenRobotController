#include "RPDO.h"

RPDO::RPDO(UNSIGNED32 COBID, UNSIGNED8 transmissionType, void *dataEntry[], UNSIGNED16 dataSize[], UNSIGNED8 numMappedObjects) {
    myCOBID = COBID;
    commParam.COB_IDUsedByRPDO = COBID;
    commParam.transmissionType = transmissionType;

    lengthData = numMappedObjects;
    mappingParam.numberOfMappedObjects = numMappedObjects;

    // Required to do pointer indexing because of the stupid way OD_RPDOMappingParameter_t is defined.
    uint32_t *pMap = &mappingParam.mappedObject1;
    for (int i = 0; i < numMappedObjects; i++) {
        // Change the dataRecord pointers
        dataRecord[i+1].pData = dataEntry[i];
        dataRecord[i + 1].length = dataSize[i];

        // Change the mapping parameter to include subindex and data size (in bits now...)
        *pMap = 0x00000100*(i+1) + 0x08*dataSize[i];
        pMap++;
    }

    // Then set up the OD
    CO_setRPDO(&commParam, &mappingParam, commRecord, dataRecord, mappingRecord);

}

UNSIGNED32 RPDO::getCOBID(){
    return myCOBID;
}
