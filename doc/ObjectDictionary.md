# Modifying the Object Dictionary for PDO Messages (DEPRECIATED)

> Note: This has changed. The PDOs are now programmatic. To create a PDO, one needs to create an RPDO or TPDO object with the appropriate parameters. The PDO is linked directly to a variable in the constructor of the object. 

The CORC implementation uses the CANOpenNode Stack, and their implementation of the object dictionary. 

The CORC implementation of PDOs is currently quite poor - although functions exist to set up PDOs on CAN Nodes, the local object dictionary (which must also be configured to mirror these PDOs) is hard coded in. Thus if new devices/PDOs are added, modifications to the Object Dictionary must be made. These are in the files `CO_OD.c` and `CO_OD.h` in `src\core\CANopen\objDict`. 

The goal of this file is to provide some information about how to do this. Note that this does not present a complete picture. 

There are 2 main changes that must be made:

1. Creating an entry in the Object Dictionary (allowing it to be accessed by the CANOpenNode Stack)
2. Configuring the RPDO or TPDO parameters

This is done through the already-implemented example of statusword for the motor drives. 

## 1. Creating an entry in the Object Dictionary

### 1.1 Data Structures and Allocation of Memory

The following is how we have defined the status word datatype in `CO_OD.h`:
```C++
/*6041    */ typedef struct
{
    UNSIGNED8 numberOfMotors;
    UNSIGNED16 motor1;
    UNSIGNED16 motor2;
    UNSIGNED16 motor3;
    UNSIGNED16 motor4;
    UNSIGNED16 motor5;
    UNSIGNED16 motor6;
} OD_statusWords_t;
```

Note that we have chosen to put this at OD Address 6041 to mirror its location on the motor drives, however, this is arbitrary. Note also that we can include more motors than we intend to use - these variables will simply not be accessed. 

The typedef for the datatype is then used within the struct used for the object dictionary (again in `CO_OD.h`). 
```C++
/***** Structure for RAM variables ********************************************/
struct sCO_OD_RAM {
    UNSIGNED32 FirstWord;

    /*1000      */ UNSIGNED32 deviceType;
    .
    .
    /*6040      */ OD_controlWords_t controlWords;
    /*6041      */ OD_statusWords_t statusWords;
    /*6064      */ OD_actualMotorPositions_t actualMotorPositions;
    .
    .
    /*6411      */ INTEGER16 writeAnalogueOutput16Bit[8];

    UNSIGNED32 LastWord;
};
```

The CO_OD_RAM is then instantiated in `CO_OD.c` using the above `sCO_OD_RAM` struct:
```C++
struct sCO_OD_RAM CO_OD_RAM = {
    CO_OD_FIRST_LAST_WORD,
    
    /*1000*/ 0x0000L,
    .
    .
    /*6005*/ 0x0000,
    /*6040*/ {0x6L, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    /*6041*/ {0x6L, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    /*6064*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*606c*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    .
    .
    CO_OD_FIRST_LAST_WORD,
};
```
Note that the order of the definitions are important, and the data initialised in the RAM needs to be of the correct size. 

### 1.2 Creating methods to access the memory on the CORC-side Code
In `CO_OD.c`, the following allows the data to be accessed by the CANopenNode methods - the items are a pointer to the location in memory of the item in the OD, an attribute for the item (this allows it to be mapped to a PDO, among other things - see `CO_SDO_OD_attributes_t` for details), and the length of the data in bytes. 
```C++
/*0x6041*/ const CO_OD_entryRecord_t OD_record6041[7] = {
    {(void *)&CO_OD_RAM.statusWords.numberOfMotors, 0x06, 0x1},
    {(void *)&CO_OD_RAM.statusWords.motor1, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.statusWords.motor2, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.statusWords.motor3, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.statusWords.motor4, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.statusWords.motor5, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.statusWords.motor6, 0xfe, 0x2},
};
```

The following in `CO_OD.c` combines everything to a single array as the "Object Dictionary". 
```C++
extern const CO_OD_entry_t CO_OD[CO_OD_NoOfElements] = {

    {0x1000, 0x00, 0x86, 4, (void *)&CO_OD_RAM.deviceType},
    .
    .
    {0x6040, 0x06, 0x00, 0, (void *)&OD_record6040},
    {0x6041, 0x06, 0x00, 0, (void *)&OD_record6041},
    {0x6064, 0x06, 0x00, 0, (void *)&OD_record6064},
    .
    .
    {0x6411, 0x08, 0x8e, 2, (void *)&CO_OD_RAM.writeAnalogueOutput16Bit[0]},
};
```
Items in each line (structure) are the index, the maximum subIndex, the attributes (see `CO_SDO_OD_attributes_t`), the length, and a pointer. In this code, it seems that some items have been set to 0, and this doesn't seem to affect functionality at the moment. I believe this might be because we do not access these items through SDOs. More details can be found in `CO_OD_entry_t`. 


## 2. Configuring the PDOs

The configurations are done in the instantiation of the Object Dictionary in `CO_OD.c`. Particularly, RPDOs are configured using address `0x14nn` and `0x16nn` (`nn` representing PDO number), and TPDOs are configured using `0x18nn` and `0x1ann`. The OD is set up to allow up to 16 RPDOs an 16 TPDOs (I am not sure if more are permitted).  

## RPDO Configuration

```C++
/***** Definition for RAM variables ********************************************/
struct sCO_OD_RAM CO_OD_RAM = {
    CO_OD_FIRST_LAST_WORD,
    .
    .
    /*1400*/ {{0x2L, 0x0181L, 0xffL},
    /*1401*/ {0x2L, 0x0182L, 0xffL},
    /*1402*/ {0x2L, 0x0183L, 0xffL},
    /*1403*/ {0x2L, 0x0184L, 0xffL},
    /*1404*/ {0x2L, 0x0185L, 0xfeL},
    /*1405*/ {0x2L, 0x0186L, 0xfeL},
    .
    .
    /*1600*/ {{0x1L, 0x60410110L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*1601*/ {0x1L, 0x60410210L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*1602*/ {0x1L, 0x60410310L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*1603*/ {0x1L, 0x60410410L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*1604*/ {0x1L, 0x60410510L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*1605*/ {0x1L, 0x60410610L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    .
```

The elements for the `OD_RPDOCommunicationParameter_t` (`0x14nn`) are: maxSubIndex, COB_ID, Transmission Type. The `StatusWord` PDO are configured to RPDOs 0-5, with each assigned a maximum subindex of 2 (this is required for RPDOs), the COB-ID of 181-186, and the 0xff as transmission type - which corresponds to asynchronous update. Note that this can also be changed to 1-240, whcih corresponds to updating the OD once every n SYNC messages. We also have two set to 0xfe = 254. This is "manufacturer specific", so am not sure what this means on this device. 

The elements for `OD_RPDOMappingParameter_t` (`0x16nn`) are: number of mapped objects, and mappedobject1, mappedobject2, etc. In the case of RPDO0, this is (0x60410110), which corresponds to address 0x6041 (in the local object dictionary), Subindex 01, and a data size of 10 = 16 bits.

## TPDO Configuration


```C++
/***** Definition for RAM variables ********************************************/
struct sCO_OD_RAM CO_OD_RAM = {
    CO_OD_FIRST_LAST_WORD,
    .
    .
    /*1800*/ {{0x6L, 0x0201L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
    /*1801*/ {0x6L, 0x0202L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
    .
    .
    /*1a00*/ {{0x1L, 0x60400110L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*1a01*/ {0x1L, 0x60400210L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    .
    .

```
This is a similar configuration to the RPDO, except the `OD_TPDOCommunicationParameter_t` (`0x18nn`) includes additional parameters relating to how and when the tranmissions occur. The `CO_RPDOMapPar_t` (`0x1ann`) is the same as for the RPDO. 
