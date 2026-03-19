/**
 * \file I2CDevice.h
 * \author Vincent Crocher
 * \brief  Generic I2CDevice class adapted from Michael Brookes library.
 *
 * \version 0.1
 * \date 2026-03-04
 *
 * \copyright Copyright (c) 2026
 *
 */

#ifndef I2CDEVICE_H
#define I2CDEVICE_H

#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/stat.h>

#include "logging.h"


#define I2C_BUS_COUNT   1               //!< How many I2C buses are available. Add paths below as required.
#define I2C_1           "/dev/i2c-1"    //!< File Path for the i2c-1 bus (one used on BB)

#define ONE_BYTE        1               //!< Used for specifying how many bytes to read
#define TWO_BYTES       2               //!< Used for specifying how many bytes to write
#define MAX_BUFF        1024            //!< Used to store fatal error.


/**
 * @ingroup IO
 * \brief Class describing an I2C device. Based on Michael Brookes library.
 * Typical use:
 *   1) Create I2CDevice object
 *   2) setDeviceAddress()
 *   3) setBusId()
 *   4) InitDevice (will check and connect to device)
 *   5) Read/Write.
 *
 * Use i2cdetect -y -r 1 command to list adresses of devices connected on the bus 1 (/dev/i2c-1) of the BB.
 */
class I2CDevice {
    public:
        /**
         \fn Public Destructor
         */
        ~I2CDevice();

        /**
         \fn Public Constructor
         */
        I2CDevice();

        /**
         \brief Requires the device address and bus id to be configured.
         */
        void initDevice();

        /**
         \brief Writes the register that you want to read, then performs a read on that register.
         \param unsigned char _RegisterValue
         */
        short getValueFromRegister(unsigned char _RegisterAddress);

		/**
         \brief Query value of given register using I2C_RDWR (faster)
         \param unsigned char _RegisterValue
         */
		short readRegisterRdwr(unsigned char _RegisterAddress);

		/**
         \brief Query two consecutive register and concatenate using I2C_RDWR (faster)
         \param unsigned char _RegisterValue
         */
		uint16_t readRegister16Rdwr(unsigned char _RegisterAddress);

		/**
         \brief Query Nx2 consecutive registers using I2C_RDWR (faster) and return as array of N signed values
         \param unsigned char _RegisterValue of first register of Nx2 to read
         \param uint16_t nbVal Number of 16bits values to read
         \param int16_t *val array to store the N values
         */
		void readBlockValues16RdwrS(unsigned char _RegisterAddress, uint16_t nbVal, int16_t val[]);

        /**
         \brief Query Nx2 consecutive registers using I2C_RDWR (faster) and return as array of N UNsigned values
         \param unsigned char _RegisterValue of first register of Nx2 to read
         \param uint16_t nbVal Number of 16bits values to read
         \param uint16_t *val array to store the N values
         */
		void readBlockValues16RdwrU(unsigned char _RegisterAddress, uint16_t nbVal, uint16_t val[]);

        /**
         \brief Set the value that will next be written to the I2C device.
         \param unsigned char _RegisterValue
         */
        void setRegisterValue( unsigned char _RegisterValue );

        /**
         \brief Set the Register address that the _RegisterValue will be written to.
         \param unsigned char _RegisterAddress
         */
        void setRegisterAddress( unsigned char _RegisterAddress );

        /**
         \brief Perform the write request. The _BufferSize is used to differentiate between a read or write request.
         \param size_t _BufferSize
         */
        int writeToDevice( size_t _BufferSize );

        /**
         \brief Convenience, wrapping, method to write to a register
         \param unsigned char addr Register address to write to
         \param uint8_t val Value to write to register
         */
        void writeReg(unsigned char addr, uint8_t val) {
                setRegisterValue(val);
                setRegisterAddress(addr);
                writeToDevice(2);
        };

    private:

        /**
         \fn Private Struct I2CBus
         \brief used to store Bus Paths for ease of access.
         */
        struct I2CBus { const char * BusPath; } _Bus[ I2C_BUS_COUNT ];

        /**
         \brief used to check file paths.
         */
        struct stat buffer;

    protected:

        /**
         \brief Returns the current FileHandle for reading and writing to the I2C device.
         \param none
         */
        int getDeviceFileHandle( );

        /**
         \brief Returns then FilePath for accessing the I2C device.
         \param none
         */
        const char * getFilePath( );

        /**
         \brief Set Path to all the available buses. As set with I2CBus (struct) and Defines.
         \param none
         */
        void setBusPaths( );

        /**
         \brief Make sure the BusId being used is valid.
         \param int _BusId
         */
        int validateBusId( );

        /**
         \brief Make sure the BusPath exists.
         \param I2CBus _BusId
         */
        char * validateBusPath( char * _BusProposedPath );

        /**
         \brief Select which Bus Path we can find your I2C device at.
         \param none
         */
        void selectABusPath( );

        /**
         \brief Used to store the device address (Hex)
         \param int _DeviceAddress
         */
        virtual void setDeviceAddress( unsigned char _DeviceAddress ) = 0;

        /**
         \brief Used to store the bus id (int)
         \param int _BusId
         */
        virtual void setBusId( int _BusId ) = 0;

        /**
         \brief Sets up an IOCTL connection to the I2C device as a Slave for Linux.
         \param none
         */
        int connectToDevice( ) { return ioctl( this->FileHandle, I2C_SLAVE, this->DeviceAddress ); };

        /**
        * \brief Attempt to open the FileHandle.
        * \param none
        */
        int openDevice( );

        /**
         \brief Reads the current buffer from the I2C device - first writes the register address that will be read.
         \param size_t _BufferSize
         */
        short readDevice( size_t _BufferSize );

        const char * DeviceBusPath;

        unsigned char DeviceAddress;
        unsigned char RegisterValue;
        unsigned char RegisterAddress;

        char ReadAndWriteBuffer[ TWO_BYTES ];
        char WriteBufferOnly[ ONE_BYTE ];

        char ErrMessage[ MAX_BUFF ];
        int FileHandle;
        int BusId;

        bool DeviceInitialised;

};

#endif //I2CDEVICE_H

