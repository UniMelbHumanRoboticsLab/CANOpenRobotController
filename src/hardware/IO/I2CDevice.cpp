#include "./I2CDevice.h"
#include <chrono>


I2CDevice::I2CDevice( ) {
    this->DeviceAddress = 0x00;
    this->BusId = 0;
    this->DeviceInitialised = false;
}

I2CDevice::~I2CDevice( ) { if(FileHandle) close(FileHandle); }

void I2CDevice::initDevice( ) {
    if(!this->DeviceAddress) {
        spdlog::error("I2C Device Not Configured ( try : 'obj->SetDeviceAddress([hex address])' )");
        return;
    }
    if(!this->BusId) {
        spdlog::error("I2C Device Not Configured ( try : 'obj->SetBusId([bus number])' )");
        return;
    }

    /*
     * ** ## -- Setup Stage -- ## ** *
     * SetBusPaths : Saves the file paths to the available buses for ease of access.
     */
    this->setBusPaths( );

    /*
     * ** ## -- Assignment Stage ( based on args ) -- ## ** *
     * ValidateBusId : Make sure we have a valid bus ID before proceeding.
     * SelectABusPath : Used to specify which bus your I2C device is on.
     * SetDeviceAddress: Hex value for your specific I2C Device.
     */
    this->validateBusId( );
    this->selectABusPath( );

    /*
     * ** ## -- Init Stage -- ## ** *
     * OpenDevice : Creates a file handle for the device, should it be closed? Probably... :)
     * ConnectToDevice : Assigns the device as an I2C Slave and checks availability using IOCTL
     *
     * More info on IOCTL : http://man7.org/linux/man-pages/man2/ioctl.2.html
     */
    this->openDevice( );
    this->connectToDevice( );

    this->DeviceInitialised = true;
}

void I2CDevice::setBusPaths( ) {
    this->_Bus[ 1 ].BusPath = validateBusPath( (char *)I2C_1 );
}

void I2CDevice::selectABusPath( ) { this->DeviceBusPath = _Bus[ this->BusId ].BusPath; }

void I2CDevice::setRegisterAddress( unsigned char _RegisterAddress ) { this->RegisterAddress = _RegisterAddress; }

void I2CDevice::setRegisterValue( unsigned char _RegisterValue ){ this->RegisterValue = _RegisterValue; }

const char * I2CDevice::getFilePath( ) { return this->DeviceBusPath; }

int I2CDevice::getDeviceFileHandle( ) { return this->FileHandle; }

int I2CDevice::validateBusId( ) {
    if( this->BusId > I2C_BUS_COUNT || this->BusId < 1 ) {
        spdlog::error("Bus ID : {}  is not a valid BUS for this device.", this->BusId);
        return -1;
    } else {
        return EXIT_SUCCESS;
    }
}

char * I2CDevice::validateBusPath( char * _I2CBusProposedPath ) {
    if( stat ( _I2CBusProposedPath, &buffer) == 0 )
        return _I2CBusProposedPath;
    else {
        spdlog::error("Fatal I2C Error - Unable to locate the I2C Bus file : {}", _I2CBusProposedPath);
        return NULL;
    }
}
short I2CDevice::getValueFromRegister( unsigned char _RegisterAddress ) {
    if(!this->DeviceInitialised) {
        spdlog::error("I2C Device Not Initialised ( try : 'obj->InitDevice()' )");
        return -1;
    }

    setRegisterAddress( _RegisterAddress );
    WriteBufferOnly[ 0 ] = RegisterAddress;

    if( write( getDeviceFileHandle( ), WriteBufferOnly, 1 ) == 1 ) {
        return readDevice( ONE_BYTE );
    } else {
        spdlog::error("Fatal I2C Error - Unable to write to file : {}", getFilePath());
        return -1;
    }
}

short I2CDevice::readDevice( size_t _BufferSize ) {
    if(!DeviceInitialised) {
        spdlog::error("I2C Device Not Initialised ( try : 'obj->InitDevice()' )");
        return -1;
    }

    unsigned char buff[ _BufferSize ];
    if( read( this->getDeviceFileHandle( ), buff, _BufferSize ) != _BufferSize ) {
        spdlog::error("Fatal I2C Error - Unable to read from file : {}", getFilePath());
        return -1;
    } else {
        return buff[ 0 ];
    }
}

short I2CDevice::readRegisterRdwr(unsigned char _RegisterAddress) {
    if (!DeviceInitialised) {
        spdlog::error("I2C Device Not Initialised ( try : 'obj->InitDevice()' )");
        return -1;
    }

    unsigned char outbuf[1] = { _RegisterAddress };
    unsigned char inbuf[1]  = { 0 };

    struct i2c_msg messages[2];
    struct i2c_rdwr_ioctl_data packets;

    messages[0].addr  = static_cast<__u16>(this->DeviceAddress);
    messages[0].flags = 0;                 // write
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = outbuf;

    messages[1].addr  = static_cast<__u16>(this->DeviceAddress);
    messages[1].flags = I2C_M_RD;          // read
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = inbuf;

    packets.msgs  = messages;
    packets.nmsgs = 2;

    if (ioctl(this->getDeviceFileHandle(), I2C_RDWR, &packets) < 0) {
        spdlog::error("I2C_RDWR ioctl failed for {}", getFilePath());
        return -1;
    }

    return static_cast<short>(inbuf[0]);
}

uint16_t I2CDevice::readRegister16Rdwr(unsigned char _RegisterAddress) {
    if (!DeviceInitialised) {
        spdlog::error("I2C Device Not Initialised ( try : 'obj->InitDevice()' )");
        return -1;
    }

    unsigned char outbuf[1] = { _RegisterAddress };
    unsigned char inbuf[2]  = { 0, 0 };

    struct i2c_msg messages[2];
    struct i2c_rdwr_ioctl_data packets;

    messages[0].addr  = static_cast<__u16>(this->DeviceAddress);
    messages[0].flags = 0;                 // write register pointer
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = outbuf;

    messages[1].addr  = static_cast<__u16>(this->DeviceAddress);
    messages[1].flags = I2C_M_RD;          // read two bytes
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = inbuf;

    packets.msgs  = messages;
    packets.nmsgs = 2;

    if (ioctl(getDeviceFileHandle(), I2C_RDWR, &packets) < 0) {
        spdlog::error("I2C_RDWR ioctl failed for {}", getFilePath());
        return -1;
    }

    // Combine as little-endian (LSB first)
    uint16_t value = (static_cast<uint16_t>(inbuf[1]) << 8)
                   |  static_cast<uint16_t>(inbuf[0]);

    return value;
}

void I2CDevice::readBlockValues16RdwrS(unsigned char _RegisterAddress, uint16_t nbVal, int16_t val[]) {
    if (!DeviceInitialised) {
        spdlog::error("I2C Device Not Initialised ( try : 'obj->InitDevice()' )");
        return;
    }

    unsigned char outbuf[1] = { _RegisterAddress };
    unsigned char inbuf[2*nbVal];

    struct i2c_msg messages[2];
    struct i2c_rdwr_ioctl_data packets;

    messages[0].addr  = static_cast<__u16>(this->DeviceAddress);
    messages[0].flags = 0;                 // write register pointer
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = outbuf;

    messages[1].addr  = static_cast<__u16>(this->DeviceAddress);
    messages[1].flags = I2C_M_RD;          // read two bytes
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = inbuf;

    packets.msgs  = messages;
    packets.nmsgs = 2;

    if (ioctl(getDeviceFileHandle(), I2C_RDWR, &packets) < 0) {
        spdlog::error("I2C_RDWR ioctl failed for {}", getFilePath());
        return;
    }

    // Combine each pair as little-endian (LSB first)
    for(int i=0; i<nbVal; i++) {
        val[i] = static_cast<int16_t>(static_cast<uint16_t>(inbuf[2*i+1]) << 8)
                   |  static_cast<uint16_t>(inbuf[2*i]);

    }
}

void I2CDevice::readBlockValues16RdwrU(unsigned char _RegisterAddress, uint16_t nbVal, uint16_t val[]) {
    if (!DeviceInitialised) {
        spdlog::error("I2C Device Not Initialised ( try : 'obj->InitDevice()' )");
        return;
    }

    unsigned char outbuf[1] = { _RegisterAddress };
    unsigned char inbuf[2*nbVal];

    struct i2c_msg messages[2];
    struct i2c_rdwr_ioctl_data packets;

    messages[0].addr  = static_cast<__u16>(this->DeviceAddress);
    messages[0].flags = 0;                 // write register pointer
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = outbuf;

    messages[1].addr  = static_cast<__u16>(this->DeviceAddress);
    messages[1].flags = I2C_M_RD;          // read two bytes
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = inbuf;

    packets.msgs  = messages;
    packets.nmsgs = 2;

    if (ioctl(getDeviceFileHandle(), I2C_RDWR, &packets) < 0) {
        spdlog::error("I2C_RDWR ioctl failed for {}", getFilePath());
        return;
    }

    // Combine each pair as little-endian (LSB first)
    for(int i=0; i<nbVal; i++) {
        val[i] = static_cast<uint16_t>(static_cast<uint16_t>(inbuf[2*i+1]) << 8)
                   |  static_cast<uint16_t>(inbuf[2*i]);

    }
}

int I2CDevice::openDevice( ) {
    FileHandle = open( getFilePath( ), O_RDWR );
    if( this->FileHandle == 0 ) {
        spdlog::error("Fatal I2C Error - Unable to open file : {}",  getFilePath());
        return -1;
    }
    return this->FileHandle;
}

int I2CDevice::writeToDevice( size_t _BufferSize ) {
    if(!DeviceInitialised) {
        spdlog::error("I2C Device Not Initialised ( try : 'obj->InitDevice()' )");
        return -1;
    }

    if( _BufferSize > ONE_BYTE ) {
        this->ReadAndWriteBuffer[ 0 ] = this->RegisterAddress;
        this->ReadAndWriteBuffer[ 1 ] = this->RegisterValue;
        write( getDeviceFileHandle( ), this->ReadAndWriteBuffer, _BufferSize );
    } else {
        this->WriteBufferOnly[ 0 ] = this->RegisterAddress;
        write( getDeviceFileHandle( ), this->WriteBufferOnly, _BufferSize );
    }
    return EXIT_SUCCESS;
}
