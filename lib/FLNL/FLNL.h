/*! \mainpage FLNL client/server library documentation
 *
 *  \section intro_sec Inroduction
 *  This documentation describes the different classes of the FLNL client/server library. This library ables to send and receive double values and simple commands between two systems (a client and a server) in a RT fashion (no buffering).<BR>
 *  The library depends on pthread and uses BSD sockets. It can be compiled for Windows and Linux.
 *
 *
 *  \section contact Contact
 *  vcrocher@unimelb.edu.au
 *  <BR><BR>
 *
 */

/**
 * \file FLNL.h
 * \brief Network classes declaration
 * \author Vincent Crocher
 * \version 1.2
 * \date November 2020
 *
 *
 */

#ifndef FLNL_H_INCLUDED
#define FLNL_H_INCLUDED


#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <string>

#if defined(_WIN32) || defined(WIN32) || defined(X64)
    #define WINDOWS
#endif

#ifdef WINDOWS
    #include <winsock2.h>
    #include "pthread.h"
    //Fix for error on macro expension on windows pthread
    #define pthread_cleanup_pop(E) (*pthread_getclean() = _pthread_cup.next,(E?_pthread_cup.func((pthread_once_t *)_pthread_cup.arg):void(0)));}
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <pthread.h>
#endif

#define MESSAGE_SIZE 255        //!< Messages (frame) size in bytes
#define EXPECTED_DOUBLE_SIZE 8  //!< Size expected for the doubles: will be checked at startup and should be same on server and client size
#define CMD_SIZE 4              //!< Commands length in chars
//#define VERBOSE //Talkative ? (connection, every missed message...)

const double HANDSHAKE_VALUE = 42.0+M_PI;
const std::string HANDSHAKE_CMD("Hand");

void * receiving(void *c);

//! A network base class able to send and receive data asynchronously as well as basic commands
//! Allows to send and receive two types of messages (both of total fixed length MESSAGE_SIZE bytes):
//! 1) Asynchronous data values (doubles) without buffering (only latest one arrived is kept)
//! The datagram is in the form 'V' - [NbValues] - [DoubleValue1InBytes] - ... - [DoubleValueNInBytes] - [0 padding] - [ChecksumByte]
//! 2) Asynchronous command messages (4 characters) with arbitary nb of parameters (doubles), without buffering (only latest one arrived is kept)
//! The datagram is in the form 'C' - [NbParams] - [CMD4CHARACTERS] - [DoubleValue1InBytes] - ... - [DoubleValueNInBytes] - [0 padding] - [ChecksumByte]
class baseSocket
{
    friend void * receiving(void *c);

    public:
        //!@name Constructor and destructor
        //@{
            baseSocket();
            virtual ~baseSocket();
        //@}

        //!@name Connecting and disconnecting methods
        //@{
            virtual int Connect(const char * addr, short int port) = 0;
            virtual int Reconnect() {return 0;};
            virtual int Disconnect();
            bool IsConnected();
        //@}

        //!@name Sending methods
        //@{
            int Send(const std::vector<double> &values);
            int Send(const std::string &cmd, const std::vector<double> &params={});
        //@}

        //!@name Receiving methods
        //@{
            bool IsReceivedValues();
            int GetReceivedValues(std::vector<double> &vals);
            bool IsReceivedCmd();
            int GetReceivedCmd(std::string &cmd, std::vector<double> &vals);
            void ClearReceivedCmd();
        //@}

    protected:
        int Socket;                                             //!< Local socket
        struct sockaddr_in sin;                                 //!< Socket parameters structure
        bool Connected;                                         //!< TRUE if client is connected to a server, FALSE otherwise
        const unsigned short int MaxNbValues;                   //!< Max possible number of doubles to send in a frame (calculated from MESSAGE_SIZE and double size)
        unsigned short int NbReceivedValues;                    //!< Number of double values received
        unsigned short int NbReceivedCmdParams;                 //!< Number of parameters received with last command
        const unsigned char InitValueCode = 'V';
        const unsigned char InitCommandCode = 'C';

        double * ReceivedValues;
        bool IsValues;                                          //!< TRUE if last values are received (since last GetReceivedValues()), FALSE otherwise
        char * ReceivedCmd;
        double * ReceivedCmdParams;
        bool IsCmd;
        pthread_mutex_t ReceivedMutex;                          //!< Mutex protecting read/writes to received values
        pthread_mutex_t ReceivedCmdMutex;                       //!< Mutex protecting read/writes to received cmds
        pthread_t ReceivingThread;                              //!< Receiving pthread

    private:
        unsigned char FullMessageOut[MESSAGE_SIZE];             // Preallocated buffer for outgoing messages
        unsigned char FullMessageIn[MESSAGE_SIZE];              // Preallocated buffer for incoming messages
        unsigned char MessageRemainingToProcess[MESSAGE_SIZE];  // Buffer for incoming messages processing
        unsigned char ToProcess[MESSAGE_SIZE];                  // Buffer for incoming messages processing
};


//! A network client able to send and receive data (doubles)
class client: public baseSocket
{
    public:
        //!@name Constructor and destructor
        //@{
            client(): baseSocket(){};
        //@}

        //!@name Connecting and disconnecting methods
        //@{
            int Connect(const char * addr, short int port = 2048);
        //@}
};

void * accepting(void *c);

//! A network server able to send and receive data (doubles)
class server: public baseSocket
{
    friend void * accepting(void *c);

    public:
        //!@name Constructor and destructor
        //@{
            server(): baseSocket(), Waiting(false) { };
        //@}

        //!@name Connecting and disconnecting methods
        //@{
            int Connect(const char * addr, short int port = 2048);
            int Disconnect();
            int Reconnect();
        //@}

    private:
        int ServerSocket;                   //!< Server (accepting) socket
        bool Waiting;                       //!< TRUE if server is currently waiting for incoming client, FALSE otherwise
        pthread_t AcceptingThread;          //!< Accepting connection (from client) pthread
};

#endif // FLNL_H_INCLUDED
