/*! \mainpage FLNL client/server library documentation
 *
 *  \section intro_sec Inroduction
 *  This documentation describes the different classes of a network client/server library. This library ables to send double values between two systems (a client and a server). <BR>
 * A server have to be created in a first time, waiting for a connection. Then a client on a remote machine could connect to it (with ip address) and so the communication could began.<BR>
 * The library could be compiled under Linux and Windows (choose the corresponding target in Code::Blocks project).
 *
 *  \subsection sub_linux Linux
 *  For Linux the project generates the file libFLNL.so in the unix directory.
 *  \subsection sub_win Windows
 *  For Windows the project generates the files libFLNL.a and libFLNL.dll in the win directory.
 * <BR>
 *
 *  \section contact Contact
 *  vincent.crocher@isir.upmc.fr
 *  <BR><BR>
 *
 */

/**
 * \file server.hpp
 * \brief Network server class declaration
 * \author Vincent Crocher
 * \version 0.5
 * \date March 2010
 *
 *
 */

#ifndef SERVER_HPP_INCLUDED
#define SERVER_HPP_INCLUDED

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>


#ifdef WIN32
    #define WINDOWS
#endif
#ifdef X64
    #define WINDOWS
#endif

#ifdef WINDOWS
    #include <winsock.h>
    #include "pthread/pthread.h"
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <pthread.h>
#endif




void * ServerReceiving(void *c);
void * Accepting(void *c);


//! A network server able to send and receive data (doubles)
class server
{
    friend void * ServerReceiving(void *c);
    friend void * Accepting(void *c);

    public:
        //!@name Constructor and destructor
        //@{
            server(int nb_values_to_send, int nb_values_to_receive);
            ~server();
        //@}

        //!@name Connecting and disconnecting methods
        //@{
            int Connect(char * addr);
            int Disconnect();
            bool IsConnected();
        //@}

        //!@name Sending methods
        //@{
            int Send(double * values);
        //@}

        //!@name Receiving methods
        //@{
            bool IsReceivedValues();
            double * GetReceivedValues();
        //@}

    private:
        int Socket;                 //!< Local socket
        int ClientSocket;           //!< Distant client socket
        struct sockaddr_in sin;     //!< Socket parameters structure
        bool Connected;             //!< TRUE if server as client connected, FALSE otherwise
        int NbValuesToSend;         //!< Number of double values the server send to the client
        int NbValuesToReceive;      //!< Number of double values the server receive from the client
        double * ReceivedValues;    //!< Tab of the last received value from the client
        bool IsValues;              //!< TRUE if last values are received (since last GetReceivedValues()), FALSE otherwise
        pthread_t ReceivingThread;  //!< Receiving pthread
        pthread_t AcceptingThread;  //!< Accepting connection (from client) pthread
};

#endif // SERVER_HPP_INCLUDED
