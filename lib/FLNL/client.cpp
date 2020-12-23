/**
 * \file client.cpp
 * \brief Network client class implementation
 * \author Vincent Crocher
 * \version 0.8
 * \date July 2020
 *
 *
 */

#include "FLNL.h"

/*-----------------------------------------------------------------------------------------------------------------*/
/*###################################   CONNECTING AND DISCONNECTING METHODS    ###################################*/
/*-----------------------------------------------------------------------------------------------------------------*/
//! Try to connect to the server and create a receiving thread
//! \param addr : The server ip address to connect to
//! \param port : The communication port (default is 2048)
//! \return 0 if OK
//! \return -1 if connect() error
//! \return -2 if error creating the receiving thread
int client::Connect(const char * addr, short int port)
{
    //Initialise server address and port
    sin.sin_addr.s_addr = inet_addr(addr);
    sin.sin_port = htons(port);

    //Initialise socket
    Socket = socket(AF_INET, SOCK_STREAM, 0);

    //Try to connect
    if (connect(Socket,(struct sockaddr*)&sin,sizeof(sin)) == -1) {
        //Connection error
        perror("FLNL::client::Connect() :");
        return -1;
    }
    else
    {
        //Connection OK
        #ifdef VERBOSE
        printf("FLNL::client::Connected to %s.\n", addr);
        #endif
        Connected=true;

        //Create a receiving thread
        if(pthread_create(&ReceivingThread, NULL, receiving, (void*)this)!=0) {
            printf("FLNL::client::Connect() : Error creating receiving thread\n");
            return -2;
        }

        //OK
        return 0;
    }
}
/*#################################################################################################################*/


