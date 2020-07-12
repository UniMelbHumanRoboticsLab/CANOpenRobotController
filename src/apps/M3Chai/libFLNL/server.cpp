/**
 * \file server.cpp
 * \brief Network server class implementation
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
//! Open a connection and wait for a client launching an Accepting thread
//! \param addr : The server ip address (local address)
//! \return 0 if OK
//! \return -1 if bind() error
//! \return -2 if error creating the accepting thread
int server::Connect(char * addr)
{
    //Initialise server address
    sin.sin_addr.s_addr = inet_addr(addr);

    //Initialise
    Waiting=false;
    if(bind(Socket, (struct sockaddr*)&sin, sizeof(sin))==-1)
    {
        #ifdef WINDOWS
            printf("WSA Error code : %d\t", WSAGetLastError());
        #endif
        perror("FLNL::server::bind() :");
        close(Socket);
        return -1;
    }
    else
    {
       return Reconnect();
    }
}

//! Wait for a client launching an Accepting thread. Can be called if socket initialised but disconnected
//! \return 0 if OK
//! \return -1 if already connected or already waiting
//! \return -2 if error creating the accepting thread
int server::Reconnect()
{
    if(IsConnected() || Waiting)
    {
        return -1;
    }

    //Ecoute d'une seule connexion
    listen(Socket, 1);

    //Creation d'un thread d'attente de connexion du client
    if(pthread_create(&AcceptingThread, NULL, accepting, (void*)this)!=0)
    {
        printf("FLNL::server::Connect() : Error creating accepting thread\n");
        return -2;
    }

    //OK
    return 0;
}

//! Thread function waiting for a client connection
//! \param c : A pointer on server object
//! \return NULL
void * accepting(void * c)
{
    server * local_server = (server*)c;

    local_server->Waiting=true;

    #ifdef WINDOWS
        int taille = sizeof(SOCKADDR_IN);
        local_server->Socket=-1;
        SOCKADDR_IN csin;
        while(local_server->Socket==INVALID_SOCKET)
            local_server->Socket = accept(local_server->Socket, (struct sockaddr*)&csin, &taille);
    #else
        socklen_t taille = sizeof(local_server->Socket);
        local_server->Socket = accept(local_server->Socket, (struct sockaddr*)&local_server->Socket, &taille);
    #endif

    //Connection OK
    printf("FLNL::server::Connected.\n");
    local_server->Connected=true;

    //Create a receiving thread
    if(pthread_create(&local_server->ReceivingThread, NULL, receiving, (void*)local_server)!=0)
    {
        printf("FLNL::server::Connect() : Error creating receiving thread\n");
        local_server->Connected=false;
    }
    local_server->Waiting=false;

    return NULL;
}
/*#################################################################################################################*/
