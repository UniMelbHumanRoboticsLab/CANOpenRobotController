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
//! \param addr : The server ip address (local address to listen on)
//! \param port : The server port (default is 2048)
//! \return 0 if OK
//! \return -1 if bind() error
//! \return -2 if error creating the accepting thread
int server::Connect(const char * addr, short int port)
{
    //Initialise server address and ports
    sin.sin_addr.s_addr = inet_addr(addr);
    sin.sin_port = htons(port);

    //Initialise server socket
    ServerSocket = socket(AF_INET, SOCK_STREAM, 0);

    //Initialise
    Waiting=false;
    if(bind(ServerSocket, (struct sockaddr*)&sin, sizeof(sin))==-1) {
        #ifdef WINDOWS
            printf("WSA Error code : %d\t", WSAGetLastError());
        #endif
        perror("FLNL::server::bind() :");
        close(ServerSocket);
        return -1;
    }
    else {
       return Reconnect();
    }
}

//! Wait for a client launching an Accepting thread. Can be called if socket initialised but disconnected
//! \return 0 if OK
//! \return -1 if already connected or already waiting
//! \return -2 if error creating the accepting thread
int server::Reconnect()
{
    if(IsConnected() || Waiting) {
        return -1;
    }

    //Ecoute d'une seule connexion
    listen(ServerSocket, 1);

    //Creation d'un thread d'attente de connexion du client
    Waiting=true;
    if(pthread_create(&AcceptingThread, NULL, accepting, (void*)this)!=0) {
        Waiting=false;
        printf("FLNL::server::Connect() : Error creating accepting thread\n");
        return -2;
    }

    //OK
    return 0;
}

//! Disconnect and close the socket
//! \return the close(Socket) return value
int server::Disconnect()
{
    pthread_cancel(ReceivingThread);
    pthread_cancel(AcceptingThread);
    Connected=false;
    Waiting=false;


    int ret=0;
    if(Socket>0) {
        ret=close(Socket);
        if(ret==0) {
            Socket = -1;
            #ifdef VERBOSE
            printf("FLNL::Disconnected.\n");
            #endif
            #ifdef WINDOWS
                WSACleanup();
            #endif
        }
        else {
            printf("FLNL::Error closing connection.\n");
        }
    }

    return ret;
}

//! Thread function waiting for a client connection: terminates when a client is connected
//! \param c : A pointer on server object
//! \return NULL
void * accepting(void * c)
{
    server * local_server = (server*)c;

    //Ensure client socket is closed
    if(local_server->Socket!=-1)
        close(local_server->Socket);
    local_server->Socket=-1;
    //Wait for new incoming connection
    while(local_server->Socket<0 && local_server->Waiting)
        local_server->Socket = accept(local_server->ServerSocket, NULL, NULL);

    if(!local_server->Waiting)
        return NULL;

    //Connection OK
    local_server->Connected=true;
    #ifdef VERBOSE
    printf("FLNL::server::Connected.\n");
    #endif

    //Create a receiving thread
    if(pthread_create(&local_server->ReceivingThread, NULL, receiving, (void*)local_server)!=0) {
        printf("FLNL::server::Connect() : Error creating receiving thread\n");
        local_server->Connected=false;
    }
    local_server->Waiting=false;

    return NULL;
}
/*#################################################################################################################*/
