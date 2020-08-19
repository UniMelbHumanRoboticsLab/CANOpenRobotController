/**
 * \file server.cpp
 * \brief Network server class implementation
 * \author Vincent Crocher
 * \version 0.5
 * \date March 2010
 *
 *
 */

#include "server.h"

/*-----------------------------------------------------------------------------------------------------------------*/
/*#########################################   CONSTRUCTOR AND DESTRUCTOR   ########################################*/
/*-----------------------------------------------------------------------------------------------------------------*/
//! Constructor initializing data and creating a socket
//! \param nb_values_to_send : Number of double values the server send to the client
//! \param nb_values_to_receive : Number of double values the server receive from the client
server::server(int nb_values_to_send, int nb_values_to_receive)
{
    //Copy values
    NbValuesToSend=nb_values_to_send;
    NbValuesToReceive=nb_values_to_receive;

    //Initialise sin
    sin.sin_family = AF_INET;
    sin.sin_port = htons(2048);

    //Initialise and allocates privates
    IsValues=false;
    Connected=false;
    ReceivedValues=new double[NbValuesToReceive];

    #ifdef WINDOWS
        WSADATA WSAData;
        WSAStartup(MAKEWORD(2,0), &WSAData);
    #endif

    //Initialise socket
    Socket = socket(AF_INET, SOCK_STREAM, 0);
}

//! Destructor releasing memory and closing the socket
server::~server()
{
    Connected=false;

    if(close(Socket)==0)
    {
        printf("server::Connection closed.\n");
        #ifdef WINDOWS
            WSACleanup();
        #endif
    }
    else
    {
        printf("server::Error closing connection.\n");
    }

    delete[] ReceivedValues;
}
/*#################################################################################################################*/



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
    if(bind(Socket, (struct sockaddr*)&sin, sizeof(sin))==-1)
    {
        #ifdef WINDOWS
            printf("WSA Error code : %d\t", WSAGetLastError());
        #endif
        perror("server::bind() :");
        close(Socket);
        return -1;
    }
    else
    {
        //Ecoute d'une seulle connexion
        listen(Socket, 1);

        //Creation d'un thread d'attente de connexion du client
        if(pthread_create(&AcceptingThread, NULL, Accepting, (void*)this)!=0)
        {
            printf("server::Connect() : Error creating accepting thread\n");
            return -2;
        }

        //OK
        return 0;
    }
}

//! Thread function waiting for a client connection
//! \param c : A pointer on server object
//! \return NULL
void * Accepting(void * c)
{
    server * local_server;
    local_server=(server*)c;

    #ifdef WINDOWS
        int taille = sizeof(SOCKADDR_IN);
        local_server->ClientSocket=-1;
        SOCKADDR_IN csin;
        while(local_server->ClientSocket==INVALID_SOCKET)
            local_server->ClientSocket = accept(local_server->Socket, (struct sockaddr*)&csin, &taille);
    #else
        socklen_t taille = sizeof(local_server->ClientSocket);
        local_server->ClientSocket = accept(local_server->Socket, (struct sockaddr*)&local_server->ClientSocket, &taille);
    #endif


    //Connection OK
    printf("server::Connected.\n");
    local_server->Connected=true;

    //Create a receiving thread
    if(pthread_create(&local_server->ReceivingThread, NULL, ServerReceiving, (void*)local_server)!=0)
    {
        printf("server::Connect() : Error creating receiving thread\n");
        local_server->Connected=false;
    }

    pthread_exit(NULL);

    return NULL;
}

//! Disconnect from the client and so close the socket
//! \return the close(Socket) return value
int server::Disconnect()
{
    Connected=false;

    return close(Socket);
}

//! Tell if a client is connected
//! \return TRUE if the server have a client connected to, FALSE otherwise
bool server::IsConnected()
{
    return Connected;
}
/*#################################################################################################################*/



/*-----------------------------------------------------------------------------------------------------------------*/
/*###############################################   SENDING METHODS   #############################################*/
/*-----------------------------------------------------------------------------------------------------------------*/
//! Send double values to the client
//! \param values : A pointer on a tab of doubles of NbValuesToSend elements
//! \return the send() return value
int server::Send(double * values)
{
    //Convert double values in char tab
	unsigned char values_in_char[NbValuesToSend*sizeof(double)];
	//ATTENTION, PEUT ETRE ENLEVER LE UNSIGNED POUR WINDOWS
    memcpy(values_in_char, values, NbValuesToSend*sizeof(double));
    //Send the char tab
    return send(ClientSocket, values_in_char, NbValuesToSend*sizeof(double), 0);
}
/*#################################################################################################################*/



/*-----------------------------------------------------------------------------------------------------------------*/
/*############################################  RECIVING METHODS  #################################################*/
/*-----------------------------------------------------------------------------------------------------------------*/
//! Tell if values have been received since last GetReceivedValues()
//! \return TRUE if values have been received from the client, FALSE otherwise
bool server::IsReceivedValues()
{
    return IsValues;
}

//! Return the last received values from the client
//! \return A tab of doubles of NbValuesToReceive elements, received from the client
double * server::GetReceivedValues()
{
    IsValues=false;
    return ReceivedValues;
}

//! Thread function waiting for data from the client
//! \param c : A pointer on the server object
//! \return NULL
void * ServerReceiving(void * c)
{
    server * local_server;
    local_server=(server*)c;

    unsigned char values_in_char[local_server->NbValuesToReceive*sizeof(double)];

    while(local_server->Connected)
    {
        int ret=recv(local_server->ClientSocket, values_in_char, local_server->NbValuesToReceive*sizeof(double), 0);
        if(ret>0)
        {
            //Recopie des valeurs recues s'il y en a
            memcpy(local_server->ReceivedValues, values_in_char, local_server->NbValuesToReceive*sizeof(double));
            local_server->IsValues=true;
        }
        else if(ret<0)
        {
            #ifdef WINDOWS
                printf("WSA Error code : %d\t", WSAGetLastError());
            #endif
            perror("server::Error reciving\n");

        }
    }

    printf("server::Disconnected.\n");

    pthread_exit(NULL);

    return NULL;
}
/*#################################################################################################################*/
