/**
 * \file baseSocket.cpp
 * \brief Network base class implementation
 * \author Vincent Crocher
 * \version 0.8
 * \date July 2020
 *
 *
 */

#include "FLNL.h"


/*-----------------------------------------------------------------------------------------------------------------*/
/*#########################################   CONSTRUCTOR AND DESTRUCTOR   ########################################*/
/*-----------------------------------------------------------------------------------------------------------------*/
//! Constructor initializing data and creating a socket
//! \param nb_values_to_send : Number of double values the server send to the client
//! \param nb_values_to_receive : Number of double values the server receive from the client
baseSocket::baseSocket(unsigned char nb_values_to_send, unsigned char nb_values_to_receive):
                        NbValuesToSend(nb_values_to_send),
                        NbValuesToReceive(nb_values_to_receive)
{
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
baseSocket::~baseSocket()
{
    Connected=false;

    if(close(Socket)==0)
    {
        printf("FLNL::Connection closed.\n");
        #ifdef WINDOWS
            WSACleanup();
        #endif
    }
    else
    {
        printf("FLNL::Error closing connection.\n");
    }

    delete[] ReceivedValues;
}
/*#################################################################################################################*/




/*-----------------------------------------------------------------------------------------------------------------*/
/*###################################   CONNECTING AND DISCONNECTING METHODS    ###################################*/
/*-----------------------------------------------------------------------------------------------------------------*/
//! Disconnect and close the socket
//! \return the close(Socket) return value
int baseSocket::Disconnect()
{
    Connected=false;
    return close(Socket);
    printf("FLNL::Disconnected.\n");
}

//! Tell if connected
//! \return TRUE if connected to the server, FALSE otherwise
bool baseSocket::IsConnected()
{
    return Connected;
}
/*#################################################################################################################*/



/*-----------------------------------------------------------------------------------------------------------------*/
/*###############################################   SENDING METHODS   #############################################*/
/*-----------------------------------------------------------------------------------------------------------------*/
//! Send double values accross
//! \param values : A pointer on a tab of doubles of NbValuesToSend elements
//! \return the send() return value
int baseSocket::Send(double * values)
{
    //Convert double values in char tab
    unsigned int size_of_values_in_char = NbValuesToSend*sizeof(double);
    unsigned char values_in_char[size_of_values_in_char];
    memcpy(values_in_char, values, size_of_values_in_char);
    unsigned char full_command[size_of_values_in_char + 3];
    full_command[0]=InitCode;
    full_command[1]=NbValuesToSend;
    unsigned char command_hash = 0;
    for(unsigned int i=0; i<size_of_values_in_char; i++) {
        full_command[2+i] = values_in_char[i];
        command_hash ^= values_in_char[i];
    }
    full_command[2+size_of_values_in_char]=command_hash;
    //Send the char array
    return send(Socket, full_command, size_of_values_in_char+3, 0);
}
/*#################################################################################################################*/




/*-----------------------------------------------------------------------------------------------------------------*/
/*############################################  RECIVING METHODS  #################################################*/
/*-----------------------------------------------------------------------------------------------------------------*/
//! Tell if values have been received since last GetReceivedValues()
//! \return TRUE if values have been received from the server, FALSE otherwise
bool baseSocket::IsReceivedValues()
{
    return IsValues;
}

//! Return the last received values from the server
//! \return A tab of doubles of NbValuesToReceive elements, received from the server
double * baseSocket::GetReceivedValues()
{
    IsValues=false;
    return ReceivedValues;
}

//! Thread function waiting for data from remote side
//! \param c : A pointer on the baseSocket object
//! \return NULL
void * receiving(void * c)
{
    baseSocket * local=(baseSocket*)c;

    unsigned char buffer[local->NbValuesToReceive*sizeof(double)+3];

    while(local->Connected) {
        int ret=recv(local->Socket, buffer, local->NbValuesToReceive*sizeof(double)+3, 0);
        if(ret>0) {
            unsigned int start_pos=0;
            //for(start_pos=0; start_pos<ret-1; start_pos++)
            //Check init code and number of values are matching
            if(buffer[start_pos]==local->InitCode && buffer[start_pos+1]==local->NbValuesToReceive) {
                //Compute and check hash
                unsigned char command_hash = 0;
                unsigned int i;
                for(i=start_pos+2; i<ret-1; i++) {
                    command_hash ^= buffer[i];
                }

                if(command_hash==buffer[i]) {
                    //Recopie des valeurs recues s'il y en a
                    memcpy(local->ReceivedValues, &buffer[start_pos+2], local->NbValuesToReceive*sizeof(double));
                    local->IsValues=true;
                }
                else {
                    //Incorrect values
                    printf("FLNL::Error receiving (wrong hash).\n");
                }
            }
            else {
                //Incorrect values
                printf("FLNL::Error receiving (wrong data format).\n");
            }

        }
        else if(ret<0) {
            #ifdef WINDOWS
                printf("WSA Error code : %d\t", WSAGetLastError());
            #endif
            perror("FLNL::Error receiving");
            if(errno==EBADF) { //Connection has been closed by client
                local->Disconnect();
            }
        } else if(ret==0) {
            //Connection has been reseted
            local->Disconnect();
        }
    }

    return NULL;
}
/*#################################################################################################################*/
