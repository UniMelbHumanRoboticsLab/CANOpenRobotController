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
    //Ensure standard size of double
    assert(sizeof(double) == EXPECTED_DOUBLE_SIZE);

    //Initialise sin
    sin.sin_family = AF_INET;
    sin.sin_port = htons(2048);

    //Initialise and allocates privates
    Socket=-1;
    IsValues=false;
    Connected=false;
    ReceivedValues=new double[NbValuesToReceive];

    pthread_mutex_init(&received_mutex, NULL);

    #ifdef WINDOWS
        WSADATA WSAData;
        WSAStartup(MAKEWORD(2,0), &WSAData);
    #endif
}

//! Destructor releasing memory and closing the socket
baseSocket::~baseSocket()
{
    Disconnect();
    pthread_mutex_destroy(&received_mutex);
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
    pthread_cancel(ReceivingThread);
    Connected=false;

    int ret=close(Socket);
    if(ret==0) {
        printf("FLNL::Disconnected.\n");
        #ifdef WINDOWS
            WSACleanup();
        #endif
    }
    else {
        printf("FLNL::Error closing connection.\n");
    }

    return ret;
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
    //Send the char array (MSG_NOSIGNAL required to avoid SIGPIPE signal which will break server on lost connection)
    return send(Socket, full_command, size_of_values_in_char+3, MSG_NOSIGNAL);
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
//! \param An array (allocated) of doubles of NbValuesToReceive elements
void baseSocket::GetReceivedValues(double val[])
{
    pthread_mutex_lock(&received_mutex);
    for(unsigned int i=0; i<NbValuesToReceive; i++)
        val[i]=ReceivedValues[i];
    pthread_mutex_unlock(&received_mutex);
    IsValues=false;
}

//! Thread function waiting for data from remote side
//! \param c : A pointer on the baseSocket object
//! \return NULL
void * receiving(void * c)
{
    baseSocket * local=(baseSocket*)c;

    short int msg_length=local->NbValuesToReceive*sizeof(double)+3;
    unsigned char rcvchars[msg_length];
    unsigned char toprocess[msg_length];
    unsigned char remainingtoprocess[msg_length];
    short int remainingtoprocess_n=0;

    while(local->Connected) {
        int ret=recv(local->Socket, rcvchars, msg_length, 0);
        if(ret==msg_length) {
            //If just recv mssg looks ok (first characters)
            if(rcvchars[0]==local->InitCode && rcvchars[1]==local->NbValuesToReceive) {
                //then use it as it is
                memcpy(toprocess, rcvchars, msg_length);
                //discard any remaining data
                remainingtoprocess_n=0;
            }
            else {
                //Use stored init part of message in buffer if any
                memcpy(toprocess, remainingtoprocess, remainingtoprocess_n);
                //concatenate with the begining of received sequence and use that
                memcpy(&toprocess[remainingtoprocess_n], rcvchars, msg_length-remainingtoprocess_n);

                /*printf("Pre:");
                for(int k=0; k<remainingtoprocess_n; k++)
                    printf("%02X", remainingtoprocess[k]);
                printf(" + ");
                for(int k=0; k<ret; k++)
                    printf("%02X", rcvchars[k]);
                printf(" = \nPre:");
                for(int k=0; k<msg_length; k++)
                    printf("%02X", toprocess[k]);
                printf(" (%d+%d=?%d)\n", remainingtoprocess_n, ret, msg_length);*/

                //Store end of message for later use in buffer
                unsigned int i;
                for(i=msg_length-1; i>=0; i--) {
                    remainingtoprocess_n=msg_length-i;
                    if(rcvchars[i]==local->InitCode)
                        break;
                }
                memcpy(remainingtoprocess, &rcvchars[i], remainingtoprocess_n);

                /*for(int k=0; k<remainingtoprocess_n; k++)
                    printf("%02X", remainingtoprocess[k]);
                printf(" = \n");
                for(int k=0; k<remainingtoprocess_n; k++)
                    printf("%02X", rcvchars[i+k]);
                printf(" (%d)\n\n", remainingtoprocess_n);*/
            }

            //Check init code and number of values are matching
            if(toprocess[0]==local->InitCode && toprocess[1]==local->NbValuesToReceive) {
                //Compute and check hash
                unsigned char command_hash = 0;
                unsigned int i;
                for(i=2; i<ret-1; i++) {
                    command_hash ^= toprocess[i];
                }

                if(command_hash==toprocess[i]) {
                    //Copy received values
                    pthread_mutex_lock(&local->received_mutex);
                    pthread_cleanup_push((void (*)(void *))pthread_mutex_unlock, (void *)&local->received_mutex); //Ensure that mutex will be unlock on thread cancelation (disconnect)
                    memcpy(local->ReceivedValues, &toprocess[2], local->NbValuesToReceive*sizeof(double));
                    pthread_mutex_unlock(&local->received_mutex);
                    pthread_cleanup_pop(1); //no unlock on cancellation required anymore at this point
                    local->IsValues=true;
                }
                else {
                    //Incorrect values
                    #ifdef VERBOSE
                    printf("FLNL::Error receiving (wrong hash).\n");
                    #endif
                }
            }
            else {
                //Incorrect values
                #ifdef VERBOSE
                printf("FLNL::Error receiving (wrong data format).\n");
                #endif
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
