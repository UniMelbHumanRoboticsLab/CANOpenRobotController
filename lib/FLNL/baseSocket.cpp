/**
 * \file baseSocket.cpp
 * \brief Network base class implementation
 * \author Vincent Crocher
 * \version 1.1
 * \date November 2020
 *
 *
 */

#include "FLNL.h"

/*-----------------------------------------------------------------------------------------------------------------*/
/*#########################################   CONSTRUCTOR AND DESTRUCTOR   ########################################*/
/*-----------------------------------------------------------------------------------------------------------------*/
//! Constructor initializing data and creating a socket
baseSocket::baseSocket():
    MaxNbValues(floor((MESSAGE_SIZE-3-CMD_SIZE)/(float)sizeof(double))) {
    //Ensure standard size of double
    assert(sizeof(double) == EXPECTED_DOUBLE_SIZE);

    //Initialise sin
    sin.sin_family = AF_INET;

    //Initialise and allocates privates
    Socket=-1;
    IsValues=false;
    Connected=false;
    ReceivedValues=new double[MaxNbValues];
    ReceivedCmd=new char[CMD_SIZE+1];
    ReceivedCmdParams=new double[MaxNbValues];

    pthread_mutex_init(&ReceivedMutex, NULL);
    pthread_mutex_init(&ReceivedCmdMutex, NULL);

#ifdef WINDOWS
    WSADATA WSAData;
    WSAStartup(MAKEWORD(2,0), &WSAData);
#endif
}

//! Destructor releasing memory and closing the socket
baseSocket::~baseSocket() {
    if(Connected)
        Disconnect();
    pthread_mutex_destroy(&ReceivedMutex);
    pthread_mutex_destroy(&ReceivedCmdMutex);
    delete[] ReceivedValues;
    delete[] ReceivedCmd;
    delete[] ReceivedCmdParams;
}
/*#################################################################################################################*/




/*-----------------------------------------------------------------------------------------------------------------*/
/*###################################   CONNECTING AND DISCONNECTING METHODS    ###################################*/
/*-----------------------------------------------------------------------------------------------------------------*/
//! Disconnect and close the socket
//! \return the close(Socket) return value
int baseSocket::Disconnect() {
    pthread_cancel(ReceivingThread);
    Connected=false;

    int ret=0;
    if(Socket>0) {
        ret=close(Socket);
        if(ret==0) {
            Socket = -1;
            printf("FLNL::Disconnected.\n");
    #ifdef WINDOWS
            WSACleanup();
    #endif
        } else {
            printf("FLNL::Error closing connection.\n");
        }
    }
    return ret;
}

//! Tell if connected
//! \return TRUE if connected to the server, FALSE otherwise
bool baseSocket::IsConnected() {
    return Connected;
}
/*#################################################################################################################*/


/*-----------------------------------------------------------------------------------------------------------------*/
/*###################################################   HELPERS   #################################################*/
/*-----------------------------------------------------------------------------------------------------------------*/
//! Compute and return the checksum for a message
unsigned char Checksum(unsigned char *full_message)
{
    unsigned char command_hash = 0;
    for(unsigned int i=2; i<MESSAGE_SIZE-1; i++) {
        command_hash ^= full_message[i];
    }
    return command_hash;
}
/*#################################################################################################################*/


/*-----------------------------------------------------------------------------------------------------------------*/
/*###############################################   SENDING METHODS   #############################################*/
/*-----------------------------------------------------------------------------------------------------------------*/
//! Send a vector of double values accross
//! \param values : A vector of doubles to send. Should be less values than MaxNbValues.
//! \return -1 if wrong values size, the send() return value otherwise
int baseSocket::Send(const std::vector<double> &values) {
    if(values.size()>MaxNbValues)
        return -1;

    //Init codes and checksum
    unsigned int idx=0;
    FullMessageOut[idx++]='V';
    FullMessageOut[idx++]=values.size();
    //Add values
    for(unsigned int i=0; i<values.size(); i++) {
        memcpy(FullMessageOut+idx, &values[i], sizeof(double));
        idx+=sizeof(double);
    }
    //Pad with 0s
    for(unsigned int i=idx; i<MESSAGE_SIZE; i++) {
        FullMessageOut[i]=0;
    }

    //Compute and add hash
    FullMessageOut[MESSAGE_SIZE-1]=Checksum(FullMessageOut);

    //Send the char array (MSG_NOSIGNAL required to avoid SIGPIPE signal which will break server on lost connection)
    #ifdef WINDOWS
        int signals=0;
    #else
        int signals=MSG_NOSIGNAL;
    #endif
    return send(Socket, (char *)FullMessageOut, MESSAGE_SIZE, signals);
}


//! Send a command with parameters
//! \param cmd : The command string to send (4 characters)
//! \param param: A vector of doubles containing the command parameters to send (optional)
//! \return -1 if wrong params size, the send() return value otherwise
int baseSocket::Send(const std::string &cmd, const std::vector<double> &params) {
    if(params.size()>MaxNbValues)
        return -1;

    //Init codes
    unsigned int idx=0;
    FullMessageOut[idx++]='C';
    FullMessageOut[idx++]=params.size();
    //Copy cmd (pad if less than CMD_SIZE)
    if(cmd.size()<CMD_SIZE) {
        memcpy(FullMessageOut+idx, cmd.c_str(), cmd.size());
        for(unsigned int i=cmd.size(); i<CMD_SIZE; i++)
            FullMessageOut[idx+i] = '\0';
    }
    else {
       memcpy(FullMessageOut+idx, cmd.c_str(), CMD_SIZE);
    }
    idx+=CMD_SIZE;
    //Add parameters
    for(unsigned int i=0; i<params.size(); i++) {
        memcpy(FullMessageOut+idx, &params[i], sizeof(double));
        idx+=sizeof(double);
    }
    //Pad with 0s
    for(unsigned int i=idx; i<MESSAGE_SIZE; i++) {
        FullMessageOut[i]=0;
    }

    //Compute and add hash
    FullMessageOut[MESSAGE_SIZE-1]=Checksum(FullMessageOut);

    //Send the char array (MSG_NOSIGNAL required to avoid SIGPIPE signal which will break server on lost connection)
    #ifdef WINDOWS
        int signals=0;
    #else
        int signals=MSG_NOSIGNAL;
    #endif
    return send(Socket, (char *)FullMessageOut, MESSAGE_SIZE, signals);
}
/*#################################################################################################################*/




/*-----------------------------------------------------------------------------------------------------------------*/
/*############################################  RECEIVING METHODS  #################################################*/
/*-----------------------------------------------------------------------------------------------------------------*/
//! Tell if values have been received since last GetReceivedValues()
//! \return TRUE if values have been received, FALSE otherwise
bool baseSocket::IsReceivedValues() {
    return IsValues;
}

//! Return the last received values
//! \param A reference to a vector to store values
//! \return The number of values received (size of vals)
int baseSocket::GetReceivedValues(std::vector<double> &vals) {
    pthread_mutex_lock(&ReceivedMutex);
    {
        if(vals.size()!=NbReceivedValues)
            vals.resize(NbReceivedValues);

        for(unsigned int i=0; i<NbReceivedValues; i++)
            vals[i]=ReceivedValues[i];
    }
    pthread_mutex_unlock(&ReceivedMutex);

    IsValues=false;

    return vals.size();
}

//! Tell if a new command has been received since last GetReceivedCmd()
//! \return TRUE if command has been received, FALSE otherwise
bool baseSocket::IsReceivedCmd() {
    return IsCmd;
}

//! Return the last received cmd
//! \param A string reference to be receive the command string (4 characters)
//! \param A reference to a vector to store command parameters
//! \return The number of parameters received (size of params)
int baseSocket::GetReceivedCmd(std::string &cmd, std::vector<double> &params) {
    pthread_mutex_lock(&ReceivedCmdMutex);
    {
        if(params.size()!=NbReceivedCmdParams)
            params.resize(NbReceivedCmdParams);

        for(unsigned int i=0; i<NbReceivedCmdParams; i++)
            params[i]=ReceivedCmdParams[i];

        cmd.assign(ReceivedCmd);
    }
    pthread_mutex_unlock(&ReceivedCmdMutex);

    IsCmd=false;

    return params.size();
}

void unlock_mutex(void * m) {
    pthread_mutex_t * mut = (pthread_mutex_t *)m;
    pthread_mutex_unlock(mut);
}

//! Thread function waiting for data from remote side
//! \param c : A pointer on the baseSocket object
//! \return NULL
void * receiving(void * c) {
    baseSocket * local=(baseSocket*)c;

    while(local->Connected) {
        //Read a full frame
        int ret=recv(local->Socket, (char *)local->FullMessageIn, MESSAGE_SIZE, 0);
        if(ret==MESSAGE_SIZE) {
            bool cksum_ok = false;
            if(Checksum(local->FullMessageIn)==local->FullMessageIn[MESSAGE_SIZE-1])
                cksum_ok = true;

            //Is it a CMD?
            if(local->FullMessageIn[0]==local->InitCommandCode && cksum_ok) {
                //then use it as a command
                unsigned short int nb_params = local->FullMessageIn[1];
                if(nb_params<=local->MaxNbValues) {
                    pthread_mutex_lock(&local->ReceivedCmdMutex);
                    pthread_cleanup_push(unlock_mutex, (void *)&local->ReceivedCmdMutex); //Ensure that mutex will be unlock on thread cancelation (disconnect)
                    {
                        //Process
                        local->NbReceivedCmdParams = nb_params;
                        memcpy(local->ReceivedCmd, &local->FullMessageIn[2], CMD_SIZE);
                        local->ReceivedCmd[CMD_SIZE]='\0';
                        //parameters
                        memcpy(local->ReceivedCmdParams, &local->FullMessageIn[2+CMD_SIZE], nb_params*sizeof(double));
                    }
                    pthread_cleanup_pop(1); //unlock mutex
                    local->IsCmd=true;
                }
                #ifdef VERBOSE
                else {
                    //Incorrect values
                    printf("FLNL::Error receiving (wrong number of params).\n");
                }
                #endif
            }
            //Is it values?
            else if(local->FullMessageIn[0]==local->InitValueCode && cksum_ok) {
                unsigned short int nb_values = local->FullMessageIn[1];
                if(nb_values<=local->MaxNbValues) {
                    pthread_mutex_lock(&local->ReceivedMutex);
                    pthread_cleanup_push(unlock_mutex, (void *)&local->ReceivedMutex); //Ensure that mutex will be unlock on thread cancelation (disconnect)
                    {
                        local->NbReceivedValues = nb_values;
                        memcpy(local->ReceivedValues, &local->FullMessageIn[2], nb_values*sizeof(double));
                    }
                    pthread_cleanup_pop(1); //unlock mutex
                    local->IsValues=true;
                }
                #ifdef VERBOSE
                else {
                    //Incorrect values
                    printf("FLNL::Error receiving (wrong number of values).\n");
                }
                #endif
            }
        }
        else if(ret<0) {
            #ifdef VERBOSE
            #ifdef WINDOWS
            printf("WSA Error code : %d\n", WSAGetLastError());
            #endif
            perror("FLNL::Error receiving");
            #endif
            if(errno==EBADF) { //Connection has been closed by client
                local->Disconnect();
            }
        }
        else if(ret==0) {
            //Connection has been reseted
            local->Disconnect();
        }
    }

    return NULL;
}
/*#################################################################################################################*/
