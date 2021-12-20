#ifndef FLNLHELPER_H
#define FLNLHELPER_H

#include <Eigen/Dense>
#include <string>
#include <typeinfo>
#include <FLNL.h>

#include "Robot.h"
#include "RobotM3.h"
#include "RobotM2.h"

//Predefined hash of types for further comparison
const size_t doubleType = typeid(double).hash_code();
const size_t stdVectorType = typeid(std::vector<double>).hash_code();
const size_t EigenVectorType = typeid(Eigen::VectorXd).hash_code();


/**
* \brief Helper class to establish a network communication allowing to publish robot state and send/recive commands using libFLNL
*
*/
class FLNLHelper
{
    public:
        /**
        * \brief Default constructor: no state registration done (use registerState methods after)
        * \param ip: server (local) ip address to use
        * \param port: communication port
        */
        FLNLHelper(std::string ip, int port = 2048) {
            //Start server and wait for incoming connection
            FLNLServer.Connect(ip.c_str(), port);

            spdlog::info("Initialised default network communication server ({}:{}) ", ip, port);
        }

        /**
        * \brief Specialised constructor also registering joint states
        * \param ip: server (local) ip address to use
        * \param port: communication port
        */
        FLNLHelper(Robot &robot, std::string ip, int port = 2048) {
            //Start server and wait for incoming connection
            FLNLServer.Connect(ip.c_str(), port);

            initTime = std::chrono::steady_clock::now();

            registerState(runningTime);
            registerState(robot.getPosition());
            registerState(robot.getVelocity());
            registerState(robot.getTorque());

            spdlog::info("Initialised network communication server ({}:{}) for default robot (state size: {})", ip, port, stateValues.size());
        }

        /**
        * \brief Dedicated constructor for M3 registering end effector state instead of joint ones
        * \param ip: server (local) ip address to use
        * \param port: communication port
        */
        FLNLHelper(RobotM3 &robot, std::string ip, int port = 2048) {
            //Start server and wait for incoming connection
            FLNLServer.Connect(ip.c_str(), port);

            initTime = std::chrono::steady_clock::now();

            registerState(runningTime);
            registerState(robot.getEndEffPosition());
            registerState(robot.getEndEffVelocity());
            registerState(robot.getInteractionForce());

            spdlog::info("Initialised network communication server ({}:{}) for M3 robot (state size: {})", ip, port, stateValues.size());
        }

         /**
        * \brief Dedicated constructor for M2 registering end effector state instead of joint ones
        * \param ip: server (local) ip address to use
        * \param port: communication port
        */
        FLNLHelper(RobotM2 &robot, std::string ip, int port = 2048) {
            //Start server and wait for incoming connection
            FLNLServer.Connect(ip.c_str(), port);

            initTime = std::chrono::steady_clock::now();

            registerState(runningTime);
            registerState(robot.getEndEffPosition());
            registerState(robot.getEndEffVelocity());
            registerState(robot.getInteractionForce());

            spdlog::info("Initialised network communication server ({}:{}) for M2 robot (state size: {})", ip, port, stateValues.size());
        }

        /**
        * \brief Close existing connection
        */
        void closeConnection() {
            if(FLNLServer.IsConnected())
                FLNLServer.Disconnect();
        }

        /**
        * \brief Default destructor also closing connection
        */
        ~FLNLHelper() {
            if(FLNLServer.IsConnected())
                FLNLServer.Disconnect();
        }


        /**
        * \brief Register a double value within state to be send regularly
        * \return registered state size after addition
        */
        int registerState(const double &v) {
            stateValues.push_back(v); //resize and initialise value
            stateReferences.push_back((void*)&v); //store reference
            stateReferencesType.push_back(typeid(v).hash_code()); //store type
            return stateValues.size();
        }

        /**
        * \brief Register a vector of doubles within state to be send regularly
        * \return registered state size after addition
        */
        int registerState(const std::vector<double> &v) {
            for(unsigned int i=0; i<v.size(); i++)//resize and initialise value
                stateValues.push_back(v[i]);
            stateReferences.push_back((void*)&v); //store reference
            stateReferencesType.push_back(typeid(v).hash_code()); //store type
            return stateValues.size();
        }

        /**
        * \brief Register an Eigen vector of doubles within state to be send regularly. WARNING: cannot take a fixed size Vector (e.g. Vector3d)
        * \return registered state size after addition
        */
        int registerState(const Eigen::VectorXd &v) {
            for(int i=0; i<v.size(); i++)//resize and initialise value
                stateValues.push_back(v[i]);
            stateReferences.push_back((void*)&v); //store reference
            stateReferencesType.push_back(typeid(v).hash_code()); //store type
            return stateValues.size();
        }


        /**
        * \brief Send registerd state values
        */
        void sendState() {

            if(FLNLServer.IsConnected()) {
                //Update time
                runningTime = (std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now() - initTime).count()) / 1e6;

                //Prepare vector of values to send
                int k=0;
                for(unsigned int i=0; i<stateReferences.size(); i++) {
                    //Get current values (based on type)
                    if(stateReferencesType[i] == doubleType) {
                        stateValues[k]=*((double*)stateReferences[i]);
                        k++;
                    }
                    else if (stateReferencesType[i] == stdVectorType){
                        std::vector<double>* sv = static_cast<std::vector<double>*>(stateReferences[i]);
                        for(unsigned int j=0; j<sv->size(); j++) {
                            stateValues[k]=(*sv)[j];
                            k++;
                        }
                    }
                    else if (stateReferencesType[i] == EigenVectorType){
                        Eigen::VectorXd* ev = static_cast<Eigen::VectorXd*>(stateReferences[i]);
                        for(int j=0; j<ev->size(); j++) {
                            stateValues[k]=(*ev)[j];
                            k++;
                        }
                    }
                    else {
                        //Not supported type
                        return;
                    }
                }
                //Send it
                FLNLServer.Send(stateValues);
            }
        }

        /**
        * \brief Send a string command (4 characters) to client without parameters
        */
        void sendCmd(const std::string &cmd) {
            FLNLServer.Send(cmd);
        }

        /**
        * \brief Send a string command (4 characters) and double parameters
        */
        void sendCmd(const std::string &cmd, const std::vector<double> &v) {
            FLNLServer.Send(cmd, v);
        }

        /**
        * \brief Send a string command (4 characters) and double parameters
        */
        void sendCmd(const std::string &cmd, const Eigen::VectorXd &v) {
            std::vector<double> stdv(v.data(), v.data() + v.rows() * v.cols());
            FLNLServer.Send(cmd, stdv);
        }

        /**
        * \brief New command received?
        */
        bool isCmd() {
            return FLNLServer.IsReceivedCmd();
        }

        /**
        * \brief Return latest received command
        */
        void getCmd(std::string &cmd, std::vector<double> &v) {
            FLNLServer.GetReceivedCmd(cmd, v);
        }

        /**
        * \brief Clear received command flag (to call when command is succesfully consumed)
        */
        void clearCmd() {
            FLNLServer.ClearReceivedCmd();
        }

        /**
        * \brief Test a specific last cmd received, and if yes consumes it (clearCmd())
        * \param cmd Command to test
        * \param v Associated parameters values vector
        */
        bool isCmd(std::string cmd, std::vector<double> &v) {
            if(isCmd()) {
                std::string cmd_r;
                FLNLServer.GetReceivedCmd(cmd_r, v);
                if(cmd_r==cmd) {
                    clearCmd();
                    return true;
                }
            }
            return false;
        }

        /**
        * \brief Test a specific last cmd received, and if yes consumes it (clearCmd())
        * \param cmd Command to test
        */
        bool isCmd(std::string cmd) {
            if(isCmd()) {
                std::string cmd_r;
                std::vector<double> v;
                FLNLServer.GetReceivedCmd(cmd_r, v);
                if(cmd_r==cmd) {
                    clearCmd();
                    return true;
                }
            }
            return false;
        }

    private:
        server FLNLServer;

        std::vector<void*> stateReferences;             //!< Hodling references of values to send (double, vector<double> or eigen vector)
        std::vector<std::size_t> stateReferencesType;   //!< Holding hash_code of typeinfo for further retrieval of type
        std::vector<double> stateValues;                //!< All state double values to be sent
        std::chrono::steady_clock::time_point initTime;
        double runningTime = 0;                         //!< Time since initialisation in s
};

#endif //FLNLHELPER_H
