#include <Eigen/Dense>
#include <string>
#include <typeinfo>
#include <FLNL/FLNL.h>

#include "Robot.h"
#include "RobotM3.h"

//Predefined hash of types for further comparison
const size_t doubleType = typeid(double).hash_code();
const size_t stdVectorType = typeid(std::vector<double>).hash_code();
const size_t EigenVectorType = typeid(Eigen::VectorXd).hash_code(); //TODO: check if VectorXd and Vector3d have same type and/or are perceived as VectorXd in register method

//! Helper class to establish a network communication allowing to publish robot state and send/recive commands using libFLNL
class FLNLHelper
{
	public:
		//! Default constructor: no state registration done (use registerState methods after)
		//! \param ip: server (local) ip address to use
		//! \param port: communication port
		FLNLHelper(std::string ip, int port = 2048) {
			//Start server and wait for incoming connection
			FLNLServer.Connect(ip.c_str(), port);
			
			spdlog::info("Initialised default network communication server ({}) ", ip);
		}
		
		//! Specialised constructor also registering joint states
		//! \param ip: server (local) ip address to use
		//! \param port: communication port
		FLNLHelper(Robot * robot, std::string ip, int port = 2048) {
			//Start server and wait for incoming connection
			FLNLServer.Connect(ip.c_str(), port);
			
			registerState(robot->getPosition());
			registerState(robot->getVelocity());
			registerState(robot->getTorque());
			
			spdlog::info("Initialised network communication server ({}) for default robot (state size: {})", ip, stateValues.size());
		}
		
		//! Dedicated constructor for M3 registering end effector state instead of joint ones
		//! \param ip: server (local) ip address to use
		//! \param port: communication port
		FLNLHelper(RobotM3 * robot, std::string ip, int port = 2048) {
			//Start server and wait for incoming connection
			FLNLServer.Connect(ip.c_str(), port);
			
			registerState(robot->getPosition());
			registerState(robot->getVelocity());
			registerState(robot->getTorque());
			
			spdlog::info("Initialised network communication server ({}) for M3 robot (state size: {})", ip, stateValues.size());
		}
		
		
		~FLNLHelper(){
			FLNLServer.Disconnect();
		}
		
		
		//! \return registered state size after addition
		int registerState(const double &v) {
			stateValues.push_back(v); //resize and initialise value
			stateReferences.push_back((void*)&v); //store reference
			stateReferencesType.push_back(typeid(v).hash_code()); //store type
			return stateValues.size();
		}
		
		//! \return registered state size after addition
		int registerState(const std::vector<double> &v) {
			std::cout << typeid(v).name() << std::endl;
			for(unsigned int i=0; i< v.size(); i++)//resize and initialise value
				stateValues.push_back(v[i]);
			stateReferences.push_back((void*)&v); //store reference
			stateReferencesType.push_back(typeid(v).hash_code()); //store type
			return stateValues.size();
		}
		
		//! \return registered state size after addition
		int registerState(const Eigen::VectorXd &v) {
			for(int i=0; i< v.size(); i++)//resize and initialise value
				stateValues.push_back(v[i]);
			stateReferences.push_back((void*)&v); //store reference
			stateReferencesType.push_back(typeid(v).hash_code()); //store type 
			return stateValues.size();
		}
		
		
		//! Send registerd state values
		void sendState() {
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
		
		void sendCmd(const std::string &cmd) {
			FLNLServer.Send(cmd);
		}
		
		void sendCmd(const std::string &cmd, const std::vector<double> &v) {
			FLNLServer.Send(cmd, v);
		}
		
		void sendCmd(const std::string &cmd, const Eigen::VectorXd &v) {
			std::vector<double> stdv(v.data(), v.data() + v.rows() * v.cols());
			FLNLServer.Send(cmd, stdv);
		}

		//! New command received?
		bool isCmd() {
			return FLNLServer.IsReceivedCmd();
		}
		
		//! Return latest received command
		void getCmd(std::string &cmd, std::vector<double> &v) {
			FLNLServer.GetReceivedCmd(cmd, v);
		}

		/*//! Return latest received command
		void getCmd(std::string &cmd, Eigen::VectorXd &v) {
		}*/


	private:
		server FLNLServer;

		std::vector<void*> stateReferences;				//!< Hodling references of values to send (double, vector<double> or eigen vector)
		std::vector<std::size_t> stateReferencesType; 	//!< Holding hash_code of typeinfo for further retrieval of type
		std::vector<double> stateValues;				//!< All state double values to be sent
	
};



