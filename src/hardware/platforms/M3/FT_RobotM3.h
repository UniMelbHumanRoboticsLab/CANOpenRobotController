/**
 * \file FT_RobotM3.h
 * \author JQ
 * \version 0.3
 * \date 2026-08-5
 * \copyright Copyright (c) 2020 - 2026
 *
 * \brief  The FT_RobotM3 class represents an M3 Robot derived class with a Force Torque Sensor.
 *
 */

#ifndef FT_RobotM3_H_INCLUDED
#define FT_RobotM3_H_INCLUDED

#include "RobotM3.h"
#include "RobotousRFT.h"


typedef Eigen::Vector3d VM3; //!< Convenience alias for double  Vector of length 3
typedef Eigen::VectorXd VX; //!< Generic (dynamic) size version required for compatibility w/ other libraries (FLNL)

class FT_RobotM3 : public RobotM3 {
   private:
    int num_FT = 1;
    // -- Variables assocaited with standalone sensors -- //
    std::vector<RobotousRFT *> FT_Sensors;
    Eigen::VectorXd wrenches;  //6xN Vector containing raw FT readings
    Eigen::VectorXd correctedWrenches;  //6xN Vector containing corrected FT readings
    Eigen::VectorXd olStatWrenches;  //6xN Vector containing all FT status
    bool sensorsOn = false;
    bool print_states = false;
    
   public:
    /**
      * \brief Default FT_RobotM3 constructor.
      * Creates joints and inputs.
      * \param yaml_config_file the name of a valide YAML file describing kinematic and dynamic parameters of the M3. If absent or incomplete default parameters are used instead.
      */
    FT_RobotM3(std::string robot_name="", std::string yaml_config_file="");
    ~FT_RobotM3();
    /**
       * \brief update current state of the robot, including input and output devices.
       * Overloaded Method from the RobotM3 Class.
       */
    void updateRobot();
    /**
     * @brief Corrects local copy of forces
     */
    void correctWrenches();

    /**
     * @brief Corrects local copy of forces, and returns them
     * @brief get the overload status of the sensors
     * @return Eigen::VectorXd& a 6xN (N is number of crutches) of crutch sensor readings
     * @return Eigen::VectorXd& a N (N is number of crutches) of overload readings
     */
    Eigen::VectorXd &getWrenches();
    Eigen::VectorXd &getOlStatWrenches();

    /**
     * @brief prints the states of the robot
     *
     */
    void printStatus();
    void printWrenches();

    /**
     * @Configure FT sensors
     *
     */
    void setWrenchesOffset(Eigen::VectorXd offsets);
    bool startFT_Sensors();
    bool stopFT_Sensors();
    bool setFT_SensorsFilter();
    bool configureMasterPDOs();
};
#endif /*FT_RobotM3_H*/