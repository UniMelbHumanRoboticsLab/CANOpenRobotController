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
    Eigen::VectorXd FT_readings;  //6xN Vector containing all FT readings
    Eigen::VectorXd FT_olStats;  //6xN Vector containing all FT status
    bool sensorsOn = false;
    
   public:
    /**
      * \brief Default FT_RobotM3 constructor.
      * Creates joints and inputs.
      * \param yaml_config_file the name of a valide YAML file describing kinematic and dynamic parameters of the M3. If absent or incomplete default parameters are used instead.
      */
    FT_RobotM3(std::string robot_name="", std::string yaml_config_file="");
    ~FT_RobotM3();
    
    void printStatus();
    
    /**
     * @brief Updates local copy of forces, and returns them
     *
     * @return Eigen::VectorXd& a 6xN (N is number of crutches) of crutch sensor readings
     */
    Eigen::VectorXd &getFT_readings();

    /**
     * @brief Takes the forces from the FT sensors and updates a local copy
     *
     */
    void updateFT_readings();

    /**
     * @brief prints the forces from the FT sensors
     *
     */
    void printFT_readings(Eigen::VectorXd readings);

    /**
     * @brief Updates and corrects the local copy of forces, and returns them
     *
     * @return Eigen::VectorXd& a 6xN (N is number of crutches) of crutch sensor readings
     */
    Eigen::VectorXd &getCorrectedFT_readings();
    Eigen::VectorXd &getFT_OLStatus();

    void setFTOffsets(Eigen::VectorXd offsets);
    bool startFT_Sensors();
    bool stopFT_Sensors();
    bool setFT_SensorsFilter();
    bool configureMasterPDOs();
};
#endif /*FT_RobotM3_H*/