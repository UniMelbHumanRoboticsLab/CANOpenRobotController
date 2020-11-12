/**
 * \file Robot.h
 * \author William Campbell, Justin Fong, Vincent Crocher
 * \brief  The <code>Robot</code> class is a abstract class which is a software representation of a Robot
 * with a flexible representation in terms of number of joints, number of sensors and type of I/O
 * the real world or virtual robot has. The class specificall represents a robot with an underlying
 * bus network connecting components to a master node, being the robots computer or processor.
 * Implementations have been designed <code>ExoRobot<code> under CANOpen protocol, however others
 * may be implemented by future developers.
 *
 * \version 0.2
 * \date 2020-10-14
 * \copyright Copyright (c) 2020
 */
/**
 *  @defgroup Robot Robot Module
 *  A group of abstract classes, acting as the software representation of a robot.
 */
#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED
#include <vector>
#include <Eigen/Dense>

#include "InputDevice.h"
#include "Joint.h"

/**
 * @ingroup Robot
 * \brief Abstract Class representing a robot. Includes vectors of Joint and InputDevice.
 *
 */
class Robot {
   protected:
    /**
    * \brief Vector of pointers to Abstract <class>Joint<class> Objects, number and type must be specified by
    * Software design in <class>Robot<class> Implementation.
    * Note: Use pointers to the joint objects here, so that the derived objects are not cast to Joint, truncating
    * any of their explicit implementations.
    *
    */
    std::vector<Joint *> joints;
    std::vector<InputDevice *> inputs;

    Eigen::VectorXd jointPositions_;
    Eigen::VectorXd jointVelocities_;
    Eigen::VectorXd jointTorques_;

   public:
    /** @name Constructors and Destructors */
    //@{
    /**
    * \brief Default <code>Robot</code> constructor.
    */
    Robot();
    virtual ~Robot();
    //@}

    /** @name Initialisation Methods */
    //@{
    /**
     * \brief Initialize memory for the designed <code>Robot</code> classes specific
     * <code>Joint</code> objects + sensors (if available) using the pure virtual initialiseJoints()
     * implemented by the robot designer. Based on the given Joints, initNetwork() will configure
     * these joints for CAN PDO messaging and Load the specififed Controller, by default set to Positio.
     *
     * \return true if successful
     * \return false if unsuccessful
     */
    bool initialise();

    /**
     * \brief Stop the robot: disable all actuated joints.
     *
     * \return true if successful
     * \return false if unsuccessful
     */
    virtual bool disable();

    /**
     * \brief Pure Virtual function, implemeted by robot designer with specified number of each concrete joint classes
     * for the robot hardware desired.
     *
     */
    virtual bool initialiseJoints() = 0;
    /**
     * \brief Pure Virtual function, implemeted by robot designer with specified number of each concrete input classes
     * for the robot hardware desired.
     *
     * \return true if successful
     * \return flase if unsuccesful
     *
     */
    virtual bool initialiseInputs() = 0;
    /**
     * \brief For each <class>Joint</class> in the robots joints Vector.
     * Individually set up the underlying CANopen PDO messaging to and from
     * the hardware attached.
     *
     * \return true if successful
     * \return false if unsuccessful
     */
    virtual bool initialiseNetwork() = 0;
    //@}

    /** @name Core Update and State Methods */
    //@{
    /**
    * \brief Update all of this <code>Robot<code> software joint positions
    * from object dictionary entries.
    *
    */
    virtual void updateRobot();

    /**
    * \brief Get the latest joints position
    *
    * \return Eigen::VectorXd a reference to the vector of actual joint positions
    */
    Eigen::VectorXd& getPosition();

    /**
    * \brief Get the latest joints velocity
    *
    * \return Eigen::VectorXd a reference to the vector of actual joint positions
    */
    Eigen::VectorXd& getVelocity();

    /**
    * \brief Get the latest joints torque
    *
    * \return Eigen::VectorXd a reference to the vector of actual joint positions
    */
    Eigen::VectorXd& getTorque();

    /**
    * \brief print out status of robot and all of its joints
    *
    */
    void printStatus();
    /**
    * \brief print out status of <code>Joint<code> J_i
    *
    */
    void printJointStatus(int J_i);
    //@}


    /** @name Core Control Methods */
    //@{
    /**
    * @brief Initialises position control on this robot. Default function is to report failure
    *
    * @return true If successful
    * @return false If unsuccesful
    */
    virtual bool initPositionControl() { return false; };

    /**
    * @brief Initialises position control on this robot. Default function is to report failure
    *
    * @return true If successful
    * @return false If unsuccesful
    */
    virtual bool initVelocityControl() { return false; };

    /**
    * @brief Initialises position control on this robot. Default function is to report failure
    *
    * @return true If successful
    * @return false If unsuccesful
    */
    virtual bool initTorqueControl() { return false; };

    /**
    * @brief Set the target positions for each of the joints
    *
    * @param positions a vector of target positions - applicable for each of the actauted joints
    * @return MovementCode representing success or failure of the application
    */
    virtual setMovementReturnCode_t setPosition(std::vector<double> positions) { return INCORRECT_MODE; };

    /**
    * @brief Set the target velocities for each of the joints
    *
    * @param positions a vector of target velocities - applicable for each of the actauted joints
    * @return MovementCode representing success or failure of the application
    */
    virtual setMovementReturnCode_t setVelocity(std::vector<double> velocities) { return INCORRECT_MODE; };

    /**
    * @brief Set the target torques for each of the joints
    *
    * @param positions a vector of target torques - applicable for each of the actauted joints
    * @return MovementCode representing success or failure of the application
    */
    virtual setMovementReturnCode_t setTorque(std::vector<double> torques) { return INCORRECT_MODE; };
    //@}



    /** @name Logging methods */
    //@{
    /**
     * /todo The default logging function has not yet been implemented.
     *
     */

    /**
    * \brief Initialises Logging to specified file
    *
    */
    void initialiseLog();
    /**
    * \brief Log input data point to currently open log file
    *
    */
    void logDataPoint(std::string data);
    /**
    * \brief Save and close any currently open logging files
    *
    */
    bool closeLog();
    //@}
};

#endif  //ROBOT_H
