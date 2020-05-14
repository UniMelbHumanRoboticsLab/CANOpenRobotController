/**
 * \file Robot.h
 * \author William Campbell
 * \brief  The <code>Robot</code> class is a abstract class which is a software representation of a Robot
 * with a flexible representation in terms of number of joints, number of sensors and type of I/O 
 * the real world or virtual robot has. The class specificall represents a robot with an underlying
 * bus network connecting components to a master node, being the robots computer or processor.
 * Implementations have been designed <code>ExoRobot<code> under CANOpen protocol, however others
 * may be implemented by future developers.
 * 
 * \version 0.1
 * \date 2020-04-09
 * \version 0.1
 * \copyright Copyright (c) 2020
 */
/**
 *  @defgroup Robot Robot Module
 *  A group of abstract classes, acting as the software representation of a robot.
 */
#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED
#include <vector>

#include "InputDevice.h"
#include "Joint.h"
#include "TrajectoryGenerator.h"
using namespace std;

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
    vector<Joint *> joints;

    vector<InputDevice *> inputs;
    /**
 * \brief Trajectory Generator  
 * 
 */
    TrajectoryGenerator *trajectoryGenerator;

   public:
    //Setup
    /**
 * \brief Default <code>Robot</code> constructor.
 */
    Robot(TrajectoryGenerator *tj);
    ~Robot();
    /**
     * \brief Initialize memory for the designed <code>Robot</code> classes specific
     * <code>Joint</code> objects + sensors (if available) using the pure virtual initialiseJoints()
     * implemented by the robot designer. Based on the given Joints, initNetwork() will configure 
     * these joints for CAN PDO messaging and Load the specififed Controller, by default set to Positio.
     * 
     * 
     * \return true 
     * \return false 
     */
    bool initialise();
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
     */
    virtual bool initialiseInputs() = 0;
    /**
     * \brief For each <class>Joint</class> in the robots joints Vector.
     * Individually set up the underlying CANopen PDO messaging to and from 
     * the hardware attached.
     * 
     * 
     * \return true 
     * \return false 
     */
    virtual bool initialiseNetwork() = 0;

    //Robot objects

    //Core  functions
    /**
    * \brief Update all of this <code>Robot<code> software joint positions 
    * from object dictionary entries.
    * 
    */
    virtual void updateRobot();
    /**
 * \brief print out status of robot and all of its joints
 * 
 */
    void printStatus();
    /**
 * \brief print out status of <code>Joint<code> J_i
 * 
 */
    void getJointStatus(int J_i);

    ////Logging
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
};

#endif  //ROBOT_H
