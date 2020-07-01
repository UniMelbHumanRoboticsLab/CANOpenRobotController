#include "RobotM3.h"

#include "DebugMacro.h"

using namespace Eigen;

short int sign(double val) {return (val>0)?1:((val<0)?-1:0); }

RobotM3::RobotM3() : Robot(), calibrated(false) {

    //Define the robot structure: each joint with limits and drive: should be in constructor
    double max_speed=360*M_PI/180.;
    double tau_max=1.9*23;
    joints.push_back(new JointM3(0, -45*M_PI/180., 45*M_PI/180., 1, -max_speed, max_speed, -tau_max, tau_max));
    joints.push_back(new JointM3(1, -15*M_PI/180., 70*M_PI/180., 1, -max_speed, max_speed, -tau_max, tau_max));//Todo
    joints.push_back(new JointM3(2, 0*M_PI/180., 95*M_PI/180., -1, -max_speed, max_speed, -tau_max, tau_max));//Todo
}

RobotM3::~RobotM3() {
    DEBUG_OUT("Delete RobotM3 object begins")
    for (auto p : joints) {
        DEBUG_OUT("Delete Joint ID: " << p->getId())
        delete p;
    }
    joints.clear();
    for (auto p : inputs) {
        DEBUG_OUT("Deleting Input")
        delete p;
    }
    DEBUG_OUT("RobotM3 deleted")
}


bool RobotM3::initialiseJoints() {

    return true;
}
bool RobotM3::initialiseNetwork() {
    DEBUG_OUT("RobotM3::initialiseNetwork()");

    bool status;
    for (auto joint : joints) {
        status = joint->initNetwork();
        if (!status)
            return false;
    }
    //Give time to drives PDO initialisation
    DEBUG_OUT("...");
    for(int i=0; i<5; i++) {
        DEBUG_OUT(".");
        usleep(10000);
    }

    return true;
}
bool RobotM3::initialiseInputs() {
    //inputs.push_back(&keyboard);
    return true;
}


bool RobotM3::stop() {
    std::cout << "Stopping M3 robot..." << std::endl;
    for (auto p : joints) {
        ((JointM3 *)p)->disable();
    }
    return true;
}

void RobotM3::applyCalibration() {
    for (int i=0; i<joints.size(); i++) {
        ((JointM3*)joints[i])->setPositionOffset(qCalibration[i]);
    }
    calibrated = true;
}

void RobotM3::updateRobot() {
    Robot::updateRobot();
}


void RobotM3::printStatus() {
    std::cout << std::setprecision(3) << std::fixed;
    std::cout <<"X=[ " << getEndEffPos().transpose() << " ]\t";
    std::cout <<"dX=[ " << getEndEffVel().transpose() << " ]\t";
    std::cout <<"F=[ " << getEndEffFor().transpose() << " ]\t";
    std::cout << std::endl;
}


void RobotM3::printJointStatus() {
    std::cout << std::setprecision(1)<< std::fixed;
    std::cout <<"q=[ " << getJointPos().transpose()*180/M_PI << " ]\t";
    std::cout <<"dq=[ " << getJointVel().transpose()*180/M_PI << " ]\t";
    std::cout <<"tau=[ " << getJointTor().transpose() << " ]\t";
    std::cout << "{";
    for (auto joint: joints)
        std::cout << "0x" << std::hex << ((JointM3*)joint)->getDriveStatus() << "; ";
    std::cout << "}" << std::endl;
}

bool RobotM3::initPositionControl() {
    DEBUG_OUT("Initialising Position Control on all joints ")
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM3 *)p)->setMode(POSITION_CONTROL, posControlMotorProfile) != POSITION_CONTROL) {
            // Something bad happened if were are here
            DEBUG_OUT("Something bad happened")
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM3 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM3 *)p)->enable();
    }
    return returnValue;
}
bool RobotM3::initVelocityControl() {
    DEBUG_OUT("Initialising Velocity Control on all joints ")
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM3 *)p)->setMode(VELOCITY_CONTROL, posControlMotorProfile) != VELOCITY_CONTROL) {
            // Something bad happened if were are here
            DEBUG_OUT("Something bad happened")
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM3 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM3 *)p)->enable();
    }
    return returnValue;
}
bool RobotM3::initTorqueControl() {
    DEBUG_OUT("Initialising Torque Control on all joints ")
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM3 *)p)->setMode(TORQUE_CONTROL) != TORQUE_CONTROL) {
            // Something bad happened if were are here
            DEBUG_OUT("Something bad happened")
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM3 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM3 *)p)->enable();
    }
    return returnValue;
}

setMovementReturnCode_t RobotM3::applyPosition(std::vector<double> positions) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    if(!calibrated) {
        returnValue = NOT_CALIBRATED;
    } else {
        for (auto p : joints) {
            setMovementReturnCode_t setPosCode = ((JointM3 *)p)->setPosition(positions[i]);
            if (setPosCode == INCORRECT_MODE) {
                std::cout << "Joint " << p->getId() << ": is not in Position Control " << std::endl;
                returnValue = INCORRECT_MODE;
            } else if (setPosCode != SUCCESS) {
                // Something bad happened
                std::cout << "Joint " << p->getId() << ": Unknown Error " << std::endl;
                returnValue = UNKNOWN_ERROR;
            }
            i++;
        }
    }
    return returnValue;
}

setMovementReturnCode_t RobotM3::applyVelocity(std::vector<double> velocities) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setVelCode = ((JointM3 *)p)->setVelocity(velocities[i]);
        if (setVelCode == INCORRECT_MODE) {
            std::cout << "Joint " << p->getId() << ": is not in Velocity Control " << std::endl;
            returnValue = INCORRECT_MODE;
        } else if (setVelCode != SUCCESS) {
            // Something bad happened
            std::cout << "Joint " << p->getId() << ": Unknown Error " << std::endl;
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

setMovementReturnCode_t RobotM3::applyTorque(std::vector<double> torques) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setTorCode = ((JointM3 *)p)->setTorque(torques[i]);
        if (setTorCode == INCORRECT_MODE) {
            std::cout << "Joint " << p->getId() << ": is not in Torque Control " << std::endl;
            returnValue = INCORRECT_MODE;
        } else if (setTorCode != SUCCESS) {
            // Something bad happened
            std::cout << "Joint " << p->getId() << ": Unknown Error " << std::endl;
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

Vector3d RobotM3::directKinematic(Vector3d q) {
    Vector3d X;

	float *L = LinkLengths;

	double F1 = (L[2]*sin(q[1]) + L[4]*cos(q[2]) + L[0]);

	X[0] = -F1 * cos(q[0]);
	X[1] = -F1 * sin(q[0]);
	X[2] = L[2]*cos(q[1]) - L[4]*sin(q[2]);

    return X;
}

Vector3d RobotM3::inverseKinematic(Vector3d X) {
    Vector3d q;

    float *L = LinkLengths;

    //Check accessible workspace
    double normX=sqrt(X[0]*X[0]+X[1]*X[1]+X[2]*X[2]);
    if( (L[4]<L[2] && normX<L[2]-L[4]) || (L[4]>L[2] && normX<sqrt(L[4]*L[4]-L[2]*L[2])) || normX>(L[2]+L[4]+L[0]) || X[0]>0 )
    {
        DEBUG_OUT("RobotM3::inverseKinematic() error: Point not accessible. NaN returned.");
        q[0]=q[1]=q[3]=nan("");
        return q;
    }

    //Compute first joint
    q[0]=-atan2(X[1], -X[0]);

    //Project onto parallel mechanism plane
    Vector3d tmpX;
    if(X[0]>0)
    {
        //should never happen as outside of workspace...
        tmpX[0]=sqrt(X[0]*X[0]+X[1]*X[1]);
    }
    else
    {
        tmpX[0]=-sqrt(X[0]*X[0]+X[1]*X[1]);
    }
    //Remove offset along -x
    tmpX[0]=tmpX[0]+L[0];
    tmpX[1]=X[1];
    tmpX[2]=X[2];

    //Calculate joints 2 and 3
    double beta=acos( (L[2]*L[2]+L[4]*L[4] - tmpX[0]*tmpX[0] - tmpX[2]*tmpX[2]) / (2.*(L[2]*L[4])) );
    q[1]=acos( L[4] * sin(beta)/sqrt(tmpX[0]*tmpX[0]+tmpX[2]*tmpX[2]) ) - atan2(tmpX[2],-tmpX[0]);
    q[2]=M_PI/2.+q[1]-beta;

    return q;
}

Matrix3d RobotM3::J() {
    Matrix3d J;
    Vector3d q;
    for(unsigned int i=0; i<3; i++) {
        q(i) = ((JointM3 *)joints[i])->getPosition();
    }

	float *L = LinkLengths;

	//Pre calculate factors for optimisation
	double F1 = (L[1] + L[3])*sin(q[2]);
	double F2 = - L[2]*cos(q[1]);
	double F3 = (L[1] + L[3])*cos(q[2]) + L[2]*sin(q[1]) + L[0];


	//Jacobian matrix elements
	J(0,0) = F3*sin(q[0]);
	J(0,1) = F2*cos(q[0]);
 	J(0,2) = F1*cos(q[0]);

	J(1,0) = - F3*cos(q[0]);
 	J(1,1) =   F2*sin(q[0]);
 	J(1,2) =   F1*sin(q[0]);

	J(2,0) = 0;
	J(2,1) = - L[2]*sin(q[1]);
	J(2,2) = - (L[1]+L[3])*cos(q[2]);

    return J;
}


Vector3d RobotM3::calculateGravityTorques() {
	Vector3d tau_g;

	//For convenience
	float *L = LinkLengths;
	float *M = LinkMasses;

	float g = 9.81; //Gravitational constant: remember to change it if using the robot on the Moon or another planet

	//Get current configuration
	Vector3d q;
    for(unsigned int i=0; i<3; i++) {
        q(i) = ((JointM3 *)joints[i])->getPosition();
    }

	//Calculate gravitational torques
	tau_g[0] = 0;
	tau_g[1] = -L[2]/2.0f*sin(q[1]) * (M[1]+M[2]+M[3]+M[4]) * g;
	tau_g[2] = -(L[1]/2.0f*(M[0]+M[3]) + L[1]*M[2] + L[4]/2.0f*M[4]) * cos(q[2]) * g;

	return tau_g;
}

Vector3d RobotM3::getJointPos() {
    return Vector3d({((JointM3*)joints[0])->getPosition(), ((JointM3*)joints[1])->getPosition(), ((JointM3*)joints[2])->getPosition()});
}

Vector3d RobotM3::getJointVel() {
    return Vector3d({((JointM3*)joints[0])->getVeloctiy(), ((JointM3*)joints[1])->getVeloctiy(), ((JointM3*)joints[2])->getVeloctiy()});
}

Vector3d RobotM3::getJointTor() {
    return Vector3d({((JointM3*)joints[0])->getTorque(), ((JointM3*)joints[1])->getTorque(), ((JointM3*)joints[2])->getTorque()});
}

Vector3d RobotM3::getEndEffPos() {
     return directKinematic(getJointPos());
}

Vector3d RobotM3::getEndEffVel() {
     return J()*getJointVel();
}

Vector3d RobotM3::getEndEffFor() {
     return (J().transpose()).inverse()*getJointTor();
}


setMovementReturnCode_t RobotM3::setJointPos(Vector3d q) {
    std::vector<double> pos{q(0), q(1), q(2)};
    return applyPosition(pos);
}

setMovementReturnCode_t RobotM3::setJointVel(Vector3d dq) {
    std::vector<double> vel{dq(0), dq(1), dq(2)};
    return applyVelocity(vel);
}

setMovementReturnCode_t RobotM3::setJointTor(Vector3d tau) {
    std::vector<double> tor{tau(0), tau(1), tau(2)};
    return applyTorque(tor);
}

setMovementReturnCode_t RobotM3::setEndEffPos(Vector3d X) {
    if(!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if(!1) {
        return OUTSIDE_LIMITS;
    }

    Vector3d q = inverseKinematic(X);
    if(std::isnan(q[0])||std::isnan(q[1])||std::isnan(q[2])) {
        return OUTSIDE_LIMITS;
    }
    else {
        return setJointPos(q);
    }
}

setMovementReturnCode_t RobotM3::setEndEffVel(Vector3d dX) {
    if(!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if(!1) {
        return OUTSIDE_LIMITS;
    }

    Vector3d dq = J().inverse()*dX;
    return setJointVel(dq);
}

setMovementReturnCode_t RobotM3::setEndEffFor(Vector3d F) {
    if(!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if(!1) {
        return OUTSIDE_LIMITS;
    }

    Vector3d tau = J().transpose()*F;
    return setJointTor(tau);

}

setMovementReturnCode_t RobotM3::setEndEffForWithCompensation(Vector3d F) {
    if(!calibrated) {
        return NOT_CALIBRATED;
    }

    //TODO: add a limit check
    if(!1) {
        return OUTSIDE_LIMITS;
    }
    Vector3d tau_g = calculateGravityTorques(); //Gravity compensation torque
    Vector3d tau_f; //Friction compensation torque
    double alpha = 0.7, beta = 0.05, threshold = 0.01;
    for(unsigned int i=0; i<3; i++) {
        double dq = ((JointM3 *)joints[i])->getVeloctiy();
        if(abs(dq)>threshold) {
            tau_f(i) = alpha*sign(dq) + beta*dq;
        }
    }

    Vector3d tau = J().transpose()*F + tau_g + tau_f;
    return setJointTor(tau);

}
