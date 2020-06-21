#include "RobotM3.h"

#include "DebugMacro.h"

using namespace Eigen;

RobotM3::RobotM3() : Robot() {

    //Define the robot structure: each joint with limits and drive: should be in constructor
    joints.push_back(new JointM3(0, -45/M_PI*180, 45/M_PI*180));
    joints.push_back(new JointM3(1, -45/M_PI*180, 45/M_PI*180));//Todo
    joints.push_back(new JointM3(2, -45/M_PI*180, 45/M_PI*180));//Todo
}

RobotM3::~RobotM3() {
    DEBUG_OUT("Delete RobotM3 object begins")
    for (auto p : joints) {
        DEBUG_OUT("Delete Joint ID: " << p->getId())
        delete p;
    }
    for (auto p : inputs) {
        DEBUG_OUT("Deleting Input")
        delete p;
    }
    joints.clear();
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

    return true;
}
bool RobotM3::initialiseInputs() {
    inputs.push_back(&keyboard);
    return true;
}

void RobotM3::updateRobot() {
    Robot::updateRobot();
}

void RobotM3::printStatus() {
    std::setprecision(3);
    std::cout << std::fixed;
    std::cout << "q=[ ";
    for (auto joint : joints)
        std::cout << ((JointM3 *)joint)->getQ() << "\t ";
    std::cout <<"]\tdq=[ ";
    for (auto joint : joints)
        std::cout << ((JointM3 *)joint)->getDq() << "\t ";
    std::cout <<"]\ttau=[ ";
    for (auto joint : joints)
        std::cout << ((JointM3 *)joint)->getTau() << "\t ";
    std::cout <<"]" << std::endl;
}



bool RobotM3::initPositionControl() {
    DEBUG_OUT("Initialising Position Control on all joints ")
    bool returnValue = true;
    for (auto p : joints) {
        if (((ActuatedJoint *)p)->setMode(POSITION_CONTROL, posControlMotorProfile) != POSITION_CONTROL) {
            // Something back happened if were are here
            DEBUG_OUT("Something bad happened")
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((ActuatedJoint *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((ActuatedJoint *)p)->enable();
    }
    return returnValue;
}
bool RobotM3::initTorqueControl() {
    DEBUG_OUT("Initialising Torque Control on all joints ")
    bool returnValue = true;
    for (auto p : joints) {
        if (((ActuatedJoint *)p)->setMode(TORQUE_CONTROL) != TORQUE_CONTROL) {
            // Something back happened if were are here
            DEBUG_OUT("Something bad happened")
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((ActuatedJoint *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((ActuatedJoint *)p)->enable();
    }
    return returnValue;
}

setMovementReturnCode_t RobotM3::setPosition(std::vector<double> positions) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((ActuatedJoint *)p)->setPosition(positions[i]);
        if (setPosCode == INCORRECT_MODE) {
            std::cout << "Joint ID " << p->getId() << ": is not in Position Control " << std::endl;
            returnValue = INCORRECT_MODE;
        } else if (setPosCode != SUCCESS) {
            // Something bad happened
            std::cout << "Joint " << p->getId() << ": Unknown Error " << std::endl;
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}



Vector3d RobotM3::directKinematic(Vector3d q)
{
    Vector3d X;

	float *L = LinkLengths;

	double F1 = (L[2]*sin(q[1]) + L[4]*cos(q[2]) + L[0]);

	X[0] = -F1 * cos(q[0]);
	X[1] = -F1 * sin(q[0]);
	X[2] = L[2]*cos(q[1]) - L[4]*sin(q[2]);

    return X;
}

Vector3d RobotM3::inverseKinematic(Vector3d X)
{
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

Matrix3d RobotM3::J()
{
    Matrix3d J;
    Vector3d q;
    for(unsigned int i=0; i<3; i++)
        q(i) = joints[i]->getQ();

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





setMovementReturnCode_t RobotM3::setJointPos(Vector3d q) {
    std::vector<double> pos{q(0), q(1), q(2)};
    return setPosition(pos);
}

setMovementReturnCode_t RobotM3::setJointVel(Vector3d q) {
}

setMovementReturnCode_t RobotM3::setJointTorque(Vector3d tau) {
}

setMovementReturnCode_t RobotM3::setEndEffPos(Vector3d X) {
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
    //TODO: add a limit check
    if(!1) {
        return OUTSIDE_LIMITS;
    }

    Vector3d dq = J().inverse()*dX;
    return setJointVel(dq);
}

setMovementReturnCode_t RobotM3::setEndEffForce(Vector3d F) {
    //TODO: add a limit check
    if(!1) {
        return OUTSIDE_LIMITS;
    }

    Vector3d tau = J().transpose()*F;
    return setJointTorque(tau);

}
