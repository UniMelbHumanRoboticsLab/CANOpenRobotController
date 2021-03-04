#include "JointM2.h"


JointM2::JointM2(int jointID, double q_min, double q_max, short int sign_, double dq_min, double dq_max, double tau_min, double tau_max, KincoDrive *drive, const std::string& name) :   Joint(jointID, q_min, q_max, drive, name),
                                                                                                                                                                sign(sign_),
                                                                                                                                                                qMin(q_min), qMax(q_max),
                                                                                                                                                                dqMin(dq_min), dqMax(dq_max),
                                                                                                                                                                tauMin(tau_min), tauMax(tau_max)
                                                                                                                                                                {
                                                                                                                                                                    spdlog::debug("MY JOINT ID: {} ({})", this->id, name);
                                                                                                                                                                }

JointM2::~JointM2() {
    // This delete is because drives aren't instantiated in M2Robot (as they are in other examples. )
    delete drive;
}

setMovementReturnCode_t JointM2::safetyCheck() {
    if (velocity > dqMax || velocity < dqMin) {
        return OUTSIDE_LIMITS;
    }
    if (torque > tauMax || torque < tauMin) {
        return OUTSIDE_LIMITS;
    }
    return SUCCESS;
}

setMovementReturnCode_t JointM2::setPosition(double qd) {
    if (calibrated) {
        if (qd >= qMin && qd <= qMax && std::isfinite(qd)) {
            return Joint::setPosition(qd);
        } else {
            return OUTSIDE_LIMITS;
        }
    } else {
        return NOT_CALIBRATED;
    }
}

setMovementReturnCode_t JointM2::setVelocity(double dqd) {
    //Position protection first only if calibrated
    if (calibrated) {
        if (position <= qMin && dqd < 0) {
            dqd = 0;
        }
        if (position >= qMax && dqd > 0) {
            dqd = 0;
        }
    }
    //Caped velocity
    if (dqd >= dqMin && dqd <= dqMax && std::isfinite(dqd)) {
        return Joint::setVelocity(dqd);
    } else {
        return OUTSIDE_LIMITS;
    }
}

setMovementReturnCode_t JointM2::setTorque(double taud) {
    //Position protection first only if calibrated
    if (calibrated) {
        if (position <= qMin && taud < 0) {
            taud = 0;
        }
        if (position >= qMax && taud > 0) {
            taud = 0;
        }
    }
    //Caped torque
    if (taud >= tauMin && taud <= tauMax && std::isfinite(taud)) {
        return Joint::setTorque(taud);
    } else {
        return OUTSIDE_LIMITS;
    }
}

bool JointM2::initNetwork() {
    spdlog::debug("JointM2::initNetwork()");
    if (((KincoDrive *)drive)->init(posControlMotorProfile)) {
        return true;
    } else {
        return false;
    }
}
