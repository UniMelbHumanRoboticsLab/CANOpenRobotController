#include "M1DemoStates.h"


double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

//minJerk(X0, Xf, T, t, &X, &dX)

//Eigen::Vector3d impedance(Eigen::Matrix3d K, Eigen::Matrix3d D, Eigen::Vector3d X0, Eigen::Vector3d X, Eigen::Vector3d dX) {
//    return K*(X0-X) - D*dX;
//}

void IdleState::entry(void) {
//    chaiServer = new server(3, 3);
//    if(chaiServer->Connect(IP_ADDRESS)!=0) {
//        std::cout /*cerr is banned*/ << "M3ChaiCommunication: Unable to initialise socket... Quitting." <<std::endl;
//        raise(SIGTERM); //Clean exit
//    }
    std::cout
            << "==================================" << std::endl
            << " WELCOME TO THE TEST STATE MACHINE" << std::endl
            << "==================================" << std::endl
            << std::endl
            << "========================" << std::endl
            << " PRESS X to start monitoring" << std::endl
            << " PRESS S to start demo state" << std::endl
            << " PRESS A to start position control" << std::endl
            << "========================" << std::endl;
}

void IdleState::during(void) {
//    if (chaiServer->IsConnected()) {
//        double *x = new double[3]{0, 1, 2}; //Chai representation frame is: X towards the operator when facing device, Y towards right hand side and Z up
//        chaiServer->Send(x);
//    }

}

void IdleState::exit(void) {
//    delete chaiServer;
    robot->stop();
    std::cout << "Idle State Exited" << std::endl;
}

//******************************* Monitoring **************************
void Monitoring::entry(void) {
    robot->applyCalibration();
    robot->initMonitoring();
    robot->m1ForceSensor->calibrate();
}

void Monitoring::during(void) {
    if(iterations++%100==1) {
        robot->printJointStatus();
        JointVec tau = robot->getJointTor_s();
        std::cout << std::dec << iterations << ": " << std::setprecision(2) << tau(0) << std::endl;
    }
}

void Monitoring::exit(void) {
    robot->stop();
    std::cout << "Idle State Exited" << std::endl;
}

//******************************* Demo state **************************
void M1PositionTracking::entryCode(void) {
    mode = 4;
    robot->applyCalibration();
    switch(mode){
        case 1:
            robot->initPositionControl();
            freq = 0.5;
            counter = 1;
            magnitude = 20;
            break;
        case 2:
            robot->initVelocityControl();
//            robot->admittanceControl();
            freq = 0.2;
            magnitude = 8;   // degree per second
            break;
        case 3:
            robot->initTorqueControl();
            freq = 0.2;
            magnitude = 0.04;   // degree per second
            break;
        case 4:
            robot->initVelocityControl();
            robot->m1ForceSensor->calibrate();
            Ks = 0;
            B = 0.01;
            dt = 0.01;
            Mass = 0.01;
            gain = 1;
            break;
        default:
            std::cout << "Wrong mode !" << std::endl;
    }

}

void M1PositionTracking::duringCode(void) {
    if(iterations%100==1) {
        //std::cout << "Doing nothing for "<< elapsedTime << "s..." << std::endl;
        robot->printJointStatus();
    }

    if(robot->status != R_SUCCESS){
        status = false;
        std::cout << "Robot error !" << std::endl;
    }

    switch(mode){
        case 1:
            positionControl();
            break;
        case 2:
            velocityControl();
            break;
        case 3:
            torqueControl();
            break;
        case 4:
            admittanceControl();
            break;
        default:
            std::cout << "Wrong mode !" << std::endl;
    }
}

void M1PositionTracking::exitCode(void) {
    switch(mode) {
        case 1:
            robot->setJointPos(JointVec::Zero());
            break;
        case 2:
            robot->setJointVel(JointVec::Zero());
            break;
        case 3:
            robot->setJointTor(JointVec::Zero());
            break;
        case 4:
            robot->setJointVel(JointVec::Zero());
            break;
        default:
            std::cout << "Wrong mode !" << std::endl;
    }
    robot->stop();
}

void M1PositionTracking::positionControl(void){
    q=robot->getJointPos();
    std::cout << q(0) << " <-> ";
//    q(0) = magnitude*sin(2*M_PI*freq*iterations/100);
    if (iterations <= 1000){
        q(0) = 90;
    }
    std::cout << q(0) << std::endl;
    if(robot->setJointPos(q) != SUCCESS){
        std::cout << "Error: " << std::endl;
    }
}

void M1PositionTracking::velocityControl(void){
    dq=robot->getJointVel();
//    dq(0) = magnitude*sin(2*M_PI*freq*iterations/100);
//     velocity control, differential velocity and command velocity
//    std::cout << std::dec << iterations << ": " << (robot->getJointPos() - q)*100 << " - " << dq(0) << std::endl;

    q=robot->getJointPos();
//    std::cout << std::dec << iterations << ": " << q(0) << " - " << q(0) << std::endl;
    if (iterations%100 ==0)
    {
        std::cout << std::dec << iterations << ": " << q(0) << " - " << q(0) << std::endl;
    }
    // give velocity command for 5 s and check the position changes
    if (iterations > 0 & iterations <= 500) {
        dq(0) = magnitude;
    }
    else
    {
        dq(0) = 0;
    }

    if(robot->setJointVel(dq) != SUCCESS){
        std::cout << "Error: " << std::endl;
    }
}

void M1PositionTracking::torqueControl(void){
    tau=robot->getJointTor();
    tau(0) = magnitude*sin(2*M_PI*freq*iterations/100);
    q=robot->getJointPos();
    std::cout << std::dec << iterations << ": " << tau(0) << " - " << q(0) << std::endl;
//    tau(0) = magnitude;
//    if (iterations == 6000){
//        magnitude = 0;
//    }
    if(robot->setJointTor(tau) != SUCCESS){
        std::cout << "Error: " << std::endl;
    }
}

void M1PositionTracking::admittanceControl(void){
    tau = (2*tau+robot->getJointTor_s())/3;
    if (tau(0) > 100 || tau(0) < -100){
        std::cout << "Torque sensor:: reading out of limits " << std::endl;
    }
    else
    {
        B = 0.01;
        Mass = 1;
        q = robot->getJointPos();
        dq = robot->getJointVel();
    B = 0;
        net_tau = tau(0) - Ks*q(0) - B*dq(0);
        acc = net_tau/Mass;
        dq(0) = dq(0) + gain*acc*dt;

        if (iterations%5==0){
            std::cout << std::dec << iterations << ": " << std::setprecision(2) << tau(0) << " ~ " << q(0) << " ~ " << dq(0)<< std::endl;
        }
        if (q(0) > 45 or q(0) <-45){
            dq(0) = dq(0)*0.5;
        }
        if(robot->setJointVel(dq) != SUCCESS){
            std::cout << "Error: " << std::endl;
        }
    }
}

//******************************* Demo state **************************
void M1DemoState::entryCode(void) {
    robot->applyCalibration();
    robot->initPositionControl();
    //robot->initVelocityControl();
//    robot->initTorqueControl();
//    qi=robot->getJointPos();
//    Xi=robot->getEndEffPos();
    freq = 0.1;
    counter = 1;
//    qi(0) = 45;
//    robot->setJointPos(qi);
}

void M1DemoState::duringCode(void) {
    if(iterations%100==1) {
        //std::cout << "Doing nothing for "<< elapsedTime << "s..." << std::endl;
        robot->printJointStatus();
        //robot->printStatus();
        /* */
//        std::cout << std::dec << iterations << std::endl;
//        counter = counter + 1;
//        std::cout << dt << std::endl;
//        qi(0) = 10*sin(2*M_PI*freq*counter/100);
////        qi=robot->getJointPos();
//        if(robot->setJointPos(qi) == SUCCESS){
//            std::cout << "Set new position "<< qi(0) << std::endl;
//        }
    }

    if(robot->status == R_SUCCESS && iterations%4==0) {
        // control frequency is 400 hz
        uint sample = iterations/4;
//        std::cout << qi(0) << std::endl;
        qi=robot->getJointPos();
        std::cout << qi(0) << " <-> ";
        qi(0) = 20*sin(2*M_PI*freq*sample/100);
        std::cout << qi(0) << std::endl;
        JointVec dq_t;
        dq_t(0) = 1;
        if(robot->setJointPos(qi) != SUCCESS){
            std::cout << "Error: " << std::endl;
        }
//        std::cout << "Velocity _1: " << dq_t(0) << std::endl;
//        robot->setJointVel(dq_t);
    }

//    qi(0) = 30*sin(2*M_PI*freq*iterations/200);
//    if(robot->setJointPos(qi) != SUCCESS){
//        std::cout << "Error: " << std::endl;
//    }

//    int frequency = 1./dt;
//    std::cout << "Frequency: " << std::dec << frequency << std::endl;

//    qi(0) = 20*sin(2*M_PI*freq*elapsedTime);
//    if(robot->setJointPos(qi) == SUCCESS){
//        std::cout << "Set new position "<< qi(0) << std::endl;
//    }

//    JointVec tau;
//    tau(0) = 0.5;
//    robot->setJointTor(tau);
//    JointVec pos;
//    pos(0) = 30;

    /*Eigen::Vector3d q = robot->getJointPos();
    q(1)=68*M_PI/180.-0.1*elapsedTime;*/
    //std::cout << q.transpose() <<std::endl;
    //robot->setJointPos(qi-Eigen::Vector3d(0.03,0.03,0.03));
    //double v=-sin(2*M_PI*1./10*elapsedTime);
    //double v=-0.1;
    //robot->setJointVel(Eigen::Vector3d(0,0,0));

    //robot->printStatus();

    /*Eigen::Vector3d dX(-0.02,0.05,0.1);
    if(robot->getEndEffPos()(2)<0) {
        robot->setEndEffVel(dX);
    }
    else {
        robot->setEndEffVel(Eigen::Vector3d(0,0,0));
    }*/


    /*Eigen::Vector3d Dq;
    if(elapsedTime<5)
        Dq={0,0.015*elapsedTime,0.015*elapsedTime};
    else
        Dq={0,0.015*5.,0.015*5.};
    robot->setJointPos(qi-Dq);*/

    /*Eigen::Vector3d tau(0,-5.0,0);*/
    //robot->setJointTor(robot->calculateGravityTorques());
    /*Eigen::Vector3d F(0,0,0);
    robot->setEndEffForWithCompensation(F);*/
}

void M1DemoState::exitCode(void) {
//    robot->setJointVel(JointVec::Zero());
//    robot->setJointTor(JointVec::Zero());
    robot->setJointPos(JointVec::Zero());
//    robot->setEndEffForWithCompensation(Eigen::Vector3d(0,0,0));
}

/*
void M1CalibState::entryCode(void) {
    calibDone=false;
    for(unsigned int i=0; i<3; i++) {
        stop_reached_time[i] = .0;
        at_stop[i] = false;
    }
    robot->decalibrate();
    robot->initTorqueControl();
    std::cout << "Calibrating (keep clear)...";
}
//Move slowly on each joint until max force detected
void M1CalibState::duringCode(void) {
    Eigen::Vector3d tau(0, 0, 0);

    //Apply constant torque (with damping) unless stop has been detected for more than 0.5s
    Eigen::Vector3d vel=robot->getJointVel();
    double b = 3.;
    for(unsigned int i=0; i<3; i++) {
        tau(i) = std::min(std::max(2 - b * vel(i), .0), 2.);
        if(stop_reached_time(i)>0.5) {
            at_stop[i]=true;
        }
        if(vel(i)<0.01) {
            stop_reached_time(i) += dt;
        }
    }

    //Switch to gravity control when done
    if(robot->isCalibrated()) {
        robot->setEndEffForWithCompensation(Eigen::Vector3d(0,0,0));
        robot->printJointStatus();
        calibDone=true; //Trigger event
    }
    else {
        //If all joints are calibrated
        if(at_stop[0] && at_stop[1] && at_stop[2]) {
            robot->applyCalibration();
            std::cout << "OK." << std::endl;
        }
        else {
            robot->setJointTor(tau);
        }
    }
}
void M1CalibState::exitCode(void) {
    robot->setEndEffForWithCompensation(Eigen::Vector3d(0,0,0));
}





void M1MassCompensation::entryCode(void) {

    std::cout << "Press S to decrease mass (-100g), W to increase (+100g)." << mass << std::endl;
}
void M1MassCompensation::duringCode(void) {

    //Smooth transition in case a mass is set at startup
    double settling_time = 3.0;
    double t=elapsedTime>settling_time?1.0:elapsedTime/settling_time;

    //Bound mass to +-5kg
    if(mass>5.0) {
        mass = 5;
    }
    if(mass<-5) {
        mass = -5;
    }

    //Apply corresponding force
    robot->setEndEffForWithCompensation(Eigen::Vector3d(0,0,t*mass*9.8));

    //Mass controllable through keyboard inputs
    if(robot->keyboard->getS()) {
        mass -=0.1;
        std::cout << "Mass: " << mass << std::endl;
    }
    if(robot->keyboard->getW()) {
        mass +=0.1;
        std::cout << "Mass: " << mass << std::endl;
    }
}
void M1MassCompensation::exitCode(void) {
    robot->setEndEffForWithCompensation(Eigen::Vector3d(0,0,0));
}





void M1EndEffDemo::entryCode(void) {
    robot->initVelocityControl();
}
void M1EndEffDemo::duringCode(void) {
    Eigen::Vector3d dX(0,0,0);

    if(elapsedTime<1.0)
    {
        //Go towards the center
        dX={-0.1,0.1,0.1};
    }
    else {
        //Joystick driven
        for(unsigned int i=0; i<3; i++) {
            dX(i)=robot->joystick->getAxis(i)/2.;
        }
    }

    //Apply
    robot->setEndEffVel(dX);

    if(iterations%20==1) {
        robot->printStatus();
    }
}
void M1EndEffDemo::exitCode(void) {
    robot->setEndEffVel(Eigen::Vector3d(0,0,0));
}






void M1DemoImpedanceState::entryCode(void) {
    robot->initTorqueControl();
    std::cout << "Press Q to select reference point, S/W to tune K gain and A/D for D gain" << std::endl;
}
void M1DemoImpedanceState::duringCode(void) {

    //Select start point
    if(robot->keyboard->getQ()) {
        Xi=robot->getEndEffPos();
        init=true;
    }

    //K tuning
    if(robot->keyboard->getS()) {
        k -= 5;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }
    if(robot->keyboard->getW()) {
        k += 5;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }
    Eigen::Matrix3d K = k*Eigen::Matrix3d::Identity();

    //D tuning
    if(robot->keyboard->getD()) {
        d -= 1;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }
    if(robot->keyboard->getA()) {
        d += 1;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }
    Eigen::Matrix3d D = d*Eigen::Matrix3d::Identity();

    //Apply impedance control
    if(init) {
        std::cout << "K=" << k << " D=" << d << " => F=" << impedance(K, Eigen::Matrix3d::Zero(), Xi, robot->getEndEffPos(), robot->getEndEffVel()).transpose() << " N" <<std::endl;
        robot->setEndEffForWithCompensation(impedance(K, D, Xi, robot->getEndEffPos(), robot->getEndEffVel()));
    }
    else {
        robot->setEndEffForWithCompensation(Eigen::Vector3d(0,0,0));
    }
}
void M1DemoImpedanceState::exitCode(void) {
    robot->setEndEffForWithCompensation(Eigen::Vector3d::Zero());
}





void M1SamplingEstimationState::entryCode(void) {
    robot->initTorqueControl();
    robot->setEndEffForWithCompensation(Eigen::Vector3d::Zero());
    std::cout << "Move robot around while estimating time" << std::endl;
}
void M1SamplingEstimationState::duringCode(void) {
    //Apply gravity compensation
    robot->setEndEffForWithCompensation(Eigen::Vector3d::Zero());

    //Save dt
    if(iterations<nb_samples) {
        //Do some math for fun
        Eigen::Matrix3d K = 2.36*Eigen::Matrix3d::Identity();
        Eigen::Matrix3d D = 0.235*Eigen::Matrix3d::Identity();
        impedance(K, Eigen::Matrix3d::Zero(), Eigen::Vector3d(-0.5, 0.23, 0.65), robot->getEndEffPos(), robot->getEndEffVel()).transpose();
        robot->J().inverse()*Eigen::Vector3d::Zero()+2*robot->inverseKinematic(Eigen::Vector3d(-0.5, 0, 0));

        //Get time and actual value read from CAN to get sampling rate
        dts[iterations] = dt;
        dX[iterations] = robot->getEndEffVel().norm();
        if(dX[iterations-1]!=dX[iterations]){ //Value has actually been updated
            new_value++;
        }
    }
    else if(iterations==nb_samples) {
        std::cout << "Done." <<std::endl;
        double dt_avg=0;
        double dt_max=0;
        double dt_min=50000;
        for(unsigned int i=0; i<nb_samples; i++) {
            dt_avg+=dts[i]/(double)nb_samples;
            if(dts[i]>dt_max)
                dt_max=dts[i];
            if(i>1 && dts[i]<dt_min)
                dt_min=dts[i];
        }
        std::cout<< std::dec<<std::setprecision(3) << "Loop time (min, avg, max): " << dt_min*1000 << " < " << dt_avg*1000 << " < " << dt_max*1000 << " (ms). Actual CAN sampling: " << nb_samples*dt_avg / (double) new_value*1000 <<  std::endl;
    }
}
void M1SamplingEstimationState::exitCode(void) {
    robot->setEndEffForWithCompensation(Eigen::Vector3d::Zero());

}
*/
