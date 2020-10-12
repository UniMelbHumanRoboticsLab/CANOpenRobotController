#include "M3DemoStates.h"

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

V3 impedance(Eigen::Matrix3d K, Eigen::Matrix3d D, V3 X0, V3 X, V3 dX, V3 dXd=V3::Zero()) {
    return K*(X0-X) + D*(dXd-dX);
}

//minJerk(X0, Xf, T, t, &X, &dX)
double JerkIt(V3 X0, V3 Xf, double T, double t, V3 &Xd, V3 &dXd) {
    t = std::max(std::min(t, T), .0); //Bound time
    double tn=std::max(std::min(t/T, 1.0), .0);//Normalised time bounded 0-1
    double tn3=pow(tn,3.);
    double tn4=tn*tn3;
    double tn5=tn*tn4;
    Xd = X0 + ( (X0-Xf) * (15.*tn4-6.*tn5-10.*tn3) );
    dXd = (X0-Xf) * (4.*15.*tn4-5.*6.*tn5-10.*3*tn3)/t;
    return tn;
}


void M3DemoState::entryCode(void) {
    //robot->applyCalibration();
    //robot->initPositionControl();
    //robot->initVelocityControl();
    //robot->initTorqueControl();
    qi=robot->getJointPosition();
    Xi=robot->getEndEffPosition();
    //robot->setJointVelocity(V3::Zero());
    //robot->setEndEffForceWithCompensation(V3::Zero(), false);
    robot->printJointStatus();
}
void M3DemoState::duringCode(void) {
    if(iterations%100==1) {
        //std::cout << "Doing nothing for "<< elapsedTime << "s..." << std::endl;
        std::cout << elapsedTime << " ";
        //robot->printJointStatus();
        robot->printStatus();
    }
    robot->setEndEffForceWithCompensation(V3(0,0,0));
    /*V3 q = robot->getJointPos();
    q(1)=68*M_PI/180.-0.1*elapsedTime;*/
    //std::cout << q.transpose() <<std::endl;
    //robot->setJointPos(qi-V3(0.03,0.03,0.03));
    //double v=-sin(2*M_PI*1./10*elapsedTime);
    //double v=-0.1;
    //robot->setJointVel(V3(0,0,0));

    //robot->printStatus();

    /*V3 dX(-0.02,0.05,0.1);
    if(robot->getEndEffPosition()(2)<0) {
        robot->setEndEffVel(dX);
    }
    else {
        robot->setEndEffVel(V3(0,0,0));
    }*/


    /*V3 Dq;
    if(elapsedTime<5)
        Dq={0,0.015*elapsedTime,0.015*elapsedTime};
    else
        Dq={0,0.015*5.,0.015*5.};
    robot->setJointPos(qi-Dq);*/

    /*V3 tau(0,-5.0,0);*/
    //robot->setJointTor(robot->calculateGravityTorques());



    /*float k_i=1.;
    V3 Xf(-0.4, 0, 0);
    V3 Xd, dXd;
    JerkIt(Xi, Xf, 5., elapsedTime, Xd, dXd);
    robot->setEndEffVelocity(dXd+k_i*(Xd-robot->getEndEffPosition()));
    std::cout << (Xd-robot->getEndEffPosition()).norm() << std::endl;*/
}
void M3DemoState::exitCode(void) {
    robot->setJointVelocity(V3::Zero());
    robot->setEndEffForceWithCompensation(V3(0,0,0));
}



void M3CalibState::entryCode(void) {
    calibDone=false;
    for(unsigned int i=0; i<3; i++) {
        stop_reached_time[i] = .0;
        at_stop[i] = false;
    }
    robot->decalibrate();
    robot->initTorqueControl();
    robot->printJointStatus();
    std::cout << "Calibrating (keep clear)..." << std::flush;
}
//Move slowly on each joint until max force detected
void M3CalibState::duringCode(void) {
    V3 tau(0, 0, 0);

    //Apply constant torque (with damping) unless stop has been detected for more than 0.5s
    V3 vel=robot->getJointVelocity();
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
        robot->setEndEffForceWithCompensation(V3(0,0,0));
        calibDone=true; //Trigger event
    }
    else {
        //If all joints are calibrated
        if(at_stop[0] && at_stop[1] && at_stop[2]) {
            robot->applyCalibration();
            std::cout << "OK." << std::endl;
        }
        else {
            robot->setJointTorque(tau);
            if(iterations%100==1) {
                std::cout << "." << std::flush;
            }
        }
    }
}
void M3CalibState::exitCode(void) {
    robot->setEndEffForceWithCompensation(V3(0,0,0));
}



void M3MassCompensation::entryCode(void) {
    robot->initTorqueControl();
    std::cout << "Press S to decrease mass (-100g), W to increase (+100g)." << std::endl;
}
void M3MassCompensation::duringCode(void) {

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
    robot->setEndEffForceWithCompensation(V3(0,0,t*mass*9.8));

    //Mass controllable through keyboard inputs
    if(robot->keyboard->getS()) {
        mass -=0.1;robot->printStatus();
        std::cout << "Mass: " << mass << std::endl;
    }
    if(robot->keyboard->getW()) {
        mass +=0.1;robot->printStatus();
        std::cout << "Mass: " << mass << std::endl;
    }
}
void M3MassCompensation::exitCode(void) {
    robot->setEndEffForceWithCompensation(V3(0,0,0));
}



void M3EndEffDemo::entryCode(void) {
    robot->initVelocityControl();
}
void M3EndEffDemo::duringCode(void) {

    //Joystick driven
    V3 dXd(0,0,0);
    for(unsigned int i=0; i<3; i++) {
        dXd(i)=robot->joystick->getAxis(i)/2.;
    }

    //Apply
    robot->setEndEffVelocity(dXd);

    if(iterations%100==1) {
        std::cout << dXd.transpose() << "  ";
        robot->printStatus();
    }
}
void M3EndEffDemo::exitCode(void) {
    robot->setEndEffVelocity(V3(0,0,0));
}



void M3DemoImpedanceState::entryCode(void) {
    robot->initTorqueControl();
    std::cout << "Press Q to select reference point, S/W to tune K gain and A/D for D gain" << std::endl;
}
void M3DemoImpedanceState::duringCode(void) {

    //Select start point
    if(robot->keyboard->getQ()) {
        Xi=robot->getEndEffPosition();
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
        std::cout << "K=" << k << " D=" << d << " => F=" << impedance(K, Eigen::Matrix3d::Zero(), Xi, robot->getEndEffPosition(), robot->getEndEffVelocity()).transpose() << " N" <<std::endl;
        robot->setEndEffForceWithCompensation(impedance(K, D, Xi, robot->getEndEffPosition(), robot->getEndEffVelocity()));
    }
    else {
        robot->setEndEffForceWithCompensation(V3(0,0,0));
    }
}
void M3DemoImpedanceState::exitCode(void) {
    robot->setEndEffForceWithCompensation(V3::Zero());
}



void M3DemoPathState::entryCode(void) {
    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(V3::Zero(), false);
    Xi=robot->getEndEffPosition();
}
void M3DemoPathState::duringCode(void) {
    V3 X=robot->getEndEffPosition();
    V3 dX=robot->getEndEffVelocity();
    V3 Xd=Xi;
    V3 Fd(0,0,0);

    //Find closest point on path
    V3 PathUnitV=(Xf-Xi)/(Xf-Xi).norm();
    Xd = Xi + ( (X-Xi).dot(PathUnitV)*PathUnitV);

    //Progress along path
    double progress = sign((X-Xi).dot(PathUnitV))*(Xd-Xi).norm()/(Xf-Xi).norm();
    double b=0;
    if(progress<=0) {
        //"Before" Xi
        Xd = Xi;
        b = 0.;
    }
    if(progress>=0 && progress<1) {
        //In between Xi and Xf
        if(progress>0.5)
            b = 1-progress;
        else
            b = progress;
    }
    if(progress>=1) {
        //After "Xf"
        Xd = Xf;
        b = 0.;
    }

    //Velocity vector projection
    V3 dX_path = dX.dot(PathUnitV)*PathUnitV;
    V3 dX_ortho = dX - dX_path;

    //Impedance towards path  point with only ortho velocity component
    Eigen::Matrix3d K = k*Eigen::Matrix3d::Identity();
    Eigen::Matrix3d D = d*Eigen::Matrix3d::Identity();
    Fd = impedance(K, D, Xd, X, dX_ortho);

    //Assitive viscosity in path direction
    Fd+=viscous_assistance*b*dX_path;

    //Apply force with gravity compensation but w/o friction compensation
    robot->setEndEffForceWithCompensation(Fd, false);

    //Display progression
    if(iterations%100==1) {
        std::cout << "Path progress: |";
        for(int i=0; i<round(progress*50.); i++)
            std::cout << "=";
        for(int i=0; i<round((1-progress)*50.); i++)
            std::cout << "-";

        std::cout << "| (" << progress*100 << "%)" << std::endl;
    }
}
void M3DemoPathState::exitCode(void) {
    robot->setEndEffForceWithCompensation(V3::Zero());
}



void M3SamplingEstimationState::entryCode(void) {
    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(V3::Zero());
    std::cout << "Move robot around while estimating time" << std::endl;
}
void M3SamplingEstimationState::duringCode(void) {
    //Apply gravity compensation
    robot->setEndEffForceWithCompensation(V3::Zero());

    //Save dt
    if(iterations>1 && iterations<nb_samples+2) {
        //Do some math for fun
        Eigen::Matrix3d K = 2.36*Eigen::Matrix3d::Identity();
        Eigen::Matrix3d D = 0.235*Eigen::Matrix3d::Identity();
        impedance(K, Eigen::Matrix3d::Zero(), V3(-0.5, 0.23, 0.65), robot->getEndEffPosition(), robot->getEndEffVelocity()).transpose();
        robot->J().inverse()*V3::Zero()+2*robot->inverseKinematic(V3(-0.5, 0, 0));

        //Get time and actual value read from CAN to get sampling rate
        dts[iterations-2] = dt;
        dX[iterations-2] = robot->getJointVelocity()[1];
        if(dX[iterations-2]!=dX[iterations-3] && iterations>2 ){ //Value has actually been updated
            new_value++;
        }
    }
    else if(iterations==nb_samples+2) {
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
void M3SamplingEstimationState::exitCode(void) {
    robot->setEndEffForceWithCompensation(V3::Zero());
}



void M3DemoMinJerkPosition::entryCode(void) {
    //Setup velocity control for position over velocity loop
    robot->initVelocityControl();
    robot->setJointVelocity(V3::Zero());
    //Initialise to first target point
    TrajPtIdx=0;
    startTime=elapsedTime;
    Xi=robot->getEndEffPosition();
    Xf=TrajPt[TrajPtIdx];
    T=TrajTime[TrajPtIdx];
    k_i=1.;
}
void M3DemoMinJerkPosition::duringCode(void) {

    V3 Xd, dXd;
    //Compute current desired interpolated point
    double status=JerkIt(Xi, Xf, T, elapsedTime-startTime, Xd, dXd);
    //Apply position control
    robot->setEndEffVelocity(dXd+k_i*(Xd-robot->getEndEffPosition()));


    //Have we reached a point?
    if(status>=1.) {
        //Go to next point
        TrajPtIdx++;
        if(TrajPtIdx>=TrajNbPts){
            TrajPtIdx=0;
        }
        //From where we are
        Xi=robot->getEndEffPosition();
        //To next point
        Xf=TrajPt[TrajPtIdx];
        T=TrajTime[TrajPtIdx];
        startTime=elapsedTime;
    }


   /* //Display progression
    if(iterations%100==1) {
        std::cout << "Progress (Point "<< TrajPtIdx << ") |";
        for(int i=0; i<round(status*50.); i++)
            std::cout << "=";
        for(int i=0; i<round((1-status)*50.); i++)
            std::cout << "-";

        std::cout << "| (" << status*100 << "%)  ";
        robot->printStatus();
    }*/
}
void M3DemoMinJerkPosition::exitCode(void) {
    robot->setJointVelocity(V3::Zero());
}



