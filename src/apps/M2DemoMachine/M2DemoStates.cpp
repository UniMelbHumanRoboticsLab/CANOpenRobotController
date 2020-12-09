#include "M2DemoStates.h"
#include "M2DemoMachine.h"

#define OWNER ((M2DemoMachine *)owner)

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

VM2 impedance(Eigen::Matrix2d K, Eigen::Matrix2d D, VM2 X0, VM2 X, VM2 dX, VM2 dXd=VM2::Zero()) {
    return K*(X0-X) + D*(dXd-dX);
}

//minJerk(X0, Xf, T, t, &X, &dX)
double JerkIt(VM2 X0, VM2 Xf, double T, double t, VM2 &Xd, VM2 &dXd) {
    t = std::max(std::min(t, T), .0); //Bound time
    double tn=std::max(std::min(t/T, 1.0), .0);//Normalised time bounded 0-1
    double tn3=pow(tn,3.);
    double tn4=tn*tn3;
    double tn5=tn*tn4;
    Xd = X0 + ( (X0-Xf) * (15.*tn4-6.*tn5-10.*tn3) );
    dXd = (X0-Xf) * (4.*15.*tn4-5.*6.*tn5-10.*3*tn3)/t;
    return tn;
}


void M2DemoState::entryCode(void) {
    //robot->applyCalibration();
    //robot->initPositionControl();
    //robot->initVelocityControl();
    //robot->initTorqueControl();
    qi=robot->getPosition();
    Xi=robot->getEndEffPosition();
    //robot->setJointVelocity(VM2::Zero());
    //robot->setEndEffForceWithCompensation(VM2::Zero(), false);
    robot->printJointStatus();
}
void M2DemoState::duringCode(void) {
    if(iterations%100==1) {
        //std::cout << "Doing nothing for "<< elapsedTime << "s..." << std::endl;
        std::cout << elapsedTime << " ";
        //robot->printJointStatus();
        robot->printStatus();
    }
    robot->setEndEffForceWithCompensation(VM2::Zero());
    /*VM2 q = robot->getJointPos();
    q(1)=68*M_PI/180.-0.1*elapsedTime;*/
    //std::cout << q.transpose() <<std::endl;
    //robot->setJointPos(qi-VM2(0.03,0.03,0.03));
    //double v=-sin(2*M_PI*1./10*elapsedTime);
    //double v=-0.1;
    //robot->setJointVel(VM2(0,0,0));

    //robot->printStatus();

    /*VM2 dX(-0.02,0.05,0.1);
    if(robot->getEndEffPosition()(2)<0) {
        robot->setEndEffVel(dX);
    }
    else {
        robot->setEndEffVel(VM2(0,0,0));
    }*/


    /*VM2 Dq;
    if(elapsedTime<5)
        Dq={0,0.015*elapsedTime,0.015*elapsedTime};
    else
        Dq={0,0.015*5.,0.015*5.};
    robot->setJointPos(qi-Dq);*/

    /*VM2 tau(0,-5.0,0);*/
    //robot->setJointTor(robot->calculateGravityTorques());



    /*float k_i=1.;
    VM2 Xf(-0.4, 0, 0);
    VM2 Xd, dXd;
    JerkIt(Xi, Xf, 5., elapsedTime, Xd, dXd);
    robot->setEndEffVelocity(dXd+k_i*(Xd-robot->getEndEffPosition()));
    std::cout << (Xd-robot->getEndEffPosition()).norm() << std::endl;*/
}
void M2DemoState::exitCode(void) {
    robot->setJointVelocity(VM2::Zero());
    robot->setEndEffForceWithCompensation(VM2::Zero());
}



void M2CalibState::entryCode(void) {
    calibDone=false;
    for(unsigned int i=0; i<2; i++) {
        stop_reached_time[i] = .0;
        at_stop[i] = false;
    }
    robot->decalibrate();
    robot->initTorqueControl();
    robot->printJointStatus();
    std::cout << "Calibrating (keep clear)..." << std::flush;
}
//Move slowly on each joint until max force detected
void M2CalibState::duringCode(void) {
    VM2 tau(0, 0);

    //Apply constant torque (with damping) unless stop has been detected for more than 0.5s
    VM2 vel=robot->getVelocity();
    double b = 3.;
    for(unsigned int i=0; i<vel.size(); i++) {
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
        robot->setEndEffForceWithCompensation(VM2::Zero());
        calibDone=true; //Trigger event
    }
    else {
        //If all joints are calibrated
        if(at_stop[0] && at_stop[1]) {
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
void M2CalibState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}



void M2Transparent::entryCode(void) {
    robot->initTorqueControl();
}
void M2Transparent::duringCode(void) {

    //Smooth transition in case a mass is set at startup
    double settling_time = 3.0;
    double t=elapsedTime>settling_time?1.0:elapsedTime/settling_time;

    //Apply corresponding force
    robot->setEndEffForceWithCompensation(VM2::Zero());
}
void M2Transparent::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}



void M2EndEffDemo::entryCode(void) {
    robot->initVelocityControl();
}
void M2EndEffDemo::duringCode(void) {

    //Joystick driven
    VM2 dXd = VM2::Zero();
    for(unsigned int i=0; i<dXd.size(); i++) {
        dXd(i)=robot->joystick->getAxis(i)/2.;
    }

    //Apply
    robot->setEndEffVelocity(dXd);

    if(iterations%100==1) {
        std::cout << dXd.transpose() << "  ";
        robot->printStatus();
    }
}
void M2EndEffDemo::exitCode(void) {
    robot->setEndEffVelocity(VM2::Zero());
}



void M2DemoImpedanceState::entryCode(void) {
    robot->initTorqueControl();
    std::cout << "Press Q to select reference point, S/W to tune K gain and A/D for D gain" << std::endl;
}
void M2DemoImpedanceState::duringCode(void) {

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
    Eigen::Matrix2d K = k*Eigen::Matrix2d::Identity();

    //D tuning
    if(robot->keyboard->getD()) {
        d -= 1;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }
    if(robot->keyboard->getA()) {
        d += 1;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }
    Eigen::Matrix2d D = d*Eigen::Matrix2d::Identity();

    //Apply impedance control
    if(init) {
        std::cout << "K=" << k << " D=" << d << " => F=" << impedance(K, Eigen::Matrix2d::Zero(), Xi, robot->getEndEffPosition(), robot->getEndEffVelocity()).transpose() << " N" <<std::endl;
        robot->setEndEffForceWithCompensation(impedance(K, D, Xi, robot->getEndEffPosition(), robot->getEndEffVelocity()));
    }
    else {
        robot->setEndEffForceWithCompensation(VM2::Zero());
    }
}
void M2DemoImpedanceState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}



void M2DemoPathState::entryCode(void) {
    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM2::Zero(), false);
    Xi=robot->getEndEffPosition();
}
void M2DemoPathState::duringCode(void) {
    VM2 X=robot->getEndEffPosition();
    VM2 dX=robot->getEndEffVelocity();
    VM2 Xd=Xi;
    VM2 Fd(0,0);

    //Find closest point on path
    VM2 PathUnitV=(Xf-Xi)/(Xf-Xi).norm();
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
    VM2 dX_path = dX.dot(PathUnitV)*PathUnitV;
    VM2 dX_ortho = dX - dX_path;

    //Impedance towards path  point with only ortho velocity component
    Eigen::Matrix2d K = k*Eigen::Matrix2d::Identity();
    Eigen::Matrix2d D = d*Eigen::Matrix2d::Identity();
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
void M2DemoPathState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}



void M2SamplingEstimationState::entryCode(void) {
    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM2::Zero());
    std::cout << "Move robot around while estimating time" << std::endl;
}
void M2SamplingEstimationState::duringCode(void) {
    //Apply gravity compensation
    robot->setEndEffForceWithCompensation(VM2::Zero());

    //Save dt
    if(iterations>1 && iterations<nb_samples+2) {
        //Do some math for fun
        Eigen::Matrix2d K = 2.36*Eigen::Matrix2d::Identity();
        Eigen::Matrix2d D = 0.235*Eigen::Matrix2d::Identity();
        impedance(K, Eigen::Matrix2d::Zero(), VM2(-0.5, 0.23), robot->getEndEffPosition(), robot->getEndEffVelocity()).transpose();
        robot->J().inverse()*VM2::Zero()+2*robot->inverseKinematic(VM2(-0.5, 0));

        //Get time and actual value read from CAN to get sampling rate
        dts[iterations-2] = dt;
        dX[iterations-2] = robot->getVelocity()[1];
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
void M2SamplingEstimationState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}



void M2DemoMinJerkPosition::entryCode(void) {
    //Setup velocity control for position over velocity loop
    robot->initVelocityControl();
    robot->setJointVelocity(VM2::Zero());
    //Initialise to first target point
    TrajPtIdx=0;
    startTime=elapsedTime;
    Xi=robot->getEndEffPosition();
    Xf=TrajPt[TrajPtIdx];
    T=TrajTime[TrajPtIdx];
    k_i=1.;
}
void M2DemoMinJerkPosition::duringCode(void) {

    VM2 Xd, dXd;
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
void M2DemoMinJerkPosition::exitCode(void) {
    robot->setJointVelocity(VM2::Zero());
}



