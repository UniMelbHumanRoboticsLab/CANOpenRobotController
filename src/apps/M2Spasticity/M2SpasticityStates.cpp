#include "M2SpasticityStates.h"
#include "M2Spasticity.h"

#define OWNER ((M2Spasticity *)owner)

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
    /**
    *what does the following mean?
    */
    double tn3=pow(tn,3.);
    double tn4=tn*tn3;
    double tn5=tn*tn4;
    Xd = X0 + ( (X0-Xf) * (15.*tn4-6.*tn5-10.*tn3) );
    dXd = (X0-Xf) * (4.*15.*tn4-5.*6.*tn5-10.*3*tn3)/t;
    return tn;
}



void M2Calib::entryCode(void) {
    calibDone=false;
    for(unsigned int i=0; i<2; i++) {
        stop_reached_time[i] = .0;
        at_stop[i] = false;
    }
    robot->decalibrate();
    robot->initTorqueControl();
   // robot -> printStatus();
    robot->printJointStatus();
    std::cout << "Calibrating (keep clear)..." << std::flush;
}
//Move slowly on each joint until max force detected
void M2Calib::duringCode(void) {
    VM2 tau(0, 0);

    //Apply constant torque (with damping) unless stop has been detected for more than 0.5s
    VM2 vel=robot->getVelocity();
    double b = 3;
    for(unsigned int i=0; i<vel.size(); i++) {
        tau(i) = -std::min(std::max(20 - b * vel(i), .0), 20.);
        if(stop_reached_time(i)>1) {
            at_stop[i]=true;
        }
        if(abs(vel(i))<0.005) {
            stop_reached_time(i) += dt;
        }
    }

    //Switch to gravity control when done
    if(robot->isCalibrated()) {
        robot->setEndEffForceWithCompensation(VM2::Zero(), false);
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
void M2Calib::exitCode(void) {
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
    robot->setEndEffForceWithCompensation(VM2::Zero(), true);

    if(iterations%100==1) {
        robot->printStatus();
     //   robot->printJointStatus();
    }
}
void M2Transparent::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM2::Zero());
}


void M2ArcCircle::entryCode(void) {
    robot->initVelocityControl();
    //Initialise values (from network command) and sanity check
    //theta_s =
    //dTheta_t =
    //radius =
    //centerPt =
    theta_s = 180.0;
    dTheta_t = 10;
    radius = 0.4;
    centerPt[0] = 0.4;
    centerPt[1] = .0;

    thetaRange=90;
    ddTheta=200;

    //Arc starting point
    finished = false;
    theta = theta_s;
	startingPt[0]=centerPt[0]+radius*cos(theta_s*M_PI/180.);
	startingPt[1]=centerPt[1]+radius*sin(theta_s*M_PI/180.);
	//Initialise profile timing
	double t_init = 1.0; //waiting time before movement starts (need to be at least 0.8 because drives have a lag...)
	double t_end_accel = t_init + dTheta_t/ddTheta; //acceleration phase to reach constant angular velociy
	double t_end_cstt = t_end_accel + (thetaRange-(dTheta_t*dTheta_t)/ddTheta)/dTheta_t; //constant angular velocity phase: ensure total range is theta_range
	double t_end_decel = t_end_cstt + dTheta_t/ddTheta; //decelaration phase

	//Define sign of movement based on starting angle
	sign=1;
	if(theta_s>90)
		sign=-1;
}
void M2ArcCircle::duringCode(void) {

    //Define velocity profile phase based on timing
    double dTheta = 0;
    VM2 dXd, Xd, dX;
    double t = elapsedTime;
    if(t<t_init)
    {
        dTheta=0;
    }
    else
    {
        if(t<t_end_accel)
        {
            //Acceleration phase
            dTheta=(t-t_init)*ddTheta;
        }
        else
        {
            if(t<=t_end_cstt)
            {
                //Constant phase
                dTheta=dTheta_t;
            }
            else
            {
                if(t<t_end_decel)
                {
                    //Deceleration phase
                    dTheta=dTheta_t-(t-t_end_cstt)*ddTheta;
                }
                else
                {
                    //Profile finished
                    dTheta=0;
                    finished = true;
                }
            }
        }
    }
    dTheta*=sign;

    //Integrate to keep mobilisation angle
    theta += dTheta*dt;

    //Transform to end effector space
    //desired velocity
    dXd[0] = -radius*sin(theta*M_PI/180.)*dTheta*M_PI/180.;
    dXd[1] = radius*cos(theta*M_PI/180.)*dTheta*M_PI/180.;
    // dXd[0] = 0.1;
    // dXd[1] = 0.1;
    //desired position
    Xd[0] = centerPt[0]+radius*cos(theta*M_PI/180.);
    Xd[1] = centerPt[1]+radius*sin(theta*M_PI/180.);
    //PI in velocity-position
    /**
    *what does K mean?
    *what does the below formula mean?
    */
    float K=5.0;
    //dX=dXd;
     dX = dXd + K*(Xd-robot->getEndEffPosition());

/**
* set both 0.1 to test joint movement
*/
//dX[0]=0.1;
//dX[1]=0.1;

    //Apply
    robot->setEndEffVelocity(dX);

    if(iterations%100==1) {
        std::cout << dXd.transpose() << "  ";
        robot->printStatus();
    }
}
void M2ArcCircle::exitCode(void) {
    robot->setEndEffVelocity(VM2::Zero());
}



void M2Recording::entryCode(void) {
    robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM2::Zero());

    //Define Variables
    RecordingPoint=0;
    //PositionRecording=robot->getEndEffPosition();
    //PositionTesting[10000];

}
void M2Recording::duringCode(void) {
    //Apply 0 force
    robot->setEndEffForceWithCompensation(VM2::Zero());

	//Record stuff...
	PositionRecording=robot->getEndEffPosition();
	PositionTesting[RecordingPoint]=PositionRecording;
	RecordingPoint++;

    if(iterations%100==1) {
        robot->printStatus();}
}

void M2Recording::exitCode(void) {
	//Identify circle??
	/**
	what we care about
	theta_s
	radius
	center point
	*/

    robot->setEndEffForceWithCompensation(VM2::Zero());
}



void M2MinJerkPosition::entryCode(void) {
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
void M2MinJerkPosition::duringCode(void) {

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
void M2MinJerkPosition::exitCode(void) {
    robot->setJointVelocity(VM2::Zero());
}



