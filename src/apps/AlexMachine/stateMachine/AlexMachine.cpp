
#include "AlexMachine.h"

#define OWNER ((AlexMachine *)owner)

AlexMachine::AlexMachine()
{
    trajectoryGenerator = new AlexTrajectoryGenerator(6);
    robot = new AlexRobot(trajectoryGenerator);
    // Events
    endTraj = new EndTraj(this);
    startExo = new StartExo(this);
    feetTogether = new FeetTogether(this);
    standSelect = new StandSelect(this);
    sitSelect = new SitSelect(this);
    walkSelect = new WalkSelect(this);
    backStep = new BackStep(this);
    upStairSelect = new UpStairSelect(this);
    downStairSelect = new DownStairSelect(this);
    isRPressed = new IsRPressed(this);
    resetButtonsPressed = new ResetButtons(this);

#ifdef VIRTUAL
    debugTransition = new DebugTransition(this);
#endif

    //States
    initState = new InitState(this, robot, trajectoryGenerator);
    initialSitting = new InitialSitting(this, robot, trajectoryGenerator);
    standing = new Standing(this, robot, trajectoryGenerator);
    sitting = new Sitting(this, robot, trajectoryGenerator);
    standingUp = new StandingUp(this, robot, trajectoryGenerator);
    sittingDwn = new SittingDwn(this, robot, trajectoryGenerator);
    steppingFirstLeft = new SteppingFirstLeft(this, robot, trajectoryGenerator);
    leftForward = new LeftForward(this, robot, trajectoryGenerator);
    steppingRight = new SteppingRight(this, robot, trajectoryGenerator);
    rightForward = new RightForward(this, robot, trajectoryGenerator);
    steppingLeft = new SteppingLeft(this, robot, trajectoryGenerator);
    steppingLastRight = new SteppingLastRight(this, robot, trajectoryGenerator);
    steppingLastLeft = new SteppingLastLeft(this, robot, trajectoryGenerator);
    errorState = new ErrorState(this, robot, trajectoryGenerator);
    backStepLeft = new BackStepLeft(this, robot, trajectoryGenerator);
    backStepRight = new BackStepRight(this, robot, trajectoryGenerator);
    steppingRightStair = new SteppingRightStair(this, robot, trajectoryGenerator);
    steppingLeftStair = new SteppingLeftStair(this, robot, trajectoryGenerator);
    steppingRightStairDown = new SteppingRightStairDown(this, robot, trajectoryGenerator);
    steppingLeftStairDown = new SteppingLeftStairDown(this, robot, trajectoryGenerator);
#ifdef VIRTUAL
    debug = new DebugState(this, robot, trajectoryGenerator);
#endif
    /**
     * \brief Moving Trajectory Transitions
     *
     */
    NewTransition(initialSitting, endTraj, sitting);
    NewTransition(standingUp, endTraj, standing);
    NewTransition(sittingDwn, endTraj, sitting);
    /* Stepping*/
    NewTransition(steppingRight, endTraj, rightForward);
    NewTransition(steppingFirstLeft, endTraj, leftForward);
    NewTransition(steppingLeft, endTraj, leftForward);
    NewTransition(steppingLastRight, endTraj, standing);
    NewTransition(steppingLastLeft, endTraj, standing);
    /**
     * \brief Stationary to moving state transitions (OD.NM controls type of walk)
     *
     */
    NewTransition(initState, startExo, initialSitting);
    NewTransition(sitting, standSelect, standingUp);
    NewTransition(standing, sitSelect, sittingDwn);
    NewTransition(standing, walkSelect, steppingFirstLeft);
    NewTransition(leftForward, walkSelect, steppingRight);
    NewTransition(rightForward, walkSelect, steppingLeft);
    NewTransition(leftForward, feetTogether, steppingLastRight);
    NewTransition(rightForward, feetTogether, steppingLastLeft);
    /*Back Stepping transitions*/
    //Currently just a left backstep from standing
    /*\todo add backstep feet together (1/2 step x size) + starting w/back right step*/
    NewTransition(standing, backStep, backStepLeft);
    NewTransition(backStepLeft, endTraj, rightForward);
    NewTransition(rightForward, backStep, backStepRight);
    NewTransition(backStepRight, endTraj, leftForward);

#ifdef VIRTUAL
    NewTransition(initState, debugTransition, debug);
    NewTransition(debug, debugTransition, initState);
#endif

    /*Stair stepping transitions*/
    NewTransition(standing, upStairSelect, steppingLeftStair);
    NewTransition(steppingLeftStair, endTraj, leftForward);
    NewTransition(leftForward, upStairSelect, steppingRightStair);
    NewTransition(steppingRightStair, endTraj, standing);
    NewTransition(standing, downStairSelect, steppingLeftStairDown);
    NewTransition(steppingLeftStairDown, endTraj, rightForward);
    NewTransition(rightForward, downStairSelect, steppingRightStairDown);
    NewTransition(steppingRightStairDown, endTraj, standing);
    /**
     * \brief  Error State Transitions
     *
     */
    NewTransition(errorState, resetButtonsPressed, initState);
    NewTransition(sitting, isRPressed, errorState);
    NewTransition(standing, isRPressed, errorState);
    NewTransition(standingUp, isRPressed, errorState);
    NewTransition(sittingDwn, isRPressed, errorState);
    NewTransition(steppingFirstLeft, isRPressed, errorState);
    NewTransition(leftForward, isRPressed, errorState);
    NewTransition(steppingRight, isRPressed, errorState);
    NewTransition(rightForward, isRPressed, errorState);
    NewTransition(steppingLeft, isRPressed, errorState);
    NewTransition(steppingLastRight, isRPressed, errorState);
    NewTransition(steppingLastLeft, isRPressed, errorState);
    NewTransition(steppingRightStair, isRPressed, errorState);
    NewTransition(steppingLeftStair, isRPressed, errorState);
    NewTransition(steppingRightStairDown, isRPressed, errorState);
    NewTransition(steppingLeftStairDown, isRPressed, errorState);
    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(initState);
}
/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void AlexMachine::init()
{
    robot->initialise();
    running = true;
}

void AlexMachine::activate()
{
    StateMachine::activate();
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------------
///////////////////////////////////////////////////////////////
/**
     * \brief poll the trajectory Generators flag to see if the currently loaded motion is complete
     *
     */
bool AlexMachine::EndTraj::check()
{
    /*For alex Traj Generator*/
    if (OWNER->trajectoryGenerator->isTrajectoryFinished(OWNER->robot->getCurrTrajProgress()) && !OWNER->robot->getGo())
    {
        return true;
    }
    else
    {
        // testing with keyboard
        if (OWNER->robot->keyboard->getW() == true)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

bool AlexMachine::StartExo::check(void)
{
    if (OWNER->robot->keyboard->getD() == true)
    {
        spdlog::info("LEAVING INIT and entering init Sitting");
        return true;
    }
    else if (OWNER->robot->getCurrentMotion() == RobotMode::INITIAL && OWNER->robot->getGo())
    {
        spdlog::info("LEAVING INIT and entering init Sitting");
        return true;
    }
    return false;
}
bool AlexMachine::FeetTogether::check(void)
{
    if (OWNER->robot->keyboard->getA())
    {
        return true;
    }
    if (OWNER->robot->getCurrentMotion() == RobotMode::FTTG && OWNER->robot->getGo())
    {
        spdlog::debug("Feet together  selected by crutch!");
        return true;
    }
    return false;
}

bool AlexMachine::StandSelect::check(void)
{
    if (OWNER->robot->getResetFlag())
    {
        if (OWNER->robot->keyboard->getA())
        {
            spdlog::debug("Stand selected by keyboard! Begin standing up");
            return true;
        }
        if (OWNER->robot->getCurrentMotion() == RobotMode::STNDUP && OWNER->robot->getGo())
        {
            spdlog::debug("Stand selected by crutch! Begin standing up");
            return true;
        }
    }
    return false;
}
bool AlexMachine::SitSelect::check(void)
{
    if (OWNER->robot->getResetFlag())
    {
        if (OWNER->robot->keyboard->getA())
        {
            spdlog::debug("Sit selected! Begin standing up");
            return true;
        }
        else if (OWNER->robot->getCurrentMotion() == RobotMode::SITDWN && OWNER->robot->getGo())
        {
            spdlog::debug("Sit selected by crutch! Begin standing up");
            return true;
        }
        else
        {
            return false;
        }
    }
}
bool AlexMachine::WalkSelect::check(void)
{
    // \todo change to switch statement
    if (OWNER->robot->getCurrentMotion() == RobotMode::NORMALWALK && OWNER->robot->keyboard->getS())
    {
        spdlog::debug("Normal walk selected begin left step");
        return true;
    }
    else if (OWNER->robot->getCurrentMotion() == RobotMode::UNEVEN && OWNER->robot->getGo())
    {
        spdlog::debug("Uneven step selected begin left step");
        return true;
    }
    else if (OWNER->robot->getCurrentMotion() == RobotMode::TILTUP && OWNER->robot->getGo())
    {
        spdlog::debug("Ramp up step selected begin left step");
        return true;
    }
    else if (OWNER->robot->getCurrentMotion() == RobotMode::TILTDWN && OWNER->robot->getGo())
    {
        spdlog::debug("Ramp dwn step selected begin left step");
        return true;
    }
    else if (OWNER->robot->getCurrentMotion() == RobotMode::NORMALWALK && OWNER->robot->getGo())
    {
        spdlog::debug("Normal walk selected by crutch!");
        return true;
    }
    else
    {
        return false;
    }
}
bool AlexMachine::BackStep::check(void)
{
    if (OWNER->robot->getCurrentMotion() == RobotMode::BKSTEP && OWNER->robot->getGo())
    {
        spdlog::debug("Backstep selected by crutch!");
        return true;
    }
    else
    {
        return false;
    }
}
bool AlexMachine::UpStairSelect::check(void)
{
    if (OWNER->robot->getCurrentMotion() == RobotMode::UPSTAIR && OWNER->robot->getGo())
    {
        spdlog::debug("up stair step selected");
        return true;
    }
    else
    {
        return false;
    }
}
bool AlexMachine::DownStairSelect::check(void)
{
    if (OWNER->robot->getCurrentMotion() == RobotMode::DWNSTAIR && OWNER->robot->getGo())
    {
        spdlog::debug("down stair step selected");
        return true;
    }
    else
    {
        return false;
    }
}
bool AlexMachine::IsRPressed::check(void)
{
#ifdef KEYBOARD
    return OWNER->robot->keyboard->getR();
#endif
#ifndef KEYBOARD
    return OWNER->robot->buttons.getErrorButton();
#endif
}
bool AlexMachine::ResetButtons::check(void)
{
#ifdef KEYBOARD
    return OWNER->robot->keyboard->getR();
#endif
#ifndef KEYBOARD
    return !(OWNER->robot->buttons.getErrorButton());
#endif
}

#ifdef VIRTUAL
bool AlexMachine::DebugTransition::check(void)
{
    return OWNER->robot->keyboard->getS();
}
#endif

/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void AlexMachine::hwStateUpdate(void)
{
    robot->updateRobot();
}

void AlexMachine::configureMasterPDOs() {
    spdlog::debug("AlexMachine::configureMasterPDOs()");
    robot->configureMasterPDOs();
}

void AlexMachine::end() {
    spdlog::debug("Ending AlexMachine");
    delete robot;
}