# Designing Finite State Machines using CORC

## What is a Finite State Machine (FSM)

- A reactive system whose response to a particular stimulus (a signal, or a piece of input) is not the same on every occasion, depending on its current “state”.
  - For example, in the case of a Vending machine, it will not despense a chocolate bar when you make a selection unless you have already inserted the correct amount of money. Thus, the response to the selection button depends on the previous history of the use of the system.

## Why FSM in Robotics

- Robot behaviour is inherently reactive. Response to an event at any moment, is dependent on the “state” of the robot.
- Safety: To avoid states (motion or behaviour) with disastrous consequences, FSM only allow pre-determined state transitions.
  - Never begin a motion unless in a safe starting position.
  - Never keep rocket engines on for more than n seconds.
  - (Therac-25 accident)[An investigation of the Therac-25 accidents - IEEE Journals & Magazine](https://ieeexplore.ieee.org/document/274940)

## How to Structure State Machine Code

CORC provides a structured way to build event driven FSMs. The following explains how to represent states and events and how to put them together into a CORC state machine. It is best practice to start with a state transition diagram and build each component individually. We will use the ExoTestMachine example provided in the CORC library [link](https://github.com/UniMelbHumanRoboticsLab/CANOpenRobotController/blob/master/src/apps/stateMachine/ExoTestMachine.cpp) presented bellow.

![exoTestMachine](https://exoembedded.readthedocs.io/en/latest/img/exoTestMachine.png)

- An Input Device (e.g. keyboard, controller etc) triggers Events. The desired behaviour is to move the exoskeleton between a sitting position and standing position. Using two moving states (sittingDwn and standingUp. We don't care how those motions happen, only the separation of individual State. This both seperates programatically and prevents unwanted behaviours.

## States

- States can either have an associated action with them (sitting Down), or no action (sitting), which differ them from each other.
- Programatically we design each state as a separate class which must implement the following three functions.
- The presiding Statemachine calls entry, exit and during their existence. In this view a state can be thought of as the description and behaviour of a system that is waiting to execute a transition.

```C++
      virtual void entry(void);
            -> 1 time set ups
        virtual void durring(void);
            -> Called by update method of the state machine
            -> *CONTINUOUS UPDATE AND CONTROL LIVES HERE*
                *MOTOR CONTROL*
        virtual void exit(void);
            -> transition out of the state triggers this, release resources.
```

For example the SittingDown class:

```C++
void SittingDwn::entry(void) {
    cout << " GREEN -> SIT DOWN " << endl
    trajectoryGenerator->initialiseTrajectory(SIT, 1);
    robot->startNewTraj();
}
void SittingDwn::during(void) {
    robot->moveThroughTraj();
}
void SittingDwn::exit(void) {
    std::cout << "Sitting Down State Exited " << endl;
}
```

## Events

Events could a sensor trigger, timer finishing, end of a trajectory or anything in between.
When an event happens:

- The event is checked against the **current state**’s transitions.
- If a transition matches the event, that transition “happens”.
- By virtue of a transition “happening”, states are **exited**, and **entered** and the relevant actions are performed
- The machine immediately **is in** the new state, ready to process the next event.
- CORC represents events as single method objects comprising as Boolean functions `virtual Boolean method: check()`. For example:

```C++
bool ExoTestMachine::EndTraj::check() {
    return OWNER->trajectoryGenerator->isTrajectoryFinished();
}
bool ExoTestMachine::IsAPressed::check(void) {
    if (OWNER->robot->keyboard.getA() == true) {
        return true;
    }
    return false;
}
```

## Transitions

A transition represents a shift from one State to another, triggered by a specific Event (arrow in the above diagram).

- Transition objects have a pointer to the Event and the toState, which for a transition object is called a target. Simply a target State triggered by an Event.
- Programatically this is captured by the fromState holding all possible transition objects in an archList. A list of all possible transitions this state can transition too. - The overlaying State machine asks the current state to check its Archlist for any Triggered transitions. If a transition event is triggered, the transition object provides the stjatemachine a pointer to its ToState. This now becomes the current state.
  Define each transition by specifying :
  - FromState - the starting state for this transition
  - ToState - the end state for this transition
  - Event( Condition) - a callable which when it returns True means this transition is valid
  - Use `NewTransition(_from_, _event_, _to_)` MACRO to achieve this.

```C++
\*Define sitting transitions*\
    NewTransition(initState, startExo, sitting);
    NewTransition(sitting, startStand, standingUp);
```

## Putting everything together in a stateMachine class.

The base state machine class relies on cyclic calls to update(). - Each statemachine starts with a call to initialize(state) setting the currentState pointer to the fed in state object, - After the initialization call each loop of a program must cyclically call statemachine.update() and a program will transition from state to state based on the events which occur. - The update() function achieves this with two calls - Checks the currentStates ArcList for any triggered Transitions (that transition objects Event). - If an Event has been triggered, a sequence of calls exits the current state and sets the target to the current state. - Else if no Event triggers a transition, the currentStates during function is called. - This sequence is repeated until program exit.

```C++
void StateMachine::update()
{
    Transition *t = currentState->getActiveArc();
    if (t != NULL)
    {
        currentState->exit();
        currentState = t->target;
        currentState->entry();
    }
    currentState->during();
}
```

## Designing your own stateMachine

Following the ExoTestMachine example [link](https://github.com/UniMelbHumanRoboticsLab/CANOpenRobotController/blob/master/src/apps/stateMachine/ExoTestMachine.cpp) , the following steps should yield a functional state machine.

1. Design a state diagram with states, events and transitions (states may have more than one possible transition)
2. Create each state classes three virtual functions following the above SittingDwn.cpp as example
3. In your Statemachine class create pointers to state objects and populate with objects: `InitState *initState;`  
   &`initState = new InitState(*this*, robot, trajectoryGenerator);`
4. Use the EventObject(_name_) MACRO to create and initialize Event objects `EventObject(IsAPressed) * isAPressed;` & `isAPressed = new IsAPressed(*this*);`
5. Define Event object check() functions as above (in Statemachine class)
6. Pass your first state to the Base StateMachine `initialize(state)` function -> `StateMachine::initialize(initState)`
7. In your main program cyclically call `<yourStaeMachine>.update()`
