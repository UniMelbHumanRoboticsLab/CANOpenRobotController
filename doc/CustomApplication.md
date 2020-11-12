# Building a custom application (with a custom state machine)

Each CORC application is a dedicated state machine with its own states which have access to your specific robot.


## Setup

To create a custom application with custom state machine, two things must be done. 

First, the source code itself must be developed, by deriving the StateMachine class can be to a custom one, named `MyCustomStateMachine`, in files named `MyCustomStateMachine.h/cpp`. These must be placed in a dedicated subfolder in the apps folder with a consistent name (e.g. `MyCustomStateMachine` must be used for both the folder and the file names)

Secondly, CMakeLists.txt must be edited and the entry `set (STATE_MACHINE_NAME "ExoTestMachine")` changed to `set (STATE_MACHINE_NAME "MyCustomStateMachine")`.

That's it, simply use the same build procedure.

> Note: It is recommended to use one of the existing application (X2DemoMachine or M3DemoMachine) as a template for your new state machine: simply copy the app folder closest to your application and follow the steps above.

> Note: Additionnal code can be placed in the state machine folder. Every file placed in this folder (and subfolders) named in {.c, .cpp, .h}  will be compiled.


## State machine structure and custom states and transitions

CORC provides a structured way to build event driven Finite State Machines.

The execution flow of a typical state machine with two states A and B is shown on this diagram and detailed below.
   
   ![CORC State Machine Diagram](img/CORCStateMachineExecutionDiagram.png)
   
Yellow parts highlights the methods which needs to be overriden with custom application code whereas Green blocks represents transitions that needs to be setup.

### States

Each state is a custom class, derived from the generic `State`. It contains three main methods which should be overidden to include your custom code:
- `virtual void entry()` which is called once when entering the state
- `virtual void during())` which is called repeatidly by the main control loop and which manage the normal control execution. The code within this state should be executable within less time than the sampling period.
- `virtual void exit()` which is called once when exiting the state (either by transition or when program ends).

For example the SittingDown class in the ExoTestmachine:

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

States objects to be used should then be defined in your state machine constructor and an initialisation State should be selected:
```C++
MyCustomStateMachine::MyCustomStateMachine() {
    robot = new X2Robot(); //Create the robot object to be used by the state machine

    stateA = new StateAState(this, robot); //Create a StateA object
    stateB = new StateBState(this, robot); //Create a StateB object
    
    ...
    
    StateMachine::initialize(stateA); //Define which state to call at start of state machine
}
```

### Events and transitions

Events are used to trigger transitions from one state to another. They could be based on a sensor trigger, timer, internale state... 
Programmatically, CORC represents events as single method objects with a boolean method `virtual bool check()` which should return true when the transition(s) should happen.

To create a new event:
- Define a new event within your state machine class: `EventObject(AtoBevent_t) * AtoBevent;`
- Instantiate it in your constructor: `AtoBevent = new AtoBevent_t(this);`
- Define it's trigger boolean method:
```C++
bool MyCustomStateMachine::AtoBevent::check() {
    if (OWNER->robot->keyboard.getA() == true) {
        return true;
    }
}
```

Once events have been declared, they can be used to trigger transitions between states. This is done by creating a new transition using the MACRO `NewTransition(_from_, _event_, _to_)` :
```C++
    NewTransition(stateA, AtoBevent, stateB);
    NewTransition(stateB, backToBevent, stateA);
```

Note that the same event can be used by multiple transitions. Multiple transitions (with different events) can also be used for the same state: the go to state being selected by the event being triggered.

Finally a complete state machine constructor look like:

```C++
MyCustomStateMachine::MyCustomStateMachine() {
    robot = new X2Robot(); //Create the robot object to be used by the state machine

    stateA = new StateAState(this, robot); //Create a StateA object
    stateB = new StateBState(this, robot); //Create a StateA object
    
    AtoBevent = new AtoBevent_t(this);
    backToBevent = new backToBevent_t(this);
    
    NewTransition(stateA, AtoBevent, stateB);
    NewTransition(stateB, backToBevent, stateA);
    
    StateMachine::initialize(stateA); //Define which state to call at start of state machine
}
```


## Logging

  CORC relies on [spdlog](https://github.com/gabime/spdlog) for all logging purpose. Two loggers are available:
  
### Generic debug and information messages
  
This log is organised in several level of priorities: TRACE < DEBUG < INFO < WARN < ERROR < CRITICAL which are used in the following cases:
- TRACE: reserved for CAN level information
- DEBUG: any execution information relevant for debug purpose only
- INFO: general execution information
- WARN: general unexpected execution information (little use)
- ERROR: any error not directly leading to termination
- CRITICAL: error leading to termination
    
This log will produce outputs both on console (`cout`) and within a rotating log file (logs/CORC.log).
    
When compiling your application you can select the desired logging level in the [CMakeLists.txt](../CMakeLists.txt) by setting CORC_LOGGING_LEVEL to one of the above value: 
keeping `set(CORC_LOGGING_LEVEL INFO)` (recommended level) will produce only INFO, WARN, ERROR and CRITICAL outputs for example.
    
To use this logger in your code, use one of the dedicated function depending on the desired level: 
```C++
spdlog::trace("Trace log {}", my_value);
...
spdlog::critical("Critical error log {} => {}", my_critical_value, my_other_critical_value);
```
These functions are accessible at any point in CORC.
    
> Note: Use this log wisely to not break the realtimness of the execution. It is also recommended to run CORC with a log level > INFO when trace and debug information are not required. For state logging, please prefere the dedicated logger described below.
  
  
### State machine execution logging
  
This log allows you to record the states of the robot, sensors or any other application specific information at every iteration in a dedicated file. The logger is accessible in the state machine and should be initialised as follows within the `MyCustomStateMachine::init()`
  
  ```C++
    logHelper.initLogger("test_logger", "logs/logexample.csv", LogFormat::CSV, true);
    logHelper.add(robot_->getPosition(), "JointPositions");
    logHelper.add(robot_->getVelocity(), "JointVelocities");
    logHelper.add(robot_->getTorque(), "JointTorques");
    logHelper.startLogger();
  ```

This example will log the robot joint positions, velocities and torques in `logs/logexample.csv` at every loop execution. 
  
The logger support any basic types and Eigen vectors. References to values to log should all be registered (using `logHelper.add()`) before starting the logger (`logHelper.startLogger()`) and these references should be valid during the entire statemachine execution.
  
Additionnaly, the logger should be properly closed at the end of the state machine execution, within `MyCustomStateMachine::end()`:
  ```C++
    if(logHelper.isStarted())
       logHelper.endLog();
  ```

> Note: implementation examples of this logger are available in the X2DemoMachine and M3DemoMachine.


## Extending

### Third-party libraries

Additionnal third-party libraries required in your code can either directly be placed in the lib folder for headers-only libraries or added as packages in the CMakeList.


### ROS

To use ROS in your application, see this [specific page](Simulation.md).
