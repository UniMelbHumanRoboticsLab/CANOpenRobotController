# Logging

  CORC relies on [spdlog](https://github.com/gabime/spdlog) for all logging purpose. Two loggers are available:
  
## Generic debug and information messages
  
This log is organised in several level of priorities: TRACE < DEBUG < INFO < WARN < ERROR < CRITICAL which are used in the following cases:
- TRACE: reserved for CAN level information
- DEBUG: any execution information relevant for debug purpose only
- INFO: general execution information
- WARN: general unexpected execution information (little use)
- ERROR: any error not directly leading to termination
- CRITICAL: error leading to termination
    
This log will produce outputs both on console (`cout`) and within a rotating log file (logs/CORC.log).
    
When compiling your application you can select the desired logging level in the [CMakeLists.txt](../../CMakeLists.txt) by setting CORC_LOGGING_LEVEL to one of the above value: 
keeping `set(CORC_LOGGING_LEVEL INFO)` (recommended level) will produce only INFO, WARN, ERROR and CRITICAL outputs for example.
    
To use this logger in your code, use one of the dedicated function depending on the desired level: 
```C++
spdlog::trace("Trace log {}", my_value);
...
spdlog::critical("Critical error log {} => {}", my_critical_value, my_other_critical_value);
```
These functions are accessible at any point in CORC.
    
> Note: Use this log wisely to not break the realtimness of the execution. It is also recommended to run CORC with a log level > INFO when trace and debug information are not required. For state logging, please prefere the dedicated logger described below.
  
  
## State machine execution logging
  
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