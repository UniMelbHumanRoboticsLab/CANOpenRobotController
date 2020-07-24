# CORC Project Application Files

Contained in this directory are an example implementation of a CORC State Machine
utilizing the Robot, associated classes and CANOpen network Stack.

The example statemachine can be found in the stateMachine folder (ExoTestMachine.cpp).
The test application is created following instructions in the root directories README.

   ExoTestMachine State transition Diagram.
``` bash
          startExo             startStand
   initState +-----> sitting +---------> standingUp
                       ^                  +
            EndTraj    |                  | EndTraj
                       |                  |
                       +                  |
                  sittingDwn <---------+ standing
                               startSit
```
                               
