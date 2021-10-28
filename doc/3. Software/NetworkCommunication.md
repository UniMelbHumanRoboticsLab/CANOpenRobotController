# Network communication (using FLNL)

## Purpose
[libFLNL](https://github.com/vcrocher/libFLNL) and the FLNLHelper class can be used to established a TCP/IP communication between CORC (server) and a client. It is intended to:
- Publish the robot state, typically at the frequency of the controller
- Send and receive short commands with double value parameters

## Usage
To use it in a CORC state machine:
- Declare an FLNLHelper in your state machine class: `FLNLHelper *UIserver;   /*!< Pointer to communication server*/`
- Instantiate and initialise it in your init method: 
```
	UIserver = new FLNLHelper("192.168.7.2");       //Instantiate object and open communication (waiting for client to connect). IP adress is server address to listen on.

	registerState(myTime);                          //Reference to a time value (double)
	registerState(robot->getEndEffPositionRef());   //Reference to an Eigen vector
	registerState(myStdVector);                     //Reference to an std::vector<double>
```
- Alternatively you can also initialise it with an helper constructor: `UIserver = new FLNLHelper(robot, "192.168.7.2");` which will automatically register the time and robot state (position, velocity and torque) to be sent at each update.
- Call the update method to send the states over the network regularlry, typically in your hwStateUpdate method: `UIserver->sendState();`
- Received command can be checked and process at any point. Example in M3DemoMachine::GoToNextState::check():
```
if ( OWNER->UIserver->isCmd() ) {
        string cmd;
        vector<double> v;
        OWNER->UIserver->getCmd(cmd, v);
        if (cmd == "GTNS") { //Go To Next State command received
            //Acknowledge
            OWNER->UIserver->sendCmd(string("OK"));
            return true;
        }
    }
```
- Similarly, commands can be sent to the client at any point: `UIserver->sendCmd(string cmd, std:vector<double> param)`.

## Examples
See M3DemoMachine for an example.

Examples of Unity-C# and Matlab clients are available [here](https://github.com/UniMelbHumanRoboticsLab/CORC-UI-Demo).

## Important notes

> Note 1: FLNL is design to accept only one client per server.

> Note 2: FLNL is meant to be asynchronous and real-time: if the client and server are running at different frequencies, no buffering is performed. Only the last received command and the last received values will be available.

> Note 3: Commands are currently restricted to 4 characters and only 30 values/parameters can be sent at once.

> Note 4: By default the communication is established on Port 2048. This can be specified in the constructor.

