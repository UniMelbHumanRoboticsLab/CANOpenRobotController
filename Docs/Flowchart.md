# Multi-threaded execution

To run application tasks and network tasks (all CANopen activity and messaging) the software runs three parallel threads. Two threads to maintain up to date CANopen processes and a third running the applications program loop. A flowchart of a typical CORC implementation can be found below.

## CANopenNode threads

Using the CANopenNode framework, CORC runs both a mainline thread for executing time-consuming CANopen operations (SDO messaging, Heartbeat, LSS, etc.) and a "realtime" thread for time-critical CANopen operations (PDO and SYNC messaging), running at a constant interval (typically 1ms).

For more details on the implementation of CANopenNode please go to the projects GitHub and documentation [page](https://github.com/CANopenNode/CANopenNode).

# Application thread

The application loop speed can be altered by the programmer, however, it must run slower than the CANopen Node rt loop. This ensures any PDO reliant commands to nodes from the application are processed and sent out on the bus.

<!-- \todo: test and document accurte method for max and min program loop speed plus associated issues when approaching max (jitter etc.)-->

# Flowchart of a typical CORC implementation

![Flow Chart](Docs/img/CORC_flow_chart.png)
