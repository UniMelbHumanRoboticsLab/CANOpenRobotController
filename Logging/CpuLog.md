# Creating CPU utilization log from command line (LINUX)
This is a guide for a basic CPU utilisation experiment for a CORC Application. 

It takes output from the default Linux command `top` - sampling the current state of the system at regular intervals, and reduces the resulting output to only the CPU utilisation for the given application. This can then be post-processed to find an average across those intervals. 

## Determine PID of EXO_APP_2020
`top` requires the Process Identification (PID) of the application under investigation. This can be found with the following command.

`pidof EXO_APP_2020`

We refer to the result of this as `<PID>`. 

## Use of the top command
We use the following options for the top command: 

-p : only display top information for the specified PID process
-b : batch mode (this formats the output such that it can be written to file)
-d : Delay i.e. intervals between iterations

The suggested command format is as follows:

`top -p <PID> -b -d 0.1 > top.txt`

This produces a file which logs the state of the machine at 1 second intervals. This interval can be changed by changing the `0.1` parameter. 

This produces an output similar to that below (with the PID set to 2304):

```
top - 22:54:25 up  5:18,  3 users,  load average: 1.07, 0.71, 0.60
Tasks:   1 total,   0 running,   1 sleeping,   0 stopped,   0 zombie
%Cpu(s):  0.5 us,  0.6 sy,  0.0 ni, 98.9 id,  0.0 wa,  0.0 hi,  0.0 si,  0.0 st
KiB Mem :   494876 total,   328284 free,    71448 used,    95144 buff/cache
KiB Swap:        0 total,        0 free,        0 used.   406028 avail Mem 

  PID USER      PR  NI    VIRT    RES    SHR S %CPU %MEM     TIME+ COMMAND
 2304 root      20   0   19572   1120   1016 S 18.8  0.2   0:11.43 EXO_APP_2020

top - 22:54:25 up  5:18,  3 users,  load average: 1.07, 0.71, 0.60
Tasks:   1 total,   0 running,   1 sleeping,   0 stopped,   0 zombie
%Cpu(s): 50.0 us,  6.2 sy,  0.0 ni, 43.8 id,  0.0 wa,  0.0 hi,  0.0 si,  0.0 st
KiB Mem :   494876 total,   328284 free,    71448 used,    95144 buff/cache
KiB Swap:        0 total,        0 free,        0 used.   406028 avail Mem 

  PID USER      PR  NI    VIRT    RES    SHR S %CPU %MEM     TIME+ COMMAND
 2304 root      20   0   19572   1120   1016 S 27.3  0.2   0:11.46 EXO_APP_2020

top - 22:54:25 up  5:18,  3 users,  load average: 1.07, 0.71, 0.60
Tasks:   1 total,   0 running,   1 sleeping,   0 stopped,   0 zombie
%Cpu(s): 57.1 us,  0.0 sy,  0.0 ni, 42.9 id,  0.0 wa,  0.0 hi,  0.0 si,  0.0 st
KiB Mem :   494876 total,   328284 free,    71448 used,    95144 buff/cache
KiB Swap:        0 total,        0 free,        0 used.   406028 avail Mem 

  PID USER      PR  NI    VIRT    RES    SHR S %CPU %MEM     TIME+ COMMAND
 2304 root      20   0   19572   1120   1016 S 20.0  0.2   0:11.48 EXO_APP_2020
```

## Sample only PID info lines to a file

The following command removes all lines apart from the ones which include the PID. Note that it's possible that this also catches some 'incidental' (e.g. if your PID is 1, it will capture all lines with a 1 in it --- this can optimised with a better command).   

`cat top.txt | grep <PID> > topPIDonly.txt`

This produces an output similar to below:

```
 2304 root      20   0   19572   1120   1016 S 18.8  0.2   0:11.43 EXO_APP_2020
 2304 root      20   0   19572   1120   1016 S 27.3  0.2   0:11.46 EXO_APP_2020
 2304 root      20   0   19572   1120   1016 S 20.0  0.2   0:11.48 EXO_APP_2020
 2304 root      20   0   19572   1120   1016 S 18.2  0.2   0:11.50 EXO_APP_2020
 2304 root      20   0   19572   1120   1016 S 20.0  0.2   0:11.52 EXO_APP_2020
 ```

## Display only CPU utilization from each line and pipe to file w/ nl for time period of each data point

We can combine this with `cut -c (columns)` command to take only the relevant lines (those associated with CPU usage) - note that the relevant columns (48-52) have been manually specified. 

`cat top.txt | grep <PID> | cut -c 48-52| nl >> cpuLog.txt`

(nl adds number line to each line entry - this is optional, and it may be easier to process if you do not use this line).

This produces an output similar to below, which can then be processed to calculate the average CPU utilisation. 

```
1	18.8 
2	27.3 
3	20.0 
.
.
.
```
