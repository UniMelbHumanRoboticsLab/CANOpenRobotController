# Creating CPU utilization log from command line (LINUX)

## determine PID of EXO_APP_2020

`pidof EXO_APP_2020`

## take returned PID and pipe top output to file - this number varies each time the app starts

## -p : only display top infor that PID process

## -b : batch mode

## -d : Delay i.e. intervals between iterations (not needed for single iteration)

#### sample every second

`top -p <PID> -b -d 1 > top.csv`
ex.
`top -p 2295 -b -d 1 > top.csv`

## Sample only PID info lines to a file

`cat top.txt | grep <PID> > greplin.csv`

## Display only CPU utilization from each line and pipe to file w/ nl for time period of each data point

#### cut -c (columns): can be found looking at top.txt in standard text editor (for any other entry)

#### nl adds number line to each line entry

`cat top.csv | grep <PID> | cut -c 48-52| nl >> cpuLog.csv`
`cat top.csv | grep 2205| cut -c 48-52| nl >> cpuLog.csv`

## Output file format :

```
1 29.0
2 30.4
3 28.9
.
.
.
```
