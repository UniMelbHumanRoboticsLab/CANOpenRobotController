import pandas as pd
 # import beagle bone logged data
beagleLog = pd.read_csv(
    r'/Users/williamcampbell/Documents/GitHub/CANOpenRobotController/Logging/log.csv')
#print(beagleLog)
meanControlLoop = beagleLog['Loop_time_sec'].mean()
medianControlLoop = beagleLog['Loop_time_sec'].median()
maxControlLoop = beagleLog['Loop_time_sec'].max()
minControlLoop = beagleLog['Loop_time_sec'].min()
countControlLoop = beagleLog['Loop_time_sec'].count()
stdControlLoop = beagleLog['Loop_time_sec'].std()
varControlLoop = beagleLog['Loop_time_sec'].var()
print("=======================================")
print("             Control loop speed (sec)")
print("=======================================")
print('Mean: {}\n Max: {}\n Min: {}\n std: {}\n for {} samples\n'.format(
    meanControlLoop, maxControlLoop, minControlLoop, stdControlLoop, countControlLoop))
 # import CPU usage logged data
fields = ['time', 'CPU_u']
cpuLog = pd.read_csv(
    r'/Users/williamcampbell/Documents/GitHub/CANOpenRobotController/Logging/cpuLog.csv',engine='python',sep=None, usecols=[0,1], header = None)
#print(cpuLog)
meanCPU = cpuLog[1].mean()
meanCPU = cpuLog[1].mean()
medianCPU = cpuLog[1].median()
maxCPU = cpuLog[1].max()
minCPU = cpuLog[1].min()
countCPU = cpuLog[1].count()
stdCPU = cpuLog[1].std()
varCPU = cpuLog[1].var()
print("=======================================")
print("             CPU usage(% util)")
print("=======================================")

print('Mean: {}\n Max: {}\n Min: {}\n std: {}\n for {} samples\n'.format(
    meanCPU, maxCPU, minCPU, stdCPU, countCPU))