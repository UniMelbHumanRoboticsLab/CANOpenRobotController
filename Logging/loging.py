import pandas as pd

df = pd.read_csv(
    r'/Users/williamcampbell/Documents/GitHub/computerVission/Testing/log.csv')
print(df)
meanControlLoop = df['Loop_time_sec'].mean()
medianControlLoop = df['Loop_time_sec'].median()
maxControlLoop = df['Loop_time_sec'].max()
minControlLoop = df['Loop_time_sec'].min()
countControlLoop = df['Loop_time_sec'].count()
stdControlLoop = df['Loop_time_sec'].std()
varControlLoop = df['Loop_time_sec'].var()

print('Mean: {}\n Max: {}\n Min: {}\n std: {}\n for {} samples\n'.format(
    meanControlLoop, maxControlLoop, minControlLoop, stdControlLoop, countControlLoop))
