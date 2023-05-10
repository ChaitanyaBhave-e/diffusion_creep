import matplotlib.pyplot as plt
import csv
import numpy as np

#PLOT FORMAT SETTINGS
plt.rcParams.update({'font.family':'Arial'})
plt.rc('font', family='sans-serif',weight='bold')

#FUNCTION FOR READING CSV FILES AND EXTRACTING LABELS AND RAW DATA FROM THEM --> USES PYTHON CSV MODULE
def getRawData(fileName, delim): # extract raw data from csv file
    rawData = []
    with open(fileName, 'r') as f:
        CSVReader = csv.reader(f, delimiter = delim, skipinitialspace = True)
        labels = next(CSVReader)
        array_size = len(labels)
        rawData = [ [] for i in range(array_size)]
        for row in CSVReader:
            for (i,val) in enumerate(row):
                rawData[i].append(float(val) )
    return (labels,rawData)

fig = plt.figure(1,figsize=(3,2),dpi=300)
# # Create the Axes.
# ax = fig.add_subplot(1,1,1)

#READ CSV FILE
# csv_file = 'test_out.csv'
# labels,data = getRawData(csv_file,',')

# time = data[labels.index('time')]

# dy = data[labels.index('dy')]
# creep = (np.asarray(dy[1:]) - dy[1])/5.0

# plt.plot(time[1:],creep,'r',label='Sink term')

#READ CSV FILE
csv_file = 'test_micron_scale_out.csv' ##'test_dirichlet_bc_out.csv'
labels,data = getRawData(csv_file,',')

time = data[labels.index('time')]

dy = data[labels.index('dy')]
creep = (np.asarray(dy[1:]) - dy[1])/5.0

plt.plot(time[1:],creep,'k',label='Dirichlet BC')



plt.xlabel("Time (s)",fontweight='bold')
plt.ylabel(r"$\bf{\epsilon_{diff}}$")
plt.legend()

plt.tight_layout()
plt.show()