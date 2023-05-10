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

fig, (ax1, ax2) = plt.subplots(2)
# fig = plt.figure(1,figsize=(3,2),dpi=300)
# # Create the Axes.
# [ax1,ax2] = fig.add_subplot(2,1,1)

X = [200,100,50]
Y = []

#READ CSV FILE
csv_file = 'test_neumann_bc_out.csv'
labels,data = getRawData(csv_file,',')

time = data[labels.index('time')]

dy = data[labels.index('dy')]
creep = (np.asarray(dy[1:]) - dy[1])/100.0
Y+=[creep[-1]]

ax1.plot(time[1:],creep,'k',label='constant diff')

csv_file = 'dirichlet_with_gb_diff_out.csv'
labels,data = getRawData(csv_file,',')

time = data[labels.index('time')]

dy = data[labels.index('dy')]
creep = (np.asarray(dy[1:]) - dy[1])/1000.0
Y+=[creep[-1]]

# ax1.plot(time[1:],creep,'r--',label='GB interp diff')

# #READ CSV FILE
# csv_file = '100_mpa.csv'
# labels,data = getRawData(csv_file,',')

# time = data[labels.index('time')]

# dy = data[labels.index('dy')]
# creep = (np.asarray(dy[1:]) - dy[1])/1000.0
# Y+=[creep[-1]]

# ax1.plot(time[1:],creep,'r',label='100 MPa')

# #READ CSV FILE
# csv_file = '50_mpa.csv'
# labels,data = getRawData(csv_file,',')

# time = data[labels.index('time')]

# dy = data[labels.index('dy')]
# creep = (np.asarray(dy[1:]) - dy[1])/1000.0
# Y+=[creep[-1]]

# Y = np.asarray(Y)/time[-1]

# ax1.plot(time[1:],creep,'b',label='50 MPa')

# ax1.set_xlabel("Time (s)",fontweight='bold')
# ax1.set_ylabel(r"$\bf{\epsilon_{diff}}$")
# ax1.legend()

# ax2.plot(X,Y,'k*')


# P = np.polyfit(X,Y,1)
# ax2.plot(X,np.polyval(P,X),'r--')
# ax2.set_xlabel("Applied tensile stress (MPa)",fontweight='bold')
# ax2.set_ylabel(r"$\bf{\dot\epsilon_{diff}} (s^{-1})$")
# ax2.legend()


fig.tight_layout()
plt.show()