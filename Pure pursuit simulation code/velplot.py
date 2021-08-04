import configparser
import math
import numpy as np
import cv2 

import matplotlib.pyplot as plt


left_vel=[]
right_vel=[]

config = configparser.ConfigParser()
config.read("config.ini")
with open(config["FILE"]["FILE_LOCATION_VEL"]) as file:
    vel = [([float(x) for x in line.split(",")]) for line in file.readlines()]

for i,w in enumerate(vel):

    left_vel.append(vel[i][0]*0.001)
    right_vel.append(vel[i][1]*0.001)
    
total_time=len(vel)*float(config["ROBOT"]["SAMPLING_TIME"])

time=[]

for i in range(0,len(vel)):
    time.append((total_time/len(vel))*i)   

 
print(time[0])

fig, ax = plt.subplots()
ax.plot(time,left_vel,label='left')
ax.plot(time,right_vel,label='right')
ax.set(xlabel='time (s)', ylabel='wheel velocity m/s',
       title='left and right wheel velocity')
plt.legend()
ax.grid()
fig.savefig("test.png")
plt.show()


