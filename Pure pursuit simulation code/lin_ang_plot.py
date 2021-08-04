import configparser
import math
import numpy as np
import cv2 

import matplotlib.pyplot as plt


linear_vel=[]
ang_vel=[]
target_vel=[]

config = configparser.ConfigParser()
config.read("config.ini")

with open(config["FILE"]["FILE_LOCATION_VEL"]) as file:
    vel = [([float(x) for x in line.split(",")]) for line in file.readlines()]

with open(config["FILE"]["FILE_LOCATION_TARGET_VEL"]) as file:
    target = [([float(t) for t in line.split(",")]) for line in file.readlines()]


for i, w in enumerate(target):
    target_vel.append(target[i][0]*0.001)



for i,w in enumerate(vel):
    
    linear_vel.append(((vel[i][0]*0.001)+(vel[i][1]*0.001))/2)
    ang_vel.append(((vel[i][1]*0.001)-(vel[i][0])*0.001)/(float(config["ROBOT"]["TRACKWIDTH"])*0.001))



total_time=len(vel)*float(config["ROBOT"]["SAMPLING_TIME"])
total_time_target=len(target)*float(config["ROBOT"]["SAMPLING_TIME"])


time=[]
time_target=[]

for i in range(0,len(vel)):
    time.append((total_time/len(vel))*i)   

for i in range(0,len(target)):
    time_target.append((total_time_target/len(target))*i)   
 


ax = plt.subplot(2, 1, 1)

plt.plot(time_target,target_vel, label='target')
plt.plot(time,linear_vel, label='linear')
plt.legend()
plt.xlabel("time s")
plt.ylabel("m/s")
plt.grid(True)

# Demonstrate some more complex labels.
ax = plt.subplot(2, 1, 2)
plt.plot(time,ang_vel,label='angular')
plt.legend()
plt.xlabel("time s")
plt.ylabel("rad/s")
plt.grid(True)

plt.show()









