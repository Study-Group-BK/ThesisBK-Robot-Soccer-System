import configparser
import math
import numpy as np
import cv2 

import matplotlib.pyplot as plt

direction=[]


config = configparser.ConfigParser()
config.read("config.ini")
with open(config["PATH"]["FILE_LOCATION_DIRECTION"]) as file:
    dir = [([float(x) for x in line.split(",")]) for line in file.readlines()]

for i,w in enumerate(dir):

    direction.append(dir[i][0])
   
    
total_time=len(dir)*float(config["ROBOT"]["SAMPLING_TIME"])

time=[]

for i in range(0,len(dir)):
    time.append((total_time/len(dir))*i)   

 
print(time[0])

fig, ax = plt.subplots()
ax.plot(time,direction,label='angle')

ax.set(xlabel='time (s)', ylabel='wheel velocity m/s',
       title='left and right wheel velocity')
plt.legend()
ax.grid()
fig.savefig("test.png")
plt.show()


