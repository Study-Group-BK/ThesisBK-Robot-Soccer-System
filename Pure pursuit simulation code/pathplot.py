import configparser
import math
import numpy as np
import cv2 
import time
import matplotlib.pyplot as plt


x=[]
y=[]

posx=[]
posy=[]

config = configparser.ConfigParser()
config.read("config.ini")
with open(config["FILE"]["FILE_LOCATION"]) as file:
    path = [([float(x) for x in line.split(",")]) for line in file.readlines()]

for i,w in enumerate(path):

    x.append(path[i][0])
    y.append(path[i][1])


config.read("config.ini")
with open(config["FILE"]["FILE_LOCATION_POS"]) as file:
    pos = [([float(a) for a in line.split(",")]) for line in file.readlines()]



for i,w in enumerate(pos):
    posx.append(pos[i][0])
    posy.append(pos[i][1])
    

fig, ax = plt.subplots()
ax.plot(x,y, label ='path')
ax.plot(posx,posy, label ='robot')



ax.set(xlabel='x mm', ylabel= 'y mm',
       title='robot position')\

plt.legend()
ax.grid()

fig.savefig("test.png")
plt.show()