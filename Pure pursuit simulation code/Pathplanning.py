import cv2
import configparser
import math
import sys
import numpy as np
import cv2 

#  MAPPING FUNCTION (from 1000 by 1700 to 500 by 850 area)
def mapx(x):
    return int((x - -850) * (425 - -425) / (850 - -850) + -425)
def mapy(y):	
    return int((y - -500) * (250 - -250) / (500 - -500) + -250)




#READ CONFIG FILE
config = configparser.ConfigParser()
config.read("config.ini")
field = cv2.imread(config["FIELD_IMAGE"]["FILE_LOCATION"])
img = np.zeros((field.shape[0], field.shape[1], 3), np.uint8)

#GETTING INPUT (ROBOTS POSITION)
print('x position of the robot')
p0x = eval(input())
print('y position of the robot')
p0y = eval(input())
p0=(p0x,p0y)
#print(p0)


#GET ROBOT CURRENT DIRECTION
print('What is the direction the robot ')
ang = math.radians(eval(input())) #doi rad sang degree
dist = float(config["PATH"]["DIST"])  #distance can be choose freely (act as coefficient)
p1=(p0x+dist*math.cos(ang),p0y+dist*math.sin(ang))
#print(p1)

#GET BALL'S POSITION
print('x position of the ball')
pballx = eval(input())
print('y position of the robot')
pbally = eval(input())
pball=(pballx,pbally) #ball position
#print(pball)


if (pball[1]>0):    #if the ball is in the top half of the field
    b_ang=math.degrees(math.atan((abs(0-pball[1])/abs(-850-pball[0]))))
if (pball[1]<0):    #if the ball is in the bottom half of the field
    b_ang=360-math.degrees(math.atan((abs(0-pball[1])/abs(-850-pball[0]))))
if (pball[1]==0):    #if the ball is in the middle (rare case)
    b_ang=0
    

b_ang=math.radians(b_ang)
#print(b_ang)
#print(math.degrees(b_ang))

#CALCULATE CONTROL POINT P3
d=float(config["PATH"]["D"]) #distance before kicking the ball
p3=(pball[0]+d*math.cos(b_ang),pball[1]+d*math.sin(b_ang))
#print(p3)

#CALCULATE CONTROL POINT P2
dis=float(config["PATH"]["DIS"]) #control point distance
p2=(p3[0]+dis*math.cos(b_ang),p3[1]+dis*math.sin(b_ang))
print(p2)

#DIVIDE PATH INTO n POINT
print('number of point ') 
n=eval(input())

# INITIALIZE VALUES
waypoints=[]

waypoints.append((p0[0],p0[1])) #FIRST POINT
for i in range(1,n+1):
    t=(i)*(1/n) #t=0->1
    #quadratic bezier eqn
    
    wx=(1-t)**3*p0[0]+3*(1-t)**2*t*p1[0]+3*(1-t)*t**2*p2[0]+t**3*p3[0]
    wy=(1-t)**3*p0[1]+3*(1-t)**2*t*p1[1]+3*(1-t)*t**2*p2[1]+t**3*p3[1]

    waypoints.append((wx,wy))

#waypoints.append((mapx(pball[0]),mapy(pball[1])))

#POINT INJECTION
end_point=[]
start_point=[]
end_point = (pball[0],pball[1])
start_point = tuple(x for x in waypoints[-1])
vector=(end_point[0]-start_point[0],end_point[1]-start_point[1])
num_points_that_fit=math.ceil(float(config["PATH"]["D"])/float(config["POINT_INJECTION"]["POINT_DIST"]))
vector= vector/np.linalg.norm(vector)*float(config["POINT_INJECTION"]["POINT_DIST"])

for i in range(1, num_points_that_fit):
    waypoints.append(start_point+vector*i)

    
waypoints.append((pball[0],pball[1]))


final_waypoints  = [[w[0], w[1]] for w in waypoints] #FINAL POINT

# CALCULATE PATH DISTANCE - W[2] mm
final_waypoints [0].append(0)
for i, w in enumerate(final_waypoints [1:], start=1):
    w.append(final_waypoints [i-1][2] + math.sqrt((w[0]-final_waypoints [i-1][0])**2 + (w[1]-final_waypoints [i-1][1])**2))
    


    # CALCULATE CURVATURE - W[3] mm^-1
final_waypoints [0].append(0.0001)
final_waypoints [-1].append(0.0001)
for i, w in enumerate(final_waypoints [1:-1], start=1):
    w[0] += 0.0001
    w[1] += 0.0001
    
    k1 = .5*(w[0]+0.00001**2 + w[1]**2 - final_waypoints [i-1][0]**2 - final_waypoints [i-1][1]**2) / (w[0]+0.00001 - final_waypoints [i-1][0])
    k2 = (w[1] - final_waypoints [i-1][1]) / (w[0]+0.000001 - final_waypoints [i-1][0])
    b = .5*(final_waypoints [i-1][0]**2 - 2*final_waypoints [i-1][0]*k1 + final_waypoints [i-1][1]**2 - final_waypoints [i+1][0]**2 + 2*final_waypoints [i+1][0]*k1 - waypoints [i+1][1]**2) / (waypoints [i+1][0]*k2 - waypoints [i+1][1] + waypoints [i-1][1] - waypoints [i-1][0]*k2)
    a = k1 - k2*b
    r = math.sqrt((w[0]-a)**2 + (w[1]-b)**2)
    w.append(1/r)
    
 

# CALCULATE DESIRED VELOCITY - W[4] (this is the old velocity) mm/s
for w in final_waypoints :
    w.append(min(float(config["VELOCITY"]["MAX_VEL"]), float(config["VELOCITY"]["TURNING_CONST"])/w[3])) #MAX_VEL mm/s
    


# ADD ACCELERATION LIMITS - W[5] 

final_waypoints[-1].append(0) #SET FINAL POINT VELOCITY TO 0
#recalculate the new velocity with acceleration
for i, w in enumerate(reversed(final_waypoints [:-1]), start=1): #not take the final value #MAX_ACCEL mm/s^2
    w.append(min(w[4], math.sqrt(final_waypoints [-i][5]**2+2*float(config["VELOCITY"]["MAX_ACCEL"])* \
                                  math.sqrt((w[0]-final_waypoints [-i][0])**2 + (w[1]-final_waypoints [-i][1])**2)))) #Kinematic equation   
final_waypoints [0][5] = float(config["VELOCITY"]["STARTING_VEL"]) #set the begining point with the starting velocity


for i in range(2, num_points_that_fit-4):
    final_waypoints [-i][5] = float(config["VELOCITY"]["MAX_VEL"]) #mm/s


# WRITE RESULTS TO FILE
with open(config["FILE"]["FILE_LOCATION"], "w+") as file:
    for w in final_waypoints :
        file.write(str(w[0]) + "," + str(w[1]) + "," + str(w[5]) +   "\n")



# DISPLAY COLOR-CODED IMAGE OF PATH
start_pos = (field.shape[0]/2, field.shape[1]/2) #show the robot in the middle of the screen

cv2.circle(field,(int(start_pos[1]+mapx(p0[0])),int(start_pos[0]-mapy(p0[1]))),5,(150, 255, 0),-1)
cv2.circle(field,(int(start_pos[1]+mapx(p1[0])),int(start_pos[0]-mapy(p1[1]))),5,(255, 255, 0),-1)
cv2.circle(field,(int(start_pos[1]+mapx(p2[0])),int(start_pos[0]-mapy(p2[1]))),5,(255, 123, 0),-1)
cv2.circle(field,(int(start_pos[1]+mapx(p3[0])),int(start_pos[0]-mapy(p3[1]))),5,(0,0,255),-1)
cv2.circle(field,(int(start_pos[1]+mapx(pball[0])),int(start_pos[0]-mapy(pball[1]))),5,(255,0,0),1)

for i in range(1, len(final_waypoints)):
   cv2.circle(field,(int(start_pos[1]+mapx(final_waypoints[i][0])),int(start_pos[0]-mapy(final_waypoints[i][1]))),2, (255*(1-final_waypoints[i-1][5]/float(config["VELOCITY"]["MAX_VEL"])), 0,
                   255*final_waypoints[i-1][5]/float(config["VELOCITY"]["MAX_VEL"])), -1)
    

cv2.imshow("Field", field)

cv2.waitKey()
