import configparser
import math
import numpy as np
import cv2 
import time

#  MAPPING FUNCTION (from 1000 by 1700 to 500 by 850 area)
def mapx(x):
    return int((x - -850) * (425 - -425) / (850 - -850) + -425)
def mapy(y):    
    return int((y - -500) * (250 - -250) / (500 - -500) + -250)



#FIND CLOSEST POINT
def closest(current):
    global path, pos
    mindist = (0, math.sqrt((path[0][0] - pos[0]) ** 2 + (path[0][1] - pos[1]) ** 2)) #INITIAL MINDIST (the first waypoint to the starting pos
    for i, p in enumerate(path[current:],current): #i is index, p is the x, y of the path
        dist = math.sqrt((p[0]-pos[0])**2 + (p[1]-pos[1])**2)
        if dist < mindist[1]: #if the new distance is smaller than the previous one, the new one is the new mindist
            mindist = (i, dist) #UPDATE THE MINDIST
            
    return mindist[0] #return the waypoint index


#Find look ahead point
#1. E is the starting point of the line segment
#2. L is the end point of the line segment
#3. C is the center of circle (robot location)
#4. r is the radius of that circle (lookahead distance)

def lookahead():
    global path, t, t_i, pos
    
    for i, p in enumerate(reversed(path[:-1])): #path are read from the file, i index, p is path x and y coordinate
        i_ = len(path)-2 - i
        d = (path[i_+1][0]-path[i_][0], path[i_+1][1]-path[i_][1]) #d = L - E (Direction vector of ray, from start to end)

        f = (p[0]-pos[0], p[1]-pos[1]) #f = E - C (Vector from center sphere to ray start)
        
        a = sum(j**2 for j in d)
        b = 2*sum(j*k for j,k in zip(d,f))
        c = sum(j**2 for j in f) - float(config["PATH"]["LOOKAHEAD"])**2
        disc = b**2 - 4*a*c
        if disc >= 0:
            disc = math.sqrt(disc)
            t1 = (-b + disc)/(2*a)
            t2 = (-b - disc)/(2*a)
            
            if 0<=t1<=1:
                    t = t1
                    t_i = i_
                    return p[0]+t*d[0], p[1]+t*d[1]
                    
            if 0<=t2<=1:
                    t = t2
                    t_i = i_
                    return p[0]+t*d[0], p[1]+t*d[1] #look ahead Point = E + (t value of intersection) * d
    t = 0
    t_i = 0
    
    return path[closest(current)][0:2] 

#CALCULATE THE CURVATURE ARC
def curvature(lookahead):
    global path, pos, angle

    side = np.sign(math.sin(3.1415/2 - angle)*(lookahead[0]-pos[0]) - math.cos(3.1415/2 - angle)*(lookahead[1]-pos[1])) #cross product if negative(right), positive (left)
    a = -math.tan(3.1415/2 - angle)
    c = math.tan(3.1415/2 - angle)*pos[0] - pos[1]
    x = abs(a*lookahead[0] + lookahead[1] + c) / math.sqrt(a**2 + 1)
    return side * (2*x/(float(config["PATH"]["LOOKAHEAD"])**2))

#CALCULATE LEFT AND RIGHT WHEEL VELOCITY
def turn(curv, vel, trackwidth):
    return  [vel*(2+curv*trackwidth)/2, vel*(2-curv*trackwidth)/2]
    
    

#VISUALIZE THE SIMULATION
def draw_path(img):
    global path, start_pos
    
    cv2.circle(img, (int(start_pos[0]+mapx(path[0][0])), int(start_pos[1]-mapy(path[0][1]))), 2,
               (255*(1-path[0][2]/float(config["VELOCITY"]["MAX_VEL"])), 0, 255*path[0][2]/float(config["VELOCITY"]["MAX_VEL"])), -1)
    for i in range(1, len(path)):
        cv2.circle(img, (int(start_pos[0]+mapx(path[i][0])), int(start_pos[1]-mapy(path[i][1]))), 2,
                   (255*(1-path[i-1][2]/float(config["VELOCITY"]["MAX_VEL"])), 0, 255*path[i-1][2]/float(config["VELOCITY"]["MAX_VEL"])), -1)
        cv2.line(img, (int(start_pos[0]+mapx(path[i][0])), int(start_pos[1]-mapy(path[i][1]))),
                 (int(start_pos[0]+mapx(path[i-1][0])), int(start_pos[1]-mapy(path[i-1][1]))),
                 (255*(1-path[i-1][2]/float(config["VELOCITY"]["MAX_VEL"])), 0, 255*path[i-1][2]/float(config["VELOCITY"]["MAX_VEL"])), 1)

def draw_robot(img):
    tmp = img.copy()
    cv2.drawContours(tmp, [np.array([(start_pos[0] + (mapx(pos[0])+length/4*math.sin(angle)-width/4*math.cos(angle)),
                                      start_pos[1] + (mapy(pos[1])+length/4*math.cos(angle)+width/4*math.sin(angle))*-1),
                                     (start_pos[0] + (mapx(pos[0])+length/4*math.sin(angle)+width/4*math.cos(angle)),
                                      start_pos[1] + (mapy(pos[1])+length/4*math.cos(angle)-width/4*math.sin(angle))*-1),
                                     (start_pos[0] + (mapx(pos[0])-length/4*math.sin(angle)+width/4*math.cos(angle)),
                                      start_pos[1] + (mapy(pos[1])-length/4*math.cos(angle)-width/4*math.sin(angle))*-1),
                                     (start_pos[0] + (mapx(pos[0])-length/4*math.sin(angle)-width/4*math.cos(angle)),
                                      start_pos[1] + (mapy(pos[1])-length/4*math.cos(angle)+width/4*math.sin(angle))*-1)])
                     .reshape((-1,1,2)).astype(np.int32)], 0, (0, 255, 255), 2) #Draw robot
    cv2.circle(tmp, (int(start_pos[0]+mapx(pos[0])), int(start_pos[1]-mapy(pos[1]))), int(radius), (0, 255, 0), 1) #look ahead circle
    cv2.circle(tmp, (int(start_pos[0]+mapx(path[close][0])), int(start_pos[1]-mapy(path[close][1]))), 3,
               (255*(1-path[close][2]/float(config["VELOCITY"]["MAX_VEL"])), 9, 255*path[close][2]/float(config["VELOCITY"]["MAX_VEL"])), -1)
    cv2.circle(tmp, (int(start_pos[0]+mapx(look[0])), int(start_pos[1]-mapy(look[1]))), 4, (255, 255, 255), -1) #goal point (start point: doi diem vao giua hinh)

    cv2.circle(tmp, (int(start_pos[0]+mapx(pos[0])), int(start_pos[1]-mapy(pos[1]))),1,(255,255,0),-1) #robot center

    cv2.line(tmp,(int(start_pos[0]+mapx(pos[0])), int(start_pos[1]-mapy(pos[1]))),
            (int(start_pos[0] + (mapx(pos[0])+(50)*math.sin(angle)-math.cos(angle))),
            int(start_pos[1] - (mapy(pos[1])+(50)*math.cos(angle)+math.sin(angle)))),(255,255,255),1)
            
    try:
        x3 = (mapx(pos[0])+mapx(look[0]))/2
        y3 = -(mapy(pos[1])+mapy(look[1]))/2
        q = math.sqrt((mapx(pos[0])-mapx(look[0]))**2 + (mapy(pos[1])-mapy(look[1]))**2)
        x = x3 - math.sqrt(1/curv**2 - (q/2)**2) * (mapy(pos[1])-mapy(look[1]))/q * np.sign(curv)
        y = y3 - math.sqrt(1/curv**2 - (q/2)**2) * (mapx(pos[0])-mapx(look[0]))/q * np.sign(curv)
        cv2.circle(tmp, (int(x+start_pos[0]), int(y+start_pos[1])), int(abs(1/curv)), (255,255,255), 1) #drawing steering curve
    except:
        pass

    cv2.line(tmp,
             (int(start_pos[0] + (mapx(pos[0])+length/4*math.sin(angle)-width/4*math.cos(angle))),
              int(start_pos[1] + (mapy(pos[1])+length/4*math.cos(angle)+width/4*math.sin(angle))*-1)),
             (int(start_pos[0] + (mapx(pos[0])+(length/4+wheels[0]/5)*math.sin(angle)-width/4*math.cos(angle))),
              int(start_pos[1] + (mapy(pos[1])+(length/4+wheels[0]/5)*math.cos(angle)+width/4*math.sin(angle))*-1)),
             (0,0,255), 2) #draw left wheel velocity visual
    cv2.line(tmp,
             (int(start_pos[0] + (mapx(pos[0])+length/4*math.sin(angle)+width/4*math.cos(angle))),
              int(start_pos[1] + (mapy(pos[1])+length/4*math.cos(angle)-width/4*math.sin(angle))*-1)),
             (int(start_pos[0] + (mapx(pos[0])+(length/4+wheels[1]/5)*math.sin(angle)+width/4*math.cos(angle))),
              int(start_pos[1] + (mapy(pos[1])+(length/4+wheels[1]/5)*math.cos(angle)-width/4*math.sin(angle))*-1)),
             (0,0,255), 2) #draw right wheel velocity visual

    cv2.imshow("img", tmp)
    cv2.waitKey(5)


#READING THE CONFIG FILE
config = configparser.ConfigParser()
config.read("config.ini")

#READ THE "path.csv" FILE 
with open(config["FILE"]["FILE_LOCATION"]) as file:
    path = [([float(x) for x in line.split(",")]) for line in file.readlines()]


width = float(config["ROBOT"]["TRACKWIDTH"]) #mm
length = float(config["ROBOT"]["LENGTH"]) #mm
radius=int(config["PATH"]["LOOKAHEAD"]) * 0.5 

pos = (path[0][0],path[0][1]) #STARTING POSITION (relative to the start_pos)

angle = math.atan2(path[1][0], path[1][1]) #STARTING ANGLE (based on the first 2 points)

wheels = [0,0] #starting wheel velocity
t = 0  #t value determine wether the look ahead circle intersect with line segment
t_i = 0
dt=float(config["ROBOT"]["SAMPLING_TIME"]) #SAMPLING TIME

field = cv2.imread(config["FIELD_IMAGE"]["FILE_LOCATION"])
img = np.zeros((field.shape[0], field.shape[1], 3), np.uint8)
start_pos = (field.shape[1]/2, field.shape[0]/2) #show the robot in the middle of the screen
draw_path(img)
cv2.imshow("img", img)
cv2.waitKey(10)

velocity=[]
position=[]
robot_angle=[]
target=[]

current=0

itt = 0
while closest(current) != len(path)-1: 
    close = closest(current)
    current=close
    look = lookahead() 
    curv = curvature(look) #if t_i>close else 0.00001
    vel = path[close][2] #get velocity of the robot in the "path.csv" file
    last_wheels = wheels
    wheels = turn(curv, vel, width) #LEFT AND RIGHT WHEEL VELOCITY
    
  

    #RATE LIMITER 
    for i, w in enumerate(wheels):
        wheels[i] = last_wheels[i] + min(float(config["ROBOT"]["MAX_VEL_CHANGE"])*dt, max(-float(config["ROBOT"]["MAX_VEL_CHANGE"])*dt, w-last_wheels[i]))
        velocity.append(wheels)

        target.append(vel)
        target_vel  = [t for t in target]  

    wheel_velocity  = [[w[0], w[1]] for w in velocity]  
    
    
    point=(mapx(pos[0]+(50)*math.sin(angle)-math.cos(angle))-mapx(pos[0]),mapy( pos[1]+(50)*math.cos(angle)+math.sin(angle))-mapy(pos[1]))
    if point[1] < 0:
        robot_angle.append(360+math.degrees(math.atan2(point[1],point[0])))
    else:
        robot_angle.append(math.degrees(math.atan2(point[1],point[0])))
    direction  = [d for d in robot_angle]

    

    #UPDATE THE ROBOT  POSITION BY USING ODOMETRY
    #ODOMETRY X=DISTANCE*COSINE(ANGLE) Y=DISTANCE*SINE(ANGLE); DISTANCE=VELOCITY*T   
    #print(math.degrees(angle))
    pos = (pos[0] + (wheels[0]+wheels[1])/2*dt * math.sin(angle), pos[1] + (wheels[0]+wheels[1])/2*dt * math.cos(angle))
    angle += math.atan((wheels[0]-wheels[1])/(width/1)*dt)

    
    position.append(pos)
    robot_position  = [[a[0], a[1]] for a in position] 

    draw_robot(img)
    #print(itt)
    itt += 1 # number of loop

#WRITE DIRECTION OF THE ROBOT INT target_vel.csv
with open(config["FILE"]["FILE_LOCATION_TARGET_VEL"], "w+") as file:
    for t in target :
        file.write(str(t)  + "\n")

#WRITE DIRECTION OF THE ROBOT INT dir.csv
with open(config["FILE"]["FILE_LOCATION_DIRECTION"], "w+") as file:
    for d in direction :
        file.write(str(d)  + "\n")

#WRITE position OF THE ROBOT INT pos.csv
with open(config["FILE"]["FILE_LOCATION_POS"], "w+") as file:
    for a in robot_position :
        file.write(str(a[0]) + "," + str(a[1]) + "\n")

#WRITE VELOCITY OF THE ROBOT INT vel.csv
with open(config["FILE"]["FILE_LOCATION_VEL"], "w+") as file:
    for w in wheel_velocity :
        file.write(str(w[0]) + "," + str(w[1]) + "\n")

cv2.waitKey()


