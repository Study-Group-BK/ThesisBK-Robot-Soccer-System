import numpy as np
import cv2
from cv2 import aruco
import BQ
# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_50)

# Create grid board object we're using in our stream
cam = cv2.VideoCapture(1, cv2.CAP_DSHOW)
BQ._camset.cam_set(cam,set=1)
while(cam.isOpened()):
    # Capturing each frame of our video stream
    ret, QueryImg = cam.read()
    if ret == True:
        # grayscale image
        gray = cv2.cvtColor(QueryImg, cv2.COLOR_BGR2GRAY)
    
        # Detect Aruco markers
        corners,ids,_ = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
        POINT=[]
        # Make sure all 5 markers were detected before printing them out
        if ids is not None:
            # Print corners and ids to the console
            for i, corner in zip(ids, corners):
                #print('ID: {}; Corners: {}'.format(i, corner))
                # print(corner)
                # POINT = np.append(POINT,corner,axis=0)
                # print(POINT)
                if i==5:
                    A=np.array([[corner[0][0][0],corner[0][0][1]]]).T
                    B=np.array([[corner[0][1][0],corner[0][1][1]]]).T
                    C=np.array([[corner[0][2][0],corner[0][2][1]]]).T
                    D=np.array([[corner[0][3][0],corner[0][3][1]]]).T
                    POINT.append(A)
                    POINT.append(B)
                    POINT.append(C)
                    POINT.append(D)
            print(POINT)
            QueryImg=BQ._draw.drawPoint(QueryImg, POINT)

            # Outline all of the markers detected in our image
            #QueryImg = aruco.drawDetectedMarkers(QueryImg, corners, ids, borderColor=(0, 0, 255))
            # cX=int(corners[0])
            # cY=int(corners[1])
            # cv2.circle(QueryImg, (cX, cY), 10, (0, 0, 255), -1)
            # print(corners[0][0][0][0])
        
    # Display our image
    cv2.imshow('QueryImage', QueryImg)
    # Exit at the end of the video on the 'q' keypress
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cam.release()
del(cam)
cv2.destroyAllWindows()