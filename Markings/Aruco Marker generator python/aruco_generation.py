# import numpy as np
# import cv2

# # Select type of aruco marker (size)
# aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

# # Create an image from the marker
# # second param is ID number
# # last param is total image size
# img = cv2.aruco.drawMarker(aruco_dict, 0, 600)
# cv2.imwrite("test_marker.jpg", img)

# # Display the image to us
# cv2.imshow('frame', img)
# # Exit on any key
# cv2.waitKey(0)
# cv2.destroyAllWindows()

import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
from numba import prange
import pandas as pd
import numpy as np

# aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

# fig = plt.figure()
# nx = 4
# ny = 3
# for i in prange(1, nx*ny+1):
#     ax = fig.add_subplot(ny,nx, i)
#     img = aruco.drawMarker(aruco_dict,i, 800)
#     plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
#     ax.axis("off")
# plt.savefig("C:\\Users\\DELL\\Desktop\\aruco marker")
# plt.show()

frame=cv2.imread("C:\\Users\\DELL\\Desktop\\aruco marker\\Figure_1.png")
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters =  aruco.DetectorParameters_create()
corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
plt.figure()
plt.imshow(frame_markers)
for i in prange(len(ids)):
    c = corners[i][0]
    plt.plot([c[:,0].mean()], [c[:,1].mean()], "o", label = "id={0}".format(ids[i]))
plt.legend()
plt.show()