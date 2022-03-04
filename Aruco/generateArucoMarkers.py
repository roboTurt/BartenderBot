import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl


aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50) # selected aruco dict

fig = plt.figure()
# generate 4 markers in a 2x2 grid
nx = 2  
ny = 2
#draw markers
plt.subplots_adjust( 
                    wspace=0.5, 
                    hspace=0.9)

for i in range(1, nx*ny+1):
    ax = fig.add_subplot(ny,nx, i)
    img = aruco.drawMarker(aruco_dict,i, 220,10)
    plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
    ax.axis("off")

#save and display
plt.savefig("./markers.pdf")
plt.show()