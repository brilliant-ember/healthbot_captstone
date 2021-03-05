## https://docs.opencv.org/master/dd/d53/tutorial_py_depthmap.html



import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
imgL = cv.imread('left2.png',0)
imgR = cv.imread('right2.png',0)
stereo = cv.StereoBM_create(numDisparities=16, blockSize=5)
disparity = stereo.compute(imgL,imgR)
plt.imshow(disparity,'gray')
plt.show()
