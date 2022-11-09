import cv2
import numpy as np


luminosity = [1000,1000]
x_min_robotarium = 0
x_max_robotarium = 2.4

y_min_robotarium = 0
y_max_robotarium = 1.74

board_scale = 1000 # to align the units in cm
inverse_scale = 1/board_scale
light_sources = [[0.25*x_max_robotarium*board_scale, 0.25*y_max_robotarium*board_scale],[0.75*x_max_robotarium*board_scale, 0.75*y_max_robotarium*board_scale]]
decay = .5

def pop_sensor_mesh(h,w):

    x = np.linspace(x_min_robotarium*board_scale, x_max_robotarium*board_scale, h)
    y = np.linspace(y_min_robotarium*board_scale, y_max_robotarium*board_scale, w)
    X, Y = np.meshgrid(x, y,sparse=False)
    Z = np.zeros((w,h))
    for k in range(len(light_sources)):
        dist = np.sqrt(np.square(X -light_sources[k][0]) + np.square(Y -light_sources[k][1])) 
        Z += luminosity[k] - 20*decay*np.log(dist) #(luminosity[k]/(4 * np.pi * (dist ** decay)))
    return Z

img = pop_sensor_mesh(1920,1080)
img = cv2.applyColorMap(cv2.flip(img.astype(np.uint8),0), cv2.COLORMAP_JET)
cv2.imshow("frame",img)
cv2.waitKey(0)

