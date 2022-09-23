from textwrap import wrap
import ServerWrapper
import random
import math
import csv

wrapper = ServerWrapper.ServerWrapper(1)

start = wrapper.get_position_global()
file = open("data.csv","a",newline="")
csvwriter = csv.writer(file)
csvwriter.writerow(["Actual X","Actual Y","Target X","Target Y","Odom X","Odom Y","Camera X","Camera Y","Camera Corrected X","Camera Corrected Y"])

def find_distance(target,pos):
    x = pos[0][0]
    y = pos[0][1]

    return math.sqrt(math.pow((target[0] - x), 2) + math.pow((target[1] - y), 2))

for i in range(0,15):
    print("Test Number: ",i)
    x = random.random() * 2.413
    y = random.random() * 1.7526
    print("X:",x*39.3701)
    print("Y:",y*39.3701)
    wrapper.set_points([[x,y]])
    while find_distance([x,y],wrapper.get_position()) > .05:
        wrapper.step()
    wrapper.set_velocities([[0,0,0]])
    wrapper.step()
    current_pos = wrapper.get_position()
    odom_data = wrapper.get_odom_pos()
    actual_x = input("Input X: ")
    actual_y = input("Input Y: ")
    csvwriter.writerow([actual_x,actual_y,x,y,odom_data[0][0],odom_data[0][1],current_pos[0][0],current_pos[0][1],current_pos[0][0]-start[0][0],current_pos[0][1]-start[0][1]])

file.close()
    