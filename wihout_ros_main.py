#!/usr/bin/env python
#import rospy
import sys
import numpy as np
import cv2
from Exploration import exploration_r
#print("worked",sys.argv)

image=255*np.ones((1000,1000,3))
#print(image)

parent_orignal_data=[]

#multiply the coordinates by 100 to convert from m to cm


#Inputting coordinates

print(" Start coordinates ")
s_y=float(input(" x_coordinate : "))
s_x=float(input(" y_coordinate : "))
theta_s=int((input(" theta_start : ")))
print(" Goal coordinates ")
g_y=float(input(" x_coordinate : "))
g_x=float(input(" y_coordinate : "))
#theta_g=int(input(" theta_goal : "))
print(" Other parameters ")
#step=int(input(" step size : "))
#radius=int(input(" radius size : "))
#clearance=int(input(" clearance size : "))
rpm1=int(input(" rpm1 : "))
rpm2=int(input("rpm2 :"))
#print(s_x,s_y,g_x,g_y)0
start=[int(s_y*100),int(s_x*100)]
goal=[int(g_y*100),int(g_x*100)]
#start=[int(sys.argv[1]*100),int(sys.argv[2]*100)]
#goal=[int(sys.argv[4]*100),int(sys.argv[5]*100)]
i=0
image=image.astype(np.uint8)
#theta_s=int(sys.argv[3])
#clearance=int(sys.argv[6])
#rpm1=int(sys.argv[7])
#rpm2=int(sys.argv[8])
radius=0.0033*100
# object of class
clearance=5
exploration=exploration_r(start,goal,theta_s,clearance,radius,rpm1,rpm2)

exploration.obstacles_form(image)
r=0.033
l=0.35

theta_d=0
out = cv2.VideoWriter('AStar_exploration.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 200, (1000,1000))






image_list=[]


state=exploration.obstacle_prone_area(image)
counter=1

while state:
    i=i+1
    counter=2
    

    pos_0,pos_1,theta_d=exploration.frontier_list(image)
    cv2.imshow("img",image)
    out.write(image)
    if cv2.waitKey(1) & 0xFF==ord('q'):
       break
    #print("baher",pos_0,pos_1,theta_d)
    image_list.append(image) #appending all image frames in list
    # calls all moves
    
    image,pos_r_0,pos_r_1=exploration.action_space(image,pos_0,pos_1,theta_d)
    if exploration.goal_reached():
        break




    exploration.expanding(pos_0,pos_1,theta_d)# checks if node has been expanded/visited or not or not

#out.release()
#cv2.destroyAllWindows()

if counter ==2:
    path,image_list=exploration.backtracking(image,out,image_list)
    print(path)
    #for img in image_list:
    #    cv2.imshow("img",img)
    #    out.write(img)
        #if cv2.waitKey(1) & 0xFF==ord('q'):


            #break
else:
    print("Goal or start point in the obstacle prone area")
out.release()
cv2.destroyAllWindows()
