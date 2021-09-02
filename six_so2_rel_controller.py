#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist

#clique-based formation control of 6 agents with relative measurements
#this controller is based on "K.Sakurama et.al, multi-agent coordination via distributed pattern matching(2019), IEEE TAC"

def relpose_sub():

    for i in range(6):
        rospy.Subscriber("tb3_%i/rel_polar_vector", % i, PoseArray, callback, callback_args=i)
    
    rospy.spin()


#compute input & publish velocity for th3_num
def callback(data, tb3_num):
    n = 3 #the number of agents in one clique
    d = 2 #dimension
    
    global stateC, desC, x_star, y_star

    #desired configuration of agents
    #equilateral triangle
    x_star = np.array([0,1,2,0.5,1.5,1])
    y_star = np.array([0,0,0,0.5*math.sqrt(3),0.5*math.sqrt(3), math.sqrt(3)])

    #states & destinations of agents in each clique
    stateC = np.zeros(shape=(d,n))
    desC = np.zeros(shape=(d,n))

    #initialize the control input
    u = 0
    
#以下，場合わけを再考すべき
    #agents on vertecies of the triangle
    if tb3_num == 0 or tb3_num == 2 or tb3_num == 5:

    #agents on sides of the triangle
    if tb3_num == 1 or tb3_num == 3 or tb3_num == 4:


    


#compute reltive position of states and destinations

if __name__ = '__main__':
    try:
        rospy.init_node('six_so2_rel_controller', anonymous=True)
        relpose_sub()
    except ropsy.ROSInterruptException: pass
