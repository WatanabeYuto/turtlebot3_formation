#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist

#clique-based formation control of 4 agents with relative measurements
def relpose_sub():

    for i in range(3):
        rospy.Subscriber("tb3_%i/rel_polar_vector" % i, PoseArray, callback, callback_args=i)
    
    rospy.spin()


#compute input & publish velocity for th3_num
def callback(data, tb3_num):
    global stateC, desC, x_star, y_star, n

    n = 3 #the number of agents in one clique
    d = 2 #dimension

    #desired configuration of agents
    #equilateral triangle
    x_star = 0.6 * np.array([0,0.5,1,0.5])
    y_star = 0.6 * np.array([0,-0.5*math.sqrt(3),0,0.5*math.sqrt(3)])

    #states & destinations of agents in each clique
    stateC = np.zeros(shape=(d,n))
    desC = np.zeros(shape=(d,n))

    #define cliques 
    clique = np.zeros(shape=(2,n))
    clique[0,:] = np.array([0,1,2])
    clique[1,:] = np.array([0,2,3])

    xi_star = x_star[tb3_num]
    yi_star = y_star[tb3_num]
    i_star = np.array([xi_star,yi_star])
    i_star = i_star.reshape(-1,1)

    #states & destinations of agents in each clique
    stateC = np.zeros(shape=(d,n))
    desC = np.zeros(shape=(d,n))

    #initialize the control input
    u = np.zeros(shape=(2,1))

    if tb3_num == 1:
        clq_id = 0
        stateC, desC = rearrange_vector(data,clq_id)
        #average
        ave_stateC = np.mean(stateC,axis=1)
        ave_stateC = ave_stateC.reshape(-1,1)
        ave_desC = np.mean(desC,axis=1)
        ave_desC = ave_desC.reshape(-1,1)
        
        #center
        cen_stateC = np.mat(stateC - ave_stateC*np.array([1,1,1]))
        cen_desC = np.mat(desC - ave_desC*np.array([1,1,1]))

        tmp_mat = cen_desC.dot(np.transpose(cen_stateC))

        #SVD
        U,sigma,V_T = np.linalg.svd(tmp_mat)

        V = np.transpose(V_T)
        D = np.diag([1,np.linalg.det(U)*np.linalg.det(V)])
        #rotation matrix
        R_C = V.dot(D).dot(np.transpose(U))

        #compute the control imput
        u = ave_stateC + R_C.dot(i_star-ave_desC)

    if tb3_num == 3:
        clq_id = 1
        stateC, desC = rearrange_vector(data,clq_id)
        #average
        ave_stateC = np.mean(stateC,axis=1)
        ave_stateC = ave_stateC.reshape(-1,1)
        ave_desC = np.mean(desC,axis=1)
        ave_desC = ave_desC.reshape(-1,1)
        
        #center
        cen_stateC = np.mat(stateC - ave_stateC*np.array([1,1,1]))
        cen_desC = np.mat(desC - ave_desC*np.array([1,1,1]))

        tmp_mat = cen_desC.dot(np.transpose(cen_stateC))

        #SVD
        U,sigma,V_T = np.linalg.svd(tmp_mat)

        V = np.transpose(V_T)
        D = np.diag([1,np.linalg.det(U)*np.linalg.det(V)])
        #rotation matrix
        R_C = V.dot(D).dot(np.transpose(U))

        #compute the control imput
        u = ave_stateC + R_C.dot(i_star-ave_desC)

    if tb3_num == 0 or tb3_num == 2:
        
        for j in range(2):
            clq_id = j
            stateC, desC = rearrange_vector(data,clq_id)
            #average
            ave_stateC = np.mean(stateC,axis=1)
            ave_stateC = ave_stateC.reshape(-1,1)
            ave_desC = np.mean(desC,axis=1)
            ave_desC = ave_desC.reshape(-1,1)
            
            #center
            cen_stateC = np.mat(stateC - ave_stateC*np.array([1,1,1]))
            cen_desC = np.mat(desC - ave_desC*np.array([1,1,1]))

            tmp_mat = cen_desC.dot(np.transpose(cen_stateC))

            #SVD
            U,sigma,V_T = np.linalg.svd(tmp_mat)

            V = np.transpose(V_T)
            D = np.diag([1,np.linalg.det(U)*np.linalg.det(V)])
            #rotation matrix
            R_C = V.dot(D).dot(np.transpose(U))
            #compute the control imput
            u = u + ave_stateC + R_C.dot(i_star-ave_desC)
    
    twist = Twist()
    twist.linear.x = u[0]
    twist.angular.z = u[1] #approximately

    vel_pub = rospy.Publisher('tb3_%i/cmd_vel' % tb3_num, Twist, queue_size=10)
    print(twist)

    vel_pub.publish(twist)


#compute reltive positions of agents against agent <tb3_num>
def rearrange_vector(data,clq_id):

    #define cliques 
    clique = np.zeros(shape=(2,n))
    clique[0,:] = np.array([0,1,2])
    clique[1,:] = np.array([0,2,3])

    k = 0
    for j in  clique[clq_id,:]:
        j = int(j)

        xrel = data.poses[j].position.x
        yrel = data.poses[j].position.y
        stateC[0][k] = xrel
        stateC[1][k] = yrel
        desC[0][k] = x_star[j]
        desC[1][k] = y_star[j]

        k = k+1
    return stateC, desC


if __name__ == '__main__':
    try:
        rospy.init_node('six_so2_rel_controller', anonymous=True)
        relpose_sub()
    except ropsy.ROSInterruptException: pass