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
        rospy.Subscriber("tb3_%i/rel_polar_vector" % i, PoseArray, callback, callback_args=i)

    rospy.spin()


#compute input & publish velocity for th3_num
def callback(data, tb3_num):
    global stateC, desC, x_star, y_star, n

    n = 3 #the number of agents in one clique
    d = 2 #dimension

    #desired configuration of agents
    #equilateral triangle
    x_star = np.array([0,1,2,0.5,1.5,1])
    y_star = np.array([0,0,0,0.5*math.sqrt(3),0.5*math.sqrt(3), math.sqrt(3)])
    xi_star = np.array([x_star[tb3_num], y_star[tb3_num]])
    xi_star = xi_star.reshape(-1,1)


    #states & destinations of agents in each clique
    stateC = np.zeros(shape=(d,n))
    desC = np.zeros(shape=(d,n))

    #define cliques
    clique = np.zeros(shape=(3,n))
    clique[0,:] = np.array([0,1,3])
    clique[1,:] = np.array([1,2,4])
    clique[2,:] = np.array([3,4,5])

    #clique indexes
    vertex_clq_id = np.array([0,1,2]) #clique of agent 0,2,5
    side_clq_id = np.zeros(shape=(3,2))
    side_clq_id[0,:] = np.array([0,1]) #cliques of agent 1
    side_clq_id[1,:] = np.array([0,2]) #agent 3
    side_clq_id[2,:] = np.array([1,2]) #agent 4

    #initialize the control input
    u = np.zeros(shape=(2,1))

    #agents on vertecies of the triangle
    if tb3_num == 0 or tb3_num == 2 or tb3_num == 5:
        if tb3_num == 0:
            agent_id = 0
        elif tb3_num == 2:
            agent_id = 1
        elif tb3_num == 5:
            agent_id = 2

        clq_id = int(vertex_clq_id[agent_id])
        stateC, desC = rearrange_vector(data, clq_id)

        ave_stateC = np.mean(stateC,axis=1)
        ave_stateC = ave_stateC.reshape(-1,1)
        ave_desC = np.mean(desC,axis=1)
        ave_desC = ave_desC.reshape(-1,1)

        cen_stateC = np.mat(stateC - ave_stateC*np.array([1,1,1]))
        cen_desC = np.mat(desC - ave_desC*np.array([1,1,1]))

        tmp_mat = cen_desC.dot(np.transpose(cen_stateC))

        #SDV
        U,sigma,V_T = np.linalg.svd(tmp_mat)

        V = np.transpose(V_T)
        D = np.diag([1,np.linalg.det(U)*np.linalg.det(U)])
        #rotation matrix
        R_C = V.dot(D).dot(np.transpose(U))

        #compute the control imput
        u = ave_stateC + R_C.dot(xi_star-ave_desC)


    #agents on sides of the triangle
    if tb3_num == 1 or tb3_num == 3 or tb3_num == 4:
        if tb3_num == 1:
            agent_id = 0
        elif tb3_num == 3:
            agent_id = 1
        elif tb3_num == 4:
            agent_id = 2


        clq_ids = side_clq_id[agent_id,:]

        for id in clq_ids:
            id = int(id)
            stateC, desC = rearrange_vector(data, id)

            ave_stateC = np.mean(stateC,axis=1)
            ave_stateC = ave_stateC.reshape(-1,1)
            ave_desC = np.mean(desC,axis=1)
            ave_desC = ave_desC.reshape(-1,1)

            cen_stateC = np.mat(stateC - ave_stateC*np.array([1,1,1]))
            cen_desC = np.mat(desC - ave_desC*np.array([1,1,1]))

            tmp_mat = cen_desC.dot(np.transpose(cen_stateC))

            #Svd
            U,sigma,V_T = np.linalg.svd(tmp_mat)

            V = np.transpose(V_T)
            D = np.diag([1,np.linalg.det(V)*np.linalg.det(U)])
            #rotation matrix
            R_C = V.dot(D).dot(np.transpose(U))
            #compute the control input
            u = u + ave_stateC + R_C.dot(xi_star-ave_desC)

    twist = Twist()
    twist.linear.x = u[0]
    twist.angular.z = u[1]

    vel_pub = rospy.Publisher('tb3_%i/cmd_vel' % tb3_num, Twist, queue_size=10)
    print(twist)

    vel_pub.publish(twist)


#compute reltive positions of agents againt agent <tb3_num> in one clique
def rearrange_vector(data, clq_id):

    #define cliques
    clique = np.zeros(shape=(3,n))
    clique[0,:] = np.array([0,1,3])
    clique[1,:] = np.array([1,2,4])
    clique[2,:] = np.array([3,4,5])
    k = 0 #j

    for j in  clique[clq_id,:]:
        j = int(j)
        xrel = data.poses[j].position.x
        yrel = data.poses[j].position.y

        stateC[0,k] = xrel
        stateC[1,k] = yrel
        desC[0,k] = x_star[j]
        desC[1,k] = y_star[j]
        k = k+1

    return stateC, desC


if __name__ == '__main__':
    try:
        rospy.init_node('six_so2_rel_controller', anonymous=True)
        relpose_sub()
    except ropsy.ROSInterruptException: pass
