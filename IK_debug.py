from sympy import *
from time import time
from mpmath import radians
import numpy as np
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generate test cases can be added to the test_cases dictionary
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[[[2.153,0.00,1.946],
                  [0,0,0,1]],
                  [1.85,0,1.946],
                  [0,0,0,0,0,0]],
              5:[[[2.15265,1.24975,2.18523],
                  [0.834202,0.0630774,0.196304,0.511462]],
                  [1.87542,1.15702,2.10555],
                  [0.55,0.29,-0.44,1.05,-0.26,0.91]],
              6:[[[1.58696,1.14884,1.70597],
                  [0.0262192,-0.0866705,0.307325,0.947287]],
                  [1.34574,0.973799,1.65134],
                  [0.63,-0.12,0.31,0.00,-0.37,0.00]]}

def solve_r36 (R36):
    alpha = 0
    beta  = 0
    gamma = 0
    return alpha, beta, gamma

def homo_transform (alpha, a, d, theta):
    T = Matrix([[           cos(theta),           -sin(theta),           0,             a],
                [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d], 
                [sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d], 
                [                    0,                     0,          0,              1]])
    return T

def rot_x (q):
    T = Matrix([[1,      0,       0, 0],
                [0, cos(q), -sin(q), 0],
                [0, sin(q),  cos(q), 0],
                [0,      0,       0, 1]])
    return T

def rot_y (q):
    T = Matrix([[ cos(q), 0, sin(q), 0],
                [      0, 1,      0, 0],
                [-sin(q), 0, cos(q), 0],
                [0,      0,       0, 1]])
    return T

def rot_z (q):
    T = Matrix([[cos(q), -sin(q), 0, 0],
                [sin(q),  cos(q), 0, 0],
                [     0,       0, 1, 0],
                [0,      0,       0, 1]])
    return T


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    ########################################################################################
    ## Insert IK code here starting at: Define DH parameter symbols

    ## 
    
                # Define DH param symbols
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')

            
            # Joint angle symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')


      
            # Modified DH params
    s = {alpha0:     0, a0:      0, d1:  0.75,               
         alpha1: -pi/2, a1:   0.35, d2:     0,
         alpha2:     0, a2:   1.25, d3:     0,
         alpha3: -pi/2, a3: -0.054, d4:   1.5,
         alpha4:  pi/2, a4:      0, d5:     0,
         alpha5: -pi/2, a5:      0, d6:     0,
         alpha6:     0, a6:      0, d7: 0.303, q7: 0}

            
            # Define Modified DH Transformation matrix



            # Create individual transformation matrices
    T01 = homo_transform(alpha0, a0, d1, q1)
    T12 = homo_transform(alpha1, a1, d2, q2)
    T23 = homo_transform(alpha2, a2, d3, q3)
    T34 = homo_transform(alpha3, a3, d4, q4)
    T45 = homo_transform(alpha4, a4, d5, q5)
    T56 = homo_transform(alpha5, a5, d6, q6)
    T6G = homo_transform(alpha6, a6, d7, q7)

    T01 = T01.subs(s)
    T12 = T12.subs(s)
    T23 = T23.subs(s)
    T34 = T34.subs(s)
    T45 = T45.subs(s)
    T56 = T56.subs(s)
    T6G = T6G.subs(s)

    R01 = rot_x(alpha0) * rot_z(q1)            
    R12 = rot_x(alpha1) * rot_z(q2)            
    R23 = rot_x(alpha2) * rot_z(q3)            

    R01 = R01.subs(s)
    R12 = R12.subs(s)
    R23 = R23.subs(s)

    R03 = simplify(R01 * R12 * R23)

    T03 = simplify(T01 * T12 * T23)
    T04 = simplify(T03 * T34)
    T06 = simplify(T04 * T45 * T56)
    T0G = simplify(T03 * T34 * T45 * T56 * T6G)

    Tcorr = simplify(rot_z(pi) * rot_y(-pi/2))
    print(Tcorr)

            
    # Extract end-effector position and orientation from request
    # px,py,pz = end-effector position
    # roll, pitch, yaw = end-effector orientation
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
         req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Calculate joint angles using Geometric IK method
    Trpy = simplify(rot_z(yaw) * rot_y(pitch) * rot_x(roll) * Tcorr.T)

    wx = px - d7 * Trpy[0,2]   
    wy = py - d7 * Trpy[1,2]   
    wz = pz - d7 * Trpy[2,2]   
    wx = wx.subs(s)
    wy = wy.subs(s)
    wz = wz.subs(s)

    rad1 = atan2(wy, wx)
    #theta1 = rad1 * 180 / pi
    theta1 = rad1
            
    u = a2
    v = sqrt(a3 * a3 + d4 * d4)
    c1 = sqrt(wx * wx + wy * wy) - a1
    c2 = wz - d1
    w2 = c1 * c1 + c2 * c2
    w = sqrt(w2) 
 
    rad2 = -(atan2(c2, c1) + acos((u*u + w2- v*v)/(2*u*w)))
    rad2 = rad2.evalf(subs=s)
    theta2 = rad2 + np.pi / 2
    #theta2 = rad2 * 180 / np.pi

    rad3 = pi/2 - acos((u*u + v*v - w2)/(2*u*v)) + atan2(a3, d4)
    rad3 = rad3.evalf(subs=s)
    #theta3 = rad3 * 180 / np.pi
    theta3 = rad3

    R34 = rot_x(alpha3) * rot_z(q4)            
    R45 = rot_x(alpha4) * rot_z(q5)            
    R56 = rot_x(alpha5) * rot_z(q6)            
    R36 = simplify(R34 * R45 * R56)
    R36 = R36.subs(s)
    R36 = R36.evalf(subs={q4: test_case[2][3], q5: test_case[2][4], q6: test_case[2][5]})
    print(R36)

    R36 = simplify(R03.T * Trpy)
    R36 = R36.subs(s)
    R36 = R36.evalf(subs={q1: rad1, q2: rad2, q3: rad3})

    print(R36)
            
    if R36[1,2] == 1:
       rad4 = 0
       rad5 = 0
       rad6 = atan2(-R36[0,1], R36[0,0])
    elif R36[1,2] == -1:
       rad4 = 0
       rad5 = pi
       rad6 = atan2(R36[0,1], -R36[0,0])
    else:
       rad4 = atan2( R36[2,2], -R36[0,2])
       rad6 = atan2(-R36[1,1],  R36[1,0])
       rad5 = atan2(R36[1,0]/cos(rad6), R36[1,2])

    #rad4, rad5, rad6 = tf.transformations.euler_from_matrix(R36.tolist(), 'ryzy')

    theta4 = rad4
    theta5 = rad5
    theta6 = rad6


    ## Ending at: Populate response for the IK request
    
    ########################################################################################
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    my_ee = T0G * Matrix([[0],[0],[0],[1]])
    my_ee = my_ee.evalf(subs={q1:rad1, q2:rad2, q3:rad3, q4:rad4, q5:rad5, q6:rad6})

    print("ee location")
    print(my_ee)

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    #your_wc = [1,1,1] # <--- Load your calculated WC values in this array
    #your_ee = [1,1,1] # <--- Load your calculated end effector value from your forward kinematics
    your_wc = [wx,wy,wz] 
    your_ee = [my_ee[0],my_ee[1],my_ee[2]] 
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    print(theta4, theta5, theta6)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple posisiotns. It is best to add your forward kinmeatics to \
           \nlook at the confirm wether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
