from sympy import *
from time import time
from mpmath import radians
import tf
import numpy as np

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
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
              4:[],
              5:[]}


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
    ## 

    ## Insert IK code here!

    ### Create symbols
    
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    # The Thetas (joint angle - angle between Xi-1 to Xi), and one for the gripper
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    # (link offset - signed distance from Xi-1 to Xi measured along Zi)
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    # (Link length - distance from Zi-1 to Zi, measured along Xi-1 where Xi-1 is perpendicular to both Zi-1 and Zi)
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    # (Twist angle - angle between Zi-1 and Z- measured about Xi-1 in a right hand sense)
    
    ### Create Modified DH parameters
    
    # Parameters for KUKA KR210 #
    
    # This information is located in the URDF files
    # URDF Folder > kr210.urdf.xacro
    
    # Create a dictionary of DH Parameters
    DH_TABLE = {alpha0:         0,     a0:      0,    d1: 0.75,    q1:           q1,
                alpha1: -np.pi/2.,     a1:   0.35,    d2: 0,       q2: q2 - np.pi/2., 
                alpha2:         0,     a2:   1.25,    d3: 0,       q3:           q3,
                alpha3: -np.pi/2.,     a3: -0.054,    d4: 1.5,     q4:           q4,
                alpha4:  np.pi/2.,     a4:      0,    d5: 0,       q5:           q5,
                alpha5: -np.pi/2.,     a5:      0,    d6: 0,       q6:           q6,
                alpha6:         0,     a6:      0,    d7: 0.303,   q7:            0}
                # The decimal after the integer indicates float

            
    
    def transformationMatrix(alpha,a,d,q):
        TF = Matrix([[             cos(q),            -sin(q),            0,       a],
                  [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                  [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                  [                   0,                   0,            0,       1]])
        return TF
    
    # Homogeneous Transforms for between neighboring links: 
    # Create individual transformation matrices

    T0_1 = transformationMatrix(alpha0, a0, d1, q1).subs(DH_TABLE) # Base Link to Link1
    T1_2 = transformationMatrix(alpha1, a1, d2, q2).subs(DH_TABLE) # Link1 to Link2
    T2_3 = transformationMatrix(alpha2, a2, d3, q3).subs(DH_TABLE) # Link2 to Link3
    T3_4 = transformationMatrix(alpha3, a3, d4, q4).subs(DH_TABLE) # Link3 Link to Link4
    T4_5 = transformationMatrix(alpha4, a4, d5, q5).subs(DH_TABLE) # Link4 to Link5       
    T5_6 = transformationMatrix(alpha5, a5, d6, q6).subs(DH_TABLE) # Link5 to Link6       
    T6_G = transformationMatrix(alpha6, a6, d7, q7).subs(DH_TABLE) # Link6 to Gripper             
    
    # Composition of Homogeneous Transforms 
    T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
    # this is the transformation matrix from the base link to the end effector
    
    r, p, y = symbols('r p y')

    rot_x = Matrix([[ 1,             0,         0],
                    [ 0,        cos(r),   -sin(r)],
                    [ 0,        sin(r),    cos(r)]])
        # Rotation about X axis (Roll)
        
    rot_y = Matrix([[  cos(p),        0,   sin(p)],
                    [       0,        1,        0],
                    [ -sin(p),        0,   cos(p)]])
        # Rotation about Y axis (Pitch)
    
    rot_z = Matrix([[ cos(y),   -sin(y),        0],
                    [ sin(y),    cos(y),        0],
                    [ 0,              0,        1]])
        # Rotation about Z axis (Yaw)
        
    ROT_G = rot_z * rot_y * rot_x 
    ### Compensate for rotation discrepancy between DH parameters and Gazebo
    
    R_corr = rot_z.subs(y, radians(180)) * rot_y.subs(p, radians(-90))
    # Rotate about the Z axis by 180deg
    # Rotate about the Y axis by -90deg   
    
    ROT_G = ROT_G * R_corr
    
    # Evaluate numerically at '0' to verify FK section with Rviz (NB pg. 84 - 'Debugging FK')
    
    # gripper_testing = print(T_total.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
        # correct at d1=.75, d2=0
        
    ##### Your IK code here 

        # px,py,pz = end-effector position
     # roll, pitch, yaw = end-effector orientation
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])

       # Compensate for rotation discrepancy between DH parameters and Gazebo

    ROT_G = ROT_G.subs({'r':roll, 'p':pitch,'y':yaw})

    EE = Matrix([[px],
                 [py],
                 [pz]])
    # Put the end effector positions into a Matrix
    
    # Now need to extract Nx, Ny, Nz from Rrpy matrix to give us the WC position
   
    WC = EE - 0.303 * ROT_G[:,2]

    Wx = WC[0]
    Wy = WC[1]
    Wz = WC[2]
    
    theta1 = atan2(Wy, Wx)  
    # theta1
    
    # Now, let's determine theta2:
    
    SIDE_S = 0.054
    # Taken from the Z measurement from the URDF (joints 4 and 5), lines 344, 355. This is a constant.
    
    JOINTS_4AND5_X = 1.5 # (0.96 + 0.54)
    # This is taken from line 344, 351(joints 4 and 5) of the URDF file 'kr210.urdf.xacro'. This is a constant.
    
    # SIDES
    SIDE_A = sqrt(SIDE_S**2 + JOINTS_4AND5_X**2)
    # SIDE_A is a constant length
    sideB = sqrt((Wz-0.75)**2 + (sqrt(Wx**2 + Wy**2) -0.35)**2)          
    # sideB is a variable length
    SIDE_C = 1.25
    # SIDE_C is a constant length
    
    # ANGLES
    # We're going to leverage the Law of Cosines: c**2 = a**2 + b**2 - 2*a*b*cos(C) -> where C is the desired angle
    angleA = acos((SIDE_C**2 + sideB**2 - SIDE_A**2)/(2*SIDE_C*sideB))
    # angleA is a variable angle
    angleB = acos((SIDE_C**2 + SIDE_A**2 - sideB**2)/(2*SIDE_C*SIDE_A))
    # angleB is a variable angle            
    angleC = acos((SIDE_A**2 + sideB**2 - SIDE_C**2)/(2*SIDE_A*sideB))
    # angleC is a variable angle            
    
    angleG = atan2((Wz - 0.75), sqrt((Wx-0.35)**2 + Wy**2))
    # I've defined 'angleG' as the angle from joint2 pointing along the X axis
        # where the hypoteneuse hits the WC. Theta2(or q2) plus angleG, plus angleA
        # equals pi/2, a right angle.
    
    # We can now define theta2 since we know that theta2 + angleA + angleG = 90deg; therefore:
    theta2 = np.pi/2. - angleG - angleA     
    #theta2

    # Now, let's determine theta3:
    
    # We'll use the same logic from theta2 of defining a 90degree angle, then deducing theta3 from that.
    
    ANGLE_S = atan2(SIDE_S, JOINTS_4AND5_X)
    # I've defined 'ANGLE_S' as the angle above SIDE_A that makes angleB into a 90deg angle
        # when the Kuka KR210 is in its zero configuration. This is a constant.
        # Note, this could also have been done with 'asin(SIDE_S/SIDE_A)'

    theta3 = np.pi/2. - ANGLE_S - angleB     
    # theta3
    
    # Use tranpose instead of inv("LU")
    # Note: R0_6 = Rrpy
    
    R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    # this cuts our homogeneous transforms down to 3x3 rotation matrices
    R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
    R3_6 = R0_3.T * ROT_G
    
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
    theta6 = atan2(-R3_6[1,1],R3_6[1,0])
    



    FK = T0_G.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6:theta6})
        
   
        
    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [Wx,Wy,Wz] # <--- Load your calculated WC values in this array
    your_ee = [FK[0,3],FK[1,3],FK[2,3]] # <--- Load your calculated end effector value from your forward kinematics
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
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
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
    test_case_number = 3

    test_code(test_cases[test_case_number])
