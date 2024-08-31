# trajectories.py
import spatialmath as sm
import roboticstoolbox as rtb
import numpy as np




# ---------- define_poses ---------- #

def define_robot_poses():
    """
    Define the start pose, end pose, test poses, and the center point as SE(3) transformations.

    Returns
    -------
    dict : A dictionary containing all the defined poses.
    """

    # Define the Start Pose
    start_pose_rpy = [0.0, 1.570796326794, 0.0]  # [roll, pitch, yaw]
    start_pose_position = [0.2, 0.3, 0.07]       # [x, y, z]
    start_pose = sm.SE3(start_pose_position) * sm.SE3.RPY(start_pose_rpy, order='xyz')

    # Define the End Pose
    end_pose_rpy = [0.0, 1.570796326794, 0.0]    # [roll, pitch, yaw]
    end_pose_position = [0.378, -0.05, 0.15]     # [x, y, z]
    end_pose = sm.SE3(end_pose_position) * sm.SE3.RPY(end_pose_rpy, order='xyz')

    # Define Test Pose 1
    test_pose_1_rpy = [0.0, 1.570796326794, 0.0]  # [roll, pitch, yaw]
    test_pose_1_position = [0.2, 0.205, 0.075]    # [x, y, z]
    test_pose_1 = sm.SE3(test_pose_1_position) * sm.SE3.RPY(test_pose_1_rpy, order='xyz')

    # Define Test Pose 2
    test_pose_2_rpy = [0.0, 1.570796326794, 0.0]  # [roll, pitch, yaw]
    test_pose_2_position = [0.375, 0.205, 0.26]   # [x, y, z]
    test_pose_2 = sm.SE3(test_pose_2_position) * sm.SE3.RPY(test_pose_2_rpy, order='xyz')
    
    # Define the center point (C)
    center_point_c = (test_pose_1.t + test_pose_2.t)/2

    # Store all poses in a dictionary
    robot_poses = {
        "StartPose": start_pose,
        "EndPose": end_pose,
        "TestPose1": test_pose_1,
        "TestPose2": test_pose_2,
        "CenterPointC": center_point_c
    }

    return robot_poses



# ---------- define_robot ---------- #

def define_robot(d=0.08):
    """define robot_ARM"""
    #from base{0} to {1}}
    E1 = rtb.ET.tz(0.2433)
    E2 = rtb.ET.Rz()
    #from base{1} to {2}}
    E3 = rtb.ET.Rx(-np.pi/2)
    E4 = rtb.ET.Rz(-np.pi/2)
    E5 = rtb.ET.Rz()
    #from base{2} to {3}}
    E6 = rtb.ET.tx(0.2)
    E7 = rtb.ET.Ry(np.pi)
    E8 = rtb.ET.Rz(np.pi/2)
    E9 = rtb.ET.Rz()
    #from base{3} to {4}}
    E10 = rtb.ET.tx(0.087)
    E11 = rtb.ET.ty(-0.2276)
    E12 = rtb.ET.Rx(np.pi/2)
    E13 = rtb.ET.Rz()
    #from base{4} to {5}}
    E14 =  rtb.ET.Rx(np.pi/2)
    E15 = rtb.ET.Rz()
    #from base{5} to {6}}
    E16 = rtb.ET.ty(0.0615)
    E17 = rtb.ET.Rx(-np.pi/2)
    E18 = rtb.ET.Rz()
    #from base{6} to {E}}
    E19 = rtb.ET.tz(d)
    #compute ets for Forward kinematics
    ets = E1 * E2 * E3 * E4 * E5 * E6 * E7 * E8 * E9 * E10 * E11 * E12 * E13 * E14 * E15 * E16 * E17 * E18 * E19
    robot_ARM = rtb.Robot(ets)

    # Define the joint limits in degrees (as a 2xN array)
    qlim_degrees = np.array([
        [-360, -150, -3.5, -360, -124, -360],  # Minimum limits
        [ 360,  150,  300,  360,  124,  360]   # Maximum limits
    ])

    # Convert the joint limits to radians
    robot_ARM.qlim = np.deg2rad(qlim_degrees)

    return robot_ARM



# ---------- ikine_with_limits ---------- #

def ikine_with_limits(robot: rtb.Robot, T: sm.SE3, q0: np.ndarray = None, itr: int = 50) -> np.ndarray:
    """
    Calculates the inverse kinematics for a given end-effector pose (T)
    while considering joint limits and allowing for multiple iterations to find a valid solution.

    Parameters
    ----------
    robot : rtb.Robot
        The robot model for which the IK is calculated.
    T : sm.SE3
        The desired end-effector pose as an SE(3) object.
    q0 : np.ndarray, optional
        Initial guess for the joint angles [in radians]. If None, the robot's default is used.
    itr : int, optional
        Number of iterations to attempt in case the initial IK solution is invalid.

    Returns
    -------
    q : np.ndarray
        The joint angles [in radians] that achieve the given pose, considering joint limits.
    """
    if q0 is None:
         q0 = np.zeros(robot.n)  

    success = False
    for i in range(itr):
        # 1. Solve the inverse kinematics with the given initial guess q0
        ik_solution = robot.ikine_LM(T, q0=q0,joint_limits=True)
        if ik_solution.success:
            q = ik_solution.q
            success = True
            break
        else:
            # If IK fails, generate a random initial guess within joint limits and retry
            q0 = np.random.uniform(robot.qlim[0, :], robot.qlim[1, :])

    if not success:
        raise ValueError(f"Inverse kinematics did not converge after {itr} iterations.")

    # 2. Clip the joint angles to be within joint limits using np.clip
    if np.any(q < robot.qlim[0, :]) or np.any(q > robot.qlim[1, :]):
      print("Joint angles are out of the allowed limits.")
      q = np.clip(q, robot.qlim[0, :], robot.qlim[1, :])

    return q



# ---------- Question 3.1 ---------- #

def generateLinearTrajectory(A: sm.SE3, B: sm.SE3, n:int, robot:rtb.Robot) -> np.ndarray:
     """
     Given a start pose A, an end pose B and the number of points n, 
     this function returns the joint angles required to move the end effector from pose A to pose B in a straight line.
     The straight line is defined by the line connecting the positions of A and B.
     The orientation of the end effector frame can be any smooth interpolation between the orientations at A and B.

     Parameters
     ----------
     A
          the start pose of the end effector frame with respect to the base frame given as an SE(3) object
     B
          the end pose of the end effector frame with respect to the base frame given as an SE(3) object
     n
          number of points to generate in the trajectory including the start and end poses
     robot
          a Robot object representing the robot

     Returns
     -------
     q
          An array of shape 6,n containing n sets of joint angles [in radians], corresponding to the n points in the trajectory

     """
     
      # 1. Generate a linear Cartesian trajectory from A to B
     trajectory = rtb.ctraj(A, B, n)

     # 2. Initialize an array to store the joint angles
     q = np.zeros((n, robot.n))

     # 3. Solve the inverse kinematics for each point in the trajectory
     for i in range(n):
          try:
               q[i, :] = ikine_with_limits(robot, trajectory[i], q0=q[i-(i>0),:] ,itr=50)
          except ValueError as e:
               print(f"IK failed at point {i+1}/{n}: {e}")
               break  

     return q.T

# ---------- Question 3.2 ---------- #

def generateCircularTrajectory(A:sm.SE3, B:sm.SE3, C:np.ndarray, n:int, robot:rtb.Robot) -> np.ndarray:
     """
     Given a start pose A, an end pose B, a point C at the centre of the circle and the number of points n, 
     this function returns the joint angles required to move the end effector from pose A to pose B along a circular path.
     The circular path is defined by the circle passing through the positions of A and B with its centre at C.
     The orientation of the end effector frame can be any smooth interpolation between the orientations at A and B.

     Parameters
     ----------
     A
          the start pose of the end effector frame with respect to the base frame given as an SE(3) object
     B
          the end pose of the end effector frame with respect to the base frame given as an SE(3) object
     C
          an array of shape 3, representing the position [in meters] of the centre of the circle in the base frame
     n
          number of points to generate in the trajectory including the start and end poses
     robot
          a Robot object representing the robot

     Returns
     -------
     q
          An array of shape 6,n containing n sets of joint angles [in radians], corresponding to the n points in the trajectory

     """
     # Vectors from C to A and C to B
     CA = A.t - C
     CB = B.t - C

     # Normal vector to the plane of the circle
     normal_vector = np.cross(CA, CB)
     normal_vector /= np.linalg.norm(normal_vector)

     # Calculate the angle between CA and CB
     angle_AB = np.arccos(np.clip(np.dot(CA, CB) / (np.linalg.norm(CA) * np.linalg.norm(CB)), -1.0, 1.0))
     angles = np.linspace(0, angle_AB, n)

     q = np.zeros((n, robot.n))

     for i, theta in enumerate(angles):
          # Compute the rotation matrix around the normal vector by theta
          R =  sm.SO3.AngleAxis(theta, normal_vector).R

          # Calculate the new point on the circle
          point_on_circle = C + R @ CA

          # Create a new pose by interpolating the orientation from A to B
          current_pose = sm.SE3(point_on_circle) * sm.SE3.RPY(np.array(A.rpy()) * (1 - i/n) + np.array(B.rpy()) * (i/n), order='xyz')

          # Solve the inverse kinematics for the current pose
          try:
               q[i, :] = ikine_with_limits(robot, current_pose, q0=q[i-(i>0),:], itr=50)
          except ValueError as e:
               print(f"IK failed at point {i+1}/{n}: {e}")
               break  

     return q.T


# ---------- Question 3.3 ---------- #

def jointAnglesAtTestPoints() -> np.ndarray:
     """
     This function must return two sets of valid and safe joint angles. 
     One corresponding to pose at Test_1 and the other corresponding to pose at Test_2

     Returns
     -------
     q
          An array of shape 6,2 containing joint angles [in radians] corresponding to the two poses
     """
     poses = define_robot_poses()
     # Create a robot model
     robot = define_robot(d=0.08)

     try:
          # Compute joint angles for Test_1 and Test_2 using inverse kinematics
          joint_angles_test1 = ikine_with_limits(robot, poses['TestPose1'])
          joint_angles_test2 = ikine_with_limits(robot, poses['TestPose2'])

          # Combine the joint angles into a single array
          q = np.vstack((joint_angles_test1, joint_angles_test2)).T 

     except Exception as error:
          print(f"An error occurred while calculating joint angles: {error}")
          q = np.zeros((6, 2))  # Return a zero array as a fallback

     return q
