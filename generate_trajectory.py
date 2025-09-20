import numpy as np
import argparse

from frankapy import FrankaArm
from frankapy import utils
from record_trajectory import create_formated_skill_dict
import pickle as pkl
from frankapy import FrankaConstants as FC



# See run_dynamic_joints, may be able to generate trajectory with the different planners, and then record said trajectory
# TODO tomorrow with Abhi, we can discuss, do IK?

# for picking up ingredients, something like the run_dynamic_pose could be useful, we could generate the trajectory with some
# planner and then execute, may be able to constrain x and y with it

# DO NOT RUN BEFORE DEBUGGING, causing suddent jolts in the arm

def generate_trajectory(start_pose, end_pose, args, fa, dt=0.01):
    fa.log_info('Visiting Pose 1')
    fa.goto_pose(start_pose, use_impedance=False) # supposed to be a bit more accurate?

    q1 = fa.get_joints()

    fa.log_info('Visiting Pose 2')
    fa.goto_pose(end_pose, use_impedance=False, duration=5)
    
    q2 = fa.get_joints()
    ts = np.arange(0, args.time, dt)

    joints_traj = [utils.min_jerk(q1, q2, t, args.time) for t in ts] #could maybe use another planner
    #print(len(joints_traj))
    #print("ts", ts)
    #print("joint_traj", joints_traj)
    skill_dict = create_formated_skill_dict(joints_traj, ts)
    with open(args.file, 'wb') as pkl_f:
        pkl.dump(skill_dict, pkl_f)
        print("Did save skill dict: {}".format(args.file))

if __name__ == "__main__":
    fa = FrankaArm()
    parser = argparse.ArgumentParser()
    parser.add_argument('--time', '-t', type=float, default=3)
    parser.add_argument('--file', '-f', default='assembly2bin4.pkl') #filename to save generated trajectory
    args = parser.parse_args()

    # before anything, reset joints to home position
    fa.reset_joints()

    home_pose = FC.HOME_POSE
    start_pose = home_pose.copy()
    start_pose.translation = [0.45931555, 0.0836659, 0.55068304]
    # start_pose.rotation = np.matmul(start_pose.rotation, np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])) # add 90 degree rotation
    
    end_pose = home_pose.copy()
    end_pose.translation = [0.21549992, 0.21637546, 0.48050308]
    

    end_pose.rotation = np.matmul(end_pose.rotation, np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])) # add 90 degree rotation
    # end_pose.rotation = np.matmul(end_pose.rotation, np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])) # add 180 degree rotation - DOES NOT WORK DUE TO LIMITS

    
    generate_trajectory(start_pose, end_pose, args, fa)

    # BIN LOCATIONS:
    # Bin6: [0.61549992, 0.21637546, 0.48050308]
    # Bin5: [0.41549992, 0.21637546, 0.48050308]
    # Bin4: [0.21549992, 0.21637546, 0.48050308]

    # ASSEMBLY LOCATION: [0.45931555, 0.0836659, 0.55068304]
